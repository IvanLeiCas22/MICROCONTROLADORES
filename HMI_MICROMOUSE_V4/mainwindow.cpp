#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QBrush>
#include <QDataStream>
#include <QDebug>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QIntValidator>
#include <QLabel>
#include <QMessageBox>
#include <QMetaEnum>
#include <QMouseEvent>
#include <QPen>
#include <QTextBlock>

namespace {
constexpr quint8 SENSOR_DET_WALL_FRONT = 0x01;
constexpr quint8 SENSOR_DET_WALL_LEFT = 0x02;
constexpr quint8 SENSOR_DET_WALL_RIGHT = 0x04;
constexpr quint8 SENSOR_DET_WALL_DIAG_LEFT = 0x08;
constexpr quint8 SENSOR_DET_WALL_DIAG_RIGHT = 0x10;
constexpr quint8 SENSOR_DET_FLOOR_FRONT = 0x20;
constexpr quint8 SENSOR_DET_FLOOR_REAR = 0x40;

void setDetectionLabel(QLabel *label, bool active) {
  label->setText(active ? " " : "");
  label->setStyleSheet(
      active ? "background-color: #22c55e; border: 1px solid #15803d; border-radius: 4px;"
             : "background-color: transparent; border: 1px solid #4b5563; border-radius: 4px;");
}
} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      serialPort(new QSerialPort(this)), m_parser(new UnerbusParser(this)),
      udpSocket(nullptr), sensorUpdateTimer(new QTimer(this)),
      pwmUpdateTimer(new QTimer(this)) // Inicializar el nuevo temporizador
{
  ui->setupUi(this);

  // Inicializar QButtonGroup para los botones de navegación
  navigationButtonGroup = new QButtonGroup(this);
  navigationButtonGroup->setExclusive(
      true); // Solo un botón puede estar checked a la vez
  navigationButtonGroup->addButton(ui->btnHome,
                                   0); // ID 0 para la página de Inicio
  navigationButtonGroup->addButton(ui->btnSensors,
                                   1); // ID 1 para la página de Sensores
  navigationButtonGroup->addButton(ui->btnConfig,
                                   2); // ID 2 para la página de Configuración
  navigationButtonGroup->addButton(ui->btnControl,
                                   3); // ID 3 para la página de control
  navigationButtonGroup->addButton(
      ui->btnLabyrinth, 4); // ID 4 para la página de visual del laberinto
  // Conectar la señal buttonClicked del grupo al slot
  // on_navigationButtonClicked
  connect(navigationButtonGroup, &QButtonGroup::buttonClicked, this,
          &MainWindow::on_navigationButtonClicked);
  ui->stackedScreens->setCurrentIndex(0); // Página de inicio

  // Conectar señales y slots para la comunicación serie
  connect(serialPort, &QSerialPort::readyRead, this,
          &MainWindow::onSerialPort_ReadyRead);
  connect(serialPort, &QSerialPort::errorOccurred, this,
          &MainWindow::handleSerialError);

  // Conectar el parser
  connect(m_parser, &UnerbusParser::packetReceived, this,
          &MainWindow::onPacketReceived);

  // --- Conectar el temporizador de sensores ---
  connect(sensorUpdateTimer, &QTimer::timeout, this,
          &MainWindow::requestSensorData);
  connect(pwmUpdateTimer, &QTimer::timeout, this, &MainWindow::requestPwmData);

  updateSerialPortList();
  updateUIState(false, false);                   // Estado inicial: desconectado
  updateConnectionStatus("Desconectado", "red"); // Establecer estado inicial

  populateCMDComboBox();
  setupControlPage(); // Configurar la página de control
  setupConfigPage();
  setupActivitiesTab();

  QIntValidator *turnAngleValidator = new QIntValidator(-360, 360, this);
  ui->editTurnAngle->setValidator(turnAngleValidator);

  // ==========================================
  // INICIALIZACIÓN DEL SIMULADOR DE LABERINTO
  // ==========================================

  // 1. Crear el escenario e insertarlo en la vista del UI
  mazeScene = new QGraphicsScene(this);
  ui->mazeView->setScene(mazeScene);
  // Le pedimos interceptar los clicks de nuestro visor del laberinto
  ui->mazeView->viewport()->installEventFilter(this);
  // Inicializamos nuestras metas por defecto "apagadas" (-1)
  start_x = -1;
  start_y = -1;
  goal_x = -1;
  goal_y = -1;

  // Opcional: Le damos un fondo oscuro muy moderno al canvas
  mazeScene->setBackgroundBrush(QColor(20, 25, 30));
  // 2. Limpiar todos los mapas lógicos (llenarlos de 0)
  memset(sim_maze_map, 0, sizeof(sim_maze_map));
  memset(real_maze_map, 0, sizeof(real_maze_map));

  // 3. El robot nace en el centro lógico de la matriz de 15x15
  current_x = 7;
  current_y = 7;
  current_heading = HEADING_NORTH;

  // 4. Marcamos la celda en la que empezamos como "Visitada"
  sim_maze_map[current_x][current_y] |= CELL_VISITED;

  // 5. Preparar corazón autonómico
  autonomousTimer = new QTimer(this);
  connect(autonomousTimer, &QTimer::timeout, this, &MainWindow::autonomousTick);

  // Conectar radio buttons al repintado
  connect(ui->radioBtnRealView, &QRadioButton::clicked, this,
          &MainWindow::drawMaze);
  connect(ui->radioBtnRobotView, &QRadioButton::clicked, this,
          &MainWindow::drawMaze);

  // 6. Ordenamos pintar el mapa por primera vez
  drawMaze();
}

MainWindow::~MainWindow() {
  delete ui;
  delete navigationButtonGroup; // Liberar la memoria del QButtonGroup
}

/**
 * @brief Slot para manejar los clics en los botones de navegación.
 *        Cambia la página del QStackedWidget y asegura que el botón clicado
 * esté checked.
 * @param button Puntero al QAbstractButton que fue clicado.
 */
void MainWindow::on_navigationButtonClicked(QAbstractButton *button) {
  // Obtener el ID del botón (que hemos configurado para que sea el índice de la
  // página)
  int pageIndex = navigationButtonGroup->id(button);

  // Cambiar la página del stacked widget
  ui->stackedScreens->setCurrentIndex(pageIndex);

  // El QButtonGroup ya se encarga de que solo un botón esté checked si
  // setExclusive(true) Por lo tanto, no necesitamos setChecked(true) aquí
  // explícitamente para el botón clicado.
}

// --- Implementación de la lógica del puerto serie ---

void MainWindow::updateSerialPortList() {
  const auto ports = QSerialPortInfo::availablePorts();

  ui->comboBoxPorts->clear();
  for (const QSerialPortInfo &port : ports) {
    ui->comboBoxPorts->addItem(port.portName(), port.systemLocation());
  }
}

void MainWindow::updateUIState(bool serialConnected, bool udpConnected) {
  // Serie
  ui->comboBoxPorts->setEnabled(!serialConnected && !udpConnected);
  ui->btnRefreshPorts->setEnabled(!serialConnected && !udpConnected);
  ui->btnConnectSerie->setEnabled(!serialConnected && !udpConnected);
  ui->btnDisconnectSerie->setEnabled(serialConnected);

  // UDP
  ui->RemoteIpLineEdit->setEnabled(false);
  ui->RemotePortLineEdit->setEnabled(false);
  ui->localPortLineEdit->setEnabled(!udpConnected && !serialConnected);
  ui->btnConnectUDP->setEnabled(!udpConnected && !serialConnected);
  ui->btnDisconnectUDP->setEnabled(udpConnected);
}

void MainWindow::on_btnConnectSerie_clicked() {
  if (ui->comboBoxPorts->currentText().isEmpty()) {
    QMessageBox::critical(this, "Error",
                          "No hay un puerto serie seleccionado.");
    return;
  }

  serialPort->setPortName(ui->comboBoxPorts->currentText());
  serialPort->setBaudRate(QSerialPort::Baud115200);
  serialPort->setDataBits(QSerialPort::Data8);
  serialPort->setParity(QSerialPort::NoParity);
  serialPort->setStopBits(QSerialPort::OneStop);
  serialPort->setFlowControl(QSerialPort::NoFlowControl);

  if (serialPort->open(QIODevice::ReadWrite)) {
    updateUIState(true, false);
    updateConnectionStatus("Conectado por Serie", "green");
    // Solicitar período de PWM al conectar
    QTimer::singleShot(200, this, [this]() {
      sendUnerbusCommand(Unerbus::CommandId::CMD_GET_PWM_PERIOD);
    });
    QTimer::singleShot(300, this, [this]() {
      sendUnerbusCommand(Unerbus::CommandId::CMD_GET_ROBOT_STATUS);
    });
  } else {
    QMessageBox::critical(this, "Error de Conexión",
                          "No se pudo abrir el puerto: " +
                              serialPort->errorString());
    updateUIState(false, false);
    updateConnectionStatus("Desconectado",
                           "red"); // Asegurar estado en caso de error
  }
}

void MainWindow::on_btnDisconnectSerie_clicked() {
  if (serialPort->isOpen()) {
    serialPort->close();
  }
  updateUIState(false, false);
  updateConnectionStatus("Desconectado", "red");
}

void MainWindow::on_btnRefreshPorts_clicked() { updateSerialPortList(); }

void MainWindow::handleSerialError(QSerialPort::SerialPortError error) {
  if (error == QSerialPort::ResourceError) {
    QMessageBox::critical(
        this, "Error Crítico",
        "El puerto serie fue desconectado o cerrado inesperadamente.");
    on_btnDisconnectSerie_clicked(); // Llama a la lógica de desconexión para
                                     // resetear la UI
  }
}

void MainWindow::onSerialPort_ReadyRead() {
  // Pasamos los datos al parser
  const QByteArray data = serialPort->readAll();
  m_parser->processData(data);
  qDebug() << "Datos recibidos del puerto serie:" << data.toHex(' ');
}

// --- NUEVO SLOT PARA PROCESAR PAQUETES ---
void MainWindow::onPacketReceived(quint8 command, const QByteArray &payload) {
  // Usamos QDataStream para leer datos binarios de forma segura
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian); // ¡Muy importante! El
                                                  // protocolo es Little Endian

  qDebug() << "Paquete válido recibido! CMD:" << Qt::hex << command
           << "Payload:" << payload.toHex(' ');

  // Obtener el nombre del comando desde el enum
  QMetaEnum metaEnum = QMetaEnum::fromType<Unerbus::CommandId>();
  const char *commandNameCStr = metaEnum.valueToKey(command);
  QString commandName = commandNameCStr
                            ? QString(commandNameCStr)
                            : QString("Desconocido (0x%1)")
                                  .arg(command, 2, 16, QChar('0').toUpper());

  // Formateamos el mensaje y lo mostramos en el QPlainText "commsLog"
  QString logMessage = QString("Recibido CMD: %1, Payload: %2")
                           .arg(commandName)
                           .arg(QString(payload.toHex(' ')));
  ui->commsLog->appendPlainText(logMessage);

  if (ui->commsLog->document()->blockCount() >
      MAX_LOG_LINES) // Limita el número de líneas en el log
  {
    // Mueve el cursor al inicio del documento
    QTextCursor cursor(ui->commsLog->document()->findBlockByNumber(0));
    // Selecciona el bloque completo (la primera línea)
    cursor.select(QTextCursor::BlockUnderCursor);
    // Elimina la selección
    cursor.removeSelectedText();
    // Elimina el salto de línea que puede quedar al inicio
    cursor.deleteChar();
  }

  // Aquí va la lógica para manejar cada comando
  switch (static_cast<Unerbus::CommandId>(command)) {
  case Unerbus::CommandId::CMD_GET_ALIVE: {
    qDebug() << "Comando GET_ALIVE recibido.";
    break;
  }
  case Unerbus::CommandId::CMD_GET_IR_SENSOR_SNAPSHOT: {
    updateIrSensorsUI(payload);
    break;
  }

  case Unerbus::CommandId::CMD_GET_MPU_DATA: {
    updateMpuSensorsUI(payload);
    break;
  }

  case Unerbus::CommandId::CMD_ACK: {
    qDebug() << "Comando ACK recibido.";
    break;
  }

  case Unerbus::CommandId::CMD_UPDATE_MAZE_CELL: {
      if (payload.size() >= 4) {
          quint8 x, y_stm, walls, heading;
          stream >> x >> y_stm >> walls >> heading;

          const int y_qt = (MAZE_HEIGHT - 1) - static_cast<int>(y_stm);

          if (x < MAZE_WIDTH && y_stm < MAZE_HEIGHT && y_qt >= 0 && y_qt < MAZE_HEIGHT) {

              // Rescatar banderas exclusivas de Qt antes de sobreescribir
              uint8_t flags_exclusivas_qt = sim_maze_map[x][y_qt] & CELL_SPECIAL;

              // Actualizamos paredes y visitado del STM32, sumando las banderas de Qt
              sim_maze_map[x][y_qt] = walls | flags_exclusivas_qt;

              // Propagación simétrica de paredes a los vecinos
              if (walls & WALL_NORTH && y_qt > 0)
                  sim_maze_map[x][y_qt - 1] |= WALL_SOUTH;

              if (walls & WALL_SOUTH && y_qt < MAZE_HEIGHT - 1)
                  sim_maze_map[x][y_qt + 1] |= WALL_NORTH;

              if (walls & WALL_EAST && x < MAZE_WIDTH - 1)
                  sim_maze_map[x + 1][y_qt] |= WALL_WEST;

              if (walls & WALL_WEST && x > 0)
                  sim_maze_map[x - 1][y_qt] |= WALL_EAST;

              // Sincronizamos la posición y heading del cuadradito verde
              current_x = x;
              current_y = static_cast<uint8_t>(y_qt);
              current_heading = static_cast<Heading>(heading);

              drawMaze();
          }
      }
    break;
  }

  case Unerbus::CommandId::CMD_GET_MOTOR_PWM: {
    updatePwmUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_PWM_PERIOD: {
    if (payload.size() >= 2) {
      quint16 period;
      stream >> period;
      updatePwmControlRanges(period);
    }
    break;
  }
  case Unerbus::CommandId::CMD_GET_PID_GAINS: {
    updatePidNavUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_MAX_PWM_CORRECTION: {
    updateMaxPwmCorrectionUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_TURN_PID_GAINS: {
    updatePidTurnUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_TURN_VELOCITY_PID_GAINS: {
    updateTurnSpeedPID(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_TURN_TARGET_DPS: {
    updateTurnTargetDps(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_TURN_MAX_SPEED: {
    updateTurnMaxSpeedUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_TURN_MIN_SPEED: {
    updateTurnMinSpeedUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_MOTOR_BASE_SPEEDS: {
    updateMotorBaseSpeedsUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_MPU_CONFIG:
  case Unerbus::CommandId::CMD_SET_MPU_CONFIG: // El SET también devuelve la
                                               // configuración actual
  {
    updateMpuConfigUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_WALL_THRESHOLDS: {
    updateWallThresholdsUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_WALL_TARGET_ADC: {
    updateWallTargetAdcUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_ROBOT_STATUS: {
    updateRobotStatusUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_CRUISE_PARAMS: {
    updateCruiseParamsUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_BRAKING_PID_GAINS: {
    updatePidBrakingUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_BRAKING_PARAMS: {
    updateBrakingParamsUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_BRAKING_MAX_SPEED: {
    updateBrakingMaxSpeedUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_BRAKING_MIN_SPEED: {
    updateBrakingMinSpeedUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_BRAKING_DEAD_ZONE: {
    updateBrakingDeadZoneUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_YAW_ANGLE: {
    updateYawAngleUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_SMOOTH_TURN_CONFIG: {
    updateSmoothTurnSpeedsUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_DELAY_TICKS: {
    updateDelayTicksUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_SYNC_MAZE_COLUMN: {
      // Necesitamos 1(col) + 15(datos) + 1(x) + 1(y) + 1(heading) = 19 bytes
      if (payload.size() >= (MAZE_HEIGHT + 4)) {
          quint8 col;
          stream >> col;

          // 1. Extraemos y guardamos las 15 celdas de esta columna
          for (int i = 0; i < MAZE_HEIGHT; i++) {
              quint8 cell_data;
              stream >> cell_data;

              // MUY IMPORTANTE: Traducimos la Y de STM32 a la Y visual de Qt (invertida)
              int y_qt = (MAZE_HEIGHT - 1) - i;

              // Rescatamos las banderas de Qt (ej. celdas especiales marcadas en UI)
              uint8_t qt_flags = sim_maze_map[col][y_qt] & CELL_SPECIAL;
              sim_maze_map[col][y_qt] = cell_data | qt_flags;
          }

          // 2. Extraemos la posición y orientación actual del robot
          quint8 x, y_stm, heading;
          stream >> x >> y_stm >> heading;
          current_x = x;
          current_y = static_cast<uint8_t>((MAZE_HEIGHT - 1) - y_stm); // Traducir Y
          current_heading = static_cast<Heading>(heading);

          // 3. PING-PONG: ¿Faltan columnas?
          if (col < MAZE_WIDTH - 1) {
              requestMazeColumn(col + 1); // Pedimos la siguiente instantáneamente
          } else {
              // Si ya llegó la columna 14, terminamos. Actualizamos el dibujo.
              drawMaze();
              ui->commsLog->appendPlainText("¡Sincronización del laberinto completada con éxito!");
          }
      }
      break;
  }
  default:
    qDebug() << "Comando desconocido:" << Qt::hex << command;
    break;
  }
}

/**
 * @brief Rellena el ComboBox de comandos manualmente.
 * @note Se rellena manualmente para incluir solo los comandos de tipo GET
 *       que no requieren un payload, simplificando la interfaz de envío rápido.
 */
void MainWindow::populateCMDComboBox() {
  ui->CMDComboBox->clear();
  ui->CMDComboBox->addItem(
      "GET_ALIVE (0xF0)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_ALIVE));
  ui->CMDComboBox->addItem(
      "GET_BUTTON_STATE (0x12)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_BUTTON_STATE));
  ui->CMDComboBox->addItem(
      "GET_MPU_DATA (0xA2)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_MPU_DATA));
  ui->CMDComboBox->addItem(
      "GET_IR_SENSOR_SNAPSHOT (0xA0)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_IR_SENSOR_SNAPSHOT));
  ui->CMDComboBox->addItem(
      "GET_MOTOR_SPEEDS (0xA4)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_MOTOR_SPEEDS));
  ui->CMDComboBox->addItem(
      "GET_MOTOR_PWM (0xA6)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_MOTOR_PWM));
  ui->CMDComboBox->addItem(
      "GET_LOCAL_IP_ADDRESS (0xE0)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_LOCAL_IP_ADDRESS));
  ui->CMDComboBox->addItem(
      "SET_UART_BYPASS_CONTROL (0xDD)",
      static_cast<quint8>(Unerbus::CommandId::CMD_SET_UART_BYPASS_CONTROL));
  ui->CMDComboBox->addItem(
      "GET_PID_GAINS (0x41)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_PID_GAINS));
  ui->CMDComboBox->addItem(
      "GET_MAX_PWM_CORRECTION (0x43)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_MAX_PWM_CORRECTION));
  ui->CMDComboBox->addItem(
      "GET_MOTOR_BASE_SPEEDS (0x45)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_MOTOR_BASE_SPEEDS));
  ui->CMDComboBox->addItem(
      "GET_TURN_PID_GAINS (0x49)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_TURN_PID_GAINS));
  ui->CMDComboBox->addItem(
      "GET_WALL_THRESHOLDS (0x61)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_WALL_THRESHOLDS));
  ui->CMDComboBox->addItem(
      "GET_WALL_TARGET_ADC (0x63)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_WALL_TARGET_ADC));
}

void MainWindow::on_btnSendCMD_clicked() {
  if (!serialPort->isOpen() && !udpSocket) {
    QMessageBox::warning(
        this, "Error", "No hay un canal de comunicación activo (Serie o UDP).");
    return;
  }

  auto cmdId =
      static_cast<Unerbus::CommandId>(ui->CMDComboBox->currentData().toUInt());
  sendUnerbusCommand(cmdId);
}
void MainWindow::on_btnConnectUDP_clicked() {
  if (udpSocket)
    return; // Ya está activo

  // 1. Abrir el puerto local para escuchar (siempre necesario)
  localPort = ui->localPortLineEdit->text().toUShort();
  if (localPort == 0) {
    QMessageBox::warning(this, "Puerto Inválido",
                         "Por favor, ingrese un puerto local válido.");
    return;
  }

  udpSocket = new QUdpSocket(this);
  if (!udpSocket->bind(QHostAddress::Any, localPort)) {
    QMessageBox::critical(
        this, "Error de Conexión",
        "No se pudo abrir el puerto UDP. Puede que ya esté en uso.");
    udpSocket->deleteLater();
    udpSocket = nullptr;
    updateConnectionStatus("Desconectado",
                           "red"); // Asegurar estado en caso de error
    return;
  }
  connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::onUDPReadyRead);

  // 2. Comprobar si ya tenemos datos para una reconexión rápida
  QString remoteIpFromUI = ui->RemoteIpLineEdit->text();
  quint16 remotePortFromUI = ui->RemotePortLineEdit->text().toUShort();

  if (!remoteIpFromUI.isEmpty() && remotePortFromUI > 0) {
    // --- LÓGICA DE RECONEXIÓN ---
    // Usar los datos existentes para conectar inmediatamente
    remoteIp = remoteIpFromUI;
    remotePort = remotePortFromUI;

    updateUIState(false, true);
    updateConnectionStatus("Conectado por UDP", "green");
    ui->commsLog->appendPlainText(QString("Reconectado directamente a %1:%2")
                                      .arg(remoteIp)
                                      .arg(remotePort));
  } else {
    // --- LÓGICA DE PRIMERA CONEXIÓN ---
    // No hay datos, esperar el primer paquete del robot
    updateUIState(false, true);
    updateConnectionStatus(QString("Escuchando en puerto %1...").arg(localPort),
                           "orange");
  }
}

void MainWindow::on_btnDisconnectUDP_clicked() {
  if (udpSocket) {
    udpSocket->close();
    udpSocket->deleteLater();
    udpSocket = nullptr;

    // Limpiar variables y campos de la UI
    remoteIp.clear();
    remotePort = 0;

    updateUIState(false, false);
    updateConnectionStatus("Desconectado", "red");
  }
}

void MainWindow::onUDPReadyRead() {
  while (udpSocket->hasPendingDatagrams()) {
    QByteArray datagram;
    QHostAddress senderIp;
    quint16 senderPort;
    datagram.resize(udpSocket->pendingDatagramSize());
    udpSocket->readDatagram(datagram.data(), datagram.size(), &senderIp,
                            &senderPort);

    // Si es el primer paquete, establecemos la conexión
    if (remoteIp.isEmpty()) {
      remoteIp = senderIp.toString();
      // Manejar direcciones IPv4 mapeadas a IPv6 (común en Windows)
      if (remoteIp.startsWith("::ffff:")) {
        remoteIp = remoteIp.mid(7);
      }
      remotePort = senderPort;

      // Actualizar la UI para reflejar la conexión establecida
      ui->RemoteIpLineEdit->setText(remoteIp);
      ui->RemotePortLineEdit->setText(QString::number(remotePort));
      ui->commsLog->appendPlainText(QString("Conexión establecida con %1:%2")
                                        .arg(remoteIp)
                                        .arg(remotePort));
      updateConnectionStatus("Conectado por UDP", "green");

      // Solicitar período de PWM al conectar
      QTimer::singleShot(200, this, [this]() {
        sendUnerbusCommand(Unerbus::CommandId::CMD_GET_PWM_PERIOD);
      });
      // Solicitar estado del robot al conectar
      QTimer::singleShot(300, this, [this]() {
        sendUnerbusCommand(Unerbus::CommandId::CMD_GET_ROBOT_STATUS);
      });
    }

    // Procesar el paquete como siempre
    m_parser->processData(datagram);
  }
}

/**
 * @brief Slot para el botón "Actualizar" de la página de sensores.
 *        Solicita los datos de los sensores una sola vez.
 */
void MainWindow::on_btnRefreshSensorsValues_clicked() { requestSensorData(); }

/**
 * @brief Slot para el checkbox "Actualización automática".
 *        Inicia o detiene el temporizador que solicita datos de sensores.
 * @param checked El estado del checkbox.
 */
void MainWindow::on_chkBoxAutoRefreshSensorsValues_toggled(bool checked) {
  if (checked) {
    sensorUpdateTimer->start(
        SENSOR_UPDATE_INTERVAL_MS); // Actualiza cada 200 ms (5 Hz)
  } else {
    sensorUpdateTimer->stop();
  }
}

/**
 * @brief Solicita todos los datos de sensores al microcontrolador.
 */
void MainWindow::requestSensorData() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_IR_SENSOR_SNAPSHOT);
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_MPU_DATA);
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_YAW_ANGLE);
}

/**
 * @brief Actualiza los QProgressBar de los sensores IR con los datos recibidos.
 * @param payload El payload del paquete CMD_GET_IR_SENSOR_SNAPSHOT.
 */
void MainWindow::updateIrSensorsUI(const QByteArray &payload) {
  if (payload.size() < 17)
    return; // 8 valores uint16_t + detection_flags

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 ir_front_left, ir_front_right, ir_diag_left, ir_diag_right,
      ir_side_left, ir_side_right, ir_gnd_front, ir_gnd_rear;
  quint8 detection_flags;

  stream >> ir_side_right >> ir_diag_right >> ir_front_right >> ir_gnd_front >>
      ir_front_left >> ir_diag_left >> ir_side_left >> ir_gnd_rear >>
      detection_flags;

  ui->progBarIrFrontLeft->setValue(ir_front_left);
  ui->progBarIrFrontRight->setValue(ir_front_right);
  ui->progBarIrLeftDiag->setValue(ir_diag_left);
  ui->progBarIrRightDiag->setValue(ir_diag_right);
  ui->progBarIrLeftSide->setValue(ir_side_left);
  ui->progBarIrRightSide->setValue(ir_side_right);
  ui->progBarIrGroundFront->setValue(ir_gnd_front);
  ui->progBarIrGroundRear->setValue(ir_gnd_rear);

  setDetectionLabel(ui->lblWallFrontDetected,
                    (detection_flags & SENSOR_DET_WALL_FRONT) != 0);
  setDetectionLabel(ui->lblWallLeftDetected,
                    (detection_flags & SENSOR_DET_WALL_LEFT) != 0);
  setDetectionLabel(ui->lblWallRightDetected,
                    (detection_flags & SENSOR_DET_WALL_RIGHT) != 0);
  setDetectionLabel(ui->lblWallDiagLeftDetected,
                    (detection_flags & SENSOR_DET_WALL_DIAG_LEFT) != 0);
  setDetectionLabel(ui->lblWallDiagRightDetected,
                    (detection_flags & SENSOR_DET_WALL_DIAG_RIGHT) != 0);
  setDetectionLabel(ui->lblFloorFrontDetected,
                    (detection_flags & SENSOR_DET_FLOOR_FRONT) != 0);
  setDetectionLabel(ui->lblFloorRearDetected,
                    (detection_flags & SENSOR_DET_FLOOR_REAR) != 0);
}

/**
 * @brief Actualiza los QLineEdit del MPU6050 con los datos recibidos.
 * @param payload El payload del paquete CMD_GET_MPU_DATA.
 */
void MainWindow::updateMpuSensorsUI(const QByteArray &payload) {
  if (payload.size() < 14)
    return; // 7 valores * 2 bytes/valor

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  qint16 ax, ay, az, temp, gx, gy, gz;
  stream >> ax >> ay >> az >> temp >> gx >> gy >> gz;

  // Ignoramos 'temp' como solicitaste.

  ui->accelXLineEdit->setText(QString::number(ax));
  ui->accelYLineEdit->setText(QString::number(ay));
  ui->accelZLineEdit->setText(QString::number(az));
  ui->gyroXLineEdit->setText(QString::number(gx));
  ui->gyroYLineEdit->setText(QString::number(gy));
  ui->gyroZLineEdit->setText(QString::number(gz));
}

void MainWindow::sendUnerbusCommand(Unerbus::CommandId cmd,
                                    const QByteArray &payload) {
  if (!serialPort->isOpen() && !udpSocket) {
    // No mostramos un error aquí para no ser intrusivos durante el
    // auto-refresco
    return;
  }

  QByteArray packet;
  packet.append(Unerbus::HEADER); // "UNER"
  // El tamaño es: 1 (CMD) + N (payload) + 1 (Checksum)
  quint8 length = 1 + payload.size() + 1;
  packet.append(length);
  packet.append(Unerbus::TOKEN);           // ':'
  packet.append(static_cast<quint8>(cmd)); // CMD
  if (!payload.isEmpty()) {
    packet.append(payload); // PAYLOAD
  }

  quint8 checksum = 0;
  // El checksum se calcula sobre todo el paquete antes de añadir el checksum
  // mismo
  for (quint8 byte : packet) {
    checksum ^= byte;
  }
  packet.append(checksum); // CHECKSUM

  if (serialPort && serialPort->isOpen()) {
    serialPort->write(packet);
  } else if (udpSocket) {
    // Si la IP remota no está seteada, no podemos enviar
    if (remoteIp.isEmpty() || remotePort == 0)
      return;
    udpSocket->writeDatagram(packet, QHostAddress(remoteIp), remotePort);
  }

  qDebug() << "Paquete enviado:" << packet.toHex(' ');
}

/**
 * @brief Actualiza el texto y el color de la etiqueta de estado de conexión.
 * @param text El mensaje a mostrar.
 * @param colorName El nombre del color (ej: "green", "red", "orange").
 */
void MainWindow::updateConnectionStatus(const QString &text,
                                        const QString &colorName) {
  ui->labelCommStatus->setText(text);
  ui->labelCommStatus->setStyleSheet(QString("color: %1;").arg(colorName));
}

void MainWindow::on_btnCalibrateMPU_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_CALIBRATE_MPU);
}

/**
 * @brief Slot para el botón "Aplicar PWM".
 *        Construye y envía el comando CMD_SET_MOTOR_PWM con los valores de la
 * UI.
 */
void MainWindow::on_btnApplyPWM_clicked() {
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(
      QDataStream::LittleEndian); // El microcontrolador es Little Endian

  // Obtener los valores porcentuales (0-100) de los spinboxes.
  int right_neg_percent = ui->spinBoxRightMotorNeg->value();
  int right_pos_percent = ui->spinBoxRightMotorPos->value();
  int left_neg_percent = ui->spinBoxLeftMotorNeg->value();
  int left_pos_percent = ui->spinBoxLeftMotorPos->value();

  // Escalar los valores porcentuales al rango del período del PWM (m_pwmPeriod
  // es el 100%).
  quint16 right_neg =
      static_cast<quint16>((right_neg_percent / 100.0f) * m_pwmPeriod);
  quint16 right_pos =
      static_cast<quint16>((right_pos_percent / 100.0f) * m_pwmPeriod);
  quint16 left_neg =
      static_cast<quint16>((left_neg_percent / 100.0f) * m_pwmPeriod);
  quint16 left_pos =
      static_cast<quint16>((left_pos_percent / 100.0f) * m_pwmPeriod);

  // Escribir en el payload en el orden que espera el firmware:
  // CH1 (Derecho, Reversa), CH2 (Derecho, Avance), CH3 (Izquierdo, Reversa),
  // CH4 (Izquierdo, Avance)
  stream << right_neg << right_pos << left_neg << left_pos;

  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MOTOR_PWM, payload);
}

/**
 * @brief Slot para el botón "Detener Motores".
 *        Envía un comando para poner todos los PWM a 0 y resetea la UI.
 */
void MainWindow::on_btnStopMotor_clicked() {
  // 1. Resetear los controles de la UI a 0
  ui->hSliderRightMotorPos->setValue(0);
  ui->hSliderRightMotorNeg->setValue(0);
  ui->hSliderLeftMotorPos->setValue(0);
  ui->hSliderLeftMotorNeg->setValue(0);
  // Los spinboxes se actualizarán automáticamente gracias a las conexiones de
  // señales

  // 2. Enviar comando con todos los PWM a 0
  // Creamos un payload de 8 bytes, todos inicializados a cero.
  QByteArray payload(8, 0);
  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MOTOR_PWM, payload);
}

/**
 * @brief Configura las conexiones de señales y slots para la página de control.
 */
void MainWindow::setupControlPage() {
  // --- Sincronización de Sliders y SpinBoxes ---
  connect(ui->hSliderRightMotorPos, &QSlider::valueChanged,
          ui->spinBoxRightMotorPos, &QSpinBox::setValue);
  connect(ui->spinBoxRightMotorPos, QOverload<int>::of(&QSpinBox::valueChanged),
          ui->hSliderRightMotorPos, &QSlider::setValue);
  connect(ui->hSliderRightMotorNeg, &QSlider::valueChanged,
          ui->spinBoxRightMotorNeg, &QSpinBox::setValue);
  connect(ui->spinBoxRightMotorNeg, QOverload<int>::of(&QSpinBox::valueChanged),
          ui->hSliderRightMotorNeg, &QSlider::setValue);
  connect(ui->hSliderLeftMotorPos, &QSlider::valueChanged,
          ui->spinBoxLeftMotorPos, &QSpinBox::setValue);
  connect(ui->spinBoxLeftMotorPos, QOverload<int>::of(&QSpinBox::valueChanged),
          ui->hSliderLeftMotorPos, &QSlider::setValue);
  connect(ui->hSliderLeftMotorNeg, &QSlider::valueChanged,
          ui->spinBoxLeftMotorNeg, &QSpinBox::setValue);
  connect(ui->spinBoxLeftMotorNeg, QOverload<int>::of(&QSpinBox::valueChanged),
          ui->hSliderLeftMotorNeg, &QSlider::setValue);

  // --- Conexión para seteo en tiempo real ---
  // Conectar todos los widgets de control a un único slot
  connect(ui->hSliderRightMotorPos, &QSlider::valueChanged, this,
          &MainWindow::on_control_widget_valueChanged);
  connect(ui->hSliderRightMotorNeg, &QSlider::valueChanged, this,
          &MainWindow::on_control_widget_valueChanged);
  connect(ui->hSliderLeftMotorPos, &QSlider::valueChanged, this,
          &MainWindow::on_control_widget_valueChanged);
  connect(ui->hSliderLeftMotorNeg, &QSlider::valueChanged, this,
          &MainWindow::on_control_widget_valueChanged);
}

/**
 * @brief Slot para el botón "Obtener PWM". Solicita los datos una vez.
 */
void MainWindow::on_btnGetPWM_clicked() { requestPwmData(); }

/**
 * @brief Slot para el checkbox "Obtención automática".
 *        Inicia o detiene el temporizador y bloquea/desbloquea los controles de
 * PWM.
 */
void MainWindow::on_chkAutoGetPWM_toggled(bool checked) {
  // Habilitar/deshabilitar controles para evitar conflictos con la entrada del
  // usuario
  ui->hSliderRightMotorPos->setEnabled(!checked);
  ui->spinBoxRightMotorPos->setEnabled(!checked);
  ui->hSliderRightMotorNeg->setEnabled(!checked);
  ui->spinBoxRightMotorNeg->setEnabled(!checked);
  ui->hSliderLeftMotorPos->setEnabled(!checked);
  ui->spinBoxLeftMotorPos->setEnabled(!checked);
  ui->hSliderLeftMotorNeg->setEnabled(!checked);
  ui->spinBoxLeftMotorNeg->setEnabled(!checked);
  ui->chkRealTimeSetPWM->setEnabled(
      !checked); // Bloquear el seteo en tiempo real

  if (checked) {
    pwmUpdateTimer->start(PWM_UPDATE_INTERVAL_MS);
  } else {
    pwmUpdateTimer->stop();
  }
}

/**
 * @brief Slot para el checkbox "Seteo en tiempo real".
 *        Habilita/deshabilita el botón "Aplicar PWM" y el checkbox de obtención
 * automática.
 */
void MainWindow::on_chkRealTimeSetPWM_toggled(bool checked) {
  ui->btnApplyPWM->setEnabled(!checked);
  ui->chkAutoGetPWM->setEnabled(!checked); // Bloquear la obtención automática
}

/**
 * @brief Slot que se activa cuando cualquier slider o spinbox de control
 * cambia. Si el modo "tiempo real" está activo, envía el comando de PWM.
 */
void MainWindow::on_control_widget_valueChanged() {
  if (ui->chkRealTimeSetPWM->isChecked()) {
    on_btnApplyPWM_clicked();
  }
}

/**
 * @brief Solicita los valores de PWM actuales al microcontrolador.
 */
void MainWindow::requestPwmData() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_MOTOR_PWM);
}

/**
 * @brief Actualiza los sliders y spinboxes de PWM con los datos recibidos.
 * @param payload El payload del paquete CMD_GET_MOTOR_PWM.
 */
void MainWindow::updatePwmUI(const QByteArray &payload) {
  if (payload.size() < 8)
    return;

  // Protección contra división por cero si el período aún no se ha recibido.
  if (m_pwmPeriod == 0) {
    qWarning() << "Intento de actualizar UI de PWM con período 0. Abortando.";
    return;
  }

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 right_neg_abs, right_pos_abs, left_neg_abs, left_pos_abs;
  stream >> right_neg_abs >> right_pos_abs >> left_neg_abs >> left_pos_abs;

  // 1. Escalar los valores absolutos a un porcentaje (0-100) para la UI.
  int right_neg_percent = (right_neg_abs * 100) / m_pwmPeriod;
  int right_pos_percent = (right_pos_abs * 100) / m_pwmPeriod;
  int left_neg_percent = (left_neg_abs * 100) / m_pwmPeriod;
  int left_pos_percent = (left_pos_abs * 100) / m_pwmPeriod;

  // 2. Bloquear señales para evitar bucles de actualización o envíos no
  // deseados.
  ui->hSliderRightMotorNeg->blockSignals(true);
  ui->spinBoxRightMotorNeg->blockSignals(true);
  ui->hSliderRightMotorPos->blockSignals(true);
  ui->spinBoxRightMotorPos->blockSignals(true);
  ui->hSliderLeftMotorNeg->blockSignals(true);
  ui->spinBoxLeftMotorNeg->blockSignals(true);
  ui->hSliderLeftMotorPos->blockSignals(true);
  ui->spinBoxLeftMotorPos->blockSignals(true);

  // 3. Actualizar valores de los sliders.
  ui->hSliderRightMotorNeg->setValue(right_neg_percent);
  ui->hSliderRightMotorPos->setValue(right_pos_percent);
  ui->hSliderLeftMotorNeg->setValue(left_neg_percent);
  ui->hSliderLeftMotorPos->setValue(left_pos_percent);

  // 4. --- MODIFICACIÓN CLAVE: Actualizar también los spinboxes ---
  ui->spinBoxRightMotorNeg->setValue(right_neg_percent);
  ui->spinBoxRightMotorPos->setValue(right_pos_percent);
  ui->spinBoxLeftMotorNeg->setValue(left_neg_percent);
  ui->spinBoxLeftMotorPos->setValue(left_pos_percent);

  // 5. Desbloquear señales para que la UI vuelva a ser interactiva.
  ui->hSliderRightMotorNeg->blockSignals(false);
  ui->spinBoxRightMotorNeg->blockSignals(false);
  ui->hSliderRightMotorPos->blockSignals(false);
  ui->spinBoxRightMotorPos->blockSignals(false);
  ui->hSliderLeftMotorNeg->blockSignals(false);
  ui->spinBoxLeftMotorNeg->blockSignals(false);
  ui->hSliderLeftMotorPos->blockSignals(false);
  ui->spinBoxLeftMotorPos->blockSignals(false);
}

/**
 * @brief Actualiza los rangos máximos de los sliders y spinboxes de PWM.
 * @param new_period El nuevo valor máximo (período del timer).
 */
void MainWindow::updatePwmControlRanges(quint16 new_period) {
  // Guardar el valor del período real para los cálculos de escalado.
  m_pwmPeriod = new_period;

  // Actualizar el campo de texto informativo en la página de configuración.
  ui->lineEditPwmPeriod->setText(QString::number(new_period));
}

/**
 * @brief Slot para el botón "Configurar" del período del PWM.
 */
void MainWindow::on_btnConfigurePeriod_clicked() {
  bool ok;
  quint16 new_period = ui->lineEditPwmPeriod->text().toUShort(&ok);

  if (ok && new_period > 10) { // Validar un mínimo razonable
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << new_period;
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_PWM_PERIOD, payload);

    // Solicitar de vuelta el valor para confirmar y actualizar la UI
    QTimer::singleShot(200, this, [this]() {
      sendUnerbusCommand(Unerbus::CommandId::CMD_GET_PWM_PERIOD);
    });
  } else {
    QMessageBox::warning(
        this, "Valor Inválido",
        "Por favor, ingrese un número de período válido (mayor a 100).");
  }
}

void MainWindow::drawMaze() {
  mazeScene->clear();

  if (!autonomousTimer->isActive()) {
    int current_target_x = is_returning ? start_x : goal_x;
    int current_target_y = is_returning ? start_y : goal_y;
    calculateFastestPath(current_x, current_y, current_target_x,
                         current_target_y);
  }

  int cellSize = 50; // Cada celda medirá 50x50 píxeles

  // 1. Configuramos los "Lápices" (Pens)
  // Lápiz tenue para celdas no visitadas o estructura básica
  QPen faintPen(QColor(50, 60, 70), 1, Qt::DotLine);

  // Lápiz brillante (Cian/Neón) para las paredes reales que detectó el robot
  QPen wallPen(QColor(0, 255, 255), 3, Qt::SolidLine);

  // Lápiz del Robot (Violeta Claro Sólido) para que resalte sobre todo
  QBrush robotBrush(QColor(125, 217, 111));
  QPen robotPen(QColor(142, 255, 127), 2);
  QPen textPen(QColor(150, 150, 150)); // Gris tenue

  // Etiquetas de Coordenadas ===
  QFont numberFont("Arial", 10, QFont::Bold);

  for (int i = 0; i < MAZE_WIDTH; i++) {
    // Coordenadas X en el techo del mapa
    QGraphicsRectItem *topAnchor =
        mazeScene->addRect(0, 0, 0, 0, Qt::NoPen, Qt::NoBrush);
    topAnchor->setPos(i * cellSize + (cellSize / 2.0), -15);
    topAnchor->setFlag(QGraphicsItem::ItemIgnoresTransformations);

    QGraphicsTextItem *topText =
        new QGraphicsTextItem(QString::number(i), topAnchor);
    topText->setFont(numberFont);
    topText->setDefaultTextColor(textPen.color());
    topText->setPos(-topText->boundingRect().width() / 2.0,
                    -topText->boundingRect().height() / 2.0);

    // Coordenadas Y al costado izquierdo del mapa
    QGraphicsRectItem *leftAnchor =
        mazeScene->addRect(0, 0, 0, 0, Qt::NoPen, Qt::NoBrush);
    leftAnchor->setPos(-15, i * cellSize + (cellSize / 2.0));
    leftAnchor->setFlag(QGraphicsItem::ItemIgnoresTransformations);

    QGraphicsTextItem *leftText =
        new QGraphicsTextItem(QString::number(i), leftAnchor);
    leftText->setFont(numberFont);
    leftText->setDefaultTextColor(textPen.color());
    leftText->setPos(-leftText->boundingRect().width() / 2.0,
                     -leftText->boundingRect().height() / 2.0);
  }

  // 2. DIBUJAR LA GRILLA Y LAS PAREDES
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {

      int px = x * cellSize;
      int py = y * cellSize; // En Qt, la directriz Y crece hacia abajo
      uint8_t current_map_cell = 0;

      if (ui->radioBtnRealView->isChecked()) {
        current_map_cell = real_maze_map[x][y];
      } else {
        current_map_cell = sim_maze_map[x][y];
      }

      // === Colores Especiales de Baldosa ===
      QColor cellBgColor = Qt::transparent; // Vacío por defecto

      // ¿Es celda de Inicio?
      if (x == start_x && y == start_y) {
        cellBgColor = QColor(20, 120, 20); // Verde vibrante sólido
      }
      // ¿Es celda Objetivo?
      else if (x == goal_x && y == goal_y) {
        cellBgColor = QColor(150, 20, 20); // Rojo sólido oscuro
      }
      // ¿Es celda Especial?
      else if (current_map_cell & CELL_SPECIAL) {
        cellBgColor = QColor(255, 215, 0); // Dorado
      }
      // ¿Al menos está visitada?
      else if (current_map_cell & CELL_VISITED) {
        cellBgColor = QColor(255, 255, 255, 10); // Blancuzco tenue
      }

      // Dibujamos el cuadrado base y guardamos el puntero
      QGraphicsRectItem *cellRect =
          mazeScene->addRect(px, py, cellSize, cellSize, faintPen, cellBgColor);

      // Elevamos las placas de A y B (Z=5) para que tapen el nivel de suelo
      // (Z=0)
      if ((x == start_x && y == start_y) || (x == goal_x && y == goal_y)) {
        cellRect->setZValue(5);
      }

      // Textos 'A' y 'B' centrados a prueba de rotación
      if (x == start_x && y == start_y) {
        QGraphicsRectItem *anchor =
            mazeScene->addRect(0, 0, 0, 0, Qt::NoPen, Qt::NoBrush);
        anchor->setZValue(
            6); // La letra debe flotar apenas sobre su placa (Z=5)
        anchor->setPos(px + (cellSize / 2.0), py + (cellSize / 2.0));
        anchor->setFlag(QGraphicsItem::ItemIgnoresTransformations);

        QGraphicsTextItem *tx = new QGraphicsTextItem("A", anchor);
        tx->setFont(QFont("Arial", 16, QFont::Bold));
        tx->setDefaultTextColor(QColor(150, 255, 150));
        tx->setPos(-tx->boundingRect().width() / 2.0,
                   -tx->boundingRect().height() / 2.0);
      }
      if (x == goal_x && y == goal_y) {
        QGraphicsRectItem *anchor =
            mazeScene->addRect(0, 0, 0, 0, Qt::NoPen, Qt::NoBrush);
        anchor->setZValue(
            6); // La letra debe flotar apenas sobre su placa (Z=5)
        anchor->setPos(px + (cellSize / 2.0), py + (cellSize / 2.0));
        anchor->setFlag(QGraphicsItem::ItemIgnoresTransformations);

        QGraphicsTextItem *tx = new QGraphicsTextItem("B", anchor);
        tx->setFont(QFont("Arial", 16, QFont::Bold));
        tx->setDefaultTextColor(QColor(255, 150, 150));
        tx->setPos(-tx->boundingRect().width() / 2.0,
                   -tx->boundingRect().height() / 2.0);
      }

      // Si el robot visitó esta celda, dibujamos sus paredes reales
      uint8_t cellData = current_map_cell;

      // Si estamos en Visión Física (Real) vemos TODO.
      // Si estamos en Visión Robot (Sim), vemos solo la memoria visitada.
      bool isGodMode = ui->radioBtnRealView->isChecked();

      if (isGodMode || (cellData & CELL_VISITED)) {
        // Rellenamos ligeramente el fondo para indicar que esta celda es
        // "conocida"
        mazeScene->addRect(px, py, cellSize, cellSize, QPen(Qt::NoPen),
                           QBrush(QColor(255, 255, 255, 10)));

        // --- Pared Norte ---
        if (cellData & WALL_NORTH) {
          QGraphicsLineItem *w =
              mazeScene->addLine(px, py, px + cellSize, py, wallPen);
          w->setZValue(8);
        }

        // --- Pared Sur ---
        if (cellData & WALL_SOUTH) {
          QGraphicsLineItem *w = mazeScene->addLine(
              px, py + cellSize, px + cellSize, py + cellSize, wallPen);
          w->setZValue(8);
        }

        // --- Pared Este ---
        if (cellData & WALL_EAST) {
          QGraphicsLineItem *w = mazeScene->addLine(
              px + cellSize, py, px + cellSize, py + cellSize, wallPen);
          w->setZValue(8);
        }

        // --- Pared Oeste ---
        if (cellData & WALL_WEST) {
          QGraphicsLineItem *w =
              mazeScene->addLine(px, py, px, py + cellSize, wallPen);
          w->setZValue(8);
        }
      }
    }
  }

  // ==========================================
  // 3. DIBUJAR LA RUTA RÁPIDA (Estilo GPS)
  // ==========================================
  if (path_length > 1) {
    // Configuramos la "Cinta" del GPS:
    // Color amarillo vibrante, translúcida, 6px de grosor y con uniones suaves
    QPen gpsPen(QColor(91, 94, 255, 180), 6, Qt::SolidLine, Qt::RoundCap,
                Qt::RoundJoin);

    // Iteramos sobre todos los puntos encadenándolos con líneas
    for (int i = 0; i < path_length - 1; i++) {
      // Desempaquetar la celda origen ("i")
      int bx1 = best_path[i] >> 4;
      int by1 = best_path[i] & 0x0F;
      qreal cx1 = (bx1 * cellSize) + (cellSize / 2.0);
      qreal cy1 = (by1 * cellSize) + (cellSize / 2.0);

      // Desempaquetar la celda contigua ("i+1")
      int bx2 = best_path[i + 1] >> 4;
      int by2 = best_path[i + 1] & 0x0F;
      qreal cx2 = (bx2 * cellSize) + (cellSize / 2.0);
      qreal cy2 = (by2 * cellSize) + (cellSize / 2.0);

      // Trazar el segmento
      mazeScene->addLine(cx1, cy1, cx2, cy2, gpsPen);
    }
  }

  // 4. DIBUJAR AL ROBOT
  // El robot debe estar en la coordenada (current_x, current_y).
  // Para que se vea centrado y más chico que la celda:
  int robotSize = cellSize / 2; // Por ej: 25x25 pixeles
  int rx = (current_x * cellSize) + (cellSize / 4);
  int ry = (current_y * cellSize) + (cellSize / 4);

  // Dibujar el cuerpo blindado con Z-Index Máximo (10)
  QGraphicsRectItem *robotBody =
      mazeScene->addRect(rx, ry, robotSize, robotSize, robotPen, robotBrush);
  robotBody->setZValue(10); // Sobre el suelo y sobre todo lo demás

  // Dibujar un indicador de "Hacia donde mira" (Línea Fucsia Brillante)
  QPen dirPen(QColor(255, 0, 0), 5, Qt::SolidLine);
  int centerX = rx + (robotSize / 2);
  int centerY = ry + (robotSize / 2);

  QGraphicsLineItem *dirLine = nullptr;

  // Calculamos a dónde apunta la línea según current_heading
  if (current_heading == HEADING_NORTH)
    dirLine = mazeScene->addLine(centerX, centerY, centerX, ry, dirPen);
  else if (current_heading == HEADING_SOUTH)
    dirLine =
        mazeScene->addLine(centerX, centerY, centerX, ry + robotSize, dirPen);
  else if (current_heading == HEADING_EAST)
    dirLine =
        mazeScene->addLine(centerX, centerY, rx + robotSize, centerY, dirPen);
  else if (current_heading == HEADING_WEST)
    dirLine = mazeScene->addLine(centerX, centerY, rx, centerY, dirPen);

  if (dirLine)
    dirLine->setZValue(10); // Flechita roja también a nivel nubes
}

void MainWindow::calculateFastestPath(int sx, int sy, int gx, int gy) {
  path_length = 0; // Reiniciar ruta
  // 0. Si no hay meta o inicio, o el mouse no está calibrado, abortar
  if (sx == -1 || gx == -1)
    return;
  // 1. Matriz topográfica (0 = Cerca; 255 = Desconocido)
  uint8_t distances[MAZE_WIDTH][MAZE_HEIGHT];
  memset(distances, 255, sizeof(distances));
  // 2. Colas estáticas Empaquetadas (X << 4 | Y). Reduce 50% la RAM.
  uint8_t queue[225];
  uint8_t head = 0;
  uint8_t tail = 0;
  // Empezamos la inundación plantando la semilla 0 en el Inicio (A)
  queue[tail] = (sx << 4) | sy;
  tail++;
  distances[sx][sy] = 0;
  bool reached_goal = false;
  // --- FASE 1: EXPANSIÓN DE LA OLA (Cálculo de Distancias) ---
  while (head < tail) {
    int cx = queue[head] >> 4;
    int cy = queue[head] & 0x0F;
    head++;
    // Si la ola acaba de chocar la meta, paramos inmediatamente!
    if (cx == gx && cy == gy) {
      reached_goal = true;
      break;
    }
    uint8_t current_dist = distances[cx][cy];
    uint8_t cell = sim_maze_map[cx][cy];
    // Vertremos agua hacia el Norte (Si no hay muro, no es borde y no se ha
    // inundado ya)
    if (!(cell & WALL_NORTH) && cy > 0 && distances[cx][cy - 1] == 255) {
      distances[cx][cy - 1] = current_dist + 1;
      queue[tail] = (cx << 4) | (cy - 1);
      tail++;
    }
    // Vertremos agua hacia el Sur
    if (!(cell & WALL_SOUTH) && cy < MAZE_HEIGHT - 1 &&
        distances[cx][cy + 1] == 255) {
      distances[cx][cy + 1] = current_dist + 1;
      queue[tail] = (cx << 4) | (cy + 1);
      tail++;
    }
    // Vertremos hacia el Este
    if (!(cell & WALL_EAST) && cx < MAZE_WIDTH - 1 &&
        distances[cx + 1][cy] == 255) {
      distances[cx + 1][cy] = current_dist + 1;
      queue[tail] = ((cx + 1) << 4) | cy;
      tail++;
    }
    // Vertremos hacia el Oeste
    if (!(cell & WALL_WEST) && cx > 0 && distances[cx - 1][cy] == 255) {
      distances[cx - 1][cy] = current_dist + 1;
      queue[tail] = ((cx - 1) << 4) | cy;
      tail++;
    }
  }
  // --- FASE 2: EL BACKTRACKING (Rastrear el río de bajada a casa) ---
  if (reached_goal) {
    int cx = gx;
    int cy = gy;

    while (cx != sx || cy != sy) {
      // Tomar foto de pisada actual y empacar en guardado métrico
      best_path[path_length] = (cx << 4) | cy;
      path_length++;
      uint8_t d = distances[cx][cy];
      uint8_t cell = sim_maze_map[cx][cy];
      // Buscar lógicamente cuál vecino estrictamente tiene un número `d-1`
      // (Bajar escalón)
      if (!(cell & WALL_NORTH) && cy > 0 && distances[cx][cy - 1] == d - 1) {
        cy--;
      } else if (!(cell & WALL_SOUTH) && cy < MAZE_HEIGHT - 1 &&
                 distances[cx][cy + 1] == d - 1) {
        cy++;
      } else if (!(cell & WALL_EAST) && cx < MAZE_WIDTH - 1 &&
                 distances[cx + 1][cy] == d - 1) {
        cx++;
      } else if (!(cell & WALL_WEST) && cx > 0 &&
                 distances[cx - 1][cy] == d - 1) {
        cx--;
      } else {
        // FALLBACK DE SEGURIDAD EXTREMA: (Previene cuelgues)
        break;
      }
    }
    // Finalmente guardamos el punto donde iniciamos
    best_path[path_length] = (sx << 4) | sy;
    path_length++;
  }
}

int MainWindow::getFloodFillDistance(int sx, int sy, int gx, int gy) {
    uint8_t dist[MAZE_WIDTH][MAZE_HEIGHT];
    memset(dist, 255, sizeof(dist));
    uint8_t q[225];
    int h = 0, t = 0;
    q[t++] = (sx << 4) | sy;
    dist[sx][sy] = 0;
    while (h < t) {
        int px = q[h] >> 4;
        int py = q[h] & 0x0F;
        h++;
        if (px == gx && py == gy)
            return dist[px][py];
        int d = dist[px][py];
        int cell = sim_maze_map[px][py];
        if (!(cell & WALL_NORTH) && py > 0 && dist[px][py - 1] == 255) {
            dist[px][py - 1] = d + 1; q[t++] = (px << 4) | (py - 1);
        }
        if (!(cell & WALL_SOUTH) && py < MAZE_HEIGHT - 1 && dist[px][py + 1] == 255) {
            dist[px][py + 1] = d + 1; q[t++] = (px << 4) | (py + 1);
        }
        if (!(cell & WALL_EAST) && px < MAZE_WIDTH - 1 && dist[px + 1][py] == 255) {
            dist[px + 1][py] = d + 1; q[t++] = ((px + 1) << 4) | py;
        }
        if (!(cell & WALL_WEST) && px > 0 && dist[px - 1][py] == 255) {
            dist[px - 1][py] = d + 1; q[t++] = ((px - 1) << 4) | py;
        }
    }
    return 9999; // Inalcanzable
}

/**
 * @brief Conecta las señales de los botones de la página de configuración a sus
 * slots.
 */
void MainWindow::setupConfigPage() {
  // Conexiones para el PID de Navegación
  connect(ui->btnGetPidNavConfig, &QPushButton::clicked, this,
          &MainWindow::on_btnGetPidNavConfig_clicked);
  connect(ui->btnSetPidNavConfig, &QPushButton::clicked, this,
          &MainWindow::on_btnSetPidNavConfig_clicked);

  // Conexiones para el PID de Giro
  connect(ui->btnGetPidTurnConfig, &QPushButton::clicked, this,
          &MainWindow::on_btnGetPidTurnConfig_clicked);
  connect(ui->btnSetPidTurnConfig, &QPushButton::clicked, this,
          &MainWindow::on_btnSetPidTurnConfig_clicked);

  // Conexiones para las Velocidades Base de los Motores
  connect(ui->btnGetBaseMotorsSpeeds, &QPushButton::clicked, this,
          &MainWindow::on_btnGetBaseMotorsSpeeds_clicked);
  connect(ui->btnSetBaseMotorsSpeeds, &QPushButton::clicked, this,
          &MainWindow::on_btnSetBaseMotorsSpeeds_clicked);

  populateMpuConfigComboBoxes();
}

/**
 * @brief Solicita todos los parámetros de configuración del PID de navegación.
 *        Implementa la sugerencia de pedir todos los datos relacionados a la
 * vez.
 */
void MainWindow::on_btnGetPidNavConfig_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_PID_GAINS);
  // Se piden los otros grupos de parámetros con un pequeño retardo para no
  // saturar el micro
  QTimer::singleShot(100, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_MAX_PWM_CORRECTION);
  });
  QTimer::singleShot(200, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_WALL_THRESHOLDS);
  });
  QTimer::singleShot(300, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_WALL_TARGET_ADC);
  });
  QTimer::singleShot(400, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_DELAY_TICKS);
  });
}

/**
 * @brief Envía todos los parámetros de configuración del PID de navegación.
 *        Implementa la sugerencia de enviar todos los datos relacionados a la
 * vez.
 */
void MainWindow::on_btnSetPidNavConfig_clicked() {
  // 1. Enviar Ganancias (Kp, Ki, Kd)
  QByteArray gainsPayload;
  QDataStream gainsStream(&gainsPayload, QIODevice::WriteOnly);
  gainsStream.setByteOrder(QDataStream::LittleEndian);

  gainsStream << static_cast<quint16>(ui->editKpNav->text().toFloat() * 100.0f);
  gainsStream << static_cast<quint16>(ui->editKiNav->text().toFloat() * 100.0f);
  gainsStream << static_cast<quint16>(ui->editKdNav->text().toFloat() * 100.0f);
  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_PID_GAINS, gainsPayload);

  // 2. Enviar Corrección Máxima de PWM
  QTimer::singleShot(100, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint16>(ui->editMaxPwmOffsetNav->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MAX_PWM_CORRECTION, payload);
  });

  // 3. Enviar Umbrales de Pared (Frontal y Lateral)
  QTimer::singleShot(200, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint16>(ui->editSetpointFront->text().toUShort());
    stream << static_cast<quint16>(ui->editSetpointSides->text().toUShort());
    stream << static_cast<quint16>(ui->editSetpointDiagonal->text().toUShort());
    stream << static_cast<quint16>(
        ui->editSetpointDiagSmoothTurn->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_WALL_THRESHOLDS, payload);
  });

  // 4. Enviar ADC Objetivo de Pared
  QTimer::singleShot(300, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint16>(ui->editSetpointOneWall->text().toUShort());
    stream << static_cast<quint16>(
        ui->editSetpointTapeDetection->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_WALL_TARGET_ADC, payload);
  });
  QTimer::singleShot(400, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint8>(ui->editWallFadeTicks->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_DELAY_TICKS, payload);
  });
}

/**
 * @brief Solicita los parámetros de configuración del PID de giro.
 */
void MainWindow::on_btnGetPidTurnConfig_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_TURN_PID_GAINS);
  // Pedimos la velocidad máxima de giro con un pequeño retardo
  QTimer::singleShot(100, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_TURN_MAX_SPEED);
  });
  QTimer::singleShot(200, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_TURN_MIN_SPEED);
  });
  QTimer::singleShot(300, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_SMOOTH_TURN_CONFIG);
  });
  QTimer::singleShot(400, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_TURN_VELOCITY_PID_GAINS);
  });
  QTimer::singleShot(500, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_TURN_TARGET_DPS);
  });
}

/**
 * @brief Envía los parámetros de configuración del PID de giro.
 */
void MainWindow::on_btnSetPidTurnConfig_clicked() {
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);

  // Convertimos de flotante (UI) a entero (protocolo) multiplicando por 100
  stream << static_cast<quint16>(ui->editKpTurn->text().toFloat() * 100.0f);
  stream << static_cast<quint16>(ui->editKiTurn->text().toFloat() * 100.0f);
  stream << static_cast<quint16>(ui->editKdTurn->text().toFloat() * 100.0f);

  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_TURN_PID_GAINS, payload);

  QTimer::singleShot(100, this, [this]() {
    QByteArray speedPayload;
    QDataStream speedStream(&speedPayload, QIODevice::WriteOnly);
    speedStream.setByteOrder(QDataStream::LittleEndian);

    speedStream << static_cast<quint16>(
        ui->editMaxTurnSpeed->text().toUShort());

    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_TURN_MAX_SPEED,
                       speedPayload);
  });

  QTimer::singleShot(200, this, [this]() {
    QByteArray minSpeedPayload;
    QDataStream minSpeedStream(&minSpeedPayload, QIODevice::WriteOnly);
    minSpeedStream.setByteOrder(QDataStream::LittleEndian);
    minSpeedStream << static_cast<quint16>(
        ui->editMinTurnSpeed->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_TURN_MIN_SPEED,
                       minSpeedPayload);
  });

  QTimer::singleShot(300, this, [this]() {
    QByteArray smoothTurnSpeedsPayload;
    QDataStream smoothTurnSpeedsStream(&smoothTurnSpeedsPayload,
                                       QIODevice::WriteOnly);
    smoothTurnSpeedsStream.setByteOrder(QDataStream::LittleEndian);
    smoothTurnSpeedsStream
        << static_cast<quint16>(ui->editFasterMotorSmooth->text().toUShort())
        << static_cast<quint16>(ui->editSlowerMotorSmooth->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_SMOOTH_TURN_CONFIG,
                       smoothTurnSpeedsPayload);
  });

  QTimer::singleShot(400, this, [this]() {
    QByteArray constantsPayload;
    QDataStream constantsStream(&constantsPayload, QIODevice::WriteOnly);
    constantsStream.setByteOrder(QDataStream::LittleEndian);
    constantsStream << static_cast<quint16>(
        ui->editKpSmoothTurn->text().toFloat() * 100.0f);
    constantsStream << static_cast<quint16>(
        ui->editKiSmoothTurn->text().toFloat() * 100.0f);
    constantsStream << static_cast<quint16>(
        ui->editKdSmoothTurn->text().toFloat() * 100.0f);
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_TURN_VELOCITY_PID_GAINS,
                       constantsPayload);
  });

  QTimer::singleShot(500, this, [this]() {
    QByteArray speedsPayload;
    QDataStream speedsStream(&speedsPayload, QIODevice::WriteOnly);
    speedsStream.setByteOrder(QDataStream::LittleEndian);
    speedsStream << static_cast<quint16>(
        ui->editTurnAngularSpeed->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_TURN_TARGET_DPS,
                       speedsPayload);
  });
}

/**
 * @brief Solicita las velocidades base de los motores.
 */
void MainWindow::on_btnGetBaseMotorsSpeeds_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_MOTOR_BASE_SPEEDS);
}

/**
 * @brief Envía las velocidades base de los motores.
 */
void MainWindow::on_btnSetBaseMotorsSpeeds_clicked() {
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);

  stream << static_cast<quint16>(
      ui->editRightMotorBaseSpeed->text().toUShort());
  stream << static_cast<quint16>(ui->editLeftMotorBaseSpeed->text().toUShort());

  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MOTOR_BASE_SPEEDS, payload);
}

/**
 * @brief Actualiza la UI con las ganancias del PID de navegación.
 */
void MainWindow::updatePidNavUI(const QByteArray &payload) {
  if (payload.size() < 6)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 kp_int, ki_int, kd_int;
  stream >> kp_int >> ki_int >> kd_int;

  // Convertimos de entero (protocolo) a flotante (UI) dividiendo por 1000
  ui->editKpNav->setText(QString::number(kp_int / 100.0, 'f', 2));
  ui->editKiNav->setText(QString::number(ki_int / 100.0, 'f', 2));
  ui->editKdNav->setText(QString::number(kd_int / 100.0, 'f', 2));
}

/**
 * @brief Actualiza la UI con la corrección máxima del PID de navegación.
 * @param payload El payload del paquete CMD_GET_MAX_PWM_CORRECTION.
 */
void MainWindow::updateMaxPwmCorrectionUI(const QByteArray &payload) {
  if (payload.size() < 2) // 1 valor * 2 bytes/valor
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 max_correction;
  stream >> max_correction;

  ui->editMaxPwmOffsetNav->setText(QString::number(max_correction));
}

/**
 * @brief Actualiza la UI con las ganancias del PID de giro.
 */
void MainWindow::updatePidTurnUI(const QByteArray &payload) {
  if (payload.size() < 6)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 kp_int, ki_int, kd_int;
  stream >> kp_int >> ki_int >> kd_int;

  // Convertimos de entero (protocolo) a flotante (UI) dividiendo por 100
  ui->editKpTurn->setText(QString::number(kp_int / 100.0, 'f', 2));
  ui->editKiTurn->setText(QString::number(ki_int / 100.0, 'f', 2));
  ui->editKdTurn->setText(QString::number(kd_int / 100.0, 'f', 2));
}

/**
 * @brief Actualiza la UI con las velocidades base de los motores.
 */
void MainWindow::updateMotorBaseSpeedsUI(const QByteArray &payload) {
  if (payload.size() < 4)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 right_speed, left_speed;
  stream >> right_speed >> left_speed;

  ui->editRightMotorBaseSpeed->setText(QString::number(right_speed));
  ui->editLeftMotorBaseSpeed->setText(QString::number(left_speed));
}

/**
 * @brief Slot para el botón "Girar".
 *        Construye y envía el comando CMD_TURN_DEGREES con el ángulo
 * especificado.
 */
void MainWindow::on_btnSendTurnAngle_clicked() {
  // 1. Validar que el campo de texto no esté vacío
  if (ui->editTurnAngle->text().isEmpty()) {
    QMessageBox::warning(this, "Entrada Inválida",
                         "Por favor, ingrese un ángulo de giro.");
    return;
  }

  // 2. Obtener el valor del ángulo como un entero de 16 bits con signo
  bool ok;
  qint16 angle = static_cast<qint16>(ui->editTurnAngle->text().toInt(&ok));

  if (!ok) {
    QMessageBox::warning(this, "Valor Inválido",
                         "El valor del ángulo no es un número válido.");
    return;
  }

  // 3. Preparar el payload
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(
      QDataStream::LittleEndian); // El microcontrolador es Little Endian

  // Escribir el ángulo en el payload. El firmware espera un int16_t.
  stream << angle;

  // 4. Enviar el comando
  sendUnerbusCommand(Unerbus::CommandId::CMD_TURN_DEGREES, payload);

  // Opcional: Limpiar el campo de texto después de enviar
  // ui->editTurnAngle->clear();
}

/**
 * @brief Actualiza la UI con la velocidad máxima de giro.
 */
void MainWindow::updateTurnMaxSpeedUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 speed;
  stream >> speed;

  ui->editMaxTurnSpeed->setText(QString::number(speed));
}

/**
 * @brief Slot para el botón "Obtener" de la configuración del MPU.
 */
void MainWindow::on_btnGetMpuConfig_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_MPU_CONFIG);
}

/**
 * @brief Slot para el botón "Establecer" de la configuración del MPU.
 */
void MainWindow::on_btnSetMpuConfig_clicked() {
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);

  // Obtener los valores de los ComboBox (el valor real, no el texto)
  quint8 accel_config = ui->comboAccelConfig->currentData().toUInt();
  quint8 gyro_config = ui->comboGyroConfig->currentData().toUInt();
  quint8 dlpf_config = ui->comboDlpfConfig->currentData().toUInt();

  stream << accel_config << gyro_config << dlpf_config;

  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MPU_CONFIG, payload);
}

/**
 * @brief Rellena los ComboBox de configuración del MPU con los valores y textos
 * correspondientes.
 */
void MainWindow::populateMpuConfigComboBoxes() {
  // --- Rango del Acelerómetro ---
  ui->comboAccelConfig->clear();
  ui->comboAccelConfig->addItem("±2g", 0x00);
  ui->comboAccelConfig->addItem("±4g", 0x08);
  ui->comboAccelConfig->addItem("±8g", 0x10);
  ui->comboAccelConfig->addItem("±16g", 0x18);

  // --- Rango del Giroscopio ---
  ui->comboGyroConfig->clear();
  ui->comboGyroConfig->addItem("±250°/s", 0x00);
  ui->comboGyroConfig->addItem("±500°/s", 0x08);
  ui->comboGyroConfig->addItem("±1000°/s", 0x10);
  ui->comboGyroConfig->addItem("±2000°/s", 0x18);

  // --- Filtro Digital Pasa-Bajos (DLPF) ---
  ui->comboDlpfConfig->clear();
  ui->comboDlpfConfig->addItem("260 Hz", 0x00);
  ui->comboDlpfConfig->addItem("184 Hz", 0x01);
  ui->comboDlpfConfig->addItem("94 Hz", 0x02);
  ui->comboDlpfConfig->addItem("44 Hz", 0x03);
  ui->comboDlpfConfig->addItem("21 Hz", 0x04);
  ui->comboDlpfConfig->addItem("10 Hz", 0x05);
  ui->comboDlpfConfig->addItem("5 Hz", 0x06);
}

/**
 * @brief Actualiza la UI con la configuración del MPU recibida.
 */
void MainWindow::updateMpuConfigUI(const QByteArray &payload) {
  if (payload.size() < 3)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint8 accel_config, gyro_config, dlpf_config;
  stream >> accel_config >> gyro_config >> dlpf_config;

  // Buscar y seleccionar el item correspondiente en cada ComboBox
  ui->comboAccelConfig->setCurrentIndex(
      ui->comboAccelConfig->findData(accel_config));
  ui->comboGyroConfig->setCurrentIndex(
      ui->comboGyroConfig->findData(gyro_config));
  ui->comboDlpfConfig->setCurrentIndex(
      ui->comboDlpfConfig->findData(dlpf_config));
}

/**
 * @brief Actualiza la UI con la velocidad mínima de giro.
 */
void MainWindow::updateTurnMinSpeedUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 speed;
  stream >> speed;

  ui->editMinTurnSpeed->setText(QString::number(speed));
}

/**
 * @brief Actualiza la UI con los umbrales de pared (frontal y lateral).
 * @param payload El payload del paquete CMD_GET_WALL_THRESHOLDS.
 */
void MainWindow::updateWallThresholdsUI(const QByteArray &payload) {
  if (payload.size() < 8) // 4 valores * 2 bytes/valor
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 threshold_front, threshold_side, threshold_diagonal,
      threshold_diagonal_turn;
  stream >> threshold_front >> threshold_side >> threshold_diagonal >>
      threshold_diagonal_turn;

  ui->editSetpointFront->setText(QString::number(threshold_front));
  ui->editSetpointSides->setText(QString::number(threshold_side));
  ui->editSetpointDiagonal->setText(QString::number(threshold_diagonal));
  ui->editSetpointDiagSmoothTurn->setText(
      QString::number(threshold_diagonal_turn));
}

/**
 * @brief Actualiza la UI con el valor ADC objetivo para el seguimiento de
 * pared.
 * @param payload El payload del paquete CMD_GET_WALL_TARGET_ADC.
 */
void MainWindow::updateWallTargetAdcUI(const QByteArray &payload) {
  if (payload.size() < 2) // 1 valor * 2 bytes/valor
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 setpoint_one_wall, setpoint_tape_detected;
  stream >> setpoint_one_wall >> setpoint_tape_detected;

  ui->editSetpointOneWall->setText(QString::number(setpoint_one_wall));
  ui->editSetpointTapeDetection->setText(
      QString::number(setpoint_tape_detected));
}

/**
 * @brief Configura las conexiones de señales y slots para la pestaña de
 * Actividades.
 */
void MainWindow::setupActivitiesTab() {
  // Conectar botones
  connect(ui->btnGetRobotStatus, &QPushButton::clicked, this,
          &MainWindow::on_btnGetRobotStatus_clicked);
  connect(ui->btnSetRobotStatus, &QPushButton::clicked, this,
          &MainWindow::on_btnSetRobotStatus_clicked);

  // Rellenar los ComboBox con los valores correspondientes
  populateRobotStatusComboBoxes();
}

/**
 * @brief Rellena los ComboBox de estado y modo del robot con valores fijos.
 */
void MainWindow::populateRobotStatusComboBoxes() {
  // --- Estado de la Aplicación (AppState) ---
  ui->comboRobotAppState->clear();
  ui->comboRobotAppState->addItem("Menú", 0);      // APP_STATE_MENU
  ui->comboRobotAppState->addItem("Corriendo", 1); // APP_STATE_RUNNING

  // --- Modo de Operación (MenuMode) ---
  ui->comboRobotMode->clear();
  ui->comboRobotMode->addItem("Inactivo (Idle)", 0); // MENU_MODE_IDLE
  ui->comboRobotMode->addItem("Buscar Celdas", 1);   // MENU_MODE_FIND_CELLS
  ui->comboRobotMode->addItem("Ir a Punto B", 2);    // MENU_MODE_GO_TO_B
}

/**
 * @brief Slot para el botón "Obtener Estado". Solicita el estado actual del
 * robot.
 */
void MainWindow::on_btnGetRobotStatus_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_ROBOT_STATUS);
}

/**
 * @brief Slot para el botón "Establecer Estado". Envía los nuevos estado y modo
 * al robot.
 */
void MainWindow::on_btnSetRobotStatus_clicked() {
  // 1. Enviar el nuevo AppState
  QByteArray appStatePayload;
  appStatePayload.append(
      static_cast<quint8>(ui->comboRobotAppState->currentData().toUInt()));
  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_APP_STATE, appStatePayload);

  // 2. Enviar el nuevo MenuMode con un pequeño retardo
  QTimer::singleShot(100, this, [this]() {
    QByteArray menuModePayload;
    menuModePayload.append(
        static_cast<quint8>(ui->comboRobotMode->currentData().toUInt()));
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_MENU_MODE, menuModePayload);
  });

  // 3. Solicitar de vuelta el estado para confirmar el cambio
  QTimer::singleShot(200, this, &MainWindow::on_btnGetRobotStatus_clicked);
}

/**
 * @brief Actualiza la UI con el estado y modo del robot recibidos.
 * @param payload El payload del paquete CMD_GET_ROBOT_STATUS.
 */
void MainWindow::updateRobotStatusUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return; // Necesitamos 2 bytes: AppState y MenuMode

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint8 app_state, menu_mode;
  stream >> app_state >> menu_mode;

  // Buscar y seleccionar el item correspondiente en cada ComboBox
  // Se bloquean las señales para evitar disparar eventos no deseados
  ui->comboRobotAppState->blockSignals(true);
  ui->comboRobotMode->blockSignals(true);

  ui->comboRobotAppState->setCurrentIndex(
      ui->comboRobotAppState->findData(app_state));
  ui->comboRobotMode->setCurrentIndex(ui->comboRobotMode->findData(menu_mode));

  ui->comboRobotAppState->blockSignals(false);
  ui->comboRobotMode->blockSignals(false);
}

/**
 * @brief Solicita los parámetros de control de crucero al robot.
 */
void MainWindow::on_btnGetCruiseParams_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_CRUISE_PARAMS);
}

/**
 * @brief Envía los parámetros de control de crucero configurados en la UI.
 */
void MainWindow::on_btnSetCruiseParams_clicked() {
  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);

  stream << static_cast<quint16>(ui->editCruiseSpeed->text().toUShort());
  stream << static_cast<quint16>(ui->editAccelThreshold->text().toUShort());
  stream << static_cast<quint16>(
      ui->editConfirmTicks->text()
          .toUShort()); // Se envía como quint16 por alineación

  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_CRUISE_PARAMS, payload);
}

/**
 * @brief Actualiza la UI con los parámetros de control de crucero recibidos.
 * @param payload El payload del paquete CMD_GET_CRUISE_PARAMS.
 */
void MainWindow::updateCruiseParamsUI(const QByteArray &payload) {
  if (payload.size() < 6) // 3 valores * 2 bytes/valor
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 cruise_speed, accel_threshold, confirm_ticks;
  stream >> cruise_speed >> accel_threshold >> confirm_ticks;

  ui->editCruiseSpeed->setText(QString::number(cruise_speed));
  ui->editAccelThreshold->setText(QString::number(accel_threshold));
  ui->editConfirmTicks->setText(QString::number(confirm_ticks));
}

/**
 * @brief Solicita todos los parámetros de configuración del PID de frenado.
 */
void MainWindow::on_btnGetPidBrakingConfig_clicked() {
  sendUnerbusCommand(Unerbus::CommandId::CMD_GET_BRAKING_PID_GAINS);
  QTimer::singleShot(100, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_BRAKING_PARAMS);
  });
  QTimer::singleShot(200, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_BRAKING_MAX_SPEED);
  });
  QTimer::singleShot(300, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_BRAKING_MIN_SPEED);
  });
  QTimer::singleShot(400, this, [this]() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_BRAKING_DEAD_ZONE);
  });
}

/**
 * @brief Envía todos los parámetros de configuración del PID de frenado.
 */
void MainWindow::on_btnSetPidBrakingConfig_clicked() {
  // 1. Enviar Ganancias (Kp, Ki, Kd)
  QByteArray gainsPayload;
  QDataStream gainsStream(&gainsPayload, QIODevice::WriteOnly);
  gainsStream.setByteOrder(QDataStream::LittleEndian);
  gainsStream << static_cast<quint16>(ui->editKpBraking->text().toFloat() *
                                      100.0f);
  gainsStream << static_cast<quint16>(ui->editKiBraking->text().toFloat() *
                                      100.0f);
  gainsStream << static_cast<quint16>(ui->editKdBraking->text().toFloat() *
                                      100.0f);
  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_BRAKING_PID_GAINS,
                     gainsPayload);

  // 2. Enviar Parámetros de Frenado (Target ADC y Umbral Accel)
  QTimer::singleShot(100, this, [this]() {
    QByteArray paramsPayload;
    QDataStream paramsStream(&paramsPayload, QIODevice::WriteOnly);
    paramsStream.setByteOrder(QDataStream::LittleEndian);
    paramsStream << static_cast<quint16>(
        ui->editStopTargetAdc->text().toUShort());
    paramsStream << static_cast<quint16>(
        ui->editAccelStopThreshold->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_BRAKING_PARAMS,
                       paramsPayload);
  });

  // 3. Enviar Velocidad Máxima de Frenado
  QTimer::singleShot(200, this, [this]() {
    QByteArray speedPayload;
    QDataStream speedStream(&speedPayload, QIODevice::WriteOnly);
    speedStream.setByteOrder(QDataStream::LittleEndian);
    speedStream << static_cast<quint16>(
        ui->editBrakingMaxSpeed->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_BRAKING_MAX_SPEED,
                       speedPayload);
  });

  // 4. Enviar Velocidad Mínima de Frenado
  QTimer::singleShot(300, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint16>(ui->editBrakingMinSpeed->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_BRAKING_MIN_SPEED, payload);
  });

  // 5. Enviar Zona Muerta de Frenado
  QTimer::singleShot(400, this, [this]() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint16>(ui->editBrakingDeadZone->text().toUShort());
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_BRAKING_DEAD_ZONE, payload);
  });
}

/**
 * @brief Actualiza la UI con las ganancias del PID de frenado.
 */
void MainWindow::updatePidBrakingUI(const QByteArray &payload) {
  if (payload.size() < 6)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 kp_int, ki_int, kd_int;
  stream >> kp_int >> ki_int >> kd_int;

  ui->editKpBraking->setText(QString::number(kp_int / 100.0, 'f', 2));
  ui->editKiBraking->setText(QString::number(ki_int / 100.0, 'f', 2));
  ui->editKdBraking->setText(QString::number(kd_int / 100.0, 'f', 2));
}

/**
 * @brief Actualiza la UI con los parámetros de frenado.
 */
void MainWindow::updateBrakingParamsUI(const QByteArray &payload) {
  if (payload.size() < 4)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 stop_target, accel_threshold;
  stream >> stop_target >> accel_threshold;

  ui->editStopTargetAdc->setText(QString::number(stop_target));
  ui->editAccelStopThreshold->setText(QString::number(accel_threshold));
}

/**
 * @brief Actualiza la UI con la velocidad máxima de frenado.
 */
void MainWindow::updateBrakingMaxSpeedUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 max_speed;
  stream >> max_speed;

  ui->editBrakingMaxSpeed->setText(QString::number(max_speed));
}

/**
 * @brief Actualiza la UI con la velocidad mínima de frenado.
 */
void MainWindow::updateBrakingMinSpeedUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 min_speed;
  stream >> min_speed;

  ui->editBrakingMinSpeed->setText(QString::number(min_speed));
}

/**
 * @brief Actualiza la UI con la zona muerta de frenado.
 */
void MainWindow::updateBrakingDeadZoneUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;
  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 dead_zone;
  stream >> dead_zone;

  ui->editBrakingDeadZone->setText(QString::number(dead_zone));
}

void MainWindow::updateYawAngleUI(const QByteArray &payload) {
  if (payload.size() < 4)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  qint32 yaw;
  stream >> yaw;

  ui->editYaw->setText(QString::number(yaw));
}

void MainWindow::updateSmoothTurnSpeedsUI(const QByteArray &payload) {
  if (payload.size() < 4)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 faster_motor_smooth_turn_speed, slower_motor_smooth_turn_speed;
  stream >> faster_motor_smooth_turn_speed >> slower_motor_smooth_turn_speed;

  ui->editFasterMotorSmooth->setText(
      QString::number(faster_motor_smooth_turn_speed));
  ui->editSlowerMotorSmooth->setText(
      QString::number(slower_motor_smooth_turn_speed));
}

void MainWindow::updateTurnSpeedPID(const QByteArray &payload) {
  if (payload.size() < 6)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  qint16 kp, ki, kd;
  stream >> kp >> ki >> kd;

  ui->editKpSmoothTurn->setText(QString::number(kp / 100.0, 'f', 2));
  ui->editKiSmoothTurn->setText(QString::number(ki / 100.0, 'f', 2));
  ui->editKdSmoothTurn->setText(QString::number(kd / 100.0, 'f', 2));
}

void MainWindow::updateTurnTargetDps(const QByteArray &payload) {
  if (payload.size() < 2)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 angular_speed;
  stream >> angular_speed;

  ui->editTurnAngularSpeed->setText(QString::number(angular_speed));
}

void MainWindow::updateDelayTicksUI(const QByteArray &payload) {
  if (payload.size() < 1)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint8 delayTicks;
  stream >> delayTicks;

  ui->editWallFadeTicks->setText(QString::number(delayTicks));
}

void MainWindow::on_btnSimTurnL_clicked() {
  // +(4-1) = +3 evita números negativos en módulo 4 de C++
  current_heading = (Heading)((current_heading + 3) % 4);
  drawMaze();
}

void MainWindow::on_btnSimFwd_clicked() {
  int next_x = static_cast<int>(current_x);
  int next_y = static_cast<int>(current_y);

  // Calculamos hacia dónde caminamos
  if (current_heading == HEADING_NORTH)
    next_y--;
  else if (current_heading == HEADING_SOUTH)
    next_y++;
  else if (current_heading == HEADING_EAST)
    next_x++;
  else if (current_heading == HEADING_WEST)
    next_x--;

  // Evitamos salirnos y provocar un acceso fuera de rango
  if (next_x < 0)
    next_x = 0;
  if (next_x >= MAZE_WIDTH)
    next_x = MAZE_WIDTH - 1;
  if (next_y < 0)
    next_y = 0;
  if (next_y >= MAZE_HEIGHT)
    next_y = MAZE_HEIGHT - 1;

  current_x = static_cast<uint8_t>(next_x);
  current_y = static_cast<uint8_t>(next_y);

  // Si NO estamos en modo desarrollador (Mundo físico), grabamos el avance en
  // la memoria.
  if (autonomousTimer->isActive()) {
    sim_maze_map[current_x][current_y] |= CELL_VISITED;
  }

  // Repintamos la escena
  drawMaze();
}

void MainWindow::on_btnSimTurnR_clicked() {
  current_heading = (Heading)((current_heading + 1) % 4);
  drawMaze();
}

void MainWindow::on_btnSimWallLeft_clicked() {
  uint8_t wall = sim_wall_lut[current_heading][2];
  real_maze_map[current_x][current_y] ^= wall;

  // Aplicamos simetría física a la celda vecina
  if (wall == WALL_NORTH && current_y > 0)
    real_maze_map[current_x][current_y - 1] ^= WALL_SOUTH;
  if (wall == WALL_SOUTH && current_y < MAZE_HEIGHT - 1)
    real_maze_map[current_x][current_y + 1] ^= WALL_NORTH;
  if (wall == WALL_EAST && current_x < MAZE_WIDTH - 1)
    real_maze_map[current_x + 1][current_y] ^= WALL_WEST;
  if (wall == WALL_WEST && current_x > 0)
    real_maze_map[current_x - 1][current_y] ^= WALL_EAST;

  drawMaze();
}

void MainWindow::on_btnSimWallFront_clicked() {
  uint8_t wall = sim_wall_lut[current_heading][0];
  real_maze_map[current_x][current_y] ^= wall;

  if (wall == WALL_NORTH && current_y > 0)
    real_maze_map[current_x][current_y - 1] ^= WALL_SOUTH;
  if (wall == WALL_SOUTH && current_y < MAZE_HEIGHT - 1)
    real_maze_map[current_x][current_y + 1] ^= WALL_NORTH;
  if (wall == WALL_EAST && current_x < MAZE_WIDTH - 1)
    real_maze_map[current_x + 1][current_y] ^= WALL_WEST;
  if (wall == WALL_WEST && current_x > 0)
    real_maze_map[current_x - 1][current_y] ^= WALL_EAST;

  drawMaze();
}

void MainWindow::on_btnSimWallRight_clicked() {
  uint8_t wall = sim_wall_lut[current_heading][1];
  real_maze_map[current_x][current_y] ^= wall;

  if (wall == WALL_NORTH && current_y > 0)
    real_maze_map[current_x][current_y - 1] ^= WALL_SOUTH;
  if (wall == WALL_SOUTH && current_y < MAZE_HEIGHT - 1)
    real_maze_map[current_x][current_y + 1] ^= WALL_NORTH;
  if (wall == WALL_EAST && current_x < MAZE_WIDTH - 1)
    real_maze_map[current_x + 1][current_y] ^= WALL_WEST;
  if (wall == WALL_WEST && current_x > 0)
    real_maze_map[current_x - 1][current_y] ^= WALL_EAST;

  drawMaze();
}

void MainWindow::on_btnSimReset_clicked() {
  if (autonomousTimer->isActive())
    autonomousTimer->stop();
  memset(sim_maze_map, 0, sizeof(sim_maze_map));
  memset(
      real_maze_map, 0,
      sizeof(real_maze_map)); // Ahora Creador y Robot pierden la memoria juntos
  current_x = 7;
  current_y = 7;
  current_heading = HEADING_NORTH;
  sim_maze_map[current_x][current_y] |= CELL_VISITED;
  is_returning = false;

  // Limpiar Meta e Inicio
  start_x = -1;
  start_y = -1;
  goal_x = -1;
  goal_y = -1;

  // Enderezar la cámara si el usuario la había dejado rotada
  ui->mazeView->resetTransform();

  drawMaze();
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event) {
  // Verificamos si alguien hizo "Click"
  if (event->type() == QEvent::MouseButtonPress) {

    // Verificamos que ese "Click" haya sido EXCLUSIVAMENTE encima de la ventana
    // del Laberinto
    if (watched == ui->mazeView->viewport()) {

      QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);

      // Obtenemos la coordenada matemática del click en el plano
      QPointF scenePos = ui->mazeView->mapToScene(mouseEvent->pos());
      int cellSize = 50;

      // Convertimos pixeles (ej. 165) a un índice en nuestra matriz 15x15 (ej.
      // 3)
      int cell_x = scenePos.x() / cellSize;
      int cell_y = scenePos.y() / cellSize;

      // Validamos que hiciste click "Adentro" del laberinto y no afuera
      if (cell_x >= 0 && cell_x < MAZE_WIDTH && cell_y >= 0 &&
          cell_y < MAZE_HEIGHT) {

        // ¿Estaba apretado el radio botón de "Fijar Inicio"?
        if (ui->radioPointerStart->isChecked()) {
          start_x = cell_x;
          start_y = cell_y;
          drawMaze();  // Repintamos la escena de inmediato
          return true; // "Consumimos" el evento
        }
        // ¿O estaba apretado el radio botón de "Fijar Meta"?
        else if (ui->radioPointerGoal->isChecked()) {
          goal_x = cell_x;
          goal_y = cell_y;
          drawMaze(); // Repintamos la escena de inmediato
          return true;
        }
        // O estaba apretado el radio botón de "Fijar Especiales"?
        else if (ui->radioPointerSpecial->isChecked()) {
          // El operador ^= alterna (toggle). Un clic la pone, otro clic la
          // quita.
          real_maze_map[cell_x][cell_y] ^= CELL_SPECIAL;
          drawMaze();
          return true;
        }
      }
    }
  }

  // Si fue un click en otra parte o no era un evento de mouse, dejamos fluir el
  // programa:
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::on_btnRotMapL_clicked() {
  // Rota toda la cámara 90 grados a la izquierda (sentido anti-horario)
  ui->mazeView->rotate(-90);
}

void MainWindow::on_btnRotMapR_clicked() {
  // Rota toda la cámara 90 grados a la derecha (sentido horario)
  ui->mazeView->rotate(90);
}

void MainWindow::autonomousTick() {
  // === 1. ACT (Movemos las ruedas) ===
  if (path_length > 1) {
    uint8_t next_cell = best_path[path_length - 2];
    uint8_t next_x = next_cell >> 4;
    uint8_t next_y = next_cell & 0x0F;

    // Girar
    if (next_y < current_y)
      current_heading = HEADING_NORTH;
    else if (next_y > current_y)
      current_heading = HEADING_SOUTH;
    else if (next_x > current_x)
      current_heading = HEADING_EAST;
    else if (next_x < current_x)
      current_heading = HEADING_WEST;

    // Mover 1 bloque
    current_x = next_x;
    current_y = next_y;
  }

  // === 2. SENSE (Los ojos infrarrojos leen el piso físico) ===
  uint8_t current_real_walls =
      real_maze_map[current_x][current_y] &
      (WALL_NORTH | WALL_SOUTH | WALL_EAST | WALL_WEST);
  // Inyectamos las paredes a la memoria RAM
  sim_maze_map[current_x][current_y] |= current_real_walls | CELL_VISITED;
  // Simetría sensorial para cuando querramos retroceder
  if (current_real_walls & WALL_NORTH && current_y > 0)
    sim_maze_map[current_x][current_y - 1] |= WALL_SOUTH;
  if (current_real_walls & WALL_SOUTH && current_y < MAZE_HEIGHT - 1)
    sim_maze_map[current_x][current_y + 1] |= WALL_NORTH;
  if (current_real_walls & WALL_EAST && current_x < MAZE_WIDTH - 1)
    sim_maze_map[current_x + 1][current_y] |= WALL_WEST;
  if (current_real_walls & WALL_WEST && current_x > 0)
    sim_maze_map[current_x - 1][current_y] |= WALL_EAST;
  // --- NUEVO: ¿Pisamos una celda con Cinta Especial? ---
  if (real_maze_map[current_x][current_y] & CELL_SPECIAL) {
    if (!(sim_maze_map[current_x][current_y] & CELL_SPECIAL)) {
      sim_maze_map[current_x][current_y] |= CELL_SPECIAL; // Grabar el hallazgo
      special_cells_found++;
      qDebug() << "Célula especial encontrada! Llevamos:" << special_cells_found
               << "de 3";
    }
  }
  // === 3. THINK (A dónde ir) ===
  int current_target_x = start_x;
  int current_target_y = start_y;
  if (!is_mode_b) {
    // MODO A: Explorar hasta hallar las 3 cintas
    if (special_cells_found < 3) {
      bool found_unvisited = false;
      int closest_dist = 9999;

      // Búsqueda de Frontera Orgánica:
      for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
          // Solo nos interesan celdas sin visitar...
          if (!(sim_maze_map[x][y] & CELL_VISITED)) {

            // ... y que estén ADYACENTES a nuestro mapa ya descubierto SIN
            // paredes bloqueando
            bool is_frontier = false;
            if (x > 0 && (sim_maze_map[x - 1][y] & CELL_VISITED) &&
                !(sim_maze_map[x][y] & WALL_WEST))
              is_frontier = true;
            if (x < MAZE_WIDTH - 1 && (sim_maze_map[x + 1][y] & CELL_VISITED) &&
                !(sim_maze_map[x][y] & WALL_EAST))
              is_frontier = true;
            if (y > 0 && (sim_maze_map[x][y - 1] & CELL_VISITED) &&
                !(sim_maze_map[x][y] & WALL_NORTH))
              is_frontier = true;
            if (y < MAZE_HEIGHT - 1 &&
                (sim_maze_map[x][y + 1] & CELL_VISITED) &&
                !(sim_maze_map[x][y] & WALL_SOUTH))
              is_frontier = true;

            if (is_frontier) {
              // De todas las opciones de expansión, elegimos la MÁS CERCANA a
              // nosotros (Distancia Manhattan)
              int dist = abs(x - current_x) + abs(y - current_y);
              if (dist < closest_dist) {
                closest_dist = dist;
                current_target_x = x;
                current_target_y = y;
                found_unvisited = true;
              }
            }
          }
        }
      }

      // Si por error de diseño se quedó encerrado sin frontera antes de hallar
      // 3, evitamos que crashee
      if (!found_unvisited) {
        autonomousTimer->stop();
        drawMaze();
        qDebug() << "ALERTA: El robot se quedó encerrado y no hay adónde ir.";
        return;
      }
    } else {
      // YA TENEMOS LAS 3! Retornamos al inicio (start_x, start_y)
      current_target_x = start_x;
      current_target_y = start_y;

      if (current_x == start_x && current_y == start_y) {
        qDebug() << "MODO A FINALIZADO. Volvió a casa tras hallar las cintas.";
        is_mode_b = true;        // Pasamos el robot a Estado de Carreras
        autonomousTimer->stop(); // Frenamos el autito!
        drawMaze();
        return;
      }
    }

    // En Modo A, calculamos la ruta paso a paso evadiendo las paredes nuevas
    calculateFastestPath(current_x, current_y, current_target_x,
                         current_target_y);
    if (path_length <= 1 &&
        (current_x != current_target_x || current_y != current_target_y)) {
      // La meta asignada es inalcanzable (está fuera del 8x6 o encerrada entre
      // paredes).
      // La engañamos marcándola como "visitada" para que el ciclo For la ignore
      // la próxima vez.
      sim_maze_map[current_target_x][current_target_y] |= CELL_VISITED;
    }
  } else {
    // ---> MODO B: TSP SPEED_RUN <---
    // PASO 1: Evaluar la mejor ruta si aún no lo ha hecho
    if (!tsp_solved) {
      // Extraemos dónde están las 3 cintas en la memoria RAM
      int sc[3][2];
      int idx = 0;
      for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
          if (sim_maze_map[x][y] & CELL_SPECIAL) {
            if (idx < 3) {
              sc[idx][0] = x;
              sc[idx][1] = y;
              idx++;
            }
          }
        }
      }

      if (idx == 3) {
        // Las 6 combinaciones (permutaciones) de visitas posibles
        int min_dist = 99999;
        int best_p[3] = {0, 1, 2};
        int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                           {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};

        // Agente Viajero (Evaluamos las 6 rutas sumando los 3 tramos)
        for (int i = 0; i < 6; i++) {
          int d1 = getFloodFillDistance(start_x, start_y, sc[perms[i][0]][0],
                               sc[perms[i][0]][1]);
          int d2 = getFloodFillDistance(sc[perms[i][0]][0], sc[perms[i][0]][1],
                               sc[perms[i][1]][0], sc[perms[i][1]][1]);
          int d3 = getFloodFillDistance(sc[perms[i][1]][0], sc[perms[i][1]][1],
                               sc[perms[i][2]][0], sc[perms[i][2]][1]);
          int totalCost = d1 + d2 + d3;

          if (totalCost < min_dist) {
            min_dist = totalCost;
            best_p[0] = perms[i][0];
            best_p[1] = perms[i][1];
            best_p[2] = perms[i][2];
          }
        }

        // Guardar el cronograma de vuelos ganador
        tsp_targets[0][0] = sc[best_p[0]][0];
        tsp_targets[0][1] = sc[best_p[0]][1];
        tsp_targets[1][0] = sc[best_p[1]][0];
        tsp_targets[1][1] = sc[best_p[1]][1];
        tsp_targets[2][0] = sc[best_p[2]][0];
        tsp_targets[2][1] = sc[best_p[2]][1];
        tsp_solved = true;
        current_tsp_index = 0;
        qDebug() << "Ruta TSP Calculada. Distancia Óptima:" << min_dist;
      } else {
        qDebug() << "ERROR: Alguna cinta se perdió.";
        autonomousTimer->stop();
        return;
      }
    }
    // PASO 2: Ejecutar el cronograma de vuelos
    if (tsp_solved && current_tsp_index < 3) {
      current_target_x = tsp_targets[current_tsp_index][0];
      current_target_y = tsp_targets[current_tsp_index][1];

      if (current_x == current_target_x && current_y == current_target_y) {
        current_tsp_index++;
        if (current_tsp_index >= 3) {
          qDebug()
              << "¡MISION CUMPLIDA! El Micromouse se detiene en la 3ra marca.";
          autonomousTimer->stop();
          drawMaze();
          return;
        } else {
          // Cargar el radar con el siguiente objetivo
          current_target_x = tsp_targets[current_tsp_index][0];
          current_target_y = tsp_targets[current_tsp_index][1];
        }
      }
    }

    // Pilotar usando nuestro propio sistema de trazado hacia el actual waypoint
    calculateFastestPath(current_x, current_y, current_target_x,
                         current_target_y);
  }

  // === 4. Draw ===
  calculateFastestPath(current_x, current_y, current_target_x,
                       current_target_y);
  drawMaze();
}

void MainWindow::on_btnToggleAutonomous_clicked() {
  if (autonomousTimer->isActive()) {
    autonomousTimer->stop();
  } else {
    // Teletransportar a la línea de largada (A) antes de comenzar
    if (start_x != -1) {
      current_x = start_x;
      current_y = start_y;
      current_heading = HEADING_NORTH;
      is_returning = false;

      // El robot revisa su propia memoria para ver cuántas cintas recuerda
      special_cells_found = 0;
      for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
          if (sim_maze_map[x][y] & CELL_SPECIAL) {
            special_cells_found++;
          }
        }
      }
      if (special_cells_found >= 3) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(
            this, "Modo B Listo",
            "El robot memorizó la ubicación de las 3 cintas.\n¿Deseas iniciar "
            "la Carrera de Velocidad (Modo B)?\n\nPresiona [No] para formatear "
            "la memoria y volver a Explorar (Modo A).",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
          is_mode_b = true;
          tsp_solved = false;    // Obligamos al TSP a re-calcular las 6 rutas
          current_tsp_index = 0; // Reseteamos el listado de visitas a 0
          qDebug() << "INICIANDO SPEED RUN (Modo B)!";
        } else {
          memset(sim_maze_map, 0, sizeof(sim_maze_map)); // Amnesia total
          special_cells_found = 0;
          is_mode_b = false;
          qDebug() << "INICIANDO EXPLORACIÓN (Modo A). Memoria borrada.";
        }
      } else {
        is_mode_b = false;
        qDebug() << "INICIANDO EXPLORACIÓN (Modo A). Buscando cintas...";
      }

      drawMaze();
    }
    autonomousTimer->start(500); // Corre a 500ms por paso
  }
}

void MainWindow::on_btnSaveMaze_clicked()
{
    QString path = QFileDialog::getSaveFileName(this, "Guardar Mapa", "", "Maze Files (*.maze)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(reinterpret_cast<char*>(&start_x), sizeof(int));
        file.write(reinterpret_cast<char*>(&start_y), sizeof(int));
        file.write(reinterpret_cast<char*>(&goal_x), sizeof(int));
        file.write(reinterpret_cast<char*>(&goal_y), sizeof(int));
        file.write(reinterpret_cast<char*>(real_maze_map), sizeof(real_maze_map));
        file.close();
    }
}


void MainWindow::on_btnLoadMaze_clicked()
{
    QString path = QFileDialog::getOpenFileName(this, "Cargar Mapa", "", "Maze Files (*.maze)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (file.open(QIODevice::ReadOnly)) {
        file.read(reinterpret_cast<char*>(&start_x), sizeof(int));
        file.read(reinterpret_cast<char*>(&start_y), sizeof(int));
        file.read(reinterpret_cast<char*>(&goal_x), sizeof(int));
        file.read(reinterpret_cast<char*>(&goal_y), sizeof(int));
        file.read(reinterpret_cast<char*>(real_maze_map), sizeof(real_maze_map));
        file.close();

        // Resetear estado del robot
        memset(sim_maze_map, 0, sizeof(sim_maze_map));
        current_x = start_x;
        current_y = start_y;
        current_heading = HEADING_NORTH;
        special_cells_found = 0;
        is_mode_b = false;

        drawMaze();
    }
}


void MainWindow::on_btnSyncMaze_clicked()
{
    if (!serialPort->isOpen() && !udpSocket) {
        QMessageBox::warning(this, "Error", "Debe estar conectado para sincronizar el laberinto.");
        return;
    }
    ui->commsLog->appendPlainText("Iniciando sincronización del laberinto...");
    requestMazeColumn(0); // Iniciamos el efecto dominó pidiendo la columna 0
}

void MainWindow::requestMazeColumn(quint8 col) {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << col;
    sendUnerbusCommand(Unerbus::CommandId::CMD_SYNC_MAZE_COLUMN, payload);
}
