#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QBrush>
#include <QDataStream>
#include <QDebug>
#include <QFormLayout>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGroupBox>
#include <QIntValidator>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaEnum>
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

constexpr quint8 PRIM_TEST_STOP = 0x00;
constexpr quint8 PRIM_TEST_START = 0x01;
constexpr quint8 PRIM_TEST_GET_STATUS = 0x02;
constexpr quint8 PRIM_TEST_SET_CONFIG = 0x03;
constexpr quint8 PRIM_TEST_GET_CONFIG = 0x04;
constexpr quint8 PRIM_TEST_SMOOTH_TURN = 0x01;
constexpr quint8 PRIM_TEST_SMOOTH_LEFT = 0x00;
constexpr quint8 PRIM_TEST_SMOOTH_RIGHT = 0x01;
constexpr quint8 PRIM_TEST_RESP_STATUS = 0x80;
constexpr quint8 PRIM_TEST_RESP_CONFIG = 0x81;
constexpr int PRIM_TEST_STATUS_SIZE = 26;
constexpr int PRIM_TEST_CONFIG_SIZE = 16;
constexpr quint8 SUPERVISOR_RUN_MODE_FIND_CELLS = 0x01;
constexpr quint8 SUPERVISOR_RUN_MODE_GO_A_TO_B = 0x02;
constexpr int SUPERVISOR_DEBUG_STATUS_SIZE = 9;

quint8 supervisorHeadingForComboIndex(int index) {
  switch (index) {
  case 1:
    return static_cast<quint8>(HEADING_EAST);
  case 2:
    return static_cast<quint8>(HEADING_SOUTH);
  case 3:
    return static_cast<quint8>(HEADING_WEST);
  case 0:
  default:
    return static_cast<quint8>(HEADING_NORTH);
  }
}

int comboIndexForSupervisorHeading(quint8 heading) {
  switch (heading) {
  case HEADING_EAST:
    return 1;
  case HEADING_SOUTH:
    return 2;
  case HEADING_WEST:
    return 3;
  case HEADING_NORTH:
  default:
    return 0;
  }
}

quint8 supervisorRunModeForComboIndex(int index) {
  return (index == 1) ? SUPERVISOR_RUN_MODE_GO_A_TO_B
                      : SUPERVISOR_RUN_MODE_FIND_CELLS;
}

QString supervisorStateToText(quint8 state) {
    switch (state) {
    case 0:
        return "IDLE";
    case 1:
        return "START_INITIAL_ADVANCE";
    case 2:
        return "RUN_INITIAL_ADVANCE";
    case 3:
        return "DECIDE";
    case 4:
        return "RUN_ADVANCE";
    case 5:
        return "RUN_APPROACH_FRONT_WALL";
    case 6:
        return "RUN_SMOOTH_LEFT";
    case 7:
        return "RUN_SMOOTH_RIGHT";
    case 8:
        return "RUN_PIVOT_180";
    case 9:
        return "ERROR";
    default:
        return QString("UNKNOWN(%1)").arg(state);
    }
}

QString supervisorActionToText(quint8 action) {
    switch (action) {
    case 0:
        return "NONE";
    case 1:
        return "INITIAL_ADVANCE";
    case 2:
        return "ADVANCE";
    case 3:
        return "APPROACH_FRONT_WALL";
    case 4:
        return "SMOOTH_LEFT";
    case 5:
        return "SMOOTH_RIGHT";
    case 6:
        return "PIVOT_180";
    default:
        return QString("UNKNOWN(%1)").arg(action);
    }
}

QString supervisorResultToText(quint8 result) {
    switch (result) {
    case 0:
        return "OK";
    case 1:
        return "INVALID_ARGUMENT";
    case 2:
        return "START_FAILED";
    case 3:
        return "PRIMITIVE_ERROR";
    case 4:
        return "UNSUPPORTED_ACTION";
    case 5:
        return "FIND_CELLS_COMPLETE";
    case 6:
        return "FIND_CELLS_INCOMPLETE_NO_FRONTIER";
    default:
        return QString("UNKNOWN(%1)").arg(result);
    }
}

QString mazeHeadingToText(quint8 heading) {
    switch (heading) {
    case HEADING_NORTH:
        return "Norte";
    case HEADING_EAST:
        return "Este";
    case HEADING_SOUTH:
        return "Sur";
    case HEADING_WEST:
        return "Oeste";
    default:
        return QString("Unknown(%1)").arg(heading);
    }
}

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
      pwmUpdateTimer(new QTimer(this)),
      primitiveTestUpdateTimer_(new QTimer(this))
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
  navigationButtonGroup->addButton(ui->btnPruebas, 4);
  navigationButtonGroup->addButton(
      ui->btnLabyrinth, 5); // ID 5 para la página de visual del laberinto
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
  connect(primitiveTestUpdateTimer_, &QTimer::timeout, this,
          &MainWindow::requestPrimitiveTestStatus);

  updateSerialPortList();
  updateUIState(false, false);                   // Estado inicial: desconectado
  updateConnectionStatus("Desconectado", "red"); // Establecer estado inicial

  populateCMDComboBox();
  setupControlPage(); // Configurar la página de control
  setupConfigPage();
  setupPrimitiveTestPage();
  setupActivitiesTab();
  setupSupervisorDebugPanel();

  QIntValidator *turnAngleValidator = new QIntValidator(-360, 360, this);
  ui->editTurnAngle->setValidator(turnAngleValidator);

  // ==========================================
  // INICIALIZACIÓN DEL SIMULADOR DE LABERINTO
  // ==========================================

  // 1. Crear el escenario e insertarlo en la vista del UI
  mazeScene = new QGraphicsScene(this);
  ui->mazeView->setScene(mazeScene);

  // Opcional: Le damos un fondo oscuro muy moderno al canvas
  mazeScene->setBackgroundBrush(QColor(20, 25, 30));
  // 2. Limpiar todos los mapas lógicos (llenarlos de 0)
  memset(robot_maze_map, 0, sizeof(robot_maze_map));

  // 3. El robot nace en el centro lógico de la matriz de 15x15
  current_x = 7;
  current_y = 7;
  current_heading = HEADING_NORTH;

  // 4. Marcamos la celda en la que empezamos como "Visitada"
  robot_maze_map[current_x][current_y] |= CELL_VISITED;

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

  if (pageIndex == 4) {
    requestPrimitiveTestConfig();
    requestPrimitiveTestStatus();
  } else if (pageIndex == 5) {
      requestSupervisorInitialPose();
      requestSupervisorDebugStatus();
  }

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

          const int logical_x = static_cast<int>(x);
          const int logical_y = static_cast<int>(y_stm);

          if (logical_x >= 0 && logical_x < MAZE_WIDTH &&
              logical_y >= 0 && logical_y < MAZE_HEIGHT) {

              // robot_maze_map usa coordenadas lógicas STM32.
              // La inversión Y se aplica solo al dibujar.
              robot_maze_map[logical_x][logical_y] = walls;

              // Propagación simétrica de paredes en coordenadas lógicas STM32.
              // NORTH = y + 1, SOUTH = y - 1.
              if ((walls & WALL_NORTH) && (logical_y < MAZE_HEIGHT - 1))
                  robot_maze_map[logical_x][logical_y + 1] |= WALL_SOUTH;

              if ((walls & WALL_SOUTH) && (logical_y > 0))
                  robot_maze_map[logical_x][logical_y - 1] |= WALL_NORTH;

              if ((walls & WALL_EAST) && (logical_x < MAZE_WIDTH - 1))
                  robot_maze_map[logical_x + 1][logical_y] |= WALL_WEST;

              if ((walls & WALL_WEST) && (logical_x > 0))
                  robot_maze_map[logical_x - 1][logical_y] |= WALL_EAST;

              current_x = x;
              current_y = y_stm;
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
  case Unerbus::CommandId::CMD_GET_APPROACH_FRONT_WALL_TARGET: {
    updateApproachFrontWallTargetUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_SUPERVISOR_INITIAL_POSE: {
    updateSupervisorInitialPoseUI(payload);
    break;
  }
  case Unerbus::CommandId::CMD_GET_SUPERVISOR_DEBUG_STATUS: {
      updateSupervisorDebugStatusUI(payload);
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

          // 1. Extraemos y guardamos las 15 celdas de esta columna.
          // IMPORTANTE: robot_maze_map usa coordenadas lógicas STM32.
          // La inversión Y se aplica solo al dibujar.
          for (int logical_y = 0; logical_y < MAZE_HEIGHT; logical_y++) {
              quint8 cell_data;
              stream >> cell_data;

              robot_maze_map[col][logical_y] = cell_data;
          }

          // 2. Extraemos la posición y orientación actual del robot.
          // current_x/current_y también quedan en coordenadas lógicas STM32.
          quint8 x, y_stm, heading;
          stream >> x >> y_stm >> heading;
          current_x = x;
          current_y = y_stm;
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
  case Unerbus::CommandId::CMD_PRIMITIVE_TEST: {
    updatePrimitiveTestResponse(payload);
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
  ui->CMDComboBox->addItem(
      "GET_NAV_DEBUG_STATUS (0x94)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_NAV_DEBUG_STATUS));
  ui->CMDComboBox->addItem(
      "GET_SUPERVISOR_DEBUG_STATUS (0x9C)",
      static_cast<quint8>(Unerbus::CommandId::CMD_GET_SUPERVISOR_DEBUG_STATUS));
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

static int logicalYToSceneRow(int logicalY)
{
    return MAZE_HEIGHT - 1 - logicalY;
}

static int sceneRowToLogicalY(int sceneRow)
{
    return MAZE_HEIGHT - 1 - sceneRow;
}

void MainWindow::drawMaze() {

  mazeScene->clear();

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

    const int logical_y_label = sceneRowToLogicalY(i);

    QGraphicsTextItem *leftText =
        new QGraphicsTextItem(QString::number(logical_y_label), leftAnchor);
    leftText->setFont(numberFont);
    leftText->setDefaultTextColor(textPen.color());
    leftText->setPos(-leftText->boundingRect().width() / 2.0,
                     -leftText->boundingRect().height() / 2.0);
  }

  // 2. DIBUJAR LA GRILLA Y LAS PAREDES
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
        const int logical_x = x;
        const int logical_y = y;

        int px = logical_x * cellSize;
        int py = logicalYToSceneRow(logical_y) * cellSize;
        uint8_t current_map_cell = 0;

        current_map_cell = robot_maze_map[logical_x][logical_y];

      // === Colores Especiales de Baldosa ===
      QColor cellBgColor = Qt::transparent; // Vacío por defecto

        if (current_map_cell & CELL_SPECIAL) {
            cellBgColor = QColor(255, 215, 0);
        }
        // ¿Al menos está visitada?
        else if (current_map_cell & CELL_VISITED) {
        cellBgColor = QColor(255, 255, 255, 10); // Blancuzco tenue
        }

      // Dibujamos el cuadrado base y guardamos el puntero
      QGraphicsRectItem *cellRect =
          mazeScene->addRect(px, py, cellSize, cellSize, faintPen, cellBgColor);

      // Si el robot visitó esta celda, dibujamos sus paredes reales
      uint8_t cellData = current_map_cell;

      if (cellData & CELL_VISITED) {
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

  // 4. DIBUJAR AL ROBOT
  // El robot debe estar en la coordenada (current_x, current_y).
  // Para que se vea centrado y más chico que la celda:
  int robotSize = cellSize / 2; // Por ej: 25x25 pixeles
  int rx = (current_x * cellSize) + (cellSize / 4);
  int ry = (logicalYToSceneRow(current_y) * cellSize) + (cellSize / 4);

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

  ui->editApproachFrontWallTargetMm->setValidator(
      new QIntValidator(10, 150, this));

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
  QTimer::singleShot(500, this, [this]() {
    sendUnerbusCommand(
        Unerbus::CommandId::CMD_GET_APPROACH_FRONT_WALL_TARGET);
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

  // 6. Enviar distancia frontal de aproximaciÃ³n para pivot de callejÃ³n.
  QTimer::singleShot(500, this,
                     &MainWindow::sendApproachFrontWallTarget);
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
void MainWindow::sendApproachFrontWallTarget() {
  quint16 target = ui->editApproachFrontWallTargetMm->text().toUShort();
  if (target < 10U)
    target = 10U;
  else if (target > 150U)
    target = 150U;

  ui->editApproachFrontWallTargetMm->setText(QString::number(target));

  QByteArray payload;
  QDataStream stream(&payload, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);
  stream << target;
  sendUnerbusCommand(Unerbus::CommandId::CMD_SET_APPROACH_FRONT_WALL_TARGET,
                     payload);
}

void MainWindow::updateApproachFrontWallTargetUI(const QByteArray &payload) {
  if (payload.size() < 2)
    return;

  QDataStream stream(payload);
  stream.setByteOrder(QDataStream::LittleEndian);

  quint16 target_mm;
  stream >> target_mm;

  ui->editApproachFrontWallTargetMm->setText(QString::number(target_mm));
}

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

void MainWindow::on_btnSimReset_clicked() {
    memset(robot_maze_map, 0, sizeof(robot_maze_map));

    current_x = 7;
    current_y = 7;
    current_heading = HEADING_NORTH;

    robot_maze_map[current_x][current_y] |= CELL_VISITED;

    ui->mazeView->resetTransform();

    drawMaze();
}

void MainWindow::on_btnRotMapL_clicked() {
  // Rota toda la cámara 90 grados a la izquierda (sentido anti-horario)
  ui->mazeView->rotate(-90);
}

void MainWindow::on_btnRotMapR_clicked() {
  // Rota toda la cámara 90 grados a la derecha (sentido horario)
  ui->mazeView->rotate(90);
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

void MainWindow::setupSupervisorDebugPanel() {
    QGroupBox *group = new QGroupBox("Estado supervisor", ui->manualControls);
    QFormLayout *layout = new QFormLayout(group);

    auto createValueLabel = [group]() {
        QLabel *label = new QLabel("-", group);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        label->setMinimumWidth(120);
        return label;
    };

    lblSupervisorActive = createValueLabel();
    lblSupervisorState = createValueLabel();
    lblSupervisorAction = createValueLabel();
    lblSupervisorResult = createValueLabel();
    lblSupervisorPose = createValueLabel();
    lblSupervisorCell = createValueLabel();
    lblSupervisorSpecials = createValueLabel();

    layout->addRow("Activo:", lblSupervisorActive);
    layout->addRow("Estado:", lblSupervisorState);
    layout->addRow("Acción:", lblSupervisorAction);
    layout->addRow("Resultado:", lblSupervisorResult);
    layout->addRow("Pose:", lblSupervisorPose);
    layout->addRow("Celda:", lblSupervisorCell);
    layout->addRow("Especiales:", lblSupervisorSpecials);

    QPushButton *btnRefreshSupervisor = new QPushButton("Actualizar estado", group);
    layout->addRow(btnRefreshSupervisor);

    connect(btnRefreshSupervisor, &QPushButton::clicked, this,
            &MainWindow::requestSupervisorDebugStatus);

    QVBoxLayout *panelLayout =
        qobject_cast<QVBoxLayout *>(ui->manualControls->layout());

    if (panelLayout != nullptr) {
        const int rotateIndex = panelLayout->indexOf(ui->groupBox_11);
        if (rotateIndex >= 0) {
            panelLayout->insertWidget(rotateIndex, group);
        } else {
            panelLayout->addWidget(group);
        }
    }
}

void MainWindow::requestMazeColumn(quint8 col) {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << col;
    sendUnerbusCommand(Unerbus::CommandId::CMD_SYNC_MAZE_COLUMN, payload);
}

void MainWindow::sendSupervisorInitialPose() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    const quint8 x = static_cast<quint8>(ui->spinInitialCellX->value());
    const quint8 y = static_cast<quint8>(ui->spinInitialCellY->value());
    const quint8 heading =
        supervisorHeadingForComboIndex(ui->comboInitialHeading->currentIndex());

    stream << x << y << heading;
    sendUnerbusCommand(Unerbus::CommandId::CMD_SET_SUPERVISOR_INITIAL_POSE,
                       payload);
}

void MainWindow::requestSupervisorInitialPose() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_SUPERVISOR_INITIAL_POSE);
}

void MainWindow::requestSupervisorDebugStatus() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_GET_SUPERVISOR_DEBUG_STATUS);
}

void MainWindow::updateSupervisorInitialPoseUI(const QByteArray &payload) {
    if (payload.size() < 3)
        return;

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 x;
    quint8 y;
    quint8 heading;
    stream >> x >> y >> heading;

    if ((x >= MAZE_WIDTH) || (y >= MAZE_HEIGHT) ||
        (heading > static_cast<quint8>(HEADING_WEST))) {
        return;
    }

    ui->spinInitialCellX->setValue(static_cast<int>(x));
    ui->spinInitialCellY->setValue(static_cast<int>(y));
    ui->comboInitialHeading->setCurrentIndex(
        comboIndexForSupervisorHeading(heading));
}

void MainWindow::updateSupervisorDebugStatusUI(const QByteArray &payload) {
    if (payload.size() < SUPERVISOR_DEBUG_STATUS_SIZE) {
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 state;
    quint8 action;
    quint8 active;
    quint8 result;
    quint8 maze_x;
    quint8 maze_y;
    quint8 maze_heading;
    quint8 maze_cell;
    quint8 special_found_count;

    stream >> state >> action >> active >> result
        >> maze_x >> maze_y >> maze_heading >> maze_cell
        >> special_found_count;

    if (lblSupervisorActive == nullptr) {
        return;
    }

    lblSupervisorActive->setText(active ? "Sí" : "No");
    lblSupervisorState->setText(
        QString("%1 (%2)").arg(supervisorStateToText(state)).arg(state));
    lblSupervisorAction->setText(
        QString("%1 (%2)").arg(supervisorActionToText(action)).arg(action));
    lblSupervisorResult->setText(
        QString("%1 (%2)").arg(supervisorResultToText(result)).arg(result));
    lblSupervisorPose->setText(
        QString("(%1, %2), %3")
            .arg(maze_x)
            .arg(maze_y)
            .arg(mazeHeadingToText(maze_heading)));
    lblSupervisorCell->setText(
        QString("0x%1").arg(maze_cell, 2, 16, QChar('0')).toUpper());
    lblSupervisorSpecials->setText(QString("%1/3").arg(special_found_count));
}

void MainWindow::on_btnSetSupervisorInitialPose_clicked() {
    sendSupervisorInitialPose();
}

void MainWindow::on_btnGetSupervisorInitialPose_clicked() {
    requestSupervisorInitialPose();
}

void MainWindow::on_btnStartSupervisorRun_clicked() {
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    stream << supervisorRunModeForComboIndex(
        ui->comboSupervisorRunMode->currentIndex());
    sendUnerbusCommand(Unerbus::CommandId::CMD_START_SUPERVISOR_RUN, payload);
    requestSupervisorDebugStatus();
}

void MainWindow::on_btnStopSupervisorRun_clicked() {
    sendUnerbusCommand(Unerbus::CommandId::CMD_STOP_SUPERVISOR_RUN);
    requestSupervisorDebugStatus();
}

void MainWindow::setupPrimitiveTestPage()
{
    ui->comboPrimitiveTest->blockSignals(true);
    ui->comboPrimitiveTest->clear();
    ui->comboPrimitiveTest->addItem("Smooth turn left", PRIM_TEST_SMOOTH_LEFT);
    ui->comboPrimitiveTest->addItem("Smooth turn right", PRIM_TEST_SMOOTH_RIGHT);
    ui->comboPrimitiveTest->blockSignals(false);

    ui->stackedPrimitiveConfig->setCurrentWidget(ui->pagePrimitiveConfigSmooth);
    ui->stackedPrimitiveTelemetry->setCurrentWidget(ui->pagePrimitiveTelemetrySmooth);

    QFormLayout *configLayout = qobject_cast<QFormLayout *>(ui->pagePrimitiveConfigSmooth->layout());
    if (configLayout == nullptr) {
        configLayout = new QFormLayout(ui->pagePrimitiveConfigSmooth);
    }

    auto createSpin = [](QWidget *parent, int minimum, int maximum, int value) {
        QSpinBox *spin = new QSpinBox(parent);
        spin->setRange(minimum, maximum);
        spin->setValue(value);
        return spin;
    };

    spinPrimSmoothKp = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 2000);
    spinPrimSmoothKi = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 500);
    spinPrimSmoothKd = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 200);
    spinPrimSmoothOutputLimit = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 6500);
    spinPrimSmoothFasterPwm = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 6000);
    spinPrimSmoothSlowerPwm = createSpin(ui->pagePrimitiveConfigSmooth, 0, 65535, 2500);
    spinPrimSmoothTargetDps = createSpin(ui->pagePrimitiveConfigSmooth, 0, 2000, 360);

    configLayout->addRow("Kp x100", spinPrimSmoothKp);
    configLayout->addRow("Ki x100", spinPrimSmoothKi);
    configLayout->addRow("Kd x100", spinPrimSmoothKd);
    configLayout->addRow("Output limit PWM", spinPrimSmoothOutputLimit);
    configLayout->addRow("Faster PWM", spinPrimSmoothFasterPwm);
    configLayout->addRow("Slower PWM", spinPrimSmoothSlowerPwm);
    configLayout->addRow("Target dps", spinPrimSmoothTargetDps);

    QFormLayout *telemetryLayout = qobject_cast<QFormLayout *>(ui->pagePrimitiveTelemetrySmooth->layout());
    if (telemetryLayout == nullptr) {
        telemetryLayout = new QFormLayout(ui->pagePrimitiveTelemetrySmooth);
    }

    auto createLabel = [](QWidget *parent) {
        QLabel *label = new QLabel("-", parent);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return label;
    };

    lblPrimTestActive = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestState = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestElapsed = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestYaw = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestYawRate = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestTargetDps = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestLeftPwm = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestRightPwm = createLabel(ui->pagePrimitiveTelemetrySmooth);
    lblPrimTestResult = createLabel(ui->pagePrimitiveTelemetrySmooth);

    telemetryLayout->addRow("Active", lblPrimTestActive);
    telemetryLayout->addRow("State", lblPrimTestState);
    telemetryLayout->addRow("Elapsed ms", lblPrimTestElapsed);
    telemetryLayout->addRow("Yaw deg", lblPrimTestYaw);
    telemetryLayout->addRow("Yaw rate dps", lblPrimTestYawRate);
    telemetryLayout->addRow("Target dps", lblPrimTestTargetDps);
    telemetryLayout->addRow("Left PWM", lblPrimTestLeftPwm);
    telemetryLayout->addRow("Right PWM", lblPrimTestRightPwm);
    telemetryLayout->addRow("Result", lblPrimTestResult);
}

quint8 MainWindow::currentPrimitiveVariant() const
{
    return static_cast<quint8>(ui->comboPrimitiveTest->currentData().toUInt());
}

void MainWindow::sendPrimitiveSmoothConfig()
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);

    stream << PRIM_TEST_SET_CONFIG;
    stream << PRIM_TEST_SMOOTH_TURN;
    stream << static_cast<quint16>(spinPrimSmoothKp->value());
    stream << static_cast<quint16>(spinPrimSmoothKi->value());
    stream << static_cast<quint16>(spinPrimSmoothKd->value());
    stream << static_cast<quint16>(spinPrimSmoothOutputLimit->value());
    stream << static_cast<quint16>(spinPrimSmoothFasterPwm->value());
    stream << static_cast<quint16>(spinPrimSmoothSlowerPwm->value());
    stream << static_cast<quint16>(spinPrimSmoothTargetDps->value());

    sendUnerbusCommand(Unerbus::CommandId::CMD_PRIMITIVE_TEST, payload);
}

void MainWindow::requestPrimitiveTestStatus()
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << PRIM_TEST_GET_STATUS;
    sendUnerbusCommand(Unerbus::CommandId::CMD_PRIMITIVE_TEST, payload);
}

void MainWindow::requestPrimitiveTestConfig()
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << PRIM_TEST_GET_CONFIG;
    stream << PRIM_TEST_SMOOTH_TURN;
    sendUnerbusCommand(Unerbus::CommandId::CMD_PRIMITIVE_TEST, payload);
}

void MainWindow::updatePrimitiveTestResponse(const QByteArray &payload)
{
    if (payload.isEmpty()) {
        return;
    }

    const quint8 responseType = static_cast<quint8>(payload.at(0));
    if (responseType == PRIM_TEST_RESP_STATUS) {
        updatePrimitiveTestStatus(payload);
    } else if (responseType == PRIM_TEST_RESP_CONFIG) {
        updatePrimitiveTestConfig(payload);
    }
}

void MainWindow::updatePrimitiveTestStatus(const QByteArray &payload)
{
    if (payload.size() < PRIM_TEST_STATUS_SIZE) {
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 responseType, testType, variant, active, state, result;
    quint32 elapsedMs;
    qint32 yawDegX10, yawRateDpsX10, targetDpsX10;
    qint16 leftPwm, rightPwm;

    stream >> responseType >> testType >> variant >> active >> state >> result;
    stream >> elapsedMs >> yawDegX10 >> yawRateDpsX10 >> targetDpsX10;
    stream >> leftPwm >> rightPwm;

    Q_UNUSED(responseType);
    Q_UNUSED(testType);
    Q_UNUSED(variant);

    lblPrimTestActive->setText(active ? "yes" : "no");
    lblPrimTestState->setText(QString::number(state));
    lblPrimTestResult->setText(QString::number(result));
    lblPrimTestElapsed->setText(QString::number(elapsedMs));
    lblPrimTestYaw->setText(QString::number(yawDegX10 / 10.0, 'f', 1));
    lblPrimTestYawRate->setText(QString::number(yawRateDpsX10 / 10.0, 'f', 1));
    lblPrimTestTargetDps->setText(QString::number(targetDpsX10 / 10.0, 'f', 1));
    lblPrimTestLeftPwm->setText(QString::number(leftPwm));
    lblPrimTestRightPwm->setText(QString::number(rightPwm));
}

void MainWindow::updatePrimitiveTestConfig(const QByteArray &payload)
{
    if (payload.size() < PRIM_TEST_CONFIG_SIZE) {
        return;
    }

    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);

    quint8 responseType, testType;
    quint16 kp, ki, kd, outputLimit, fasterPwm, slowerPwm, targetDps;

    stream >> responseType >> testType;
    stream >> kp >> ki >> kd >> outputLimit >> fasterPwm >> slowerPwm >> targetDps;

    if ((responseType != PRIM_TEST_RESP_CONFIG) ||
        (testType != PRIM_TEST_SMOOTH_TURN)) {
        return;
    }

    spinPrimSmoothKp->setValue(kp);
    spinPrimSmoothKi->setValue(ki);
    spinPrimSmoothKd->setValue(kd);
    spinPrimSmoothOutputLimit->setValue(outputLimit);
    spinPrimSmoothFasterPwm->setValue(fasterPwm);
    spinPrimSmoothSlowerPwm->setValue(slowerPwm);
    spinPrimSmoothTargetDps->setValue(targetDps);
}

void MainWindow::on_btnPrimitiveStart_clicked()
{
    sendPrimitiveSmoothConfig();

    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << PRIM_TEST_START;
    stream << PRIM_TEST_SMOOTH_TURN;
    stream << currentPrimitiveVariant();
    sendUnerbusCommand(Unerbus::CommandId::CMD_PRIMITIVE_TEST, payload);
}


void MainWindow::on_btnPrimitiveStop_clicked()
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << PRIM_TEST_STOP;
    sendUnerbusCommand(Unerbus::CommandId::CMD_PRIMITIVE_TEST, payload);
}


void MainWindow::on_btnPrimitiveApplyConfig_clicked()
{
    sendPrimitiveSmoothConfig();
}


void MainWindow::on_btnPrimitiveUpdateStatus_clicked()
{
    requestPrimitiveTestStatus();
}


void MainWindow::on_chkPrimitiveAutoUpdate_toggled(bool checked)
{
    if (checked) {
        primitiveTestUpdateTimer_->start(100);
        requestPrimitiveTestStatus();
    } else {
        primitiveTestUpdateTimer_->stop();
    }
}


void MainWindow::on_comboPrimitiveTest_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    ui->stackedPrimitiveConfig->setCurrentWidget(ui->pagePrimitiveConfigSmooth);
    ui->stackedPrimitiveTelemetry->setCurrentWidget(ui->pagePrimitiveTelemetrySmooth);
    requestPrimitiveTestConfig();
}
