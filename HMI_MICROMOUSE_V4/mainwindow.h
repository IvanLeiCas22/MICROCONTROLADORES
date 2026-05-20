#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "Comunicacion/unerbusparser.h"
#include <QButtonGroup> // Necesario para QButtonGroup
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QTimer>
#include <QtNetwork/QUdpSocket>
#include <QtSerialPort/QSerialPort>     // Incluir para QSerialPort
#include <QtSerialPort/QSerialPortInfo> // Incluir para QSerialPortInfo
#include <QMessageBox>
#include <QFileDialog>

// --- CONSTANTES DEL LABERINTO ---
#define MAZE_WIDTH 15
#define MAZE_HEIGHT 15
#define WALL_NORTH 0x01
#define WALL_SOUTH 0x02
#define WALL_EAST 0x04
#define WALL_WEST 0x08
#define CELL_VISITED 0x10
#define CELL_SPECIAL 0x20

enum Heading { HEADING_NORTH = 0, HEADING_EAST, HEADING_SOUTH, HEADING_WEST };

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void on_navigationButtonClicked(
      QAbstractButton
          *button); // Nuevo slot para manejar clics de botones de navegación

  // --- Slots para la comunicación serie ---
  void on_btnConnectSerie_clicked();
  void on_btnDisconnectSerie_clicked();
  void on_btnRefreshPorts_clicked();
  void onSerialPort_ReadyRead();
  void handleSerialError(QSerialPort::SerialPortError error);
  void on_btnConnectUDP_clicked();
  void on_btnDisconnectUDP_clicked();
  void onUDPReadyRead();

  // --- Slot para procesar paquetes del micromouse ---
  void onPacketReceived(quint8 command, const QByteArray &payload);

  // --- Slots para enviar comandos ---
  void on_btnSendCMD_clicked();

  // --- Slots para la página de sensores ---
  void on_btnRefreshSensorsValues_clicked();
  void on_chkBoxAutoRefreshSensorsValues_toggled(bool checked);
  void requestSensorData();

  // --- Slots para calibración y configuración ---
  void on_btnCalibrateMPU_clicked();

  // --- Slots para la página de control de motores ---
  void on_btnApplyPWM_clicked();
  void on_btnStopMotor_clicked();
  void on_btnGetPWM_clicked();
  void on_chkAutoGetPWM_toggled(bool checked);
  void on_chkRealTimeSetPWM_toggled(bool checked);
  void on_control_widget_valueChanged(); // Slot para seteo en tiempo real del
                                         // PWM de los motores
  void on_btnConfigurePeriod_clicked();
  void on_btnSendTurnAngle_clicked();

  // --- Slots para la página de configuración ---
  void on_btnGetBaseMotorsSpeeds_clicked();
  void on_btnSetBaseMotorsSpeeds_clicked();
  void on_btnGetPidNavConfig_clicked();
  void on_btnSetPidNavConfig_clicked();
  void on_btnGetPidTurnConfig_clicked();
  void on_btnSetPidTurnConfig_clicked();
  void on_btnGetMpuConfig_clicked();
  void on_btnSetMpuConfig_clicked();
  void on_btnGetRobotStatus_clicked();
  void on_btnSetRobotStatus_clicked();
  void on_btnGetCruiseParams_clicked();
  void on_btnSetCruiseParams_clicked();
  void on_btnGetPidBrakingConfig_clicked();
  void on_btnSetPidBrakingConfig_clicked();

  // --- Slots para la página del laberinto ---
  void on_btnSimTurnL_clicked();

  void on_btnSimFwd_clicked();

  void on_btnSimTurnR_clicked();

  void on_btnSimWallLeft_clicked();

  void on_btnSimWallFront_clicked();

  void on_btnSimWallRight_clicked();

  void on_btnSimReset_clicked();

  void on_btnRotMapL_clicked();

  void on_btnRotMapR_clicked();

  void on_btnToggleAutonomous_clicked();

  void on_btnSaveMaze_clicked();

  void on_btnLoadMaze_clicked();

  void on_btnSyncMaze_clicked();

  private:
  Ui::MainWindow *ui;
  QButtonGroup *navigationButtonGroup; // Nuevo miembro para agrupar los botones
                                       // de navegación
  QSerialPort *serialPort;             // Miembro para manejar el puerto serie
  UnerbusParser *m_parser;             // Nuevo miembro para manejar el parser

  QUdpSocket *udpSocket;
  QString remoteIp;
  quint16 remotePort;
  quint16 localPort;

  QTimer *sensorUpdateTimer; // Temporizador para actualizaciones automáticas
  QTimer *pwmUpdateTimer;    // Temporizador para actualizaciones de PWM

  quint16 m_pwmPeriod = 1000; // Almacena el período máximo de PWM para escalar
                              // los valores de la UI.

  // --- Constantes para configuración ---
  static const int SENSOR_UPDATE_INTERVAL_MS = 200;
  static const int PWM_UPDATE_INTERVAL_MS =
      200; // Intervalo para pedir PWM (5 Hz)
  static const int MAX_LOG_LINES = 200;

  // --- Variables LABERINTO ---
  // Puntero maestro del lienzo
  QGraphicsScene *mazeScene;
  // Matrices de Mapas Duales
  uint8_t real_maze_map[MAZE_WIDTH][MAZE_HEIGHT]; // El Universo (15x15)
  uint8_t sim_maze_map[MAZE_WIDTH][MAZE_HEIGHT];  // La memoria del Robot
  // Motor de Autonomía
  QTimer *autonomousTimer;
  // Coordenadas
  uint8_t current_x;
  uint8_t current_y;
  Heading current_heading;
  // Matriz de mapeo: Fila = Orientación Actual, Columna = Pared Frente(0),
  // Der(1), Izq(2)
  const uint8_t sim_wall_lut[4][3] = {
      {WALL_NORTH, WALL_EAST, WALL_WEST},  // Si el robot mira al NORTH
      {WALL_EAST, WALL_SOUTH, WALL_NORTH}, // Si el robot mira al EAST
      {WALL_SOUTH, WALL_WEST, WALL_EAST},  // Si el robot mira al SOUTH
      {WALL_WEST, WALL_NORTH, WALL_SOUTH}  // Si el robot mira al WEST
  };
  // Variables de Meta e Inicio
  int start_x, start_y;
  int goal_x, goal_y;
  bool is_returning = false;
  int special_cells_found = 0;
  bool is_mode_b = false; // False = MODO A (Explorar). True = MODO B (Speedrun)
  int tsp_targets[3][2];
  bool tsp_solved = false;
  int current_tsp_index = 0;
  // Memoria BFS Estática (Compatible con STM32 - Empaquetado)
  uint8_t best_path[225]; // Almacena (X << 4) | Y
  uint8_t path_length;

  // --- Funciones de ayuda ---
  void updateSerialPortList();
  void updateUIState(bool serialConnected, bool udpConnected);
  void populateCMDComboBox();
  void sendUnerbusCommand(
      Unerbus::CommandId cmd,
      const QByteArray &payload = QByteArray()); // Helper para enviar comandos
  void updateIrSensorsUI(const QByteArray &payload);
  void updateMpuSensorsUI(const QByteArray &payload);
  void updateConnectionStatus(
      const QString &text,
      const QString &colorName); // Helper para actualizar el estado de conexión
  void setupControlPage();       // Función de configuración del control
  void requestPwmData();         // Nueva función para pedir datos de PWM
  void updatePwmUI(
      const QByteArray &payload); // Nueva función para actualizar la UI de PWM
  void updatePwmControlRanges(quint16 new_period);

  // --- Funciones LABERINTO ---
  // Función que hará toda la magia de iluminar las paredes
  void drawMaze();
  void requestMazeColumn(quint8 col);
  void calculateFastestPath(int sx, int sy, int gx, int gy);
  int getFloodFillDistance(int sx, int sy, int gx, int gy);
  void autonomousTick();

  // --- Funciones dispatch comunicaciones ---
  void setupConfigPage();
  void updatePidNavUI(const QByteArray &payload);
  void updatePidTurnUI(const QByteArray &payload);
  void updateMotorBaseSpeedsUI(const QByteArray &payload);
  void updateTurnMaxSpeedUI(const QByteArray &payload);
  void updateTurnMinSpeedUI(const QByteArray &payload);
  void updateMpuConfigUI(const QByteArray &payload);
  void populateMpuConfigComboBoxes();
  void updateWallThresholdsUI(const QByteArray &payload);
  void updateWallTargetAdcUI(const QByteArray &payload);
  void updateMaxPwmCorrectionUI(const QByteArray &payload);
  void setupActivitiesTab();
  void populateRobotStatusComboBoxes();
  void updateRobotStatusUI(const QByteArray &payload);
  void updateCruiseParamsUI(const QByteArray &payload);
  void updatePidBrakingUI(const QByteArray &payload);
  void updateBrakingParamsUI(const QByteArray &payload);
  void updateBrakingMaxSpeedUI(const QByteArray &payload);
  void updateBrakingMinSpeedUI(const QByteArray &payload);
  void updateBrakingDeadZoneUI(const QByteArray &payload);
  void updateYawAngleUI(const QByteArray &payload);
  void updateSmoothTurnSpeedsUI(const QByteArray &payload);
  void updateTurnSpeedPID(const QByteArray &payload);
  void updateTurnTargetDps(const QByteArray &payload);
  void updateDelayTicksUI(const QByteArray &payload);

protected:
  // Filtro maestro de cualquier evento del ratón
  bool eventFilter(QObject *watched, QEvent *event) override;
};

#endif // MAINWINDOW_H
