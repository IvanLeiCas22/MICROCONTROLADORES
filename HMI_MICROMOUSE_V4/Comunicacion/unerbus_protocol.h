#ifndef UNERBUS_PROTOCOL_H
#define UNERBUS_PROTOCOL_H

#include <QObject> // Para Q_NAMESPACE
#include <QtGlobal>

namespace Unerbus
{
    Q_NAMESPACE

    // Definición del protocolo (constantes)
    const QByteArray HEADER = "UNER";
    const char TOKEN = ':';

    // Enumeración de los comandos, extraída de app_config.h en STM32
    enum class CommandId : quint8
    {
        CMD_ACK = 0x0D,
        CMD_GET_ALIVE = 0xF0,
        CMD_START_CONFIG = 0xEE,
        CMD_FIRMWARE = 0xF1,
        CMD_GET_BUTTON_STATE = 0x12,
        CMD_GET_MPU_DATA = 0xA2,
        CMD_GET_IR_SENSOR_SNAPSHOT = 0xA0,
        CMD_TEST_MOTORS = 0xA1,
        CMD_GET_MOTOR_SPEEDS = 0xA4,
        CMD_SET_MOTOR_PWM = 0xA5,
        CMD_GET_MOTOR_PWM = 0xA6,
        CMD_GET_LOCAL_IP_ADDRESS = 0xE0,
        CMD_SET_UART_BYPASS_CONTROL = 0xDD,
        CMD_CALIBRATE_MPU = 0xA3,
        CMD_SET_PID_GAINS = 0x40,
        CMD_GET_PID_GAINS = 0x41,
        CMD_SET_MAX_PWM_CORRECTION = 0x42,
        CMD_GET_MAX_PWM_CORRECTION = 0x43,
        CMD_SET_MOTOR_BASE_SPEEDS = 0x44,
        CMD_GET_MOTOR_BASE_SPEEDS = 0x45,
        CMD_CALIBRATE_MOTORS = 0x46,
        CMD_TURN_DEGREES = 0x47,
        CMD_SET_TURN_PID_GAINS = 0x48,
        CMD_GET_TURN_PID_GAINS = 0x49,
        CMD_SET_TURN_MAX_SPEED = 0x4A,
        CMD_GET_TURN_MAX_SPEED = 0x4B,
        CMD_SET_PWM_PERIOD = 0x50,
        CMD_GET_PWM_PERIOD = 0x51,
        CMD_SET_MPU_CONFIG = 0xA7,
        CMD_GET_MPU_CONFIG = 0xA8,
        CMD_SET_TURN_MIN_SPEED = 0x4C,
        CMD_GET_TURN_MIN_SPEED = 0x4D,
        CMD_SET_WALL_THRESHOLDS = 0x60,
        CMD_GET_WALL_THRESHOLDS = 0x61,
        CMD_SET_WALL_TARGET_ADC = 0x62,
        CMD_GET_WALL_TARGET_ADC = 0x63,
        CMD_SET_APP_STATE = 0x70,     // Para cambiar entre MENU y RUNNING
        CMD_GET_APP_STATE = 0x71,     // Para leer el estado de la app
        CMD_SET_MENU_MODE = 0x72,     // Para seleccionar un modo de operación
        CMD_GET_MENU_MODE = 0x73,     // Para leer el modo de operación actual
        CMD_GET_ROBOT_STATUS = 0x74,  // Para leer el estado completo (AppState y MenuMode)
        CMD_SET_CRUISE_PARAMS = 0x4E, // Configurar velocidad crucero y umbral de aceleración
        CMD_GET_CRUISE_PARAMS = 0x4F, // Leer velocidad crucero y umbral de aceleración
        CMD_GET_YAW_ANGLE = 0x75,         // Para leer el ángulo de guiñada actual
        CMD_GET_SMOOTH_TURN_CONFIG = 0x80, // Para leer la configuración de giro suave
        CMD_SET_SMOOTH_TURN_CONFIG = 0x81, // Para configurar el giro suave
        CMD_SET_TURN_VELOCITY_PID_GAINS = 0x82, // Configurar Kp, Ki, Kd del PID de velocidad de giro
        CMD_GET_TURN_VELOCITY_PID_GAINS = 0x83, // Leer Kp, Ki, Kd del PID de velocidad de giro
        CMD_SET_TURN_TARGET_DPS = 0x84,         // Configurar la velocidad angular objetivo para giros
        CMD_GET_TURN_TARGET_DPS = 0x85,         // Leer la velocidad angular objetivo
        CMD_GET_DELAY_TICKS = 0X90,             // Leer el número de ticks de retardo
        CMD_SET_DELAY_TICKS = 0X91,             // Configurar el número de ticks de retardo
        CMD_UPDATE_MAZE_CELL = 0x92,            // (STM32 -> Qt) Enviar actualización de info de celda
        CMD_SYNC_MAZE_COLUMN = 0x93,            // Sincronizar 1 columna entera del laberinto
        CMD_PRIMITIVE_TEST = 0x95,              // Banco de pruebas de primitivas
        CMD_SET_APPROACH_FRONT_WALL_TARGET = 0x96,
        CMD_GET_APPROACH_FRONT_WALL_TARGET = 0x97,
        CMD_SET_SUPERVISOR_INITIAL_POSE = 0x98,
        CMD_GET_SUPERVISOR_INITIAL_POSE = 0x99,
        CMD_START_SUPERVISOR_RUN = 0x9A,
        CMD_STOP_SUPERVISOR_RUN = 0x9B,
        CMD_GET_SUPERVISOR_DEBUG_STATUS = 0x9C,
        CMD_SET_SUPERVISOR_GOAL_CELL = 0x9D,
        CMD_GET_SUPERVISOR_GOAL_CELL = 0x9E,
        CMD_SUPERVISOR_STATUS_UPDATE = 0x9F,
        CMD_OTHERS
    };
    Q_ENUM_NS(CommandId)

} // namespace Unerbus

#endif // UNERBUS_PROTOCOL_H
