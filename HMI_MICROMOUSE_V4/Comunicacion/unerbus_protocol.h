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
        CMD_SET_BRAKING_PID_GAINS = 0x64,
        CMD_GET_BRAKING_PID_GAINS = 0x65,
        CMD_SET_BRAKING_PARAMS = 0x66,
        CMD_GET_BRAKING_PARAMS = 0x67,
        CMD_SET_BRAKING_MAX_SPEED = 0x68,
        CMD_GET_BRAKING_MAX_SPEED = 0x69,
        CMD_SET_BRAKING_MIN_SPEED = 0x6A, // Para configurar la velocidad mínima de frenado
        CMD_GET_BRAKING_MIN_SPEED = 0x6B, // Para leer la velocidad mínima de frenado
        CMD_SET_BRAKING_DEAD_ZONE = 0x6C, // Para configurar la zona muerta de frenado
        CMD_GET_BRAKING_DEAD_ZONE = 0x6D, // Para leer la zona muerta de frenado
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
        CMD_OTHERS
    };
    Q_ENUM_NS(CommandId)

} // namespace Unerbus

#endif // UNERBUS_PROTOCOL_H
