/* STM32/Test2024-master/Core/Inc/app_core.h */
#ifndef INC_APP_CORE_H_
#define INC_APP_CORE_H_

#include "main.h"

/*
 * STM32 application core / hardware adapter.
 *
 * app_core is the integration layer between CubeMX/HAL-generated code and the
 * project-specific portable modules.
 *
 * Responsibilities:
 * - initialize application-level drivers and runtime state;
 * - run the main cooperative loop;
 * - adapt STM32 sensors/timing into AppNavInput;
 * - consume AppNavOutput and drive motors;
 * - route UNERBUS commands from USB/WiFi;
 * - manage the physical button and local OLED display;
 * - start/stop primitive tests and supervisor runs.
 *
 * Navigation ownership:
 * - app_nav owns portable perception/controllers/primitives;
 * - app_nav_supervisor owns mission sequencing and logical maze updates;
 * - app_core only adapts hardware, configuration and communication.
 */

/* -------------------------------------------------------------------------- */
/* Main application entry points                                               */
/* -------------------------------------------------------------------------- */

void App_Core_Init(void);
void App_Core_Loop(void);

/* -------------------------------------------------------------------------- */
/* HAL callback wrappers                                                       */
/* -------------------------------------------------------------------------- */

/*
 * These wrappers are called from the actual HAL callbacks generated in main.c
 * or STM32 interrupt files. Keeping the application logic here keeps CubeMX
 * generated files smaller and easier to regenerate.
 */
void App_Core_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void App_Core_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void App_Core_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void App_Core_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void App_Core_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void App_Core_USB_ReceiveData(uint8_t *buf, uint16_t len);

#endif /* INC_APP_CORE_H_ */
