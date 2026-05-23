#ifndef INC_APP_NAV_H_
#define INC_APP_NAV_H_

#include "app_nav_config.h"
#include "app_nav_debug.h"
#include "app_nav_types.h"

/*
 * Portable navigation boundary.
 *
 * This file intentionally has no HAL dependencies. The STM32 application layer
 * will eventually feed AppNavInput and consume AppNavOutput. The Qt simulator
 * will use the same API through its FirmwareSimBridge.
 */

void App_Nav_Init(const AppNavConfig *config);
void App_Nav_Reset(void);
void App_Nav_StartFindCells(void);
void App_Nav_Stop(void);
void App_Nav_Tick(const AppNavInput *input, AppNavOutput *output);

#endif /* INC_APP_NAV_H_ */
