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
void App_Nav_SetConfig(const AppNavConfig *config);
void App_Nav_GetConfig(AppNavConfig *config_out);
void App_Nav_Reset(void);
void App_Nav_StartFindCells(void);
void App_Nav_Stop(void);
void App_Nav_Tick(const AppNavInput *input, AppNavOutput *output);
bool App_Nav_RecommendAction(uint32_t random_value,
                              AppNavRecommendedAction *action_out);
bool App_Nav_StartStraightDriveYawHold(int32_t yaw_target_q16_deg);
bool App_Nav_ComputeStraightDrivePwm(const AppNavInput *input,
                                     AppNavOutput *output);
bool App_Nav_StartYawHoldAdvance(int32_t yaw_target_q16_deg);
bool App_Nav_ComputeYawHoldAdvancePwm(const AppNavInput *input,
                                      uint16_t right_base_pwm,
                                      uint16_t left_base_pwm,
                                      AppNavOutput *output);
bool App_Nav_StartWallFollowAdvance(void);
bool App_Nav_ComputeWallFollowPwm(const AppNavInput *input,
                                  uint16_t right_base_pwm,
                                  uint16_t left_base_pwm,
                                  AppNavOutput *output);
bool App_Nav_StartAdvanceAction(AppNavAdvanceActionMode mode);
bool App_Nav_StartAdvanceActionWithRearTapeProfile(AppNavAdvanceActionMode mode,
                                                   AppNavRearTapeProfile rear_tape_profile);
AppNavAdvanceActionState App_Nav_TickAdvanceAction(const AppNavInput *input,
                                                   AppNavOutput *output);
void App_Nav_StopAdvanceAction(void);
bool App_Nav_StartApproachFrontWallAction(void);
AppNavApproachFrontWallActionState App_Nav_TickApproachFrontWallAction(const AppNavInput *input,
                                                                       AppNavOutput *output);
void App_Nav_StopApproachFrontWallAction(void);
bool App_Nav_StartSmoothTurn(AppNavSmoothTurnDirection direction);
bool App_Nav_ComputeSmoothTurnPwm(const AppNavInput *input,
                                  AppNavOutput *output);
bool App_Nav_StartSmoothAction(AppNavSmoothActionType action);
bool App_Nav_StartSmoothActionWithRearTapeProfile(AppNavSmoothActionType action,
                                                  AppNavRearTapeProfile rear_tape_profile);
AppNavSmoothActionState App_Nav_TickSmoothAction(const AppNavInput *input,
                                                 AppNavOutput *output);
void App_Nav_StopSmoothAction(void);
bool App_Nav_StartPivotTurn(void);
bool App_Nav_ComputePivotTurnPwm(const AppNavInput *input,
                                 int16_t target_dps,
                                 AppNavOutput *output);
bool App_Nav_StartPivotAction(AppNavPivotActionType action);
AppNavPivotActionState App_Nav_TickPivotAction(const AppNavInput *input,
                                               AppNavOutput *output);
void App_Nav_StopPivotAction(void);
bool App_Nav_StartBraking(void);
bool App_Nav_ComputeBrakingPwm(const AppNavInput *input,
                               AppNavOutput *output);

#endif /* INC_APP_NAV_H_ */
