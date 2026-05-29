#ifndef INC_APP_NAV_H_
#define INC_APP_NAV_H_

#include "app_nav_config.h"
#include "app_nav_debug.h"
#include "app_nav_types.h"

/*
 * Portable navigation primitives and perception boundary.
 *
 * This module has no HAL dependencies. It receives AppNavInput and produces
 * AppNavOutput using fixed-point / integer-only logic so it can run both on the
 * STM32 firmware and in the Qt simulator firmware_core.
 *
 * Ownership split:
 *
 * - app_nav:
 *   Low-level perception, wall/yaw controllers, and motion primitives
 *   such as advance, smooth turn, pivot and approach-front-wall.
 *
 * - app_nav_supervisor:
 *   Mission-level sequencing. It decides which primitive to run, updates the
 *   logical maze, handles FIND_CELLS, and owns high-level navigation state.
 *
 * App_Nav_Tick() is kept as a perception/debug shell. It does not own the live
 * FIND_CELLS mission; the supervisor drives the active primitive actions.
 */

/* -------------------------------------------------------------------------- */
/* Configuration and module lifecycle                                          */
/* -------------------------------------------------------------------------- */

void App_Nav_Init(const AppNavConfig *config);
void App_Nav_SetConfig(const AppNavConfig *config);
void App_Nav_GetConfig(AppNavConfig *config_out);
void App_Nav_Reset(void);

/* -------------------------------------------------------------------------- */
/* Legacy/perception shell                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Perception/debug shell.
 *
 * Current real navigation is orchestrated by app_nav_supervisor. App_Nav_Tick()
 * updates perception/debug data from AppNavInput, but it does not run a mission
 * state machine or command motors.
 */
void App_Nav_Tick(const AppNavInput *input, AppNavOutput *output);

/* -------------------------------------------------------------------------- */
/* Local recommendation policy                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Basic wall-based action recommendation. The supervisor may use this as a
 * local policy, but app_nav itself does not update the logical maze.
 */
bool App_Nav_RecommendAction(uint32_t random_value,
                              AppNavRecommendedAction *action_out);

/* -------------------------------------------------------------------------- */
/* Reusable low-level drive controllers                                         */
/* -------------------------------------------------------------------------- */

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

bool App_Nav_StartSmoothTurn(AppNavSmoothTurnDirection direction);
bool App_Nav_ComputeSmoothTurnPwm(const AppNavInput *input,
                                  AppNavOutput *output);

bool App_Nav_StartPivotTurn(void);
bool App_Nav_ComputePivotTurnPwm(const AppNavInput *input,
                                 int16_t target_dps,
                                 AppNavOutput *output);

bool App_Nav_StartBraking(void);
bool App_Nav_ComputeBrakingPwm(const AppNavInput *input,
                               AppNavOutput *output);

/* -------------------------------------------------------------------------- */
/* Complete primitive actions used by app_nav_supervisor                        */
/* -------------------------------------------------------------------------- */

/*
 * Advance until the rear floor sensor confirms the next cell boundary tape.
 *
 * Rear tape profiles distinguish normal cells from special cells, where the
 * rear sensor can see an internal black patch before the exit boundary tape.
 */
bool App_Nav_StartAdvanceAction(AppNavAdvanceActionMode mode);
bool App_Nav_StartAdvanceActionWithRearTapeProfile(AppNavAdvanceActionMode mode,
                                                   AppNavRearTapeProfile rear_tape_profile);
AppNavAdvanceActionState App_Nav_TickAdvanceAction(const AppNavInput *input,
                                                   AppNavOutput *output);
void App_Nav_StopAdvanceAction(void);

/*
 * Smooth turn action.
 *
 * The curved part may end by yaw target or diagonal/wall reference, but the
 * action remains active in POST_YAW_SEEK_REAR_TAPE until rear tape confirms
 * cell entry.
 */
bool App_Nav_StartSmoothAction(AppNavSmoothActionType action);
bool App_Nav_StartSmoothActionWithRearTapeProfile(AppNavSmoothActionType action,
                                                  AppNavRearTapeProfile rear_tape_profile);
AppNavSmoothActionState App_Nav_TickSmoothAction(const AppNavInput *input,
                                                 AppNavOutput *output);
void App_Nav_StopSmoothAction(void);

/*
 * In-cell pivot action. Pivots update orientation only; cell position is owned
 * by the supervisor according to the surrounding sequence.
 */
bool App_Nav_StartPivotAction(AppNavPivotActionType action);
AppNavPivotActionState App_Nav_TickPivotAction(const AppNavInput *input,
                                               AppNavOutput *output);
void App_Nav_StopPivotAction(void);

/*
 * Approach the front wall before an in-cell 180° pivot in a dead-end.
 * This action does not confirm cell entry by itself.
 */
bool App_Nav_StartApproachFrontWallAction(void);
AppNavApproachFrontWallActionState App_Nav_TickApproachFrontWallAction(const AppNavInput *input,
                                                                       AppNavOutput *output);
void App_Nav_StopApproachFrontWallAction(void);

/*
 * Center the robot before an in-cell 180° pivot using the front floor sensor.
 *
 * This action advances until the front floor sensor detects a valid boundary
 * tape rising edge. It does not update logical pose by itself.
 */
bool App_Nav_StartCenterByFrontTapeForPivotAction(AppNavFrontTapeProfile front_tape_profile);

AppNavCenterFrontTapeActionState App_Nav_TickCenterByFrontTapeForPivotAction(const AppNavInput *input,
                                                                             AppNavOutput *output);
void App_Nav_StopCenterByFrontTapeForPivotAction(void);

#endif /* INC_APP_NAV_H_ */
