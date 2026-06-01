#ifndef INC_APP_NAV_H_
#define INC_APP_NAV_H_

#include "app_nav_config.h"
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
 */

/* -------------------------------------------------------------------------- */
/* Configuration and module lifecycle                                          */
/* -------------------------------------------------------------------------- */

void App_Nav_Init(const AppNavConfig *config);
void App_Nav_SetConfig(const AppNavConfig *config);
void App_Nav_GetConfig(AppNavConfig *config_out);
void App_Nav_Reset(void);

/* -------------------------------------------------------------------------- */
/* Perception                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Evaluate the portable perception layer from the current sensor input.
 *
 * The function applies the same hysteresis state used by the motion primitives
 * and returns the filtered wall/tape interpretation. It does not command motors
 * and does not update the logical maze.
 */
bool App_Nav_EvaluatePerception(const AppNavInput *input,
                                AppNavPerception *perception_out);

/* -------------------------------------------------------------------------- */
/* Local recommendation policy                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Deterministic wall-based fallback recommendation. The supervisor may use
 * this as a local policy, but app_nav itself does not update the logical maze.
 *
 * Priority: front -> right -> left -> back.
 */
bool App_Nav_RecommendAction(const AppNavPerception *perception,
                              AppNavRecommendedAction *action_out);

/* -------------------------------------------------------------------------- */
/* Portable primitive-test runner                                              */
/* -------------------------------------------------------------------------- */

/*
 * Lightweight portable runner for manual primitive tests.
 *
 * The runner is intentionally independent from HMI/UNERBUS/app_core. It
 * executes complete primitive actions, not low-level controllers, so manual
 * tests exercise the same action layer used by the supervisor.
 *
 * Current first stage supports only smooth-left/right tests. Additional
 * primitives can be added here without exposing low-level controller APIs.
 */
typedef enum
{
    APP_NAV_PRIMITIVE_TEST_NONE = 0,
    APP_NAV_PRIMITIVE_TEST_SMOOTH_LEFT,
    APP_NAV_PRIMITIVE_TEST_SMOOTH_RIGHT
} AppNavPrimitiveTestType;

typedef enum
{
    APP_NAV_PRIMITIVE_TEST_IDLE = 0,
    APP_NAV_PRIMITIVE_TEST_RUNNING,
    APP_NAV_PRIMITIVE_TEST_DONE,
    APP_NAV_PRIMITIVE_TEST_TIMEOUT,
    APP_NAV_PRIMITIVE_TEST_ERROR,
    APP_NAV_PRIMITIVE_TEST_REJECTED
} AppNavPrimitiveTestState;

bool App_NavPrimitiveTest_Start(AppNavPrimitiveTestType type);
AppNavPrimitiveTestState App_NavPrimitiveTest_Tick(const AppNavInput *input,
                                                   const AppNavPerception *perception,
                                                   AppNavOutput *output);
void App_NavPrimitiveTest_Stop(void);
AppNavPrimitiveTestState App_NavPrimitiveTest_GetState(void);

/* -------------------------------------------------------------------------- */
/* Complete primitive actions used by app_nav_supervisor                        */
/* -------------------------------------------------------------------------- */

/*
 * Advance until the rear floor sensor confirms the next cell boundary tape.
 *
 * Rear tape profiles distinguish normal cells from special cells, where the
 * rear sensor can see an internal black patch before the exit boundary tape.
 */
bool App_Nav_StartAdvanceActionWithRearTapeProfile(AppNavAdvanceActionMode mode,
                                                   AppNavRearTapeProfile rear_tape_profile);
AppNavAdvanceActionState App_Nav_TickAdvanceAction(const AppNavInput *input,
                                                   const AppNavPerception *perception,
                                                   AppNavOutput *output);
void App_Nav_StopAdvanceAction(void);

/*
 * Smooth turn action.
 *
 * The curved part may end by yaw target or diagonal/wall reference, but the
 * action remains active in POST_YAW_SEEK_REAR_TAPE until rear tape confirms
 * cell entry.
 */
bool App_Nav_StartSmoothActionWithRearTapeProfile(AppNavSmoothActionType action,
                                                  AppNavRearTapeProfile rear_tape_profile);
AppNavSmoothActionState App_Nav_TickSmoothAction(const AppNavInput *input,
                                                 const AppNavPerception *perception,
                                                 AppNavOutput *output);
void App_Nav_StopSmoothAction(void);

/*
 * In-cell pivot action. Pivots update orientation only; cell position is owned
 * by the supervisor according to the surrounding sequence.
 */
bool App_Nav_StartPivotAction(AppNavPivotActionType action);
AppNavPivotActionState App_Nav_TickPivotAction(const AppNavInput *input,
                                               const AppNavPerception *perception,
                                               AppNavOutput *output);
void App_Nav_StopPivotAction(void);

/*
 * Approach the front wall before an in-cell 180° pivot.
 *
 * This is used for dead-end turnarounds and for route backtracking when the
 * current front edge is a wall. This action does not confirm cell entry by
 * itself.
 */
bool App_Nav_StartApproachFrontWallAction(void);
AppNavApproachFrontWallActionState App_Nav_TickApproachFrontWallAction(const AppNavInput *input,
                                                                       const AppNavPerception *perception,
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
                                                                             const AppNavPerception *perception,
                                                                             AppNavOutput *output);
void App_Nav_StopCenterByFrontTapeForPivotAction(void);

#endif /* INC_APP_NAV_H_ */
