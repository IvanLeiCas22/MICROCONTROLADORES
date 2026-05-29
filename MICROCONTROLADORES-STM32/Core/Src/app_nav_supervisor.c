#include "app_nav_supervisor.h"

#include "app_find_cells_policy.h"
#include "app_go_to_b_policy.h"
#include "app_maze.h"
#include "app_nav.h"

#include <stddef.h>

/*
 * Portable mission supervisor.
 *
 * This module owns high-level navigation sequencing for FIND_CELLS and
 * GO_A_TO_B:
 * - maps the current logical cell from relative wall perception;
 * - asks the active mission policy for the next decision;
 * - selects and sequences the required primitive action;
 * - updates the logical maze pose/cell after confirmed movement;
 * - detects and counts unique CELL_SPECIAL cells during FIND_CELLS;
 * - finishes missions with their public APP_NAV_SUPERVISOR_RESULT_* code.
 *
 * It does not implement low-level motor control. Motion primitives live in
 * app_nav.c and are driven through App_Nav_*Action APIs.
 *
 * It does not access HAL directly. Hardware/simulator adapters must provide
 * AppNavInput and consume AppNavOutput.
 */

#define APP_NAV_SUPERVISOR_SPECIAL_TARGET_COUNT 3U

#define APP_NAV_SUPERVISOR_YAW_180_Q16 ((int64_t)180 << 16)
#define APP_NAV_SUPERVISOR_YAW_360_Q16 ((int64_t)360 << 16)

static AppNavSupervisorDebug app_nav_supervisor_debug;
static int32_t app_nav_supervisor_action_yaw_reference_q16_deg;
static uint8_t app_nav_supervisor_action_yaw_reference_valid;
static uint8_t app_nav_supervisor_pivot_180_exit_requires_advance;
static uint8_t app_nav_supervisor_special_found_count;
static uint8_t app_nav_supervisor_initial_x;
static uint8_t app_nav_supervisor_initial_y;
static HeadingTypeDef app_nav_supervisor_initial_heading;
static uint8_t app_nav_supervisor_initial_pose_valid;
static uint8_t app_nav_supervisor_goal_x;
static uint8_t app_nav_supervisor_goal_y;
static uint8_t app_nav_supervisor_goal_valid;
static AppNavSupervisorMission app_nav_supervisor_mission =
    APP_NAV_SUPERVISOR_MISSION_FIND_CELLS;

/* -------------------------------------------------------------------------- */
/* State, output and shared utility helpers                                    */
/* -------------------------------------------------------------------------- */

static void App_NavSupervisor_ClearOutput(AppNavOutput *output)
{
    if (output == NULL)
    {
        return;
    }

    output->right_motor_pwm = 0;
    output->left_motor_pwm = 0;
    output->maze_update_valid = false;
    output->maze_x = 0U;
    output->maze_y = 0U;
    output->maze_cell_data = 0U;
    output->maze_heading = 0U;
}

static void App_NavSupervisor_StopActions(void)
{
    App_Nav_StopAdvanceAction();
    App_Nav_StopApproachFrontWallAction();
    App_Nav_StopCenterByFrontTapeForPivotAction();
    App_Nav_StopSmoothAction();
    App_Nav_StopPivotAction();
}

static void App_NavSupervisor_SetDefaultInitialPose(void)
{
    app_nav_supervisor_initial_x = APP_MAZE_DEFAULT_START_X;
    app_nav_supervisor_initial_y = APP_MAZE_DEFAULT_START_Y;
    app_nav_supervisor_initial_heading = APP_MAZE_DEFAULT_START_HEADING;
    app_nav_supervisor_initial_pose_valid = 1U;
}

static bool App_NavSupervisor_IsValidCell(uint8_t x, uint8_t y)
{
    return ((x < MAZE_WIDTH) && (y < MAZE_HEIGHT));
}

static void App_NavSupervisor_ClearActionYawReference(void)
{
    app_nav_supervisor_action_yaw_reference_q16_deg = 0;
    app_nav_supervisor_action_yaw_reference_valid = 0U;
}

static void App_NavSupervisor_ClearPivotExitLatch(void)
{
    app_nav_supervisor_pivot_180_exit_requires_advance = 0U;
}

static int32_t App_NavSupervisor_NormalizeYawDeltaQ16(int64_t delta_q16_deg)
{
    while (delta_q16_deg > APP_NAV_SUPERVISOR_YAW_180_Q16)
    {
        delta_q16_deg -= APP_NAV_SUPERVISOR_YAW_360_Q16;
    }

    while (delta_q16_deg < -APP_NAV_SUPERVISOR_YAW_180_Q16)
    {
        delta_q16_deg += APP_NAV_SUPERVISOR_YAW_360_Q16;
    }

    return (int32_t)delta_q16_deg;
}

static bool App_NavSupervisor_CaptureActionYawReference(const AppNavInput *input)
{
    if (input == NULL)
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    app_nav_supervisor_action_yaw_reference_q16_deg = input->yaw_q16_deg;
    app_nav_supervisor_action_yaw_reference_valid = 1U;
    return true;
}

static bool App_NavSupervisor_BuildActionInput(const AppNavInput *input,
                                               AppNavInput *action_input)
{
    if ((input == NULL) || (action_input == NULL) ||
        (app_nav_supervisor_action_yaw_reference_valid == 0U))
    {
        return false;
    }

    *action_input = *input;
    action_input->yaw_q16_deg = App_NavSupervisor_NormalizeYawDeltaQ16(
        (int64_t)input->yaw_q16_deg -
        (int64_t)app_nav_supervisor_action_yaw_reference_q16_deg);

    return true;
}

static void App_NavSupervisor_UpdateMazeDebug(void)
{
    uint8_t payload[APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE] = {0U};

    if (App_Maze_WriteCurrentCellUpdatePayload(payload) != APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE)
    {
        app_nav_supervisor_debug.maze_x = 0U;
        app_nav_supervisor_debug.maze_y = 0U;
        app_nav_supervisor_debug.maze_cell = 0U;
        app_nav_supervisor_debug.maze_heading = 0U;
        app_nav_supervisor_debug.special_found_count = app_nav_supervisor_special_found_count;
        return;
    }

    app_nav_supervisor_debug.maze_x = payload[0];
    app_nav_supervisor_debug.maze_y = payload[1];
    app_nav_supervisor_debug.maze_cell = payload[2];
    app_nav_supervisor_debug.maze_heading = payload[3];
    app_nav_supervisor_debug.special_found_count = app_nav_supervisor_special_found_count;
}

static void App_NavSupervisor_MapCurrentCellFromInput(const AppNavInput *input)
{
    AppNavOutput dummy_output = {0};
    AppNavDebug nav_debug = {0};

    App_Nav_Tick(input, &dummy_output);
    App_Nav_GetDebug(&nav_debug);

    App_Maze_MapCurrentCell((nav_debug.wall_front != 0U),
                            (nav_debug.wall_right != 0U),
                            (nav_debug.wall_left != 0U));
    App_NavSupervisor_UpdateMazeDebug();
}

static void App_NavSupervisor_SetState(AppNavSupervisorState state,
                                       AppNavSupervisorAction action,
                                       uint8_t result)
{
    app_nav_supervisor_debug.state = state;
    app_nav_supervisor_debug.current_action = action;
    app_nav_supervisor_debug.last_result = result;
}

static AppNavSupervisorState App_NavSupervisor_SetError(uint8_t result)
{
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();
    App_NavSupervisor_ClearPivotExitLatch();
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_ERROR,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               result);
    return app_nav_supervisor_debug.state;
}

/* -------------------------------------------------------------------------- */
/* Mission completion and special-cell detection                               */
/* -------------------------------------------------------------------------- */

static AppNavSupervisorState App_NavSupervisor_FinishMissionWithResult(AppNavOutput *output,
                                                                       uint8_t result)
{
    App_NavSupervisor_ClearOutput(output);
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();
    App_NavSupervisor_ClearPivotExitLatch();

    app_nav_supervisor_debug.active = 0U;

    App_NavSupervisor_UpdateMazeDebug();
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               result);

    return app_nav_supervisor_debug.state;
}

static AppNavSupervisorState App_NavSupervisor_FinishFindCells(AppNavOutput *output)
{
    return App_NavSupervisor_FinishMissionWithResult(
        output,
        APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE);
}

static bool App_NavSupervisor_CheckSpecialAtConfirmedCellEntry(const AppNavInput *input)
{
    if (input == NULL)
    {
        return false;
    }

    if ((input->floor_front_black == 0U) ||
        (input->floor_rear_black == 0U))
    {
        return false;
    }

    if (App_Maze_MarkCurrentCellSpecial())
    {
        App_NavSupervisor_UpdateMazeDebug();

        if (app_nav_supervisor_mission == APP_NAV_SUPERVISOR_MISSION_FIND_CELLS)
        {
            if (app_nav_supervisor_special_found_count < 255U)
            {
                app_nav_supervisor_special_found_count++;
            }
        }
    }

    if (app_nav_supervisor_mission != APP_NAV_SUPERVISOR_MISSION_FIND_CELLS)
    {
        return false;
    }

    return (app_nav_supervisor_special_found_count >= APP_NAV_SUPERVISOR_SPECIAL_TARGET_COUNT);
}

/* -------------------------------------------------------------------------- */
/* Primitive start helpers                                                     */
/* -------------------------------------------------------------------------- */

static TurnTypeDef App_NavSupervisor_SmoothTurnForState(AppNavSupervisorState state)
{
    return (state == APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT) ? TURN_LEFT : TURN_RIGHT;
}

static bool App_NavSupervisor_StartAdvanceWithState(const AppNavInput *input,
                                                    AppNavSupervisorState state,
                                                    AppNavSupervisorAction action)
{
    AppNavRearTapeProfile rear_tape_profile = APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL;

    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if ((action != APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE) &&
        App_Maze_IsCurrentCellSpecial())
    {
        rear_tape_profile =
            (app_nav_supervisor_pivot_180_exit_requires_advance != 0U)
                ? APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
                : APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL;
    }

    if (!App_Nav_StartAdvanceActionWithRearTapeProfile(APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD,
                                                       rear_tape_profile))
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(state,
                               action,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    return true;
}

static bool App_NavSupervisor_StartInitialAdvance(const AppNavInput *input)
{
    return App_NavSupervisor_StartAdvanceWithState(input,
                                                  APP_NAV_SUPERVISOR_RUN_INITIAL_ADVANCE,
                                                  APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE);
}

static bool App_NavSupervisor_StartAdvance(const AppNavInput *input)
{
    return App_NavSupervisor_StartAdvanceWithState(input,
                                                  APP_NAV_SUPERVISOR_RUN_ADVANCE,
                                                  APP_NAV_SUPERVISOR_ACTION_ADVANCE);
}

static bool App_NavSupervisor_StartApproachFrontWallForPivot(const AppNavInput *input)
{
    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if (!App_Nav_StartApproachFrontWallAction())
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT,
                               APP_NAV_SUPERVISOR_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    return true;
}

static bool App_NavSupervisor_CurrentCellHasFrontWall(void)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    HeadingTypeDef heading = HEADING_NORTH;

    if (!App_Maze_GetRobotPose(&x, &y, &heading))
    {
        return false;
    }

    return App_Maze_CellHasWall(x, y, heading);
}

static bool App_NavSupervisor_StartCenterFrontTapeForPivot(const AppNavInput *input)
{
    AppNavFrontTapeProfile front_tape_profile = APP_NAV_FRONT_TAPE_PROFILE_NORMAL_CELL;

    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if (App_Maze_IsCurrentCellSpecial())
    {
        front_tape_profile = APP_NAV_FRONT_TAPE_PROFILE_SPECIAL_CELL;
    }

    /*
     * This action prepares a 180° route-backtracking pivot when the front edge is
     * open. The actual pivot exit advance is armed only after the front-tape
     * preparation action completes.
     */
    App_NavSupervisor_ClearPivotExitLatch();

    if (!App_Nav_StartCenterByFrontTapeForPivotAction(front_tape_profile))
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT,
                               APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    return true;
}

static bool App_NavSupervisor_StartRouteBacktrackingPivotPrep(const AppNavInput *input)
{
    /*
     * BACKTRACK_REQUIRED means that the next route step is behind the robot.
     * The physical preparation depends on the current front edge:
     *
     * - front wall present: use the existing front-wall approach sequence;
     * - front open: use the front floor sensor boundary-tape sequence.
     */
    if (App_NavSupervisor_CurrentCellHasFrontWall())
    {
        return App_NavSupervisor_StartApproachFrontWallForPivot(input);
    }

    return App_NavSupervisor_StartCenterFrontTapeForPivot(input);
}

static bool App_NavSupervisor_StartPivot180(const AppNavInput *input)
{
    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if (!App_Nav_StartPivotAction(APP_NAV_PIVOT_180_RIGHT))
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_PIVOT_180,
                               APP_NAV_SUPERVISOR_ACTION_PIVOT_180,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    return true;
}

static bool App_NavSupervisor_StartSmoothWithState(const AppNavInput *input,
                                                   AppNavSmoothActionType smooth_action,
                                                   AppNavSupervisorState state,
                                                   AppNavSupervisorAction action)
{
    AppNavRearTapeProfile rear_tape_profile = APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL;

    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if (App_Maze_IsCurrentCellSpecial())
    {
        rear_tape_profile = APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL;
    }

    if (!App_Nav_StartSmoothActionWithRearTapeProfile(smooth_action,
                                                      rear_tape_profile))
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(state,
                               action,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    return true;
}

static bool App_NavSupervisor_StartRecommendedAction(AppNavRecommendedAction action,
                                                     const AppNavInput *input)
{
    switch (action)
    {
    case APP_NAV_ACTION_GO_FRONT_NAVIGATING:
    case APP_NAV_ACTION_GO_FRONT_STRAIGHT:
        return App_NavSupervisor_StartAdvance(input);

    case APP_NAV_ACTION_SMOOTH_LEFT:
        return App_NavSupervisor_StartSmoothWithState(input,
                                                      APP_NAV_SMOOTH_ACTION_LEFT,
                                                      APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT,
                                                      APP_NAV_SUPERVISOR_ACTION_SMOOTH_LEFT);

    case APP_NAV_ACTION_SMOOTH_RIGHT:
        return App_NavSupervisor_StartSmoothWithState(input,
                                                      APP_NAV_SMOOTH_ACTION_RIGHT,
                                                      APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT,
                                                      APP_NAV_SUPERVISOR_ACTION_SMOOTH_RIGHT);

    case APP_NAV_ACTION_GO_BACK:
        return App_NavSupervisor_StartApproachFrontWallForPivot(input);

    case APP_NAV_ACTION_NONE:
    default:
        return false;
    }
}

/* -------------------------------------------------------------------------- */
/* Supervisor state handlers                                                   */
/* -------------------------------------------------------------------------- */

static AppNavSupervisorState App_NavSupervisor_HandleFindCellsDecide(const AppNavInput *input,
                                                                     AppNavOutput *output)
{
    AppNavRecommendedAction recommended_action = APP_NAV_ACTION_NONE;
    AppFindCellsDecision find_cells_decision = {0};

    /*
     * Production FIND_CELLS policy:
     * - execute concrete actions returned by app_find_cells_policy;
     * - if exploration has no remaining frontier, finish the mission as
     *   incomplete instead of falling back to the old local policy;
     * - if the route to a frontier requires an initial backward step, start a
     *   180° route-backtracking preparation. The preparation method is selected
     *   from the current front edge: front-wall approach if a front wall is known,
     *   front-tape centering if the front edge is open.
     */
    if (App_FindCellsPolicy_Evaluate(&find_cells_decision) &&
        (find_cells_decision.action != APP_NAV_ACTION_NONE))
    {
        recommended_action = find_cells_decision.action;
    }
    else if (find_cells_decision.reason == APP_FIND_CELLS_DECISION_REASON_NO_FRONTIER)
    {
        return App_NavSupervisor_FinishMissionWithResult(
            output,
            APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_INCOMPLETE_NO_FRONTIER);
    }
    else if (find_cells_decision.reason == APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED)
    {
        if (!App_NavSupervisor_StartRouteBacktrackingPivotPrep(input))
        {
            return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
        }

        return app_nav_supervisor_debug.state;
    }
    else
    {
        if (!App_Nav_RecommendAction(0U, &recommended_action))
        {
            return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
        }
    }

    if (!App_NavSupervisor_StartRecommendedAction(recommended_action, input))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
    }

    return app_nav_supervisor_debug.state;
}

static AppNavSupervisorState App_NavSupervisor_HandleGoToBDecide(const AppNavInput *input,
                                                                 AppNavOutput *output)
{
    AppGoToBDecision go_to_b_decision = {0};

    if (!App_GoToBPolicy_Evaluate(app_nav_supervisor_goal_x,
                                  app_nav_supervisor_goal_y,
                                  &go_to_b_decision))
    {
        if (go_to_b_decision.reason == APP_GO_TO_B_DECISION_REASON_NO_PATH)
        {
            return App_NavSupervisor_FinishMissionWithResult(
                output,
                APP_NAV_SUPERVISOR_RESULT_GO_TO_B_NO_PATH);
        }

        if (go_to_b_decision.reason == APP_GO_TO_B_DECISION_REASON_BACKTRACK_REQUIRED)
        {
            if (!App_NavSupervisor_StartRouteBacktrackingPivotPrep(input))
            {
                return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
            }

            return app_nav_supervisor_debug.state;
        }

        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
    }

    if (go_to_b_decision.reason == APP_GO_TO_B_DECISION_REASON_GOAL_REACHED)
    {
        return App_NavSupervisor_FinishMissionWithResult(
            output,
            APP_NAV_SUPERVISOR_RESULT_GO_TO_B_COMPLETE);
    }

    if ((go_to_b_decision.reason != APP_GO_TO_B_DECISION_REASON_ROUTE_STEP) ||
        (go_to_b_decision.action == APP_NAV_ACTION_NONE))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
    }

    if (!App_NavSupervisor_StartRecommendedAction(go_to_b_decision.action, input))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
    }

    return app_nav_supervisor_debug.state;
}

static AppNavSupervisorState App_NavSupervisor_HandleDecide(const AppNavInput *input,
                                                            AppNavOutput *output)
{
    App_NavSupervisor_MapCurrentCellFromInput(input);

    switch (app_nav_supervisor_mission)
    {
    case APP_NAV_SUPERVISOR_MISSION_FIND_CELLS:
        return App_NavSupervisor_HandleFindCellsDecide(input, output);

    case APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B:
        return App_NavSupervisor_HandleGoToBDecide(input, output);

    default:
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
    }
}

static AppNavSupervisorState App_NavSupervisor_HandleStartInitialAdvance(const AppNavInput *input,
                                                                         AppNavOutput *output)
{
    App_NavSupervisor_ClearOutput(output);
    App_NavSupervisor_MapCurrentCellFromInput(input);

    if (!App_NavSupervisor_StartInitialAdvance(input))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
    }

    return app_nav_supervisor_debug.state;
}

static AppNavSupervisorState App_NavSupervisor_HandleAdvanceWithState(const AppNavInput *input,
                                                                      AppNavOutput *output,
                                                                      AppNavSupervisorState running_state,
                                                                      AppNavSupervisorAction running_action)
{
    AppNavInput action_input = {0};
    AppNavAdvanceActionState advance_state;

    if (!App_NavSupervisor_BuildActionInput(input, &action_input))
    {
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }

    advance_state = App_Nav_TickAdvanceAction(&action_input, output);

    switch (advance_state)
    {
    case APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE:
    case APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW:
    case APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD:
        App_NavSupervisor_SetState(running_state,
                                   running_action,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE:
        App_NavSupervisor_ClearOutput(output);
        App_Maze_AdvanceRobotPosition();

        if (App_NavSupervisor_CheckSpecialAtConfirmedCellEntry(input))
        {
            return App_NavSupervisor_FinishFindCells(output);
        }

        App_Nav_StopAdvanceAction();
        App_NavSupervisor_ClearActionYawReference();
        App_NavSupervisor_UpdateMazeDebug();
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_ADVANCE_ACTION_TIMEOUT:
    case APP_NAV_ADVANCE_ACTION_ERROR:
    case APP_NAV_ADVANCE_ACTION_IDLE:
    default:
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }
}

static AppNavSupervisorState App_NavSupervisor_HandleAdvance(const AppNavInput *input,
                                                             AppNavOutput *output)
{
    return App_NavSupervisor_HandleAdvanceWithState(input,
                                                    output,
                                                    APP_NAV_SUPERVISOR_RUN_ADVANCE,
                                                    APP_NAV_SUPERVISOR_ACTION_ADVANCE);
}

static AppNavSupervisorState App_NavSupervisor_HandleInitialAdvance(const AppNavInput *input,
                                                                    AppNavOutput *output)
{
    return App_NavSupervisor_HandleAdvanceWithState(input,
                                                    output,
                                                    APP_NAV_SUPERVISOR_RUN_INITIAL_ADVANCE,
                                                    APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE);
}

static AppNavSupervisorState App_NavSupervisor_HandleApproachFrontWallForPivot(const AppNavInput *input,
                                                                               AppNavOutput *output)
{
    AppNavInput action_input = {0};
    AppNavApproachFrontWallActionState approach_state;

    if (!App_NavSupervisor_BuildActionInput(input, &action_input))
    {
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }

    approach_state = App_Nav_TickApproachFrontWallAction(&action_input, output);

    switch (approach_state)
    {
    case APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_WALL_FOLLOW:
    case APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_YAW_HOLD:
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT,
                                   APP_NAV_SUPERVISOR_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_APPROACH_FRONT_WALL_ACTION_DONE_FRONT_WALL:
        App_NavSupervisor_ClearOutput(output);
        App_Nav_StopApproachFrontWallAction();
        app_nav_supervisor_pivot_180_exit_requires_advance = 1U;

        if (!App_NavSupervisor_StartPivot180(input))
        {
            return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
        }

        return app_nav_supervisor_debug.state;

    case APP_NAV_APPROACH_FRONT_WALL_ACTION_TIMEOUT:
    case APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR:
    case APP_NAV_APPROACH_FRONT_WALL_ACTION_IDLE:
    default:
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }
}

static AppNavSupervisorState App_NavSupervisor_HandleCenterFrontTapeForPivot(const AppNavInput *input,
                                                                             AppNavOutput *output)
{
    AppNavInput action_input = {0};
    AppNavCenterFrontTapeActionState center_state;

    if (!App_NavSupervisor_BuildActionInput(input, &action_input))
    {
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }

    center_state = App_Nav_TickCenterByFrontTapeForPivotAction(&action_input, output);

    switch (center_state)
    {
    case APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_WALL_FOLLOW:
    case APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_YAW_HOLD:
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT,
                                   APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_CENTER_FRONT_TAPE_ACTION_DONE_FRONT_TAPE:
        App_NavSupervisor_ClearOutput(output);
        App_Nav_StopCenterByFrontTapeForPivotAction();
        app_nav_supervisor_pivot_180_exit_requires_advance = 1U;

        if (!App_NavSupervisor_StartPivot180(input))
        {
            return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
        }

        return app_nav_supervisor_debug.state;

    case APP_NAV_CENTER_FRONT_TAPE_ACTION_TIMEOUT:
    case APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR:
    case APP_NAV_CENTER_FRONT_TAPE_ACTION_IDLE:
    default:
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }
}

static AppNavSupervisorState App_NavSupervisor_HandleSmooth(const AppNavInput *input,
                                                            AppNavOutput *output)
{
    AppNavSupervisorState current_state = app_nav_supervisor_debug.state;
    AppNavInput action_input = {0};
    TurnTypeDef turn = App_NavSupervisor_SmoothTurnForState(current_state);
    AppNavSmoothActionState smooth_state;

    if (!App_NavSupervisor_BuildActionInput(input, &action_input))
    {
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }

    smooth_state = App_Nav_TickSmoothAction(&action_input, output);

    switch (smooth_state)
    {
    case APP_NAV_SMOOTH_ACTION_TURNING:
    case APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE:
        App_NavSupervisor_SetState(current_state,
                                   app_nav_supervisor_debug.current_action,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE:
    case APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE:
        App_NavSupervisor_ClearOutput(output);
        App_Maze_UpdateRobotHeading(turn);
        App_Maze_AdvanceRobotPosition();

        if (App_NavSupervisor_CheckSpecialAtConfirmedCellEntry(input))
        {
            return App_NavSupervisor_FinishFindCells(output);
        }

        App_Nav_StopSmoothAction();
        App_NavSupervisor_ClearActionYawReference();
        App_NavSupervisor_UpdateMazeDebug();
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_SMOOTH_ACTION_DONE_WALL:
        /*
         * Defensive legacy path.
         *
         * Smooth diagonal/wall detection no longer completes the action here.
         * It enters APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE inside app_nav.c
         * and waits for rear tape confirmation before the supervisor advances the
         * logical map cell.
         *
         * If DONE_WALL reaches the supervisor, treating it as a primitive error is
         * safer than updating heading and starting a separate ADVANCE, which can
         * desynchronize physical and logical position.
         */
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);

    case APP_NAV_SMOOTH_ACTION_FRONT_WALL_SAFETY:
    case APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT:
    case APP_NAV_SMOOTH_ACTION_ERROR:
    case APP_NAV_SMOOTH_ACTION_IDLE:
    default:
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }
}

static AppNavSupervisorState App_NavSupervisor_HandlePivot(const AppNavInput *input,
                                                           AppNavOutput *output)
{
    AppNavInput action_input = {0};
    AppNavPivotActionState pivot_state;

    if (!App_NavSupervisor_BuildActionInput(input, &action_input))
    {
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }

    pivot_state = App_Nav_TickPivotAction(&action_input, output);

    switch (pivot_state)
    {
    case APP_NAV_PIVOT_ACTION_RUNNING:
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_PIVOT_180,
                                   APP_NAV_SUPERVISOR_ACTION_PIVOT_180,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_PIVOT_ACTION_DONE:
        App_NavSupervisor_ClearOutput(output);
        App_Maze_UpdateRobotHeading(TURN_AROUND);
        App_Nav_StopPivotAction();
        App_NavSupervisor_ClearActionYawReference();
        App_NavSupervisor_UpdateMazeDebug();

        if (app_nav_supervisor_pivot_180_exit_requires_advance != 0U)
        {
            if (!App_NavSupervisor_StartAdvance(input))
            {
                return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
            }

            App_NavSupervisor_ClearPivotExitLatch();
            return app_nav_supervisor_debug.state;
        }

        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_PIVOT_ACTION_TIMEOUT:
    case APP_NAV_PIVOT_ACTION_ERROR:
    case APP_NAV_PIVOT_ACTION_IDLE:
    default:
        App_NavSupervisor_ClearOutput(output);
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR);
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void App_NavSupervisor_Init(void)
{
    app_nav_supervisor_mission = APP_NAV_SUPERVISOR_MISSION_FIND_CELLS;
    app_nav_supervisor_goal_x = 0U;
    app_nav_supervisor_goal_y = 0U;
    app_nav_supervisor_goal_valid = 0U;
    App_NavSupervisor_SetDefaultInitialPose();
    App_NavSupervisor_Reset();
}

void App_NavSupervisor_Reset(void)
{
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();
    App_NavSupervisor_ClearPivotExitLatch();
    app_nav_supervisor_special_found_count = 0U;
    app_nav_supervisor_debug.special_found_count = 0U;

    if (app_nav_supervisor_initial_pose_valid != 0U)
    {
        if (!App_Maze_ResetStateWithPose(app_nav_supervisor_initial_x,
                                         app_nav_supervisor_initial_y,
                                         app_nav_supervisor_initial_heading))
        {
            App_NavSupervisor_SetDefaultInitialPose();
            (void)App_Maze_ResetStateWithPose(app_nav_supervisor_initial_x,
                                              app_nav_supervisor_initial_y,
                                              app_nav_supervisor_initial_heading);
        }
    }
    else
    {
        App_Maze_ResetState();
    }

    app_nav_supervisor_debug.active = 0U;
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    App_NavSupervisor_UpdateMazeDebug();
}

bool App_NavSupervisor_SetInitialPose(uint8_t x,
                                      uint8_t y,
                                      HeadingTypeDef heading)
{
    if (!App_Maze_IsValidPose(x, y, heading))
    {
        return false;
    }

    app_nav_supervisor_initial_x = x;
    app_nav_supervisor_initial_y = y;
    app_nav_supervisor_initial_heading = heading;
    app_nav_supervisor_initial_pose_valid = 1U;

    return true;
}

bool App_NavSupervisor_ResetWithInitialPose(uint8_t x,
                                            uint8_t y,
                                            HeadingTypeDef heading)
{
    if (!App_NavSupervisor_SetInitialPose(x, y, heading))
    {
        App_NavSupervisor_SetDefaultInitialPose();
        App_NavSupervisor_Reset();
        return false;
    }

    App_NavSupervisor_Reset();
    return true;
}

bool App_NavSupervisor_SetGoalCell(uint8_t x, uint8_t y)
{
    if (!App_NavSupervisor_IsValidCell(x, y))
    {
        app_nav_supervisor_goal_x = 0U;
        app_nav_supervisor_goal_y = 0U;
        app_nav_supervisor_goal_valid = 0U;
        return false;
    }

    app_nav_supervisor_goal_x = x;
    app_nav_supervisor_goal_y = y;
    app_nav_supervisor_goal_valid = 1U;
    return true;
}

bool App_NavSupervisor_GetGoalCell(uint8_t *x, uint8_t *y, bool *valid)
{
    if ((x == NULL) || (y == NULL) || (valid == NULL))
    {
        return false;
    }

    *x = app_nav_supervisor_goal_x;
    *y = app_nav_supervisor_goal_y;
    *valid = (app_nav_supervisor_goal_valid != 0U);

    return true;
}

bool App_NavSupervisor_SetMission(AppNavSupervisorMission mission)
{
    switch (mission)
    {
    case APP_NAV_SUPERVISOR_MISSION_FIND_CELLS:
    case APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B:
        app_nav_supervisor_mission = mission;
        return true;

    default:
        return false;
    }
}

AppNavSupervisorMission App_NavSupervisor_GetMission(void)
{
    return app_nav_supervisor_mission;
}

bool App_NavSupervisor_Start(void)
{
    uint8_t current_x = 0U;
    uint8_t current_y = 0U;
    HeadingTypeDef current_heading = HEADING_NORTH;

    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();
    App_NavSupervisor_ClearPivotExitLatch();

    if (app_nav_supervisor_mission == APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B)
    {
        if (app_nav_supervisor_goal_valid == 0U)
        {
            app_nav_supervisor_debug.active = 0U;
            App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                                       APP_NAV_SUPERVISOR_ACTION_NONE,
                                       APP_NAV_SUPERVISOR_RESULT_GO_TO_B_INVALID_TARGET);
            App_NavSupervisor_UpdateMazeDebug();
            return false;
        }

        if (App_Maze_GetRobotPose(&current_x, &current_y, &current_heading) &&
            (current_x == app_nav_supervisor_goal_x) &&
            (current_y == app_nav_supervisor_goal_y))
        {
            app_nav_supervisor_debug.active = 0U;
            App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                                       APP_NAV_SUPERVISOR_ACTION_NONE,
                                       APP_NAV_SUPERVISOR_RESULT_GO_TO_B_COMPLETE);
            App_NavSupervisor_UpdateMazeDebug();
            return true;
        }
    }
    else if (app_nav_supervisor_mission != APP_NAV_SUPERVISOR_MISSION_FIND_CELLS)
    {
        app_nav_supervisor_debug.active = 0U;
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_ERROR,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
        App_NavSupervisor_UpdateMazeDebug();
        return false;
    }

    app_nav_supervisor_debug.active = 1U;
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_START_INITIAL_ADVANCE,
                               APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    App_NavSupervisor_UpdateMazeDebug();

    return true;
}

void App_NavSupervisor_Stop(void)
{
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();
    App_NavSupervisor_ClearPivotExitLatch();

    app_nav_supervisor_debug.active = 0U;
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    App_NavSupervisor_UpdateMazeDebug();
}

AppNavSupervisorState App_NavSupervisor_Tick(const AppNavInput *input,
                                             AppNavOutput *output)
{
    if (output != NULL)
    {
        App_NavSupervisor_ClearOutput(output);
    }

    if ((input == NULL) || (output == NULL))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_INVALID_ARGUMENT);
    }

    if (app_nav_supervisor_debug.active == 0U)
    {
        if (app_nav_supervisor_debug.state != APP_NAV_SUPERVISOR_IDLE)
        {
            App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                                       APP_NAV_SUPERVISOR_ACTION_NONE,
                                       APP_NAV_SUPERVISOR_RESULT_OK);
        }

        return app_nav_supervisor_debug.state;
    }

    switch (app_nav_supervisor_debug.state)
    {
    case APP_NAV_SUPERVISOR_START_INITIAL_ADVANCE:
        return App_NavSupervisor_HandleStartInitialAdvance(input, output);

    case APP_NAV_SUPERVISOR_RUN_INITIAL_ADVANCE:
        return App_NavSupervisor_HandleInitialAdvance(input, output);

    case APP_NAV_SUPERVISOR_DECIDE:
    	return App_NavSupervisor_HandleDecide(input, output);

    case APP_NAV_SUPERVISOR_RUN_ADVANCE:
        return App_NavSupervisor_HandleAdvance(input, output);

    case APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT:
        return App_NavSupervisor_HandleApproachFrontWallForPivot(input, output);

    case APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT:
        return App_NavSupervisor_HandleCenterFrontTapeForPivot(input, output);

    case APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT:
    case APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT:
        return App_NavSupervisor_HandleSmooth(input, output);

    case APP_NAV_SUPERVISOR_RUN_PIVOT_180:
        return App_NavSupervisor_HandlePivot(input, output);

    case APP_NAV_SUPERVISOR_ERROR:
        App_NavSupervisor_ClearOutput(output);
        return app_nav_supervisor_debug.state;

    case APP_NAV_SUPERVISOR_IDLE:
    default:
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;
    }
}

void App_NavSupervisor_GetDebug(AppNavSupervisorDebug *debug_out)
{
    if (debug_out == NULL)
    {
        return;
    }

    *debug_out = app_nav_supervisor_debug;
}

bool App_NavSupervisor_IsActive(void)
{
    return (app_nav_supervisor_debug.active != 0U);
}
