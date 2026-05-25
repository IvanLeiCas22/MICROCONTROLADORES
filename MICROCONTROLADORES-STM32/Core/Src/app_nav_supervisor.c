#include "app_nav_supervisor.h"

#include "app_maze.h"
#include "app_nav.h"

#include <stddef.h>

#define APP_NAV_SUPERVISOR_RESULT_OK 0U
#define APP_NAV_SUPERVISOR_RESULT_INVALID_ARGUMENT 1U
#define APP_NAV_SUPERVISOR_RESULT_START_FAILED 2U
#define APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR 3U
#define APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION 4U
#define APP_NAV_SUPERVISOR_YAW_180_Q16 ((int64_t)180 << 16)
#define APP_NAV_SUPERVISOR_YAW_360_Q16 ((int64_t)360 << 16)

static AppNavSupervisorDebug app_nav_supervisor_debug;
static int32_t app_nav_supervisor_action_yaw_reference_q16_deg;
static uint8_t app_nav_supervisor_action_yaw_reference_valid;

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
    App_Nav_StopSmoothAction();
    App_Nav_StopPivotAction();
}

static void App_NavSupervisor_ClearActionYawReference(void)
{
    app_nav_supervisor_action_yaw_reference_q16_deg = 0;
    app_nav_supervisor_action_yaw_reference_valid = 0U;
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
        return;
    }

    app_nav_supervisor_debug.maze_x = payload[0];
    app_nav_supervisor_debug.maze_y = payload[1];
    app_nav_supervisor_debug.maze_cell = payload[2];
    app_nav_supervisor_debug.maze_heading = payload[3];
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
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_ERROR,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               result);
    return app_nav_supervisor_debug.state;
}

static TurnTypeDef App_NavSupervisor_SmoothTurnForState(AppNavSupervisorState state)
{
    return (state == APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT) ? TURN_LEFT : TURN_RIGHT;
}

static bool App_NavSupervisor_StartAdvance(const AppNavInput *input)
{
    if (!App_NavSupervisor_CaptureActionYawReference(input))
    {
        return false;
    }

    if (!App_Nav_StartAdvanceAction(APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD))
    {
        App_NavSupervisor_ClearActionYawReference();
        return false;
    }

    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_ADVANCE,
                               APP_NAV_SUPERVISOR_ACTION_ADVANCE,
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
        if (!App_NavSupervisor_CaptureActionYawReference(input))
        {
            return false;
        }
        if (!App_Nav_StartSmoothAction(APP_NAV_SMOOTH_ACTION_LEFT))
        {
            App_NavSupervisor_ClearActionYawReference();
            return false;
        }
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT,
                                   APP_NAV_SUPERVISOR_ACTION_SMOOTH_LEFT,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return true;

    case APP_NAV_ACTION_SMOOTH_RIGHT:
        if (!App_NavSupervisor_CaptureActionYawReference(input))
        {
            return false;
        }
        if (!App_Nav_StartSmoothAction(APP_NAV_SMOOTH_ACTION_RIGHT))
        {
            App_NavSupervisor_ClearActionYawReference();
            return false;
        }
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT,
                                   APP_NAV_SUPERVISOR_ACTION_SMOOTH_RIGHT,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return true;

    case APP_NAV_ACTION_GO_BACK:
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

    case APP_NAV_ACTION_NONE:
    default:
        return false;
    }
}

static AppNavSupervisorState App_NavSupervisor_HandleDecide(const AppNavInput *input)
{
    AppNavOutput dummy_output = {0};
    AppNavDebug nav_debug = {0};
    AppNavRecommendedAction recommended_action = APP_NAV_ACTION_NONE;

    App_Nav_Tick(input, &dummy_output);
    App_Nav_GetDebug(&nav_debug);

    App_Maze_MapCurrentCell((nav_debug.wall_front != 0U),
                            (nav_debug.wall_right != 0U),
                            (nav_debug.wall_left != 0U));
    App_NavSupervisor_UpdateMazeDebug();

    if (!App_Nav_RecommendAction(0U, &recommended_action))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION);
    }

    if (!App_NavSupervisor_StartRecommendedAction(recommended_action, input))
    {
        return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
    }

    return app_nav_supervisor_debug.state;
}

static AppNavSupervisorState App_NavSupervisor_HandleAdvance(const AppNavInput *input,
                                                             AppNavOutput *output)
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
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_RUN_ADVANCE,
                                   APP_NAV_SUPERVISOR_ACTION_ADVANCE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE:
        App_NavSupervisor_ClearOutput(output);
        App_Maze_AdvanceRobotPosition();
        App_Nav_StopAdvanceAction();
        App_NavSupervisor_ClearActionYawReference();
        App_NavSupervisor_UpdateMazeDebug();
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_ADVANCE_ACTION_FRONT_OBSTACLE_SAFETY:
    case APP_NAV_ADVANCE_ACTION_TIMEOUT:
    case APP_NAV_ADVANCE_ACTION_ERROR:
    case APP_NAV_ADVANCE_ACTION_IDLE:
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
        App_Nav_StopSmoothAction();
        App_NavSupervisor_ClearActionYawReference();
        App_NavSupervisor_UpdateMazeDebug();
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;

    case APP_NAV_SMOOTH_ACTION_DONE_WALL:
        App_NavSupervisor_ClearOutput(output);
        App_Maze_UpdateRobotHeading(turn);
        App_Nav_StopSmoothAction();
        App_NavSupervisor_UpdateMazeDebug();

        if (!App_NavSupervisor_StartAdvance(input))
        {
            return App_NavSupervisor_SetError(APP_NAV_SUPERVISOR_RESULT_START_FAILED);
        }

        return app_nav_supervisor_debug.state;

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

void App_NavSupervisor_Init(void)
{
    App_NavSupervisor_Reset();
}

void App_NavSupervisor_Reset(void)
{
    App_NavSupervisor_StopActions();
    App_Maze_ResetState();
    App_NavSupervisor_ClearActionYawReference();

    app_nav_supervisor_debug.active = 0U;
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    App_NavSupervisor_UpdateMazeDebug();
}

bool App_NavSupervisor_Start(void)
{
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();

    app_nav_supervisor_debug.active = 1U;
    App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_DECIDE,
                               APP_NAV_SUPERVISOR_ACTION_NONE,
                               APP_NAV_SUPERVISOR_RESULT_OK);
    App_NavSupervisor_UpdateMazeDebug();

    return true;
}

void App_NavSupervisor_Stop(void)
{
    App_NavSupervisor_StopActions();
    App_NavSupervisor_ClearActionYawReference();

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
        App_NavSupervisor_SetState(APP_NAV_SUPERVISOR_IDLE,
                                   APP_NAV_SUPERVISOR_ACTION_NONE,
                                   APP_NAV_SUPERVISOR_RESULT_OK);
        return app_nav_supervisor_debug.state;
    }

    switch (app_nav_supervisor_debug.state)
    {
    case APP_NAV_SUPERVISOR_DECIDE:
        return App_NavSupervisor_HandleDecide(input);

    case APP_NAV_SUPERVISOR_RUN_ADVANCE:
        return App_NavSupervisor_HandleAdvance(input, output);

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
