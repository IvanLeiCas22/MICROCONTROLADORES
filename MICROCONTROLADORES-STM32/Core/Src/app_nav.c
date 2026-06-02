#include "app_nav.h"
#include "pid_controller.h"

#include <string.h>

/*
 * Portable navigation primitive layer.
 *
 * This file intentionally contains no HAL calls. It is shared between the STM32
 * firmware and the Qt simulator firmware_core.
 *
 * Important ownership rule:
 *
 * - app_nav.c owns perception, low-level controllers and primitive actions.
 * - app_nav_supervisor.c owns mission sequencing, logical maze updates and
 *   FIND_CELLS completion.
 *
 * The active robot motion during FIND_CELLS / GO_A_TO_B is driven by the
 * explicit primitive action APIs: AdvanceAction, SmoothAction, PivotAction,
 * ApproachFrontWallAction and CenterByFrontTapeForPivotAction.
 */

typedef enum
{
    APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK = 0,
    APP_NAV_REAR_TAPE_GATE_WAIT_SPECIAL_PATCH_BLACK,
    APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH,
    APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE
} AppNavRearTapeGateState;

typedef enum
{
    APP_NAV_FRONT_TAPE_GATE_INIT = 0,
    APP_NAV_FRONT_TAPE_GATE_WAIT_LEAVE_CURRENT_BLACK,
    APP_NAV_FRONT_TAPE_GATE_ARMED_FOR_BOUNDARY_TAPE
} AppNavFrontTapeGateState;

typedef enum
{
    APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_WAIT_LEAVE_ENTRY = 0,
    APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING,
    APP_NAV_ADVANCE_REAR_TAPE_GATE_EXIT_DETECTED,
    APP_NAV_ADVANCE_REAR_TAPE_GATE_ERROR
} AppNavAdvanceRearTapeGateResult;

typedef enum
{
    APP_NAV_FORWARD_GUIDANCE_WALL_FOLLOW = 0,
    APP_NAV_FORWARD_GUIDANCE_YAW_HOLD
} AppNavForwardGuidanceMode;

typedef enum
{
    APP_NAV_SMOOTH_TURN_LEFT = 0,
    APP_NAV_SMOOTH_TURN_RIGHT = 1
} AppNavSmoothTurnDirection;

static AppNavConfig app_nav_config;
static AppNavPerception app_nav_perception;
static PID_Controller_t app_nav_advance_pid;
static PID_Controller_t app_nav_smooth_turn_pid;
static PID_Controller_t app_nav_pivot_turn_pid;
static AppNavAdvanceActionMode app_nav_advance_action_mode;
static AppNavAdvanceActionState app_nav_advance_action_state;
static AppNavRearTapeProfile app_nav_advance_rear_tape_profile;
static AppNavRearTapeGateState app_nav_advance_rear_tape_gate_state;
static AppNavApproachFrontWallActionState app_nav_approach_front_wall_action_state;
static AppNavCenterFrontTapeActionState app_nav_center_front_tape_action_state;
static AppNavFrontTapeProfile app_nav_center_front_tape_profile;
static AppNavFrontTapeGateState app_nav_center_front_tape_gate_state;
static AppNavSmoothTurnDirection app_nav_smooth_turn_direction;
static AppNavSmoothActionType app_nav_smooth_action_type;
static AppNavSmoothActionState app_nav_smooth_action_state;
static AppNavRearTapeProfile app_nav_smooth_rear_tape_profile;
static AppNavRearTapeGateState app_nav_smooth_rear_tape_gate_state;
static AppNavPivotActionType app_nav_pivot_action_type;
static AppNavPivotActionState app_nav_pivot_action_state;
static int32_t app_nav_straight_yaw_target_q16_deg;
static int16_t app_nav_pivot_target_yaw_degrees;
static int16_t app_nav_pivot_last_target_dps;
static uint16_t app_nav_smooth_post_yaw_ticks;
static uint32_t app_nav_pivot_elapsed_ms;
static uint8_t app_nav_straight_active;
static uint8_t app_nav_wall_follow_active;
static uint8_t app_nav_advance_action_active;
static uint8_t app_nav_advance_was_rear_tape_detected;
static uint8_t app_nav_advance_yaw_hold_started;
static uint8_t app_nav_approach_front_wall_action_active;
static uint8_t app_nav_approach_front_wall_yaw_hold_started;
static uint8_t app_nav_center_front_tape_action_active;
static uint8_t app_nav_center_front_tape_was_front_tape_detected;
static uint8_t app_nav_center_front_tape_yaw_hold_started;
static uint8_t app_nav_smooth_turn_active;
static uint8_t app_nav_smooth_action_active;
static uint8_t app_nav_smooth_was_rear_tape_detected;
static uint8_t app_nav_pivot_turn_active;
static AppNavPrimitiveTestType app_nav_primitive_test_type;
static AppNavPrimitiveTestState app_nav_primitive_test_state;
static uint8_t app_nav_primitive_test_action_started;

static bool App_Nav_StartYawHoldAdvanceInternal(int32_t yaw_target_q16_deg,
                                                uint8_t clear_smooth_action);
static bool App_Nav_ComputeYawHoldAdvancePwm(const AppNavInput *input,
                                             uint16_t right_base_pwm,
                                             uint16_t left_base_pwm,
                                             AppNavOutput *output);
static bool App_Nav_StartWallFollowAdvance(void);
static bool App_Nav_ComputeWallFollowPwm(const AppNavInput *input,
                                         const AppNavPerception *perception,
                                         uint16_t right_base_pwm,
                                         uint16_t left_base_pwm,
                                         AppNavOutput *output);
static bool App_Nav_StartSmoothTurn(AppNavSmoothTurnDirection direction);
static bool App_Nav_ComputeSmoothTurnPwm(const AppNavInput *input,
                                         AppNavOutput *output);
static bool App_Nav_StartPivotTurn(void);
static bool App_Nav_ComputePivotTurnPwm(const AppNavInput *input,
                                        int16_t target_dps,
                                        AppNavOutput *output);

/* Mirrors the legacy ADC channel order without depending on app_config.h. */
typedef enum
{
    APP_NAV_ADC_RIGHT_LAT_CH = 0U,
    APP_NAV_ADC_DIAGONAL_RIGHT_CH = 1U,
    APP_NAV_ADC_FRONT_RIGHT_CH = 2U,
    APP_NAV_ADC_FLOOR_FRONT_CH = 3U,
    APP_NAV_ADC_FRONT_LEFT_CH = 4U,
    APP_NAV_ADC_DIAGONAL_LEFT_CH = 5U,
    APP_NAV_ADC_LEFT_LAT_CH = 6U,
    APP_NAV_ADC_FLOOR_REAR_CH = 7U
} AppNavAdcChannel;

typedef enum
{
    APP_NAV_DECISION_BACK = 0U,
    APP_NAV_DECISION_FRONT = 1U,
    APP_NAV_DECISION_RIGHT = 2U,
    APP_NAV_DECISION_LEFT = 3U
} AppNavDecisionOption;

#define APP_NAV_OPTION_BACK_MASK (1U << APP_NAV_DECISION_BACK)
#define APP_NAV_OPTION_FRONT_MASK (1U << APP_NAV_DECISION_FRONT)
#define APP_NAV_OPTION_RIGHT_MASK (1U << APP_NAV_DECISION_RIGHT)
#define APP_NAV_OPTION_LEFT_MASK (1U << APP_NAV_DECISION_LEFT)
#define APP_NAV_PIVOT_COMPLETION_DEAD_ZONE_DEG 1

static void App_Nav_ApplyAdvancePidConfig(uint8_t reset_state)
{
    int32_t output_limit_pwm = app_nav_config.advance_pid_output_limit_pwm;

    if (output_limit_pwm < 0)
    {
        output_limit_pwm = 0;
    }

    PID_Config_t cfg = {
        .kp = app_nav_config.advance_pid_kp_q16,
        .ki = app_nav_config.advance_pid_ki_q16,
        .kd = app_nav_config.advance_pid_kd_q16,
        .out_min = -INT_TO_FIXED(output_limit_pwm),
        .out_max = INT_TO_FIXED(output_limit_pwm),
    };

    PID_ApplyConfig(&app_nav_advance_pid, &cfg, (reset_state != 0U));
}

static void App_Nav_ApplySmoothTurnPidConfig(uint8_t reset_state)
{
    int32_t output_limit_pwm = app_nav_config.smooth_turn_pid_output_limit_pwm;

    if (output_limit_pwm < 0)
    {
        output_limit_pwm = 0;
    }

    PID_Config_t cfg = {
        .kp = app_nav_config.smooth_turn_pid_kp_q16,
        .ki = app_nav_config.smooth_turn_pid_ki_q16,
        .kd = app_nav_config.smooth_turn_pid_kd_q16,
        .out_min = -INT_TO_FIXED(output_limit_pwm),
        .out_max = INT_TO_FIXED(output_limit_pwm),
    };

    PID_ApplyConfig(&app_nav_smooth_turn_pid, &cfg, (reset_state != 0U));
}

static void App_Nav_ApplyPivotTurnPidConfig(uint8_t reset_state)
{
    int32_t output_limit_pwm = app_nav_config.pivot_turn_pid_output_limit_pwm;

    if (output_limit_pwm < 0)
    {
        output_limit_pwm = 0;
    }

    PID_Config_t cfg = {
        .kp = app_nav_config.pivot_turn_pid_kp_q16,
        .ki = app_nav_config.pivot_turn_pid_ki_q16,
        .kd = app_nav_config.pivot_turn_pid_kd_q16,
        .out_min = -INT_TO_FIXED(output_limit_pwm),
        .out_max = INT_TO_FIXED(output_limit_pwm),
    };

    PID_ApplyConfig(&app_nav_pivot_turn_pid, &cfg, (reset_state != 0U));
}

static void App_Nav_SetSmoothTurnSetpoint(AppNavSmoothTurnDirection direction)
{
    int32_t target_dps = (int32_t)app_nav_config.turn_target_dps;

    if (direction == APP_NAV_SMOOTH_TURN_RIGHT)
    {
        target_dps = -target_dps;
    }

    PID_Set_Setpoint(&app_nav_smooth_turn_pid, target_dps);
}

static int32_t App_Nav_LimitCorrectionToMotorBases(int32_t correction,
                                                   uint16_t right_base_pwm,
                                                   uint16_t left_base_pwm)
{
    int32_t right_base = (int32_t)right_base_pwm;
    int32_t left_base = (int32_t)left_base_pwm;

    if (correction > right_base)
    {
        return right_base;
    }

    if (correction < -left_base)
    {
        return -left_base;
    }

    return correction;
}

static void App_Nav_ClearOutput(AppNavOutput *output)
{
    if (output == NULL)
    {
        return;
    }

    memset(output, 0, sizeof(*output));
}

static int32_t App_Nav_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void App_Nav_ClearPivotActionState(void)
{
    app_nav_pivot_action_type = APP_NAV_PIVOT_LEFT_90;
    app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_IDLE;
    app_nav_pivot_target_yaw_degrees = 0;
    app_nav_pivot_last_target_dps = 0;
    app_nav_pivot_elapsed_ms = 0U;
}

static void App_Nav_ClearAdvanceActionState(void)
{
    app_nav_advance_action_mode = APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD;
    app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_IDLE;
    app_nav_advance_rear_tape_profile = APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL;
    app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK;
    app_nav_advance_action_active = 0U;
    app_nav_advance_was_rear_tape_detected = 0U;
    app_nav_advance_yaw_hold_started = 0U;
}

static void App_Nav_ClearApproachFrontWallActionState(void)
{
    app_nav_approach_front_wall_action_state = APP_NAV_APPROACH_FRONT_WALL_ACTION_IDLE;
    app_nav_approach_front_wall_action_active = 0U;
    app_nav_approach_front_wall_yaw_hold_started = 0U;
}

static void App_Nav_ClearCenterFrontTapeActionState(void)
{
    app_nav_center_front_tape_action_state = APP_NAV_CENTER_FRONT_TAPE_ACTION_IDLE;
    app_nav_center_front_tape_profile = APP_NAV_FRONT_TAPE_PROFILE_NORMAL_CELL;
    app_nav_center_front_tape_gate_state = APP_NAV_FRONT_TAPE_GATE_INIT;
    app_nav_center_front_tape_action_active = 0U;
    app_nav_center_front_tape_was_front_tape_detected = 0U;
    app_nav_center_front_tape_yaw_hold_started = 0U;
}

static void App_Nav_SetAdvanceActionTerminal(AppNavAdvanceActionState terminal_state)
{
    app_nav_advance_action_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    app_nav_advance_action_state = terminal_state;
}

static void App_Nav_SetAdvanceActionRunningState(void)
{
    app_nav_advance_action_state = (app_nav_advance_yaw_hold_started != 0U)
                                       ? APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD
                                       : APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW;
}

static void App_Nav_SetApproachFrontWallActionTerminal(AppNavApproachFrontWallActionState terminal_state)
{
    app_nav_approach_front_wall_action_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    app_nav_approach_front_wall_action_state = terminal_state;
}

static void App_Nav_SetCenterFrontTapeActionTerminal(AppNavCenterFrontTapeActionState terminal_state)
{
    app_nav_center_front_tape_action_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    app_nav_center_front_tape_action_state = terminal_state;
}

static void App_Nav_ClearSmoothActionState(void)
{
    app_nav_smooth_action_type = APP_NAV_SMOOTH_ACTION_LEFT;
    app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_IDLE;
    app_nav_smooth_rear_tape_profile = APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL;
    app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK;
    app_nav_smooth_action_active = 0U;
    app_nav_smooth_was_rear_tape_detected = 0U;
    app_nav_smooth_post_yaw_ticks = 0U;
}

static void App_Nav_ClearPrimitiveTestState(void)
{
    app_nav_primitive_test_type = APP_NAV_PRIMITIVE_TEST_NONE;
    app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_IDLE;
    app_nav_primitive_test_action_started = 0U;
}

static void App_Nav_SetSmoothActionTerminal(AppNavSmoothActionState terminal_state)
{
    app_nav_smooth_action_active = 0U;
    app_nav_smooth_turn_active = 0U;
    app_nav_straight_active = 0U;
    app_nav_smooth_action_state = terminal_state;
    app_nav_smooth_post_yaw_ticks = 0U;
}

static void App_Nav_EnterSmoothPostYawSeek(const AppNavInput *input,
                                           AppNavOutput *output)
{
    app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE;
    app_nav_smooth_post_yaw_ticks = 0U;

    (void)App_Nav_StartYawHoldAdvanceInternal(input->yaw_q16_deg, 0U);

    output->right_motor_pwm = (int16_t)(app_nav_config.right_motor_base_speed);
    output->left_motor_pwm = (int16_t)(app_nav_config.left_motor_base_speed);
}

static bool App_Nav_UpdateSmoothRearTapeGate(bool current_rear_tape,
                                             bool exit_detection_enabled)
{
    switch (app_nav_smooth_rear_tape_gate_state)
    {
    case APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK:
        if (current_rear_tape)
        {
            app_nav_smooth_was_rear_tape_detected = 1U;
            return false;
        }

        app_nav_smooth_was_rear_tape_detected = 0U;

        if (app_nav_smooth_rear_tape_profile == APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL)
        {
            app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_SPECIAL_PATCH_BLACK;
        }
        else
        {
            app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE;
        }
        return false;

    case APP_NAV_REAR_TAPE_GATE_WAIT_SPECIAL_PATCH_BLACK:
        if (current_rear_tape)
        {
            app_nav_smooth_was_rear_tape_detected = 1U;
            app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH;
        }
        else
        {
            app_nav_smooth_was_rear_tape_detected = 0U;
        }
        return false;

    case APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH:
        if (current_rear_tape)
        {
            app_nav_smooth_was_rear_tape_detected = 1U;
        }
        else
        {
            app_nav_smooth_was_rear_tape_detected = 0U;
            app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE;
        }
        return false;

    case APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE:
        if (!current_rear_tape)
        {
            app_nav_smooth_was_rear_tape_detected = 0U;
            return false;
        }

        if ((app_nav_smooth_was_rear_tape_detected == 0U) &&
            exit_detection_enabled)
        {
            app_nav_smooth_was_rear_tape_detected = 1U;
            return true;
        }

        if (exit_detection_enabled)
        {
            app_nav_smooth_was_rear_tape_detected = 1U;
        }

        return false;

    default:
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return false;
    }
}

static AppNavAdvanceRearTapeGateResult App_Nav_UpdateAdvanceRearTapeGate(bool current_rear_tape)
{
    switch (app_nav_advance_rear_tape_gate_state)
    {
    case APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK:
        if (current_rear_tape)
        {
            app_nav_advance_was_rear_tape_detected = 1U;

            if (app_nav_advance_rear_tape_profile == APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT)
            {
                app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH;
                return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING;
            }

            return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_WAIT_LEAVE_ENTRY;
        }

        app_nav_advance_was_rear_tape_detected = 0U;

        if ((app_nav_advance_rear_tape_profile == APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL) ||
            (app_nav_advance_rear_tape_profile == APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT))
        {
            app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_SPECIAL_PATCH_BLACK;
        }
        else
        {
            app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE;
        }

        return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING;

    case APP_NAV_REAR_TAPE_GATE_WAIT_SPECIAL_PATCH_BLACK:
        if (current_rear_tape)
        {
            app_nav_advance_was_rear_tape_detected = 1U;
            app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH;
        }
        else
        {
            app_nav_advance_was_rear_tape_detected = 0U;
        }

        return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING;

    case APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_SPECIAL_PATCH:
        if (current_rear_tape)
        {
            app_nav_advance_was_rear_tape_detected = 1U;
        }
        else
        {
            app_nav_advance_was_rear_tape_detected = 0U;
            app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE;
        }

        return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING;

    case APP_NAV_REAR_TAPE_GATE_ARMED_FOR_EXIT_TAPE:
        if (current_rear_tape)
        {
            if (app_nav_advance_was_rear_tape_detected == 0U)
            {
                return APP_NAV_ADVANCE_REAR_TAPE_GATE_EXIT_DETECTED;
            }

            app_nav_advance_was_rear_tape_detected = 1U;
        }
        else
        {
            app_nav_advance_was_rear_tape_detected = 0U;
        }

        return APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_RUNNING;

    default:
        return APP_NAV_ADVANCE_REAR_TAPE_GATE_ERROR;
    }
}

static bool App_Nav_GetPivotActionTargets(AppNavPivotActionType action,
                                          int16_t *target_yaw_degrees,
                                          int16_t *base_target_dps)
{
    int16_t target_dps = (int16_t)app_nav_config.pivot_turn_target_dps;

    if ((target_yaw_degrees == NULL) || (base_target_dps == NULL))
    {
        return false;
    }

    switch (action)
    {
    case APP_NAV_PIVOT_LEFT_90:
        *target_yaw_degrees = -90;
        *base_target_dps = target_dps;
        return true;
    case APP_NAV_PIVOT_RIGHT_90:
        *target_yaw_degrees = 90;
        *base_target_dps = -target_dps;
        return true;
    case APP_NAV_PIVOT_180_LEFT:
        *target_yaw_degrees = -180;
        *base_target_dps = target_dps;
        return true;
    case APP_NAV_PIVOT_180_RIGHT:
        *target_yaw_degrees = 180;
        *base_target_dps = -target_dps;
        return true;
    default:
        return false;
    }
}

static uint8_t App_Nav_DetectLowWithHysteresis(uint16_t value,
                                               uint16_t threshold,
                                               uint16_t hysteresis,
                                               uint8_t was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected != 0U)
    {
        return ((uint32_t)value < release_threshold) ? 1U : 0U;
    }

    return (value < threshold) ? 1U : 0U;
}

static uint8_t App_Nav_DetectFrontWallWithHysteresis(uint16_t left_value,
                                                     uint16_t right_value,
                                                     uint16_t threshold,
                                                     uint16_t hysteresis,
                                                     uint8_t was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected != 0U)
    {
        return (((uint32_t)left_value < release_threshold) &&
                ((uint32_t)right_value < release_threshold)) ? 1U : 0U;
    }

    return ((left_value < threshold) && (right_value < threshold)) ? 1U : 0U;
}

static bool App_Nav_UpdatePerception(const AppNavInput *input)
{
    uint16_t floor_front_adc;
    uint16_t floor_rear_adc;

    if (input == NULL)
    {
        return false;
    }

    floor_front_adc = input->adc_filtered[APP_NAV_ADC_FLOOR_FRONT_CH];
    floor_rear_adc = input->adc_filtered[APP_NAV_ADC_FLOOR_REAR_CH];

    app_nav_perception.wall_front = App_Nav_DetectFrontWallWithHysteresis(input->dist_front_left_mm,
                                              	  	  	  	  	  	  	  input->dist_front_right_mm,
																		  app_nav_config.wall_threshold_mm_front,
																		  app_nav_config.wall_hysteresis_mm,
																		  app_nav_perception.wall_front);

    app_nav_perception.wall_left = App_Nav_DetectLowWithHysteresis(input->dist_left_lat_mm,
                                        							app_nav_config.wall_threshold_mm_side,
																	app_nav_config.wall_hysteresis_mm,
																	app_nav_perception.wall_left);

    app_nav_perception.wall_right = App_Nav_DetectLowWithHysteresis(input->dist_right_lat_mm,
                                        							app_nav_config.wall_threshold_mm_side,
																	app_nav_config.wall_hysteresis_mm,
																	app_nav_perception.wall_right);

    app_nav_perception.wall_diag_left = App_Nav_DetectLowWithHysteresis(input->dist_diagonal_left_mm,
                                        								app_nav_config.wall_threshold_mm_diagonal,
																		app_nav_config.wall_hysteresis_mm,
																		app_nav_perception.wall_diag_left);

    app_nav_perception.wall_diag_right = App_Nav_DetectLowWithHysteresis(input->dist_diagonal_right_mm,
                                        								app_nav_config.wall_threshold_mm_diagonal,
																		app_nav_config.wall_hysteresis_mm,
																		app_nav_perception.wall_diag_right);

    app_nav_perception.floor_front_black = App_Nav_DetectLowWithHysteresis(floor_front_adc,
                                        									app_nav_config.tape_detection_threshold_adc,
																			app_nav_config.tape_hysteresis_adc,
																			app_nav_perception.floor_front_black);

    app_nav_perception.floor_rear_black = App_Nav_DetectLowWithHysteresis(floor_rear_adc,
                                        									app_nav_config.tape_detection_threshold_adc,
																			app_nav_config.tape_hysteresis_adc,
																			app_nav_perception.floor_rear_black);

    return true;
}


static void App_Nav_ResetPerception(void)
{
    memset(&app_nav_perception, 0, sizeof(app_nav_perception));
}


/* -------------------------------------------------------------------------- */
/* Configuration, lifecycle and perception                                      */
/* -------------------------------------------------------------------------- */

void App_Nav_Init(const AppNavConfig *config)
{
    if (config != NULL)
    {
        app_nav_config = *config;
    }
    else
    {
        app_nav_config = App_Nav_DefaultConfig();
    }

    app_nav_straight_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_smooth_turn_active = 0U;
    app_nav_pivot_turn_active = 0U;
    app_nav_smooth_turn_direction = APP_NAV_SMOOTH_TURN_LEFT;
    app_nav_straight_yaw_target_q16_deg = 0;
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearCenterFrontTapeActionState();
    App_Nav_ClearSmoothActionState();
    App_Nav_ClearPivotActionState();
    App_Nav_ClearPrimitiveTestState();
    App_Nav_ApplyAdvancePidConfig(1U);
    App_Nav_ApplySmoothTurnPidConfig(1U);
    App_Nav_ApplyPivotTurnPidConfig(1U);
    App_Nav_ResetPerception();
}

void App_Nav_SetConfig(const AppNavConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    app_nav_config = *config;
    App_Nav_ApplyAdvancePidConfig(0U);
    App_Nav_ApplySmoothTurnPidConfig(0U);
    App_Nav_ApplyPivotTurnPidConfig(0U);
}

void App_Nav_GetConfig(AppNavConfig *config_out)
{
    if (config_out == NULL)
    {
        return;
    }

    *config_out = app_nav_config;
}

void App_Nav_Reset(void)
{
    app_nav_straight_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_smooth_turn_active = 0U;
    app_nav_pivot_turn_active = 0U;
    app_nav_smooth_turn_direction = APP_NAV_SMOOTH_TURN_LEFT;
    app_nav_straight_yaw_target_q16_deg = 0;
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearSmoothActionState();
    App_Nav_ClearPivotActionState();
    App_Nav_ClearPrimitiveTestState();
    PID_Reset(&app_nav_advance_pid);
    PID_Reset(&app_nav_smooth_turn_pid);
    PID_Reset(&app_nav_pivot_turn_pid);
    App_Nav_ResetPerception();
}

bool App_Nav_EvaluatePerception(const AppNavInput *input,
                                AppNavPerception *perception_out)
{
    if ((input == NULL) || (perception_out == NULL))
    {
        return false;
    }

    if (!App_Nav_UpdatePerception(input))
    {
        return false;
    }

    *perception_out = app_nav_perception;
    return true;
}


/* -------------------------------------------------------------------------- */
/* Local recommendation policy                                                  */
/* -------------------------------------------------------------------------- */

bool App_Nav_RecommendAction(const AppNavPerception *perception,
                              AppNavRecommendedAction *action_out)
{
    if ((perception == NULL) || (action_out == NULL))
    {
        return false;
    }

    if (perception->wall_front == 0U)
    {
        *action_out = APP_NAV_ACTION_GO_FRONT;
        return true;
    }

    if (perception->wall_right == 0U)
    {
        *action_out = APP_NAV_ACTION_SMOOTH_RIGHT;
        return true;
    }

    if (perception->wall_left == 0U)
    {
        *action_out = APP_NAV_ACTION_SMOOTH_LEFT;
        return true;
    }

    *action_out = APP_NAV_ACTION_GO_BACK;
    return true;
}

static bool App_Nav_StartYawHoldAdvanceInternal(int32_t yaw_target_q16_deg,
                                                uint8_t clear_smooth_action)
{
    app_nav_straight_yaw_target_q16_deg = yaw_target_q16_deg;
    app_nav_straight_active = 1U;
    app_nav_wall_follow_active = 0U;
    app_nav_smooth_turn_active = 0U;
    app_nav_pivot_turn_active = 0U;
    if (clear_smooth_action != 0U)
    {
        App_Nav_ClearAdvanceActionState();
        App_Nav_ClearApproachFrontWallActionState();
        App_Nav_ClearSmoothActionState();
    }
    App_Nav_ClearPivotActionState();

    PID_Reset(&app_nav_advance_pid);
    PID_Set_Setpoint_Fixed(&app_nav_advance_pid, yaw_target_q16_deg);

    return true;
}

/* -------------------------------------------------------------------------- */
/* Reusable low-level drive controllers                                         */
/* -------------------------------------------------------------------------- */

static bool App_Nav_ComputeYawHoldAdvancePwm(const AppNavInput *input,
                                      uint16_t right_base_pwm,
                                      uint16_t left_base_pwm,
                                      AppNavOutput *output)
{
    int32_t pid_output_fixed;
    int32_t correction;
    int32_t correction_limit;
    int32_t right_pwm;
    int32_t left_pwm;

    if ((input == NULL) || (output == NULL))
    {
        return false;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_straight_active == 0U)
    {
        return false;
    }

    PID_Set_Setpoint_Fixed(&app_nav_advance_pid,
                           app_nav_straight_yaw_target_q16_deg);
    pid_output_fixed = PID_Update_Fixed(&app_nav_advance_pid,
                                        input->yaw_q16_deg,
                                        input->dt_ms);
    correction = FIXED_TO_INT(pid_output_fixed);
    correction_limit = (right_base_pwm < left_base_pwm) ? (int32_t)right_base_pwm : (int32_t)left_base_pwm;

    if (correction > correction_limit)
    {
        correction = correction_limit;
    }
    else if (correction < -correction_limit)
    {
        correction = -correction_limit;
    }

    right_pwm = (int32_t)right_base_pwm - correction;
    left_pwm = (int32_t)left_base_pwm + correction;

    output->right_motor_pwm = (int16_t)right_pwm;
    output->left_motor_pwm = (int16_t)left_pwm;

    return true;
}


static bool App_Nav_StartWallFollowAdvance(void)
{
    app_nav_straight_active = 0U;
    app_nav_wall_follow_active = 1U;
    app_nav_smooth_turn_active = 0U;
    app_nav_pivot_turn_active = 0U;
    app_nav_straight_yaw_target_q16_deg = 0;
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearSmoothActionState();
    App_Nav_ClearPivotActionState();

    PID_Reset(&app_nav_advance_pid);
    PID_Set_Setpoint_Fixed(&app_nav_advance_pid, 0);

    return true;
}

/* -------------------------------------------------------------------------- */
/* Smooth turn controller and SmoothAction                                      */
/* -------------------------------------------------------------------------- */

static bool App_Nav_StartSmoothTurn(AppNavSmoothTurnDirection direction)
{
    if ((direction != APP_NAV_SMOOTH_TURN_LEFT) &&
        (direction != APP_NAV_SMOOTH_TURN_RIGHT))
    {
        return false;
    }

    app_nav_smooth_turn_direction = direction;
    app_nav_smooth_turn_active = 1U;
    app_nav_straight_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_pivot_turn_active = 0U;
    app_nav_straight_yaw_target_q16_deg = 0;
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearCenterFrontTapeActionState();
    App_Nav_ClearSmoothActionState();
    App_Nav_ClearPivotActionState();

    PID_Reset(&app_nav_smooth_turn_pid);
    App_Nav_SetSmoothTurnSetpoint(direction);

    return true;
}

static bool App_Nav_ComputeSmoothTurnPwm(const AppNavInput *input,
                                  AppNavOutput *output)
{
    int16_t base_right;
    int16_t base_left;
    int32_t pid_output_fixed;
    int16_t correction;
    int16_t right_speed;
    int16_t left_speed;

    if ((input == NULL) || (output == NULL))
    {
        return false;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_smooth_turn_active == 0U)
    {
        return false;
    }

    if (app_nav_smooth_turn_direction == APP_NAV_SMOOTH_TURN_LEFT)
    {
        base_right = (int16_t)app_nav_config.faster_motor_smooth_turn_speed;
        base_left = (int16_t)app_nav_config.slower_motor_smooth_turn_speed;
    }
    else if (app_nav_smooth_turn_direction == APP_NAV_SMOOTH_TURN_RIGHT)
    {
        base_right = (int16_t)app_nav_config.slower_motor_smooth_turn_speed;
        base_left = (int16_t)app_nav_config.faster_motor_smooth_turn_speed;
    }
    else
    {
        return false;
    }

    App_Nav_SetSmoothTurnSetpoint(app_nav_smooth_turn_direction);
    pid_output_fixed = PID_Update(&app_nav_smooth_turn_pid,
                                  input->yaw_rate_dps,
                                  input->dt_ms);
    correction = (int16_t)FIXED_TO_INT(pid_output_fixed);

    right_speed = base_right + correction;
    left_speed = base_left - correction;

    output->right_motor_pwm = right_speed;
    output->left_motor_pwm = left_speed;

    return true;
}

void App_Nav_StopSmoothAction(void)
{
    app_nav_smooth_turn_active = 0U;
    app_nav_straight_active = 0U;
    App_Nav_ClearSmoothActionState();
    PID_Reset(&app_nav_smooth_turn_pid);
    PID_Reset(&app_nav_advance_pid);
}

bool App_Nav_StartSmoothActionWithRearTapeProfile(AppNavSmoothActionType action,
                                                  AppNavRearTapeProfile rear_tape_profile)
{
    AppNavSmoothTurnDirection direction;

    if ((rear_tape_profile != APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL) &&
        (rear_tape_profile != APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL))
    {
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return false;
    }

    if (action == APP_NAV_SMOOTH_ACTION_LEFT)
    {
        direction = APP_NAV_SMOOTH_TURN_LEFT;
    }
    else if (action == APP_NAV_SMOOTH_ACTION_RIGHT)
    {
        direction = APP_NAV_SMOOTH_TURN_RIGHT;
    }
    else
    {
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return false;
    }

    if (!App_Nav_StartSmoothTurn(direction))
    {
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return false;
    }

    app_nav_smooth_action_type = action;
    app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_TURNING;
    app_nav_smooth_rear_tape_profile = rear_tape_profile;
    app_nav_smooth_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK;
    app_nav_smooth_action_active = 1U;
    app_nav_smooth_was_rear_tape_detected = app_nav_perception.floor_rear_black;
    app_nav_smooth_post_yaw_ticks = 0U;

    return true;
}

AppNavSmoothActionState App_Nav_TickSmoothAction(const AppNavInput *input,
                                                 const AppNavPerception *perception,
                                                 AppNavOutput *output)
{
    int32_t yaw_deg;
    bool current_rear_tape;
    bool rear_tape_detected = false;
    bool wall_detected = false;
    int32_t yaw_completion_threshold;
    bool yaw_target_reached;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return app_nav_smooth_action_state;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_IDLE)
    {
        return APP_NAV_SMOOTH_ACTION_IDLE;
    }

    if ((app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE) ||
        (app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE) ||
        (app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT) ||
        (app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_ERROR))
    {
        return app_nav_smooth_action_state;
    }

    if (app_nav_smooth_action_active == 0U)
    {
        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_ERROR;
        return app_nav_smooth_action_state;
    }

    yaw_deg = FIXED_TO_INT(input->yaw_q16_deg);
    current_rear_tape = (perception->floor_rear_black != 0U);

    rear_tape_detected = App_Nav_UpdateSmoothRearTapeGate(
        current_rear_tape,
        (App_Nav_AbsInt32(yaw_deg) > (int32_t)app_nav_config.smooth_rear_tape_min_yaw_deg));

    if (app_nav_smooth_action_type == APP_NAV_SMOOTH_ACTION_LEFT)
    {
        wall_detected = (input->dist_diagonal_left_mm < app_nav_config.after_turn_wall_threshold_mm);
    }
    else if (app_nav_smooth_action_type == APP_NAV_SMOOTH_ACTION_RIGHT)
    {
        wall_detected = (input->dist_diagonal_right_mm < app_nav_config.after_turn_wall_threshold_mm);
    }
    else
    {
        App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_ERROR);
        return app_nav_smooth_action_state;
    }

    yaw_completion_threshold = 90 - (int32_t)app_nav_config.smooth_turn_completion_dead_zone_deg;
    if (yaw_completion_threshold < 0)
    {
        yaw_completion_threshold = 0;
    }
    yaw_target_reached = (App_Nav_AbsInt32(yaw_deg) >= yaw_completion_threshold);

    if (app_nav_smooth_action_state == APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE)
    {
        if (rear_tape_detected)
        {
            App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE);
            return app_nav_smooth_action_state;
        }

        app_nav_smooth_post_yaw_ticks++;
        if (app_nav_smooth_post_yaw_ticks >= app_nav_config.smooth_post_yaw_seek_timeout_ticks)
        {
            App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT);
            return app_nav_smooth_action_state;
        }

        if (!App_Nav_ComputeYawHoldAdvancePwm(input,
                                              app_nav_config.right_motor_base_speed,
                                              app_nav_config.left_motor_base_speed,
                                              output))
        {
            App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_ERROR);
            return app_nav_smooth_action_state;
        }

        app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE;
        return app_nav_smooth_action_state;
    }

    if (rear_tape_detected)
    {
        App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE);
        return app_nav_smooth_action_state;
    }

    if (wall_detected)
    {
        App_Nav_EnterSmoothPostYawSeek(input,
                                       output);
        return app_nav_smooth_action_state;
    }

    if (yaw_target_reached)
    {
        App_Nav_EnterSmoothPostYawSeek(input,
                                       output);
        return app_nav_smooth_action_state;
    }

    if (!App_Nav_ComputeSmoothTurnPwm(input, output))
    {
        App_Nav_SetSmoothActionTerminal(APP_NAV_SMOOTH_ACTION_ERROR);
        return app_nav_smooth_action_state;
    }

    app_nav_smooth_action_state = APP_NAV_SMOOTH_ACTION_TURNING;
    return app_nav_smooth_action_state;
}

/* -------------------------------------------------------------------------- */
/* Pivot turn controller and PivotAction                                        */
/* -------------------------------------------------------------------------- */

static bool App_Nav_StartPivotTurn(void)
{
    app_nav_pivot_turn_active = 1U;
    app_nav_straight_active = 0U;
    app_nav_wall_follow_active = 0U;
    app_nav_smooth_turn_active = 0U;
    app_nav_straight_yaw_target_q16_deg = 0;
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearCenterFrontTapeActionState();
    App_Nav_ClearSmoothActionState();

    PID_Reset(&app_nav_pivot_turn_pid);

    return true;
}

static bool App_Nav_ComputePivotTurnPwm(const AppNavInput *input,
                                 int16_t target_dps,
                                 AppNavOutput *output)
{
    int32_t pid_output_fixed;
    int16_t correction_pwm;

    if ((input == NULL) || (output == NULL))
    {
        return false;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_pivot_turn_active == 0U)
    {
        return false;
    }

    PID_Set_Setpoint(&app_nav_pivot_turn_pid, target_dps);
    pid_output_fixed = PID_Update(&app_nav_pivot_turn_pid,
                                  input->yaw_rate_dps,
                                  input->dt_ms);
    correction_pwm = (int16_t)FIXED_TO_INT(pid_output_fixed);

    output->right_motor_pwm = correction_pwm;
    output->left_motor_pwm = -correction_pwm;

    return true;
}

void App_Nav_StopPivotAction(void)
{
    app_nav_pivot_turn_active = 0U;
    App_Nav_ClearPivotActionState();
    PID_Reset(&app_nav_pivot_turn_pid);
}

bool App_Nav_StartPivotAction(AppNavPivotActionType action)
{
    int16_t target_yaw_degrees;
    int16_t base_target_dps;

    if (!App_Nav_GetPivotActionTargets(action,
                                       &target_yaw_degrees,
                                       &base_target_dps))
    {
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        return false;
    }

    if (!App_Nav_StartPivotTurn())
    {
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        return false;
    }

    app_nav_pivot_action_type = action;
    app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_RUNNING;
    app_nav_pivot_target_yaw_degrees = target_yaw_degrees;
    app_nav_pivot_last_target_dps = base_target_dps;
    app_nav_pivot_elapsed_ms = 0U;

    return true;
}

AppNavPivotActionState App_Nav_TickPivotAction(const AppNavInput *input,
                                               const AppNavPerception *perception,
                                               AppNavOutput *output)
{
    int32_t current_yaw_degrees;
    int32_t target_abs;
    int32_t current_abs;
    int16_t base_target_dps;
    int16_t target_dps;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        return app_nav_pivot_action_state;
    }

    App_Nav_ClearOutput(output);

    (void)perception;

    if (app_nav_pivot_action_state == APP_NAV_PIVOT_ACTION_IDLE)
    {
        return APP_NAV_PIVOT_ACTION_IDLE;
    }

    if ((app_nav_pivot_action_state == APP_NAV_PIVOT_ACTION_DONE) ||
        (app_nav_pivot_action_state == APP_NAV_PIVOT_ACTION_TIMEOUT) ||
        (app_nav_pivot_action_state == APP_NAV_PIVOT_ACTION_ERROR))
    {
        return app_nav_pivot_action_state;
    }

    if ((UINT32_MAX - app_nav_pivot_elapsed_ms) >= input->dt_ms)
    {
        app_nav_pivot_elapsed_ms += input->dt_ms;
    }
    else
    {
        app_nav_pivot_elapsed_ms = UINT32_MAX;
    }

    if (!App_Nav_GetPivotActionTargets(app_nav_pivot_action_type,
                                       &app_nav_pivot_target_yaw_degrees,
                                       &base_target_dps))
    {
        app_nav_pivot_turn_active = 0U;
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        return app_nav_pivot_action_state;
    }

    current_yaw_degrees = FIXED_TO_INT(input->yaw_q16_deg);
    target_abs = App_Nav_AbsInt32(app_nav_pivot_target_yaw_degrees);
    current_abs = App_Nav_AbsInt32(current_yaw_degrees);

    if (target_abs == 0)
    {
        app_nav_pivot_turn_active = 0U;
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        return app_nav_pivot_action_state;
    }

    target_dps = (int16_t)(((int32_t)base_target_dps *
                            (int32_t)(((((target_abs - current_abs) * (int16_t)100) /
                                        target_abs) +
                                       (int16_t)40)) /
                           100));
    app_nav_pivot_last_target_dps = target_dps;

    if (current_abs >= (target_abs - APP_NAV_PIVOT_COMPLETION_DEAD_ZONE_DEG))
    {
        app_nav_pivot_turn_active = 0U;
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_DONE;
        return app_nav_pivot_action_state;
    }

    if (!App_Nav_ComputePivotTurnPwm(input, target_dps, output))
    {
        app_nav_pivot_turn_active = 0U;
        app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_ERROR;
        App_Nav_ClearOutput(output);
        return app_nav_pivot_action_state;
    }

    app_nav_pivot_action_state = APP_NAV_PIVOT_ACTION_RUNNING;
    return app_nav_pivot_action_state;
}

static bool App_Nav_ComputeWallFollowPwm(const AppNavInput *input,
                                  const AppNavPerception *perception,
                                  uint16_t right_base_pwm,
                                  uint16_t left_base_pwm,
                                  AppNavOutput *output)
{
    int32_t measured_diff;
    int32_t pid_output_fixed;
    int32_t correction;
    int32_t right_pwm;
    int32_t left_pwm;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        return false;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_wall_follow_active == 0U)
    {
        return false;
    }

    if ((perception->wall_diag_left != 0U) &&
        (perception->wall_diag_right != 0U) &&
        (perception->wall_left != 0U) &&
        (perception->wall_right != 0U))
    {
        measured_diff = (int32_t)input->dist_left_lat_mm -
                        (int32_t)input->dist_right_lat_mm;
    }
    else if ((perception->wall_diag_right != 0U) &&
             (perception->wall_right != 0U))
    {
        measured_diff = ((int32_t)app_nav_config.wall_target_mm -
                         (int32_t)input->dist_right_lat_mm) *
                        2;
    }
    else if ((perception->wall_diag_left != 0U) &&
             (perception->wall_left != 0U))
    {
        measured_diff = ((int32_t)input->dist_left_lat_mm -
                         (int32_t)app_nav_config.wall_target_mm) *
                        2;
    }
    else
    {
        return false;
    }

    PID_Set_Setpoint_Fixed(&app_nav_advance_pid, 0);
    pid_output_fixed = PID_Update(&app_nav_advance_pid, measured_diff, input->dt_ms);
    correction = App_Nav_LimitCorrectionToMotorBases(FIXED_TO_INT(pid_output_fixed),
                                                     right_base_pwm,
                                                     left_base_pwm);

    right_pwm = (int32_t)right_base_pwm - correction;
    left_pwm = (int32_t)left_base_pwm + correction;

    output->right_motor_pwm = (int16_t)right_pwm;
    output->left_motor_pwm = (int16_t)left_pwm;

    return true;
}

static bool App_Nav_ComputeForwardGuidedPwm(const AppNavInput *input,
                                            const AppNavPerception *perception,
                                            AppNavOutput *output,
                                            uint8_t force_yaw_hold,
                                            uint8_t *yaw_hold_started,
                                            AppNavForwardGuidanceMode *guidance_mode)
{
    if ((input == NULL) ||
        (perception == NULL) ||
        (output == NULL) ||
        (yaw_hold_started == NULL) ||
        (guidance_mode == NULL))
    {
        return false;
    }

    if ((force_yaw_hold != 0U) || (*yaw_hold_started != 0U))
    {
        if (*yaw_hold_started == 0U)
        {
            (void)App_Nav_StartYawHoldAdvanceInternal(input->yaw_q16_deg, 0U);
            *yaw_hold_started = 1U;
        }

        if (!App_Nav_ComputeYawHoldAdvancePwm(input,
                                              app_nav_config.right_motor_base_speed,
                                              app_nav_config.left_motor_base_speed,
                                              output))
        {
            return false;
        }

        *guidance_mode = APP_NAV_FORWARD_GUIDANCE_YAW_HOLD;
        return true;
    }

    if (App_Nav_ComputeWallFollowPwm(input,
                                     perception,
                                     app_nav_config.right_motor_base_speed,
                                     app_nav_config.left_motor_base_speed,
                                     output))
    {
        *guidance_mode = APP_NAV_FORWARD_GUIDANCE_WALL_FOLLOW;
        return true;
    }

    (void)App_Nav_StartYawHoldAdvanceInternal(input->yaw_q16_deg, 0U);
    *yaw_hold_started = 1U;

    if (!App_Nav_ComputeYawHoldAdvancePwm(input,
                                          app_nav_config.right_motor_base_speed,
                                          app_nav_config.left_motor_base_speed,
                                          output))
    {
        return false;
    }

    *guidance_mode = APP_NAV_FORWARD_GUIDANCE_YAW_HOLD;
    return true;
}

static bool App_Nav_ComputeAdvanceActionPwm(const AppNavInput *input,
                                            const AppNavPerception *perception,
                                            AppNavOutput *output)
{
    AppNavForwardGuidanceMode guidance_mode;
    uint8_t force_yaw_hold;

    force_yaw_hold =
        (app_nav_advance_action_state == APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD) ? 1U : 0U;

    if (!App_Nav_ComputeForwardGuidedPwm(input,
                                         perception,
                                         output,
                                         force_yaw_hold,
                                         &app_nav_advance_yaw_hold_started,
                                         &guidance_mode))
    {
        return false;
    }

    if (guidance_mode == APP_NAV_FORWARD_GUIDANCE_YAW_HOLD)
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD;
    }
    else
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW;
    }

    return true;
}

void App_Nav_StopAdvanceAction(void)
{
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    App_Nav_ClearAdvanceActionState();
    PID_Reset(&app_nav_advance_pid);
}

/* -------------------------------------------------------------------------- */
/* Portable primitive-test runner                                              */
/* -------------------------------------------------------------------------- */

bool App_NavPrimitiveTest_Start(AppNavPrimitiveTestType type)
{
    if (app_nav_primitive_test_state == APP_NAV_PRIMITIVE_TEST_RUNNING)
    {
        return false;
    }

    if ((type != APP_NAV_PRIMITIVE_TEST_SMOOTH_LEFT) &&
        (type != APP_NAV_PRIMITIVE_TEST_SMOOTH_RIGHT))
    {
        app_nav_primitive_test_type = APP_NAV_PRIMITIVE_TEST_NONE;
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_REJECTED;
        app_nav_primitive_test_action_started = 0U;
        return false;
    }

    app_nav_primitive_test_type = type;
    app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_RUNNING;
    app_nav_primitive_test_action_started = 0U;

    return true;
}

AppNavPrimitiveTestState App_NavPrimitiveTest_Tick(const AppNavInput *input,
                                                   const AppNavPerception *perception,
                                                   AppNavOutput *output)
{
    AppNavSmoothActionType smooth_action;
    AppNavSmoothActionState smooth_state;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_ERROR;
        app_nav_primitive_test_action_started = 0U;
        return app_nav_primitive_test_state;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_primitive_test_state != APP_NAV_PRIMITIVE_TEST_RUNNING)
    {
        return app_nav_primitive_test_state;
    }

    if (app_nav_primitive_test_type == APP_NAV_PRIMITIVE_TEST_SMOOTH_LEFT)
    {
        smooth_action = APP_NAV_SMOOTH_ACTION_LEFT;
    }
    else if (app_nav_primitive_test_type == APP_NAV_PRIMITIVE_TEST_SMOOTH_RIGHT)
    {
        smooth_action = APP_NAV_SMOOTH_ACTION_RIGHT;
    }
    else
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_REJECTED;
        app_nav_primitive_test_action_started = 0U;
        return app_nav_primitive_test_state;
    }

    if (app_nav_primitive_test_action_started == 0U)
    {
        if (!App_Nav_StartSmoothActionWithRearTapeProfile(
                smooth_action,
                APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL))
        {
            app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_ERROR;
            app_nav_primitive_test_action_started = 0U;
            return app_nav_primitive_test_state;
        }

        app_nav_primitive_test_action_started = 1U;
    }

    smooth_state = App_Nav_TickSmoothAction(input, perception, output);

    if ((smooth_state == APP_NAV_SMOOTH_ACTION_TURNING) ||
        (smooth_state == APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE))
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_RUNNING;
    }
    else if ((smooth_state == APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE) ||
             (smooth_state == APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE))
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_DONE;
        app_nav_primitive_test_action_started = 0U;
    }
    else if (smooth_state == APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT)
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_TIMEOUT;
        app_nav_primitive_test_action_started = 0U;
    }
    else
    {
        app_nav_primitive_test_state = APP_NAV_PRIMITIVE_TEST_ERROR;
        app_nav_primitive_test_action_started = 0U;
    }

    return app_nav_primitive_test_state;
}

void App_NavPrimitiveTest_Stop(void)
{
    if ((app_nav_primitive_test_type == APP_NAV_PRIMITIVE_TEST_SMOOTH_LEFT) ||
        (app_nav_primitive_test_type == APP_NAV_PRIMITIVE_TEST_SMOOTH_RIGHT))
    {
        App_Nav_StopSmoothAction();
    }

    App_Nav_ClearPrimitiveTestState();
}

AppNavPrimitiveTestState App_NavPrimitiveTest_GetState(void)
{
    return app_nav_primitive_test_state;
}

/* -------------------------------------------------------------------------- */
/* AdvanceAction: drive until rear tape confirms next cell boundary             */
/* -------------------------------------------------------------------------- */

bool App_Nav_StartAdvanceActionWithRearTapeProfile(AppNavAdvanceActionMode mode,
                                                   AppNavRearTapeProfile rear_tape_profile)
{
    if (mode != APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD)
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_ERROR;
        return false;
    }

    if ((rear_tape_profile != APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL) &&
        (rear_tape_profile != APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL) &&
        (rear_tape_profile != APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT))
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_ERROR;
        return false;
    }

    if (!App_Nav_StartWallFollowAdvance())
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_ERROR;
        return false;
    }

    App_Nav_ClearCenterFrontTapeActionState();

    app_nav_advance_action_mode = mode;
    app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE;
    app_nav_advance_rear_tape_profile = rear_tape_profile;
    app_nav_advance_rear_tape_gate_state = APP_NAV_REAR_TAPE_GATE_WAIT_LEAVE_ENTRY_BLACK;
    app_nav_advance_action_active = 1U;
    app_nav_advance_was_rear_tape_detected = 0U;
    app_nav_advance_yaw_hold_started = 0U;

    return true;
}

AppNavAdvanceActionState App_Nav_TickAdvanceAction(const AppNavInput *input,
                                                   const AppNavPerception *perception,
                                                   AppNavOutput *output)
{
    bool current_rear_tape;
    AppNavAdvanceRearTapeGateResult rear_tape_gate_result;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_ERROR;
        return app_nav_advance_action_state;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_advance_action_state == APP_NAV_ADVANCE_ACTION_IDLE)
    {
        return APP_NAV_ADVANCE_ACTION_IDLE;
    }

    if ((app_nav_advance_action_state == APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE) ||
        (app_nav_advance_action_state == APP_NAV_ADVANCE_ACTION_TIMEOUT) ||
        (app_nav_advance_action_state == APP_NAV_ADVANCE_ACTION_ERROR))
    {
        return app_nav_advance_action_state;
    }

    if ((app_nav_advance_action_active == 0U) ||
        (app_nav_advance_action_mode != APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD))
    {
        App_Nav_SetAdvanceActionTerminal(APP_NAV_ADVANCE_ACTION_ERROR);
        return app_nav_advance_action_state;
    }

    current_rear_tape = (perception->floor_rear_black != 0U);
    rear_tape_gate_result = App_Nav_UpdateAdvanceRearTapeGate(current_rear_tape);

    if (rear_tape_gate_result == APP_NAV_ADVANCE_REAR_TAPE_GATE_EXIT_DETECTED)
    {
        App_Nav_SetAdvanceActionTerminal(APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE);
        return app_nav_advance_action_state;
    }

    if (rear_tape_gate_result == APP_NAV_ADVANCE_REAR_TAPE_GATE_ERROR)
    {
        App_Nav_SetAdvanceActionTerminal(APP_NAV_ADVANCE_ACTION_ERROR);
        return app_nav_advance_action_state;
    }

    if (rear_tape_gate_result == APP_NAV_ADVANCE_REAR_TAPE_GATE_COMPUTE_WAIT_LEAVE_ENTRY)
    {
        if (!App_Nav_ComputeAdvanceActionPwm(input, perception, output))
        {
            App_Nav_SetAdvanceActionTerminal(APP_NAV_ADVANCE_ACTION_ERROR);
            return app_nav_advance_action_state;
        }

        app_nav_advance_action_state = APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE;
        return app_nav_advance_action_state;
    }

    App_Nav_SetAdvanceActionRunningState();

    if (!App_Nav_ComputeAdvanceActionPwm(input, perception, output))
    {
        App_Nav_SetAdvanceActionTerminal(APP_NAV_ADVANCE_ACTION_ERROR);
        return app_nav_advance_action_state;
    }

    return app_nav_advance_action_state;
}

void App_Nav_StopApproachFrontWallAction(void)
{
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    App_Nav_ClearApproachFrontWallActionState();
    PID_Reset(&app_nav_advance_pid);
}

/* -------------------------------------------------------------------------- */
/* ApproachFrontWallAction: prepare in-cell 180 pivot using front wall          */
/* -------------------------------------------------------------------------- */

bool App_Nav_StartApproachFrontWallAction(void)
{
    if (!App_Nav_StartWallFollowAdvance())
    {
        app_nav_approach_front_wall_action_state = APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR;
        return false;
    }

    app_nav_approach_front_wall_action_state = APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_WALL_FOLLOW;
    app_nav_approach_front_wall_action_active = 1U;
    app_nav_approach_front_wall_yaw_hold_started = 0U;

    return true;
}

AppNavApproachFrontWallActionState App_Nav_TickApproachFrontWallAction(const AppNavInput *input,
                                                                       const AppNavPerception *perception,
                                                                       AppNavOutput *output)
{
    AppNavForwardGuidanceMode guidance_mode;
    uint16_t front_avg_mm;
    uint8_t force_yaw_hold;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_approach_front_wall_action_state = APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR;
        return app_nav_approach_front_wall_action_state;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_approach_front_wall_action_state == APP_NAV_APPROACH_FRONT_WALL_ACTION_IDLE)
    {
        return APP_NAV_APPROACH_FRONT_WALL_ACTION_IDLE;
    }

    if ((app_nav_approach_front_wall_action_state == APP_NAV_APPROACH_FRONT_WALL_ACTION_DONE_FRONT_WALL) ||
        (app_nav_approach_front_wall_action_state == APP_NAV_APPROACH_FRONT_WALL_ACTION_TIMEOUT) ||
        (app_nav_approach_front_wall_action_state == APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR))
    {
        return app_nav_approach_front_wall_action_state;
    }

    if (app_nav_approach_front_wall_action_active == 0U)
    {
        App_Nav_SetApproachFrontWallActionTerminal(APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR);
        return app_nav_approach_front_wall_action_state;
    }

    front_avg_mm = (uint16_t)(((uint32_t)input->dist_front_left_mm +
                               (uint32_t)input->dist_front_right_mm) /
                              2U);

    if (front_avg_mm <= app_nav_config.approach_front_wall_target_mm)
    {
        App_Nav_SetApproachFrontWallActionTerminal(APP_NAV_APPROACH_FRONT_WALL_ACTION_DONE_FRONT_WALL);
        return app_nav_approach_front_wall_action_state;
    }

    force_yaw_hold =
        (app_nav_approach_front_wall_action_state ==
         APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_YAW_HOLD) ? 1U : 0U;

    if (!App_Nav_ComputeForwardGuidedPwm(input,
                                         perception,
                                         output,
                                         force_yaw_hold,
                                         &app_nav_approach_front_wall_yaw_hold_started,
                                         &guidance_mode))
    {
        App_Nav_SetApproachFrontWallActionTerminal(APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR);
        return app_nav_approach_front_wall_action_state;
    }

    if (guidance_mode == APP_NAV_FORWARD_GUIDANCE_YAW_HOLD)
    {
        app_nav_approach_front_wall_action_state =
            APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_YAW_HOLD;
    }
    else
    {
        app_nav_approach_front_wall_action_state =
            APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_WALL_FOLLOW;
    }

    return app_nav_approach_front_wall_action_state;
}

/* -------------------------------------------------------------------------- */
/* CenterByFrontTapeForPivotAction: prepare in-cell 180 pivot in open cell      */
/* -------------------------------------------------------------------------- */

static bool App_Nav_UpdateCenterFrontTapeGate(bool current_front_tape)
{
    switch (app_nav_center_front_tape_gate_state)
    {
    case APP_NAV_FRONT_TAPE_GATE_INIT:
        if (current_front_tape)
        {
            /*
             * If the front floor sensor is already black at action start,
             * never finish immediately. First leave the current black region,
             * then arm detection for the next rising edge.
             *
             * This protects special-cell black patches and any unexpected
             * initial black condition in normal cells.
             */
            app_nav_center_front_tape_was_front_tape_detected = 1U;
            app_nav_center_front_tape_gate_state = APP_NAV_FRONT_TAPE_GATE_WAIT_LEAVE_CURRENT_BLACK;
        }
        else
        {
            app_nav_center_front_tape_was_front_tape_detected = 0U;
            app_nav_center_front_tape_gate_state = APP_NAV_FRONT_TAPE_GATE_ARMED_FOR_BOUNDARY_TAPE;
        }
        return false;

    case APP_NAV_FRONT_TAPE_GATE_WAIT_LEAVE_CURRENT_BLACK:
        if (current_front_tape)
        {
            app_nav_center_front_tape_was_front_tape_detected = 1U;
            return false;
        }

        app_nav_center_front_tape_was_front_tape_detected = 0U;
        app_nav_center_front_tape_gate_state = APP_NAV_FRONT_TAPE_GATE_ARMED_FOR_BOUNDARY_TAPE;
        return false;

    case APP_NAV_FRONT_TAPE_GATE_ARMED_FOR_BOUNDARY_TAPE:
        if (current_front_tape)
        {
            if (app_nav_center_front_tape_was_front_tape_detected == 0U)
            {
                return true;
            }

            app_nav_center_front_tape_was_front_tape_detected = 1U;
        }
        else
        {
            app_nav_center_front_tape_was_front_tape_detected = 0U;
        }
        return false;

    default:
        App_Nav_SetCenterFrontTapeActionTerminal(APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR);
        return false;
    }
}

bool App_Nav_StartCenterByFrontTapeForPivotAction(AppNavFrontTapeProfile front_tape_profile)
{
    if ((front_tape_profile != APP_NAV_FRONT_TAPE_PROFILE_NORMAL_CELL) &&
        (front_tape_profile != APP_NAV_FRONT_TAPE_PROFILE_SPECIAL_CELL))
    {
        app_nav_center_front_tape_action_state = APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR;
        return false;
    }

    /*
     * Ensure this new primitive owns forward motion exclusively.
     */
    App_Nav_ClearAdvanceActionState();
    App_Nav_ClearApproachFrontWallActionState();
    App_Nav_ClearSmoothActionState();
    App_Nav_ClearPivotActionState();

    if (!App_Nav_StartWallFollowAdvance())
    {
        app_nav_center_front_tape_action_state = APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR;
        return false;
    }

    app_nav_center_front_tape_action_state = APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_WALL_FOLLOW;
    app_nav_center_front_tape_profile = front_tape_profile;
    app_nav_center_front_tape_gate_state = APP_NAV_FRONT_TAPE_GATE_INIT;
    app_nav_center_front_tape_action_active = 1U;
    app_nav_center_front_tape_was_front_tape_detected = 0U;
    app_nav_center_front_tape_yaw_hold_started = 0U;

    return true;
}

AppNavCenterFrontTapeActionState App_Nav_TickCenterByFrontTapeForPivotAction(const AppNavInput *input,
                                                                             const AppNavPerception *perception,
                                                                             AppNavOutput *output)
{
    AppNavForwardGuidanceMode guidance_mode;
    bool current_front_tape;
    uint8_t force_yaw_hold;

    if ((input == NULL) || (perception == NULL) || (output == NULL))
    {
        app_nav_center_front_tape_action_state = APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR;
        return app_nav_center_front_tape_action_state;
    }

    App_Nav_ClearOutput(output);

    if (app_nav_center_front_tape_action_state == APP_NAV_CENTER_FRONT_TAPE_ACTION_IDLE)
    {
        return APP_NAV_CENTER_FRONT_TAPE_ACTION_IDLE;
    }

    if ((app_nav_center_front_tape_action_state == APP_NAV_CENTER_FRONT_TAPE_ACTION_DONE_FRONT_TAPE) ||
        (app_nav_center_front_tape_action_state == APP_NAV_CENTER_FRONT_TAPE_ACTION_TIMEOUT) ||
        (app_nav_center_front_tape_action_state == APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR))
    {
        return app_nav_center_front_tape_action_state;
    }

    if (app_nav_center_front_tape_action_active == 0U)
    {
        App_Nav_SetCenterFrontTapeActionTerminal(APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR);
        return app_nav_center_front_tape_action_state;
    }

    /*
     * Use the same discrete floor sensor source used by the existing rear tape
     * gates. No extra debounce/timer is added here; hysteresis belongs to the
     * perception layer that provides floor_front_black.
     */
    current_front_tape = (perception->floor_front_black != 0U);

    if (App_Nav_UpdateCenterFrontTapeGate(current_front_tape))
    {
        App_Nav_SetCenterFrontTapeActionTerminal(APP_NAV_CENTER_FRONT_TAPE_ACTION_DONE_FRONT_TAPE);
        return app_nav_center_front_tape_action_state;
    }

    force_yaw_hold =
        (app_nav_center_front_tape_action_state ==
         APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_YAW_HOLD) ? 1U : 0U;

    if (!App_Nav_ComputeForwardGuidedPwm(input,
                                         perception,
                                         output,
                                         force_yaw_hold,
                                         &app_nav_center_front_tape_yaw_hold_started,
                                         &guidance_mode))
    {
        App_Nav_SetCenterFrontTapeActionTerminal(APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR);
        return app_nav_center_front_tape_action_state;
    }

    if (guidance_mode == APP_NAV_FORWARD_GUIDANCE_YAW_HOLD)
    {
        app_nav_center_front_tape_action_state =
            APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_YAW_HOLD;
    }
    else
    {
        app_nav_center_front_tape_action_state =
            APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_WALL_FOLLOW;
    }

    return app_nav_center_front_tape_action_state;
}

void App_Nav_StopCenterByFrontTapeForPivotAction(void)
{
    app_nav_wall_follow_active = 0U;
    app_nav_straight_active = 0U;
    App_Nav_ClearCenterFrontTapeActionState();
    PID_Reset(&app_nav_advance_pid);
}
