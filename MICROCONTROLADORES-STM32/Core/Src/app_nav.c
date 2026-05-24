#include "app_nav.h"
#include "pid_controller.h"

#include <string.h>

static AppNavConfig app_nav_config;
static AppNavDebug app_nav_debug;
static PID_Controller_t app_nav_advance_pid;
static bool app_nav_enabled = false;

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

static void App_Nav_ApplyAdvancePidConfig(uint8_t reset_state)
{
    PID_Config_t cfg = {
        .kp = app_nav_config.advance_pid_kp_q16,
        .ki = app_nav_config.advance_pid_ki_q16,
        .kd = app_nav_config.advance_pid_kd_q16,
        .out_min = -INT_TO_FIXED(app_nav_config.advance_pid_output_limit_pwm),
        .out_max = INT_TO_FIXED(app_nav_config.advance_pid_output_limit_pwm),
    };

    PID_ApplyConfig(&app_nav_advance_pid, &cfg, (reset_state != 0U));
}

static void App_Nav_ClearOutput(AppNavOutput *output)
{
    if (output == NULL)
    {
        return;
    }

    memset(output, 0, sizeof(*output));
}

static uint8_t App_Nav_BoolToU8(bool value)
{
    return value ? 1U : 0U;
}

static bool App_Nav_DetectLowWithHysteresis(uint16_t value,
                                            uint16_t threshold,
                                            uint16_t hysteresis,
                                            uint8_t was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected != 0U)
    {
        return ((uint32_t)value < release_threshold);
    }

    return (value < threshold);
}

static bool App_Nav_DetectFrontWallWithHysteresis(uint16_t left_value,
                                                  uint16_t right_value,
                                                  uint16_t threshold,
                                                  uint16_t hysteresis,
                                                  uint8_t was_detected)
{
    uint32_t release_threshold = (uint32_t)threshold + (uint32_t)hysteresis;

    if (was_detected != 0U)
    {
        return (((uint32_t)left_value < release_threshold) &&
                ((uint32_t)right_value < release_threshold));
    }

    return ((left_value < threshold) && (right_value < threshold));
}

static void App_Nav_UpdatePerception(const AppNavInput *input)
{
    uint16_t floor_front_adc = input->adc_filtered[APP_NAV_ADC_FLOOR_FRONT_CH];
    uint16_t floor_rear_adc = input->adc_filtered[APP_NAV_ADC_FLOOR_REAR_CH];

    app_nav_debug.floor_front_adc = floor_front_adc;
    app_nav_debug.floor_rear_adc = floor_rear_adc;
    app_nav_debug.dist_front_left_mm = input->dist_front_left_mm;
    app_nav_debug.dist_front_right_mm = input->dist_front_right_mm;
    app_nav_debug.dist_left_lat_mm = input->dist_left_lat_mm;
    app_nav_debug.dist_right_lat_mm = input->dist_right_lat_mm;
    app_nav_debug.dist_diagonal_left_mm = input->dist_diagonal_left_mm;
    app_nav_debug.dist_diagonal_right_mm = input->dist_diagonal_right_mm;

    app_nav_debug.wall_front = App_Nav_BoolToU8(
        App_Nav_DetectFrontWallWithHysteresis(input->dist_front_left_mm,
                                              input->dist_front_right_mm,
                                              app_nav_config.wall_threshold_mm_front,
                                              app_nav_config.wall_hysteresis_mm,
                                              app_nav_debug.wall_front));

    app_nav_debug.wall_left = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(input->dist_left_lat_mm,
                                        app_nav_config.wall_threshold_mm_side,
                                        app_nav_config.wall_hysteresis_mm,
                                        app_nav_debug.wall_left));

    app_nav_debug.wall_right = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(input->dist_right_lat_mm,
                                        app_nav_config.wall_threshold_mm_side,
                                        app_nav_config.wall_hysteresis_mm,
                                        app_nav_debug.wall_right));

    app_nav_debug.wall_diag_left = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(input->dist_diagonal_left_mm,
                                        app_nav_config.wall_threshold_mm_diagonal,
                                        app_nav_config.wall_hysteresis_mm,
                                        app_nav_debug.wall_diag_left));

    app_nav_debug.wall_diag_right = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(input->dist_diagonal_right_mm,
                                        app_nav_config.wall_threshold_mm_diagonal,
                                        app_nav_config.wall_hysteresis_mm,
                                        app_nav_debug.wall_diag_right));

    app_nav_debug.floor_front_black = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(floor_front_adc,
                                        app_nav_config.tape_detection_threshold_adc,
                                        app_nav_config.tape_hysteresis_adc,
                                        app_nav_debug.floor_front_black));

    app_nav_debug.floor_rear_black = App_Nav_BoolToU8(
        App_Nav_DetectLowWithHysteresis(floor_rear_adc,
                                        app_nav_config.tape_detection_threshold_adc,
                                        app_nav_config.tape_hysteresis_adc,
                                        app_nav_debug.floor_rear_black));
}

static void App_Nav_ResetDebug(void)
{
    memset(&app_nav_debug, 0, sizeof(app_nav_debug));
    app_nav_debug.mode = APP_NAV_MODE_IDLE;
    app_nav_debug.state = APP_NAV_STATE_IDLE;
    app_nav_debug.previous_state = APP_NAV_STATE_IDLE;
    app_nav_debug.yaw_target_deg = APP_NAV_YAW_TARGET_UNAVAILABLE;
}

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

    app_nav_enabled = false;
    App_Nav_ApplyAdvancePidConfig(1U);
    App_Nav_ResetDebug();
}

void App_Nav_SetConfig(const AppNavConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    app_nav_config = *config;
    App_Nav_ApplyAdvancePidConfig(0U);
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
    app_nav_enabled = false;
    PID_Reset(&app_nav_advance_pid);
    App_Nav_ResetDebug();
}

void App_Nav_StartFindCells(void)
{
    app_nav_enabled = true;
    app_nav_debug.mode = APP_NAV_MODE_FIND_CELLS;
    app_nav_debug.previous_state = app_nav_debug.state;
    app_nav_debug.state = APP_NAV_STATE_NAVIGATING;
    app_nav_debug.last_transition_reason = APP_NAV_TRANSITION_START_FIND_CELLS;
    app_nav_debug.transition_sequence++;
}

void App_Nav_Stop(void)
{
    app_nav_enabled = false;
    app_nav_debug.mode = APP_NAV_MODE_IDLE;
    app_nav_debug.previous_state = app_nav_debug.state;
    app_nav_debug.state = APP_NAV_STATE_IDLE;
    app_nav_debug.last_transition_reason = APP_NAV_TRANSITION_STOP_TO_MENU;
    app_nav_debug.pwm_right_cmd = 0;
    app_nav_debug.pwm_left_cmd = 0;
    app_nav_debug.transition_sequence++;
}

void App_Nav_Tick(const AppNavInput *input, AppNavOutput *output)
{
    App_Nav_ClearOutput(output);
    app_nav_debug.pwm_right_cmd = 0;
    app_nav_debug.pwm_left_cmd = 0;

    if ((input == NULL) || (output == NULL))
    {
        return;
    }

    App_Nav_UpdatePerception(input);

    /*
     * Perception-only mode. The live navigation still runs in app_core.c.
     * This portable layer observes sensors and exports debug perception
     * without owning motors or state-machine behavior yet.
     */
    output->right_motor_pwm = 0;
    output->left_motor_pwm = 0;

    if (!app_nav_enabled)
    {
        app_nav_debug.mode = APP_NAV_MODE_IDLE;
        app_nav_debug.state = APP_NAV_STATE_IDLE;
    }
}

void App_Nav_GetDebug(AppNavDebug *debug_out)
{
    if (debug_out == NULL)
    {
        return;
    }

    *debug_out = app_nav_debug;
}

bool App_Nav_RecommendAction(uint32_t random_value,
                              AppNavRecommendedAction *action_out)
{
    uint8_t available_options = APP_NAV_OPTION_BACK_MASK;
    uint8_t valid_options[3] = {0U, 0U, 0U};
    uint8_t valid_count = 0U;
    uint8_t choice;
    AppNavRecommendedAction action = APP_NAV_ACTION_NONE;

    if (action_out == NULL)
    {
        return false;
    }

    if (app_nav_debug.wall_front == 0U)
    {
        available_options |= APP_NAV_OPTION_FRONT_MASK;
        valid_options[valid_count] = APP_NAV_DECISION_FRONT;
        valid_count++;
    }

    if (app_nav_debug.wall_right == 0U)
    {
        available_options |= APP_NAV_OPTION_RIGHT_MASK;
        valid_options[valid_count] = APP_NAV_DECISION_RIGHT;
        valid_count++;
    }

    if (app_nav_debug.wall_left == 0U)
    {
        available_options |= APP_NAV_OPTION_LEFT_MASK;
        valid_options[valid_count] = APP_NAV_DECISION_LEFT;
        valid_count++;
    }

    app_nav_debug.available_options_mask = available_options;
    app_nav_debug.valid_option_count = valid_count;

    if (available_options == APP_NAV_OPTION_BACK_MASK)
    {
        action = APP_NAV_ACTION_GO_BACK;
    }
    else if (valid_count == 0U)
    {
        *action_out = APP_NAV_ACTION_NONE;
        app_nav_debug.last_recommended_action = (uint8_t)APP_NAV_ACTION_NONE;
        return false;
    }
    else
    {
        choice = valid_options[random_value % valid_count];

        if (choice == APP_NAV_DECISION_LEFT)
        {
            action = APP_NAV_ACTION_SMOOTH_LEFT;
        }
        else if (choice == APP_NAV_DECISION_RIGHT)
        {
            action = APP_NAV_ACTION_SMOOTH_RIGHT;
        }
        else
        {
            if ((app_nav_debug.wall_left != 0U) || (app_nav_debug.wall_right != 0U))
            {
                action = APP_NAV_ACTION_GO_FRONT_NAVIGATING;
            }
            else
            {
                action = APP_NAV_ACTION_GO_FRONT_STRAIGHT;
            }
        }
    }

    *action_out = action;
    app_nav_debug.last_recommended_action = (uint8_t)action;
    return true;
}
