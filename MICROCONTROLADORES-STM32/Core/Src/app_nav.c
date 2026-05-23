#include "app_nav.h"

#include <string.h>

static AppNavConfig app_nav_config;
static AppNavDebug app_nav_debug;
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

static void App_Nav_UpdatePerceptionShadow(const AppNavInput *input)
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
    App_Nav_ResetDebug();
}

void App_Nav_Reset(void)
{
    app_nav_enabled = false;
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

    App_Nav_UpdatePerceptionShadow(input);

    /*
     * Shadow mode only. The live navigation still runs in app_core.c.
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
