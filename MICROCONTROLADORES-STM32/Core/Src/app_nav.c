#include "app_nav.h"

#include <string.h>

static AppNavConfig app_nav_config;
static AppNavDebug app_nav_debug;
static bool app_nav_enabled = false;

static void App_Nav_ClearOutput(AppNavOutput *output)
{
    if (output == NULL)
    {
        return;
    }

    memset(output, 0, sizeof(*output));
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

    (void)app_nav_config;
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
    (void)input;

    App_Nav_ClearOutput(output);

    /*
     * Stub only. The live navigation still runs in app_core.c.
     * Keeping this as a no-op allows both repositories to agree on the API
     * before moving behavior into this module.
     */
    app_nav_debug.pwm_right_cmd = 0;
    app_nav_debug.pwm_left_cmd = 0;

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
