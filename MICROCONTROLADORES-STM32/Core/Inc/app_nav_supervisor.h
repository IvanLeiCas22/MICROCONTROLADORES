#ifndef INC_APP_NAV_SUPERVISOR_H_
#define INC_APP_NAV_SUPERVISOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"
#include "app_nav_types.h"

typedef enum
{
    APP_NAV_SUPERVISOR_IDLE = 0,
    APP_NAV_SUPERVISOR_START_INITIAL_ADVANCE,
    APP_NAV_SUPERVISOR_RUN_INITIAL_ADVANCE,
    APP_NAV_SUPERVISOR_DECIDE,
    APP_NAV_SUPERVISOR_RUN_ADVANCE,
    APP_NAV_SUPERVISOR_RUN_APPROACH_FRONT_WALL_FOR_PIVOT,
    APP_NAV_SUPERVISOR_RUN_SMOOTH_LEFT,
    APP_NAV_SUPERVISOR_RUN_SMOOTH_RIGHT,
    APP_NAV_SUPERVISOR_RUN_PIVOT_180,
    APP_NAV_SUPERVISOR_ERROR
} AppNavSupervisorState;

typedef enum
{
    APP_NAV_SUPERVISOR_ACTION_NONE = 0,
    APP_NAV_SUPERVISOR_ACTION_INITIAL_ADVANCE,
    APP_NAV_SUPERVISOR_ACTION_ADVANCE,
    APP_NAV_SUPERVISOR_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
    APP_NAV_SUPERVISOR_ACTION_SMOOTH_LEFT,
    APP_NAV_SUPERVISOR_ACTION_SMOOTH_RIGHT,
    APP_NAV_SUPERVISOR_ACTION_PIVOT_180
} AppNavSupervisorAction;

typedef struct
{
    AppNavSupervisorState state;
    AppNavSupervisorAction current_action;
    uint8_t active;
    uint8_t last_result;
    uint8_t maze_x;
    uint8_t maze_y;
    uint8_t maze_heading;
    uint8_t maze_cell;
} AppNavSupervisorDebug;

void App_NavSupervisor_Init(void);
void App_NavSupervisor_Reset(void);
bool App_NavSupervisor_SetInitialPose(uint8_t x,
                                      uint8_t y,
                                      HeadingTypeDef heading);
bool App_NavSupervisor_ResetWithInitialPose(uint8_t x,
                                            uint8_t y,
                                            HeadingTypeDef heading);
bool App_NavSupervisor_Start(void);
void App_NavSupervisor_Stop(void);
AppNavSupervisorState App_NavSupervisor_Tick(const AppNavInput *input,
                                             AppNavOutput *output);
void App_NavSupervisor_GetDebug(AppNavSupervisorDebug *debug_out);
bool App_NavSupervisor_IsActive(void);

#endif /* INC_APP_NAV_SUPERVISOR_H_ */
