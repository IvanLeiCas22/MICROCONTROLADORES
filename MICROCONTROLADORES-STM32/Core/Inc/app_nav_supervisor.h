#ifndef INC_APP_NAV_SUPERVISOR_H_
#define INC_APP_NAV_SUPERVISOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"
#include "app_nav_types.h"

/*
 * Public last_result values reported through AppNavSupervisorDebug.
 *
 * Keep these values uint8_t-compatible: AppNavSupervisorDebug.last_result is
 * intentionally stored as uint8_t to preserve compact debug layout.
 */
#define APP_NAV_SUPERVISOR_RESULT_OK 0U
#define APP_NAV_SUPERVISOR_RESULT_INVALID_ARGUMENT 1U
#define APP_NAV_SUPERVISOR_RESULT_START_FAILED 2U
#define APP_NAV_SUPERVISOR_RESULT_PRIMITIVE_ERROR 3U
#define APP_NAV_SUPERVISOR_RESULT_UNSUPPORTED_ACTION 4U
#define APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE 5U
#define APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_INCOMPLETE_NO_FRONTIER 6U

/*
 * Mission-level supervisor state.
 *
 * The supervisor owns the FIND_CELLS mission sequence and logical map updates.
 * Primitive-specific states remain in app_nav:
 * - AppNavAdvanceActionState
 * - AppNavSmoothActionState
 * - AppNavPivotActionState
 * - AppNavApproachFrontWallActionState
 */

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

/*
 * High-level action currently requested by the supervisor.
 *
 * This is a compact mission-level action label for debug/telemetry. It is not
 * the full primitive state; use the corresponding app_nav primitive debug for
 * low-level phase details.
 */

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

/*
 * Supervisor mission selector.
 *
 * FIND_CELLS is the currently implemented mission. It explores with the local
 * policy and finishes when three unique CELL_SPECIAL cells have been detected.
 *
 * GO_A_TO_B is reserved and must remain safe: starting it should fail / not
 * move the robot until the mission is implemented.
 */

typedef enum
{
    APP_NAV_SUPERVISOR_MISSION_FIND_CELLS = 0,
    APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B
} AppNavSupervisorMission;

typedef struct
{
    AppNavSupervisorState state;
    AppNavSupervisorAction current_action;

    /*
     * active != 0 means the supervisor is currently executing a mission.
     * When FIND_CELLS completes normally, active becomes 0, state returns to
     * IDLE, and last_result is APP_NAV_SUPERVISOR_RESULT_FIND_CELLS_COMPLETE.
     */
    uint8_t active;

    /*
     * One of APP_NAV_SUPERVISOR_RESULT_*.
     * Kept as uint8_t for compact telemetry/debug layout.
     */
    uint8_t last_result;

    /*
     * Snapshot of the logical maze pose/cell stored by app_maze.
     * maze_cell contains WALL_* / CELL_VISITED / CELL_SPECIAL bits.
     */
    uint8_t maze_x;
    uint8_t maze_y;
    uint8_t maze_heading;
    uint8_t maze_cell;

    /*
     * Number of unique CELL_SPECIAL cells detected during FIND_CELLS.
     */
    uint8_t special_found_count;
} AppNavSupervisorDebug;

void App_NavSupervisor_Init(void);
void App_NavSupervisor_Reset(void);
bool App_NavSupervisor_SetInitialPose(uint8_t x,
                                      uint8_t y,
                                      HeadingTypeDef heading);
bool App_NavSupervisor_ResetWithInitialPose(uint8_t x,
                                            uint8_t y,
                                            HeadingTypeDef heading);
bool App_NavSupervisor_SetMission(AppNavSupervisorMission mission);
AppNavSupervisorMission App_NavSupervisor_GetMission(void);
bool App_NavSupervisor_Start(void);
void App_NavSupervisor_Stop(void);
AppNavSupervisorState App_NavSupervisor_Tick(const AppNavInput *input,
                                             AppNavOutput *output);
void App_NavSupervisor_GetDebug(AppNavSupervisorDebug *debug_out);
bool App_NavSupervisor_IsActive(void);

#endif /* INC_APP_NAV_SUPERVISOR_H_ */
