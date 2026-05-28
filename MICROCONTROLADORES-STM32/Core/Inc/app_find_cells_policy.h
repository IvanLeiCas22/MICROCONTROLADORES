#ifndef INC_APP_FIND_CELLS_POLICY_H_
#define INC_APP_FIND_CELLS_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"
#include "app_nav_types.h"

/*
 * FIND_CELLS high-level decision policy.
 *
 * This module is production navigation logic, not shadow/debug code.
 *
 * Current implementation stage:
 * - prefer immediate unvisited neighbors in relative priority order:
 *   front -> right -> left;
 * - if no immediate unvisited neighbor is available, route toward the nearest
 *   exploration frontier using flood fill implemented as multi-source BFS;
 * - if the best next step is behind the robot, report BACKTRACK_REQUIRED and
 *   return false until the future CENTER_BY_FRONT_TAPE_FOR_PIVOT primitive is
 *   available.
 *
 * Tie-break rule:
 * - front -> right -> left -> back.
 */

typedef enum
{
    APP_FIND_CELLS_DECISION_REASON_NONE = 0,
    APP_FIND_CELLS_DECISION_REASON_IMMEDIATE_UNVISITED,
    APP_FIND_CELLS_DECISION_REASON_ROUTE_TO_FRONTIER,
    APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED,
    APP_FIND_CELLS_DECISION_REASON_NO_FRONTIER
} AppFindCellsDecisionReason;

typedef struct
{
    AppNavRecommendedAction action;
    HeadingTypeDef desired_dir;
    uint8_t target_x;
    uint8_t target_y;
    AppFindCellsDecisionReason reason;
} AppFindCellsDecision;

/*
 * Evaluate the current FIND_CELLS decision.
 *
 * Returns true only when this policy found a concrete executable action.
 * Returns false when the supervisor should use the existing local fallback.
 *
 * Important:
 * - BACKTRACK_REQUIRED is intentionally not executable yet, because the current
 *   APP_NAV_ACTION_GO_BACK path is tied to the dead-end front-wall approach.
 */
bool App_FindCellsPolicy_Evaluate(AppFindCellsDecision *decision_out);

#endif /* INC_APP_FIND_CELLS_POLICY_H_ */
