#ifndef INC_APP_GO_TO_B_POLICY_H_
#define INC_APP_GO_TO_B_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"
#include "app_nav_types.h"

/*
 * GO_A_TO_B high-level decision policy.
 *
 * This module implements the real target-cell navigation decision for a
 * partially known maze. It plans optimistically: unknown edges are considered
 * traversable, and only known present walls block the route. The supervisor
 * executes exactly one returned step and re-evaluates after each confirmed cell
 * transition.
 *
 * Tie-break rule:
 * - front -> right -> left -> back.
 */

typedef enum
{
    APP_GO_TO_B_DECISION_REASON_NONE = 0,
    APP_GO_TO_B_DECISION_REASON_GOAL_REACHED,
    APP_GO_TO_B_DECISION_REASON_ROUTE_STEP,
    APP_GO_TO_B_DECISION_REASON_BACKTRACK_REQUIRED,
    APP_GO_TO_B_DECISION_REASON_NO_PATH
} AppGoToBDecisionReason;

typedef struct
{
    AppNavRecommendedAction action;
    HeadingTypeDef desired_dir;
    uint8_t target_x;
    uint8_t target_y;
    AppGoToBDecisionReason reason;
} AppGoToBDecision;

bool App_GoToBPolicy_Evaluate(uint8_t goal_x,
                              uint8_t goal_y,
                              AppGoToBDecision *decision_out);

#endif /* INC_APP_GO_TO_B_POLICY_H_ */
