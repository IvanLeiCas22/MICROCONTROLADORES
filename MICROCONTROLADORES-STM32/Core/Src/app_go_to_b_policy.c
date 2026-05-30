#include "app_go_to_b_policy.h"

#include "app_maze.h"
#include "app_route_planner.h"
#include <stddef.h>

#define APP_GO_TO_B_INVALID_COORD 0xFFU

static const AppNavRecommendedAction app_go_to_b_relative_actions[APP_MAZE_REL_COUNT] =
{
    APP_NAV_ACTION_GO_FRONT_NAVIGATING,
    APP_NAV_ACTION_SMOOTH_RIGHT,
    APP_NAV_ACTION_SMOOTH_LEFT,
    APP_NAV_ACTION_NONE
};

static void App_GoToBPolicy_ClearDecision(AppGoToBDecision *decision)
{
    if (decision == NULL)
    {
        return;
    }

    decision->action = APP_NAV_ACTION_NONE;
    decision->desired_dir = HEADING_NORTH;
    decision->target_x = APP_GO_TO_B_INVALID_COORD;
    decision->target_y = APP_GO_TO_B_INVALID_COORD;
    decision->reason = APP_GO_TO_B_DECISION_REASON_NONE;
}

static bool App_GoToBPolicy_SelectRouteStep(uint8_t x,
                                            uint8_t y,
                                            HeadingTypeDef heading,
                                            AppGoToBDecision *decision_out)
{
    HeadingTypeDef relative_dirs[APP_MAZE_REL_COUNT];
    uint8_t current_idx = App_Maze_CellIndex(x, y);
    uint8_t current_cost = App_RoutePlanner_GetDistanceByIndex(current_idx);
    uint8_t best_cost = APP_ROUTE_DISTANCE_INF;
    uint8_t best_candidate_index = APP_GO_TO_B_INVALID_COORD;
    uint8_t best_target_x = APP_GO_TO_B_INVALID_COORD;
    uint8_t best_target_y = APP_GO_TO_B_INVALID_COORD;

    if (decision_out == NULL)
    {
        return false;
    }

    if (current_cost == APP_ROUTE_DISTANCE_INF)
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    App_Maze_BuildRelativeDirections(heading, relative_dirs);

    for (uint8_t i = 0U; i < APP_MAZE_REL_COUNT; i++)
    {
        uint8_t nx = 0U;
        uint8_t ny = 0U;
        uint8_t neighbor_idx = 0U;
        uint8_t neighbor_cost = APP_ROUTE_DISTANCE_INF;

        if (!App_RoutePlanner_CanCross(x,
                                        y,
                                        relative_dirs[i],
                                        APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED,
                                        &nx,
                                        &ny))
        {
            continue;
        }

        neighbor_idx = App_Maze_CellIndex(nx, ny);
        neighbor_cost = App_RoutePlanner_GetDistanceByIndex(neighbor_idx);

        if (neighbor_cost == APP_ROUTE_DISTANCE_INF)
        {
            continue;
        }

        if (neighbor_cost >= current_cost)
        {
            continue;
        }

        if (neighbor_cost < best_cost)
        {
            best_cost = neighbor_cost;
            best_candidate_index = i;
            best_target_x = nx;
            best_target_y = ny;
        }
    }

    if (best_candidate_index == APP_GO_TO_B_INVALID_COORD)
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    decision_out->desired_dir = relative_dirs[best_candidate_index];
    decision_out->target_x = best_target_x;
    decision_out->target_y = best_target_y;

    if (app_go_to_b_relative_actions[best_candidate_index] == APP_NAV_ACTION_NONE)
    {
        decision_out->action = APP_NAV_ACTION_NONE;
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_BACKTRACK_REQUIRED;
        return false;
    }

    decision_out->action = app_go_to_b_relative_actions[best_candidate_index];
    decision_out->reason = APP_GO_TO_B_DECISION_REASON_ROUTE_STEP;
    return true;
}

bool App_GoToBPolicy_Evaluate(uint8_t goal_x,
                              uint8_t goal_y,
                              AppGoToBDecision *decision_out)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    HeadingTypeDef heading = HEADING_NORTH;

    if (decision_out == NULL)
    {
        return false;
    }

    App_GoToBPolicy_ClearDecision(decision_out);

    if (!App_Maze_IsValidCell(goal_x, goal_y))
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    if (!App_Maze_GetRobotPose(&x, &y, &heading))
    {
        return false;
    }

    if ((x == goal_x) && (y == goal_y))
    {
        decision_out->target_x = goal_x;
        decision_out->target_y = goal_y;
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_GOAL_REACHED;
        return true;
    }

    App_RoutePlanner_Reset();

    if (!App_RoutePlanner_AddSeed(goal_x, goal_y))
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    if (!App_RoutePlanner_Run(APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED))
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    return App_GoToBPolicy_SelectRouteStep(x, y, heading, decision_out);
}
