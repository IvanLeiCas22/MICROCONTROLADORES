#include "app_go_to_b_policy.h"

#include "app_maze.h"
#include <stddef.h>

#define APP_GO_TO_B_INVALID_COORD 0xFFU
#define APP_GO_TO_B_DISTANCE_INF 0xFFU
#define APP_GO_TO_B_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)

typedef struct
{
    HeadingTypeDef dir;
    AppNavRecommendedAction action;
} AppGoToBCandidate;

static uint8_t goal_distance[APP_GO_TO_B_CELL_COUNT];
static uint8_t bfs_queue[APP_GO_TO_B_CELL_COUNT];

static HeadingTypeDef App_GoToBPolicy_RotateRight(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 1) % 4);
}

static HeadingTypeDef App_GoToBPolicy_RotateLeft(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 3) % 4);
}

static HeadingTypeDef App_GoToBPolicy_GetOppositeDirection(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 2) % 4);
}

static uint8_t App_GoToBPolicy_CellIndex(uint8_t x, uint8_t y)
{
    return (uint8_t)((y * MAZE_WIDTH) + x);
}

static uint8_t App_GoToBPolicy_IndexToX(uint8_t index)
{
    return (uint8_t)(index % MAZE_WIDTH);
}

static uint8_t App_GoToBPolicy_IndexToY(uint8_t index)
{
    return (uint8_t)(index / MAZE_WIDTH);
}

static bool App_GoToBPolicy_IsValidCell(uint8_t x, uint8_t y)
{
    return ((x < MAZE_WIDTH) && (y < MAZE_HEIGHT));
}

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

static bool App_GoToBPolicy_CanCrossOptimistic(uint8_t x,
                                               uint8_t y,
                                               HeadingTypeDef dir,
                                               uint8_t *neighbor_x,
                                               uint8_t *neighbor_y)
{
    uint8_t nx = 0U;
    uint8_t ny = 0U;

    if ((neighbor_x == NULL) || (neighbor_y == NULL))
    {
        return false;
    }

    if (!App_Maze_GetNeighbor(x, y, dir, &nx, &ny))
    {
        return false;
    }

    if (App_Maze_CellHasWall(x, y, dir))
    {
        return false;
    }

    *neighbor_x = nx;
    *neighbor_y = ny;
    return true;
}

static bool App_GoToBPolicy_RunOptimisticFloodFill(uint8_t goal_x,
                                                   uint8_t goal_y)
{
    uint16_t queue_head = 0U;
    uint16_t queue_tail = 0U;

    static const HeadingTypeDef dirs[4] =
    {
        HEADING_NORTH,
        HEADING_EAST,
        HEADING_SOUTH,
        HEADING_WEST
    };

    if (!App_GoToBPolicy_IsValidCell(goal_x, goal_y))
    {
        return false;
    }

    for (uint16_t i = 0U; i < APP_GO_TO_B_CELL_COUNT; i++)
    {
        goal_distance[i] = APP_GO_TO_B_DISTANCE_INF;
        bfs_queue[i] = 0U;
    }

    goal_distance[App_GoToBPolicy_CellIndex(goal_x, goal_y)] = 0U;
    bfs_queue[queue_tail] = App_GoToBPolicy_CellIndex(goal_x, goal_y);
    queue_tail++;

    while (queue_head < queue_tail)
    {
        uint8_t current_idx = bfs_queue[queue_head];
        uint8_t current_x = App_GoToBPolicy_IndexToX(current_idx);
        uint8_t current_y = App_GoToBPolicy_IndexToY(current_idx);
        uint8_t current_dist = goal_distance[current_idx];

        queue_head++;

        if (current_dist >= (APP_GO_TO_B_DISTANCE_INF - 1U))
        {
            continue;
        }

        for (uint8_t i = 0U; i < 4U; i++)
        {
            uint8_t nx = 0U;
            uint8_t ny = 0U;
            uint8_t neighbor_idx = 0U;

            if (!App_GoToBPolicy_CanCrossOptimistic(current_x,
                                                    current_y,
                                                    dirs[i],
                                                    &nx,
                                                    &ny))
            {
                continue;
            }

            neighbor_idx = App_GoToBPolicy_CellIndex(nx, ny);

            if (goal_distance[neighbor_idx] != APP_GO_TO_B_DISTANCE_INF)
            {
                continue;
            }

            goal_distance[neighbor_idx] = (uint8_t)(current_dist + 1U);
            bfs_queue[queue_tail] = neighbor_idx;
            queue_tail++;
        }
    }

    return true;
}

static bool App_GoToBPolicy_SelectRouteStep(uint8_t x,
                                            uint8_t y,
                                            HeadingTypeDef heading,
                                            AppGoToBDecision *decision_out)
{
    const AppGoToBCandidate candidates[4] =
    {
        {heading, APP_NAV_ACTION_GO_FRONT_NAVIGATING},
        {App_GoToBPolicy_RotateRight(heading), APP_NAV_ACTION_SMOOTH_RIGHT},
        {App_GoToBPolicy_RotateLeft(heading), APP_NAV_ACTION_SMOOTH_LEFT},
        {App_GoToBPolicy_GetOppositeDirection(heading), APP_NAV_ACTION_NONE}
    };

    uint8_t current_idx = App_GoToBPolicy_CellIndex(x, y);
    uint8_t current_cost = goal_distance[current_idx];
    uint8_t best_cost = APP_GO_TO_B_DISTANCE_INF;
    uint8_t best_candidate_index = APP_GO_TO_B_INVALID_COORD;
    uint8_t best_target_x = APP_GO_TO_B_INVALID_COORD;
    uint8_t best_target_y = APP_GO_TO_B_INVALID_COORD;

    if (decision_out == NULL)
    {
        return false;
    }

    if (current_cost == APP_GO_TO_B_DISTANCE_INF)
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t nx = 0U;
        uint8_t ny = 0U;
        uint8_t neighbor_idx = 0U;
        uint8_t neighbor_cost = APP_GO_TO_B_DISTANCE_INF;

        if (!App_GoToBPolicy_CanCrossOptimistic(x,
                                                y,
                                                candidates[i].dir,
                                                &nx,
                                                &ny))
        {
            continue;
        }

        neighbor_idx = App_GoToBPolicy_CellIndex(nx, ny);
        neighbor_cost = goal_distance[neighbor_idx];

        if (neighbor_cost == APP_GO_TO_B_DISTANCE_INF)
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

    decision_out->desired_dir = candidates[best_candidate_index].dir;
    decision_out->target_x = best_target_x;
    decision_out->target_y = best_target_y;

    if (candidates[best_candidate_index].action == APP_NAV_ACTION_NONE)
    {
        decision_out->action = APP_NAV_ACTION_NONE;
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_BACKTRACK_REQUIRED;
        return false;
    }

    decision_out->action = candidates[best_candidate_index].action;
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

    if (!App_GoToBPolicy_IsValidCell(goal_x, goal_y))
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

    if (!App_GoToBPolicy_RunOptimisticFloodFill(goal_x, goal_y))
    {
        decision_out->reason = APP_GO_TO_B_DECISION_REASON_NO_PATH;
        return false;
    }

    return App_GoToBPolicy_SelectRouteStep(x, y, heading, decision_out);
}
