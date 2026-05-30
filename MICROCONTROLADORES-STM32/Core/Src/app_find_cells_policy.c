#include "app_find_cells_policy.h"

#include "app_maze.h"
#include <stddef.h>

#define APP_FIND_CELLS_INVALID_COORD 0xFFU
#define APP_FIND_CELLS_DISTANCE_INF 0xFFU
#define APP_FIND_CELLS_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)

static const AppNavRecommendedAction app_find_cells_relative_actions[APP_MAZE_REL_COUNT] =
{
    APP_NAV_ACTION_GO_FRONT_NAVIGATING,
    APP_NAV_ACTION_SMOOTH_RIGHT,
    APP_NAV_ACTION_SMOOTH_LEFT,
    APP_NAV_ACTION_NONE
};

static uint8_t frontier_distance[APP_FIND_CELLS_CELL_COUNT];
static uint8_t bfs_queue[APP_FIND_CELLS_CELL_COUNT];

static void App_FindCellsPolicy_ClearDecision(AppFindCellsDecision *decision)
{
    if (decision == NULL)
    {
        return;
    }

    decision->action = APP_NAV_ACTION_NONE;
    decision->desired_dir = HEADING_NORTH;
    decision->target_x = APP_FIND_CELLS_INVALID_COORD;
    decision->target_y = APP_FIND_CELLS_INVALID_COORD;
    decision->reason = APP_FIND_CELLS_DECISION_REASON_NONE;
}

static bool App_FindCellsPolicy_IsReachableUnvisitedNeighbor(uint8_t x,
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

    if (!App_Maze_IsKnownOpenEdge(x, y, dir))
    {
        return false;
    }

    if (!App_Maze_GetNeighbor(x, y, dir, &nx, &ny))
    {
        return false;
    }

    if (App_Maze_IsCellVisited(nx, ny))
    {
        return false;
    }

    *neighbor_x = nx;
    *neighbor_y = ny;
    return true;
}

static bool App_FindCellsPolicy_HasOpenNonBackExit(uint8_t x,
                                                   uint8_t y,
                                                   HeadingTypeDef heading)
{
    HeadingTypeDef relative_dirs[APP_MAZE_REL_COUNT];

    App_Maze_BuildRelativeDirections(heading, relative_dirs);

    for (uint8_t i = 0U; i < APP_MAZE_REL_DIRECT_COUNT; i++)
    {
        if (App_Maze_IsKnownOpenEdge(x, y, relative_dirs[i]))
        {
            return true;
        }
    }

    return false;
}

static bool App_FindCellsPolicy_IsFrontierCell(uint8_t x, uint8_t y)
{
    static const HeadingTypeDef dirs[4] =
    {
        HEADING_NORTH,
        HEADING_EAST,
        HEADING_SOUTH,
        HEADING_WEST
    };

    if (!App_Maze_IsCellVisited(x, y))
    {
        return false;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t nx = 0U;
        uint8_t ny = 0U;

        if (!App_Maze_IsKnownOpenEdge(x, y, dirs[i]))
        {
            continue;
        }

        if (!App_Maze_GetNeighbor(x, y, dirs[i], &nx, &ny))
        {
            continue;
        }

        if (!App_Maze_IsCellVisited(nx, ny))
        {
            return true;
        }
    }

    return false;
}

static bool App_FindCellsPolicy_RunFrontierFloodFill(void)
{
    uint16_t queue_head = 0U;
    uint16_t queue_tail = 0U;

    for (uint16_t i = 0U; i < APP_FIND_CELLS_CELL_COUNT; i++)
    {
        frontier_distance[i] = APP_FIND_CELLS_DISTANCE_INF;
        bfs_queue[i] = 0U;
    }

    /*
     * Multi-source BFS:
     * all visited cells that have at least one known-open edge toward an
     * unvisited neighbor start with distance 0.
     */
    for (uint8_t y = 0U; y < MAZE_HEIGHT; y++)
    {
        for (uint8_t x = 0U; x < MAZE_WIDTH; x++)
        {
            if (App_FindCellsPolicy_IsFrontierCell(x, y))
            {
                uint8_t idx = App_Maze_CellIndex(x, y);

                frontier_distance[idx] = 0U;
                bfs_queue[queue_tail] = idx;
                queue_tail++;
            }
        }
    }

    if (queue_tail == 0U)
    {
        return false;
    }

    while (queue_head < queue_tail)
    {
        static const HeadingTypeDef dirs[4] =
        {
            HEADING_NORTH,
            HEADING_EAST,
            HEADING_SOUTH,
            HEADING_WEST
        };

        uint8_t current_idx = bfs_queue[queue_head];
        uint8_t current_x = App_Maze_IndexToX(current_idx);
        uint8_t current_y = App_Maze_IndexToY(current_idx);
        uint8_t current_dist = frontier_distance[current_idx];

        queue_head++;

        if (current_dist >= (APP_FIND_CELLS_DISTANCE_INF - 1U))
        {
            continue;
        }

        for (uint8_t i = 0U; i < 4U; i++)
        {
            uint8_t nx = 0U;
            uint8_t ny = 0U;
            uint8_t neighbor_idx = 0U;

            if (!App_Maze_IsKnownOpenEdge(current_x, current_y, dirs[i]))
            {
                continue;
            }

            if (!App_Maze_GetNeighbor(current_x, current_y, dirs[i], &nx, &ny))
            {
                continue;
            }

            /*
             * Routing to a frontier is safe-only in FIND_CELLS:
             * flood propagation crosses only already visited cells.
             */
            if (!App_Maze_IsCellVisited(nx, ny))
            {
                continue;
            }

            neighbor_idx = App_Maze_CellIndex(nx, ny);

            if (frontier_distance[neighbor_idx] != APP_FIND_CELLS_DISTANCE_INF)
            {
                continue;
            }

            frontier_distance[neighbor_idx] = (uint8_t)(current_dist + 1U);
            bfs_queue[queue_tail] = neighbor_idx;
            queue_tail++;
        }
    }

    return true;
}

static bool App_FindCellsPolicy_SelectRouteStep(uint8_t x,
                                                uint8_t y,
                                                HeadingTypeDef heading,
                                                AppFindCellsDecision *decision_out)
{
    HeadingTypeDef relative_dirs[APP_MAZE_REL_COUNT];
    uint8_t best_cost = APP_FIND_CELLS_DISTANCE_INF;
    uint8_t best_candidate_index = APP_FIND_CELLS_INVALID_COORD;
    uint8_t best_target_x = APP_FIND_CELLS_INVALID_COORD;
    uint8_t best_target_y = APP_FIND_CELLS_INVALID_COORD;

    if (decision_out == NULL)
    {
        return false;
    }

    App_Maze_BuildRelativeDirections(heading, relative_dirs);

    /*
     * Tie-break priority is encoded by the relative slot order:
     * front -> right -> left -> back.
     */
    for (uint8_t i = 0U; i < APP_MAZE_REL_COUNT; i++)
    {
        uint8_t nx = 0U;
        uint8_t ny = 0U;
        uint8_t neighbor_idx = 0U;
        uint8_t neighbor_cost = APP_FIND_CELLS_DISTANCE_INF;

        if (!App_Maze_IsKnownOpenEdge(x, y, relative_dirs[i]))
        {
            continue;
        }

        if (!App_Maze_GetNeighbor(x, y, relative_dirs[i], &nx, &ny))
        {
            continue;
        }

        if (!App_Maze_IsCellVisited(nx, ny))
        {
            continue;
        }

        neighbor_idx = App_Maze_CellIndex(nx, ny);
        neighbor_cost = frontier_distance[neighbor_idx];

        if (neighbor_cost == APP_FIND_CELLS_DISTANCE_INF)
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

    if (best_candidate_index == APP_FIND_CELLS_INVALID_COORD)
    {
        decision_out->reason = APP_FIND_CELLS_DECISION_REASON_NO_FRONTIER;
        return false;
    }

    decision_out->desired_dir = relative_dirs[best_candidate_index];
    decision_out->target_x = best_target_x;
    decision_out->target_y = best_target_y;

    if (app_find_cells_relative_actions[best_candidate_index] == APP_NAV_ACTION_NONE)
    {
    	/*
    	 * The route exists, but the next step is behind the robot.
    	 * Do not use APP_NAV_ACTION_GO_BACK here: that action remains reserved for
    	 * the local dead-end recommendation path. The supervisor handles
    	 * BACKTRACK_REQUIRED by choosing the correct 180° pivot preparation method
    	 * from the current front edge.
    	 */
        decision_out->action = APP_NAV_ACTION_NONE;
        decision_out->reason = APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED;
        return false;
    }

    decision_out->action = app_find_cells_relative_actions[best_candidate_index];
    decision_out->reason = APP_FIND_CELLS_DECISION_REASON_ROUTE_TO_FRONTIER;
    return true;
}

bool App_FindCellsPolicy_Evaluate(AppFindCellsDecision *decision_out)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    HeadingTypeDef heading = HEADING_NORTH;
    HeadingTypeDef relative_dirs[APP_MAZE_REL_COUNT];

    if (decision_out == NULL)
    {
        return false;
    }

    App_FindCellsPolicy_ClearDecision(decision_out);

    if (!App_Maze_GetRobotPose(&x, &y, &heading))
    {
        return false;
    }

    App_Maze_BuildRelativeDirections(heading, relative_dirs);

    /*
     * First priority: immediately enter an unvisited neighbor if it is already
     * reachable from the current decision point.
     *
     * Back is intentionally checked separately after the directly executable
     * front/right/left options because it requires a 180° pivot preparation
     * selected by the supervisor.
     */
    for (uint8_t i = 0U; i < APP_MAZE_REL_DIRECT_COUNT; i++)
    {
        uint8_t target_x = 0U;
        uint8_t target_y = 0U;

        if (App_FindCellsPolicy_IsReachableUnvisitedNeighbor(x,
                                                             y,
                                                             relative_dirs[i],
                                                             &target_x,
                                                             &target_y))
        {
            decision_out->action = app_find_cells_relative_actions[i];
            decision_out->desired_dir = relative_dirs[i];
            decision_out->target_x = target_x;
            decision_out->target_y = target_y;
            decision_out->reason = APP_FIND_CELLS_DECISION_REASON_IMMEDIATE_UNVISITED;
            return true;
        }
    }

    /*
     * If there is no known-open exit in front/right/left, this is a local
     * dead-end from the robot's current heading. Do not report BACKTRACK_REQUIRED
     * here: the local fallback must handle it as APP_NAV_ACTION_GO_BACK, which
     * uses the front-wall approach sequence.
     */
    if (!App_FindCellsPolicy_HasOpenNonBackExit(x, y, heading))
    {
        decision_out->reason = APP_FIND_CELLS_DECISION_REASON_NONE;
        return false;
    }

    /*
     * If the only immediate unvisited neighbor is behind the robot, report it as
     * BACKTRACK_REQUIRED. The supervisor will choose the proper 180° pivot
     * preparation method from the current front edge.
     */
    {
        uint8_t back_target_x = 0U;
        uint8_t back_target_y = 0U;
        HeadingTypeDef back_dir = relative_dirs[APP_MAZE_REL_BACK];

        if (App_FindCellsPolicy_IsReachableUnvisitedNeighbor(x,
                                                             y,
                                                             back_dir,
                                                             &back_target_x,
                                                             &back_target_y))
        {
            decision_out->action = APP_NAV_ACTION_NONE;
            decision_out->desired_dir = back_dir;
            decision_out->target_x = back_target_x;
            decision_out->target_y = back_target_y;
            decision_out->reason = APP_FIND_CELLS_DECISION_REASON_BACKTRACK_REQUIRED;
            return false;
        }
    }

    /*
     * Second priority: route through already visited known-open cells toward
     * the nearest exploration frontier.
     */
    if (!App_FindCellsPolicy_RunFrontierFloodFill())
    {
        decision_out->reason = APP_FIND_CELLS_DECISION_REASON_NO_FRONTIER;
        return false;
    }

    return App_FindCellsPolicy_SelectRouteStep(x, y, heading, decision_out);
}
