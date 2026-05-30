#include "app_route_planner.h"

#include <stddef.h>

static uint8_t route_distance[APP_ROUTE_CELL_COUNT];
static uint8_t route_queue[APP_ROUTE_CELL_COUNT];
static uint16_t route_queue_head;
static uint16_t route_queue_tail;

void App_RoutePlanner_Reset(void)
{
    for (uint16_t i = 0U; i < APP_ROUTE_CELL_COUNT; i++)
    {
        route_distance[i] = APP_ROUTE_DISTANCE_INF;
        route_queue[i] = 0U;
    }

    route_queue_head = 0U;
    route_queue_tail = 0U;
}

bool App_RoutePlanner_AddSeed(uint8_t x,
                              uint8_t y)
{
    uint8_t idx = 0U;

    if (!App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    idx = App_Maze_CellIndex(x, y);

    if (route_distance[idx] == 0U)
    {
        return true;
    }

    if (route_queue_tail >= APP_ROUTE_CELL_COUNT)
    {
        return false;
    }

    route_distance[idx] = 0U;
    route_queue[route_queue_tail] = idx;
    route_queue_tail++;

    return true;
}

bool App_RoutePlanner_CanCross(uint8_t x,
                               uint8_t y,
                               HeadingTypeDef dir,
                               AppRouteTraversalMode mode,
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

    switch (mode)
    {
    case APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY:
        if (!App_Maze_IsKnownOpenEdge(x, y, dir))
        {
            return false;
        }

        if (!App_Maze_IsCellVisited(nx, ny))
        {
            return false;
        }
        break;

    case APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED:
        if (App_Maze_CellHasWall(x, y, dir))
        {
            return false;
        }
        break;

    default:
        return false;
    }

    *neighbor_x = nx;
    *neighbor_y = ny;
    return true;
}

bool App_RoutePlanner_Run(AppRouteTraversalMode mode)
{
    static const HeadingTypeDef dirs[4] =
    {
        HEADING_NORTH,
        HEADING_EAST,
        HEADING_SOUTH,
        HEADING_WEST
    };

    if (route_queue_tail == 0U)
    {
        return false;
    }

    route_queue_head = 0U;

    while (route_queue_head < route_queue_tail)
    {
        uint8_t current_idx = route_queue[route_queue_head];
        uint8_t current_x = App_Maze_IndexToX(current_idx);
        uint8_t current_y = App_Maze_IndexToY(current_idx);
        uint8_t current_dist = route_distance[current_idx];

        route_queue_head++;

        if (current_dist >= (APP_ROUTE_DISTANCE_INF - 1U))
        {
            continue;
        }

        for (uint8_t i = 0U; i < 4U; i++)
        {
            uint8_t nx = 0U;
            uint8_t ny = 0U;
            uint8_t neighbor_idx = 0U;

            if (!App_RoutePlanner_CanCross(current_x,
                                           current_y,
                                           dirs[i],
                                           mode,
                                           &nx,
                                           &ny))
            {
                continue;
            }

            neighbor_idx = App_Maze_CellIndex(nx, ny);

            if (route_distance[neighbor_idx] != APP_ROUTE_DISTANCE_INF)
            {
                continue;
            }

            if (route_queue_tail >= APP_ROUTE_CELL_COUNT)
            {
                return false;
            }

            route_distance[neighbor_idx] = (uint8_t)(current_dist + 1U);
            route_queue[route_queue_tail] = neighbor_idx;
            route_queue_tail++;
        }
    }

    return true;
}

uint8_t App_RoutePlanner_GetDistance(uint8_t x,
                                     uint8_t y)
{
    if (!App_Maze_IsValidCell(x, y))
    {
        return APP_ROUTE_DISTANCE_INF;
    }

    return App_RoutePlanner_GetDistanceByIndex(App_Maze_CellIndex(x, y));
}

uint8_t App_RoutePlanner_GetDistanceByIndex(uint8_t index)
{
    if (index >= APP_ROUTE_CELL_COUNT)
    {
        return APP_ROUTE_DISTANCE_INF;
    }

    return route_distance[index];
}
