#ifndef INC_APP_ROUTE_PLANNER_H_
#define INC_APP_ROUTE_PLANNER_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze.h"

#define APP_ROUTE_DISTANCE_INF 0xFFU
#define APP_ROUTE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)

typedef enum
{
    APP_ROUTE_TRAVERSAL_KNOWN_OPEN_VISITED_ONLY = 0,
    APP_ROUTE_TRAVERSAL_OPTIMISTIC_UNKNOWN_ALLOWED
} AppRouteTraversalMode;

/*
 * Shared static-workspace BFS planner.
 *
 * Non-reentrant by design:
 * - reset;
 * - add one or more seeds;
 * - run;
 * - query distances before the next reset/run.
 */
void App_RoutePlanner_Reset(void);

bool App_RoutePlanner_AddSeed(uint8_t x,
                              uint8_t y);

bool App_RoutePlanner_CanCross(uint8_t x,
                               uint8_t y,
                               HeadingTypeDef dir,
                               AppRouteTraversalMode mode,
                               uint8_t *neighbor_x,
                               uint8_t *neighbor_y);

bool App_RoutePlanner_Run(AppRouteTraversalMode mode);

uint8_t App_RoutePlanner_GetDistance(uint8_t x,
                                     uint8_t y);

uint8_t App_RoutePlanner_GetDistanceByIndex(uint8_t index);

#endif /* INC_APP_ROUTE_PLANNER_H_ */
