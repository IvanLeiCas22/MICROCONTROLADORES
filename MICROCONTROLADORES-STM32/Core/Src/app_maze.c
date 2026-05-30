#include "app_maze.h"

#include <string.h>

/*
 * Portable logical maze map.
 *
 * This module stores the robot logical pose and the known maze cells. It does
 * not infer physical movement by itself: app_nav_supervisor calls movement
 * update functions only after a primitive confirms the corresponding logical
 * event, normally through rear tape detection.
 *
 * Coordinate convention:
 * - x increases to HEADING_EAST.
 * - y increases to HEADING_NORTH.
 *
 * UI/simulator screen conversions must be handled outside this module.
 */

typedef struct
{
    uint8_t x;
    uint8_t y;
    HeadingTypeDef heading;
} AppMazePosition_t;

/* -------------------------------------------------------------------------- */
/* Internal map state                                                          */
/* -------------------------------------------------------------------------- */

static uint8_t maze_map[MAZE_WIDTH][MAZE_HEIGHT];
static uint8_t maze_known_edges[MAZE_WIDTH][MAZE_HEIGHT];
static AppMazePosition_t current_pos;

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

bool App_Maze_IsValidCell(uint8_t x, uint8_t y)
{
    return ((x < MAZE_WIDTH) && (y < MAZE_HEIGHT));
}

static bool App_Maze_IsValidHeading(HeadingTypeDef heading)
{
    return ((heading == HEADING_NORTH) ||
            (heading == HEADING_EAST) ||
            (heading == HEADING_SOUTH) ||
            (heading == HEADING_WEST));
}

static uint8_t App_Maze_DirectionToWallBit(HeadingTypeDef dir)
{
    switch (dir)
    {
    case HEADING_NORTH:
        return WALL_NORTH;
    case HEADING_EAST:
        return WALL_EAST;
    case HEADING_SOUTH:
        return WALL_SOUTH;
    case HEADING_WEST:
        return WALL_WEST;
    default:
        return 0U;
    }
}

static uint8_t App_Maze_DirectionToKnownBit(HeadingTypeDef dir)
{
    switch (dir)
    {
    case HEADING_NORTH:
        return EDGE_KNOWN_NORTH;
    case HEADING_EAST:
        return EDGE_KNOWN_EAST;
    case HEADING_SOUTH:
        return EDGE_KNOWN_SOUTH;
    case HEADING_WEST:
        return EDGE_KNOWN_WEST;
    default:
        return 0U;
    }
}

HeadingTypeDef App_Maze_GetOppositeDirection(HeadingTypeDef dir)
{
    switch (dir)
    {
    case HEADING_NORTH:
        return HEADING_SOUTH;
    case HEADING_EAST:
        return HEADING_WEST;
    case HEADING_SOUTH:
        return HEADING_NORTH;
    case HEADING_WEST:
        return HEADING_EAST;
    default:
        return HEADING_NORTH;
    }
}

HeadingTypeDef App_Maze_RotateRight(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 1) % 4);
}

HeadingTypeDef App_Maze_RotateLeft(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 3) % 4);
}

uint8_t App_Maze_CellIndex(uint8_t x, uint8_t y)
{
    return (uint8_t)((y * MAZE_WIDTH) + x);
}

uint8_t App_Maze_IndexToX(uint8_t index)
{
    return (uint8_t)(index % MAZE_WIDTH);
}

uint8_t App_Maze_IndexToY(uint8_t index)
{
    return (uint8_t)(index / MAZE_WIDTH);
}

static bool App_Maze_GetNeighborInternal(uint8_t x,
                                         uint8_t y,
                                         HeadingTypeDef dir,
                                         uint8_t *neighbor_x,
                                         uint8_t *neighbor_y)
{
    if ((neighbor_x == NULL) || (neighbor_y == NULL) || !App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    switch (dir)
    {
    case HEADING_NORTH:
        if (y >= (MAZE_HEIGHT - 1U))
        {
            return false;
        }
        *neighbor_x = x;
        *neighbor_y = (uint8_t)(y + 1U);
        return true;

    case HEADING_EAST:
        if (x >= (MAZE_WIDTH - 1U))
        {
            return false;
        }
        *neighbor_x = (uint8_t)(x + 1U);
        *neighbor_y = y;
        return true;

    case HEADING_SOUTH:
        if (y == 0U)
        {
            return false;
        }
        *neighbor_x = x;
        *neighbor_y = (uint8_t)(y - 1U);
        return true;

    case HEADING_WEST:
        if (x == 0U)
        {
            return false;
        }
        *neighbor_x = (uint8_t)(x - 1U);
        *neighbor_y = y;
        return true;

    default:
        return false;
    }
}

static void App_Maze_MarkKnownEdge(uint8_t x, uint8_t y, HeadingTypeDef dir)
{
    uint8_t known_bit = App_Maze_DirectionToKnownBit(dir);

    if ((known_bit == 0U) || !App_Maze_IsValidCell(x, y))
    {
        return;
    }

    maze_known_edges[x][y] |= known_bit;

    /*
     * Mirror the known state into the adjacent cell whenever the neighbor is
     * inside the logical maze. This applies to both open and walled edges.
     */
    uint8_t neighbor_x = 0U;
    uint8_t neighbor_y = 0U;

    if (App_Maze_GetNeighborInternal(x, y, dir, &neighbor_x, &neighbor_y))
    {
        HeadingTypeDef opposite_dir = App_Maze_GetOppositeDirection(dir);
        uint8_t opposite_known_bit = App_Maze_DirectionToKnownBit(opposite_dir);

        if (opposite_known_bit != 0U)
        {
            maze_known_edges[neighbor_x][neighbor_y] |= opposite_known_bit;
        }
    }
}

static void App_Maze_MirrorWallIfNeeded(uint8_t x, uint8_t y, HeadingTypeDef dir)
{
    uint8_t neighbor_x = 0U;
    uint8_t neighbor_y = 0U;

    if (!App_Maze_GetNeighborInternal(x, y, dir, &neighbor_x, &neighbor_y))
    {
        return;
    }

    HeadingTypeDef opposite_dir = App_Maze_GetOppositeDirection(dir);
    uint8_t opposite_wall_bit = App_Maze_DirectionToWallBit(opposite_dir);

    if (opposite_wall_bit != 0U)
    {
        maze_map[neighbor_x][neighbor_y] |= opposite_wall_bit;
    }
}

/* -------------------------------------------------------------------------- */
/* Pose validation and lifecycle                                               */
/* -------------------------------------------------------------------------- */

bool App_Maze_IsValidPose(uint8_t x, uint8_t y, HeadingTypeDef heading)
{
    return (App_Maze_IsValidCell(x, y) && App_Maze_IsValidHeading(heading));
}

bool App_Maze_SetRobotPose(uint8_t x, uint8_t y, HeadingTypeDef heading)
{
    if (!App_Maze_IsValidPose(x, y, heading))
    {
        return false;
    }

    current_pos.x = x;
    current_pos.y = y;
    current_pos.heading = heading;

    return true;
}

void App_Maze_ResetRobotPosition(void)
{
    (void)App_Maze_SetRobotPose(APP_MAZE_DEFAULT_START_X,
                                APP_MAZE_DEFAULT_START_Y,
                                APP_MAZE_DEFAULT_START_HEADING);
}

void App_Maze_ResetState(void)
{
    (void)App_Maze_ResetStateWithPose(APP_MAZE_DEFAULT_START_X,
                                      APP_MAZE_DEFAULT_START_Y,
                                      APP_MAZE_DEFAULT_START_HEADING);
}

bool App_Maze_ResetStateWithPose(uint8_t x, uint8_t y, HeadingTypeDef heading)
{
    memset(maze_map, 0, sizeof(maze_map));
    memset(maze_known_edges, 0, sizeof(maze_known_edges));

    if (!App_Maze_SetRobotPose(x, y, heading))
    {
        App_Maze_ResetRobotPosition();
        return false;
    }

    return true;
}

/* -------------------------------------------------------------------------- */
/* Logical pose updates                                                        */
/* -------------------------------------------------------------------------- */

void App_Maze_AdvanceRobotPosition(void)
{
    switch (current_pos.heading)
    {
    case HEADING_NORTH:
        if (current_pos.y < (MAZE_HEIGHT - 1U))
        {
            current_pos.y++;
        }
        break;

    case HEADING_EAST:
        if (current_pos.x < (MAZE_WIDTH - 1U))
        {
            current_pos.x++;
        }
        break;

    case HEADING_SOUTH:
        if (current_pos.y > 0U)
        {
            current_pos.y--;
        }
        break;

    case HEADING_WEST:
        if (current_pos.x > 0U)
        {
            current_pos.x--;
        }
        break;
    }
}

void App_Maze_UpdateRobotHeading(TurnTypeDef turn_direction)
{
    current_pos.heading = (HeadingTypeDef)((current_pos.heading + turn_direction + 4) % 4);
}

/* -------------------------------------------------------------------------- */
/* Cell mapping and wall mirroring                                             */
/* -------------------------------------------------------------------------- */

void App_Maze_MapCurrentCell(bool front_wall_detected,
                             bool right_wall_detected,
                             bool left_wall_detected)
{
    HeadingTypeDef front_dir = current_pos.heading;
    HeadingTypeDef right_dir = App_Maze_RotateRight(current_pos.heading);
    HeadingTypeDef left_dir = App_Maze_RotateLeft(current_pos.heading);

    uint8_t cell_data = CELL_VISITED;

    /*
     * The three relative directions sensed from the current pose become known
     * edges regardless of whether each edge is open or walled.
     */
    App_Maze_MarkKnownEdge(current_pos.x, current_pos.y, front_dir);
    App_Maze_MarkKnownEdge(current_pos.x, current_pos.y, right_dir);
    App_Maze_MarkKnownEdge(current_pos.x, current_pos.y, left_dir);

    if (front_wall_detected)
    {
        cell_data |= App_Maze_DirectionToWallBit(front_dir);
    }
    if (right_wall_detected)
    {
        cell_data |= App_Maze_DirectionToWallBit(right_dir);
    }
    if (left_wall_detected)
    {
        cell_data |= App_Maze_DirectionToWallBit(left_dir);
    }

    /*
     * OR semantics preserve previously known flags such as CELL_SPECIAL and any
     * walls discovered in earlier passes.
     */
    maze_map[current_pos.x][current_pos.y] |= cell_data;

    /*
     * Mirror present walls into adjacent cells so both sides of a wall stay
     * consistent for map synchronization and future planning.
     */
    if (front_wall_detected)
    {
        App_Maze_MirrorWallIfNeeded(current_pos.x, current_pos.y, front_dir);
    }
    if (right_wall_detected)
    {
        App_Maze_MirrorWallIfNeeded(current_pos.x, current_pos.y, right_dir);
    }
    if (left_wall_detected)
    {
        App_Maze_MirrorWallIfNeeded(current_pos.x, current_pos.y, left_dir);
    }
}

/* -------------------------------------------------------------------------- */
/* Special-cell flags                                                          */
/* -------------------------------------------------------------------------- */

bool App_Maze_MarkCurrentCellSpecial(void)
{
    uint8_t *cell = &maze_map[current_pos.x][current_pos.y];

    if ((*cell & CELL_SPECIAL) != 0U)
    {
        return false;
    }

    *cell |= (CELL_VISITED | CELL_SPECIAL);
    return true;
}

bool App_Maze_IsCurrentCellSpecial(void)
{
    return ((maze_map[current_pos.x][current_pos.y] & CELL_SPECIAL) != 0U);
}

uint8_t App_Maze_GetCurrentCellData(void)
{
    return maze_map[current_pos.x][current_pos.y];
}

/* -------------------------------------------------------------------------- */
/* Read-only map query API for planning                                        */
/* -------------------------------------------------------------------------- */

bool App_Maze_GetRobotPose(uint8_t *x,
                           uint8_t *y,
                           HeadingTypeDef *heading)
{
    if ((x == NULL) || (y == NULL) || (heading == NULL))
    {
        return false;
    }

    *x = current_pos.x;
    *y = current_pos.y;
    *heading = current_pos.heading;

    return true;
}

bool App_Maze_GetCellData(uint8_t x,
                          uint8_t y,
                          uint8_t *cell_out)
{
    if ((cell_out == NULL) || !App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    *cell_out = maze_map[x][y];
    return true;
}

bool App_Maze_IsCellVisited(uint8_t x, uint8_t y)
{
    if (!App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    return ((maze_map[x][y] & CELL_VISITED) != 0U);
}

bool App_Maze_IsCellSpecial(uint8_t x, uint8_t y)
{
    if (!App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    return ((maze_map[x][y] & CELL_SPECIAL) != 0U);
}

bool App_Maze_IsEdgeKnown(uint8_t x,
                          uint8_t y,
                          HeadingTypeDef dir)
{
    uint8_t known_bit = App_Maze_DirectionToKnownBit(dir);

    if ((known_bit == 0U) || !App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    return ((maze_known_edges[x][y] & known_bit) != 0U);
}

bool App_Maze_CellHasWall(uint8_t x,
                          uint8_t y,
                          HeadingTypeDef dir)
{
    uint8_t wall_bit = App_Maze_DirectionToWallBit(dir);

    if ((wall_bit == 0U) || !App_Maze_IsValidCell(x, y))
    {
        return false;
    }

    return ((maze_map[x][y] & wall_bit) != 0U);
}

bool App_Maze_IsKnownOpenEdge(uint8_t x,
                              uint8_t y,
                              HeadingTypeDef dir)
{
    uint8_t neighbor_x = 0U;
    uint8_t neighbor_y = 0U;

    if (!App_Maze_GetNeighborInternal(x, y, dir, &neighbor_x, &neighbor_y))
    {
        return false;
    }

    if (!App_Maze_IsEdgeKnown(x, y, dir))
    {
        return false;
    }

    if (App_Maze_CellHasWall(x, y, dir))
    {
        return false;
    }

    return true;
}

bool App_Maze_GetNeighbor(uint8_t x,
                          uint8_t y,
                          HeadingTypeDef dir,
                          uint8_t *neighbor_x,
                          uint8_t *neighbor_y)
{
    return App_Maze_GetNeighborInternal(x, y, dir, neighbor_x, neighbor_y);
}

/* -------------------------------------------------------------------------- */
/* STM32/Qt synchronization payloads                                           */
/* -------------------------------------------------------------------------- */

uint8_t App_Maze_WriteCurrentCellUpdatePayload(uint8_t *buffer)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    buffer[0] = current_pos.x;
    buffer[1] = current_pos.y;
    buffer[2] = maze_map[current_pos.x][current_pos.y];
    buffer[3] = (uint8_t)current_pos.heading;

    return APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE;
}

uint8_t App_Maze_WriteColumnSyncPayload(uint8_t requested_col, uint8_t *buffer)
{
    if (buffer == NULL)
    {
        return 0U;
    }

    if (requested_col >= MAZE_WIDTH)
    {
        return 0U;
    }

    uint8_t payload_len = 0U;
    buffer[payload_len++] = requested_col;

    for (uint8_t i = 0U; i < MAZE_HEIGHT; i++)
    {
        buffer[payload_len++] = maze_map[requested_col][i];
    }

    buffer[payload_len++] = current_pos.x;
    buffer[payload_len++] = current_pos.y;
    buffer[payload_len++] = (uint8_t)current_pos.heading;

    return payload_len;
}
