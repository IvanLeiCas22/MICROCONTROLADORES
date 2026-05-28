#ifndef INC_APP_MAZE_TYPES_H_
#define INC_APP_MAZE_TYPES_H_

#include <stdint.h>

/*
 * Logical maze coordinate convention used by the portable firmware:
 *
 * - X increases toward HEADING_EAST.
 * - Y increases toward HEADING_NORTH.
 *
 * UI/simulator screen coordinates may use a different visual convention; any
 * conversion/inversion belongs to the adapter/UI layer, not to app_maze.
 */
typedef enum
{
    HEADING_NORTH = 0,
    HEADING_EAST = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST = 3
} HeadingTypeDef;

/*
 * Relative logical turns.
 *
 * Values are chosen so heading update can be computed modulo 4:
 *   new_heading = (heading + turn + 4) % 4
 */
typedef enum
{
    TURN_RIGHT = 1,
    TURN_LEFT = -1,
    TURN_AROUND = 2
} TurnTypeDef;

/*
 * Per-cell bit layout.
 *
 * Each maze cell is stored as one uint8_t:
 *
 * bit 0: north wall present
 * bit 1: south wall present
 * bit 2: east wall present
 * bit 3: west wall present
 * bit 4: visited cell
 * bit 5: detected special cell
 *
 * The full byte is used by STM32/Qt sync, so new bits must remain compatible
 * with the HMI/simulator drawing code.
 */
#define WALL_NORTH 0x01
#define WALL_SOUTH 0x02
#define WALL_EAST 0x04
#define WALL_WEST 0x08
#define CELL_VISITED 0x10
#define CELL_SPECIAL 0x20

/*
 * Known-edge bit layout.
 *
 * These bits are used in a separate internal map, not in the STM32/Qt
 * synchronized cell byte. They indicate that a direction was observed,
 * regardless of whether a wall was present or the edge was open.
 */
#define EDGE_KNOWN_NORTH 0x01
#define EDGE_KNOWN_SOUTH 0x02
#define EDGE_KNOWN_EAST 0x04
#define EDGE_KNOWN_WEST 0x08

#define MAZE_WIDTH 15
#define MAZE_HEIGHT 15

#define APP_MAZE_DEFAULT_START_X 7U
#define APP_MAZE_DEFAULT_START_Y 7U
#define APP_MAZE_DEFAULT_START_HEADING HEADING_NORTH

/*
 * Payload sizes used by the STM32/Qt map synchronization commands.
 */
#define APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE 4U
#define APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE (MAZE_HEIGHT + 4U)

#endif /* INC_APP_MAZE_TYPES_H_ */
