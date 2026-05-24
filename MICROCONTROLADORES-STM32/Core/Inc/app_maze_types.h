#ifndef INC_APP_MAZE_TYPES_H_
#define INC_APP_MAZE_TYPES_H_

#include <stdint.h>

typedef enum
{
    HEADING_NORTH = 0,
    HEADING_EAST = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST = 3
} HeadingTypeDef;

typedef enum
{
    TURN_RIGHT = 1,
    TURN_LEFT = -1,
    TURN_AROUND = 2
} TurnTypeDef;

#define WALL_NORTH 0x01
#define WALL_SOUTH 0x02
#define WALL_EAST 0x04
#define WALL_WEST 0x08
#define CELL_VISITED 0x10
#define CELL_SPECIAL 0x20

#define MAZE_WIDTH 15
#define MAZE_HEIGHT 15

#define APP_MAZE_CELL_UPDATE_PAYLOAD_SIZE 4U
#define APP_MAZE_COLUMN_SYNC_PAYLOAD_SIZE (MAZE_HEIGHT + 4U)

#endif /* INC_APP_MAZE_TYPES_H_ */
