#ifndef INC_APP_MAZE_H_
#define INC_APP_MAZE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"

/*
 * Portable logical maze map.
 *
 * app_maze owns:
 * - current logical cell x/y;
 * - current logical heading;
 * - visited cells;
 * - known/present walls;
 * - detected CELL_SPECIAL flags.
 *
 * It does not own physical movement. The supervisor calls these functions only
 * after a primitive confirms the corresponding logical event.
 */

/* -------------------------------------------------------------------------- */
/* Pose and state lifecycle                                                    */
/* -------------------------------------------------------------------------- */

bool App_Maze_IsValidPose(uint8_t x, uint8_t y, HeadingTypeDef heading);

bool App_Maze_SetRobotPose(uint8_t x,
                           uint8_t y,
                           HeadingTypeDef heading);

void App_Maze_ResetRobotPosition(void);
void App_Maze_ResetState(void);

bool App_Maze_ResetStateWithPose(uint8_t x,
                                 uint8_t y,
                                 HeadingTypeDef heading);

/* -------------------------------------------------------------------------- */
/* Logical movement                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Advance one logical cell in the current heading.
 *
 * This must be called only after a primitive confirms cell entry, normally by
 * rear floor sensor boundary-tape detection.
 */
void App_Maze_AdvanceRobotPosition(void);

/*
 * Update heading after an in-cell pivot or a confirmed smooth turn.
 */
void App_Maze_UpdateRobotHeading(TurnTypeDef turn_direction);

/* -------------------------------------------------------------------------- */
/* Cell mapping                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Mark the current cell as visited and OR in detected walls relative to the
 * current heading. Opposite wall bits are mirrored into adjacent cells.
 */
void App_Maze_MapCurrentCell(bool front_wall_detected,
                             bool right_wall_detected,
                             bool left_wall_detected);

/*
 * Mark current cell as special.
 *
 * Returns true only when CELL_SPECIAL was newly set. This allows the supervisor
 * to count unique special cells without double-counting repeated detections.
 */
bool App_Maze_MarkCurrentCellSpecial(void);

bool App_Maze_IsCurrentCellSpecial(void);
uint8_t App_Maze_GetCurrentCellData(void);

/* -------------------------------------------------------------------------- */
/* STM32/Qt synchronization payloads                                           */
/* -------------------------------------------------------------------------- */

uint8_t App_Maze_WriteCurrentCellUpdatePayload(uint8_t *buffer);
uint8_t App_Maze_WriteColumnSyncPayload(uint8_t requested_col, uint8_t *buffer);

#endif /* INC_APP_MAZE_H_ */
