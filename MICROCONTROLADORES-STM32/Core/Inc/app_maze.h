#ifndef INC_APP_MAZE_H_
#define INC_APP_MAZE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"

bool App_Maze_IsValidPose(uint8_t x, uint8_t y, HeadingTypeDef heading);
bool App_Maze_SetRobotPose(uint8_t x,
                           uint8_t y,
                           HeadingTypeDef heading);
void App_Maze_ResetRobotPosition(void);
void App_Maze_ResetState(void);
bool App_Maze_ResetStateWithPose(uint8_t x,
                                 uint8_t y,
                                 HeadingTypeDef heading);
void App_Maze_AdvanceRobotPosition(void);
void App_Maze_UpdateRobotHeading(TurnTypeDef turn_direction);
void App_Maze_MapCurrentCell(bool front_wall_detected, bool right_wall_detected, bool left_wall_detected);

bool App_Maze_MarkCurrentCellSpecial(void);
bool App_Maze_IsCurrentCellSpecial(void);
uint8_t App_Maze_GetCurrentCellData(void);

uint8_t App_Maze_WriteCurrentCellUpdatePayload(uint8_t *buffer);
uint8_t App_Maze_WriteColumnSyncPayload(uint8_t requested_col, uint8_t *buffer);

#endif /* INC_APP_MAZE_H_ */
