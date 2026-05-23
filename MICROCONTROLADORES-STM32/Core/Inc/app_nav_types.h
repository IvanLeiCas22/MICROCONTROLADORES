#ifndef INC_APP_NAV_TYPES_H_
#define INC_APP_NAV_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#define APP_NAV_ADC_CHANNEL_COUNT 8U
#define APP_NAV_YAW_TARGET_UNAVAILABLE ((int16_t)32767)

typedef enum
{
    APP_NAV_MODE_IDLE = 0,
    APP_NAV_MODE_FIND_CELLS,
    APP_NAV_MODE_MANUAL_CONTROL,
    APP_NAV_MODE_DRIVE_STRAIGHT,
    APP_NAV_MODE_GO_TO_B
} AppNavMode;

typedef enum
{
    APP_NAV_STATE_IDLE = 0,
    APP_NAV_STATE_NAVIGATING,
    APP_NAV_STATE_BRAKING,
    APP_NAV_STATE_DECIDING,
    APP_NAV_STATE_TURNING_LEFT,
    APP_NAV_STATE_TURNING_RIGHT,
    APP_NAV_STATE_SMOOTH_TURN_LEFT,
    APP_NAV_STATE_SMOOTH_TURN_RIGHT,
    APP_NAV_STATE_STRAIGHT_DRIVE,
    APP_NAV_STATE_STRAIGHT_DRIVE_DECIDING,
    APP_NAV_STATE_TURN_AROUND_RIGHT,
    APP_NAV_STATE_TURN_AROUND_LEFT,
    APP_NAV_STATE_ERROR
} AppNavState;

typedef enum
{
    APP_NAV_TRANSITION_NONE = 0,
    APP_NAV_TRANSITION_START_FIND_CELLS = 1,
    APP_NAV_TRANSITION_STOP_TO_MENU = 2,
    APP_NAV_TRANSITION_FRONT_WALL_BRAKING = 10,
    APP_NAV_TRANSITION_DIAGONALS_LOST_DECISION = 11,
    APP_NAV_TRANSITION_STRAIGHT_REAR_TAPE_NAVIGATING = 12,
    APP_NAV_TRANSITION_STRAIGHT_REAR_TAPE_DECIDING = 13,
    APP_NAV_TRANSITION_DECIDE_DEAD_END = 20,
    APP_NAV_TRANSITION_DECIDE_SMOOTH_LEFT = 21,
    APP_NAV_TRANSITION_DECIDE_SMOOTH_RIGHT = 22,
    APP_NAV_TRANSITION_DECIDE_FRONT_NAVIGATING = 23,
    APP_NAV_TRANSITION_DECIDE_FRONT_STRAIGHT = 24,
    APP_NAV_TRANSITION_BRAKING_DONE = 30,
    APP_NAV_TRANSITION_SMOOTH_DONE = 40,
    APP_NAV_TRANSITION_PIVOT_DONE = 50,
    APP_NAV_TRANSITION_TURN_START = 60
} AppNavTransitionReason;

typedef enum
{
    APP_NAV_SMOOTH_DIR_NONE = 0,
    APP_NAV_SMOOTH_DIR_LEFT = 1,
    APP_NAV_SMOOTH_DIR_RIGHT = 2
} AppNavSmoothDirection;

typedef enum
{
    APP_NAV_SMOOTH_FINISH_NONE = 0,
    APP_NAV_SMOOTH_FINISH_REAR_TAPE = 1,
    APP_NAV_SMOOTH_FINISH_YAW_TARGET = 2,
    APP_NAV_SMOOTH_FINISH_WALL = 3,
    APP_NAV_SMOOTH_FINISH_POST_YAW_REAR_TAPE = 4,
    APP_NAV_SMOOTH_FINISH_POST_YAW_TIMEOUT = 5,
    APP_NAV_SMOOTH_FINISH_FRONT_WALL_SAFETY = 6
} AppNavSmoothFinishReason;

typedef struct
{
    uint32_t dt_ms;

    uint16_t adc_filtered[APP_NAV_ADC_CHANNEL_COUNT];

    uint16_t dist_right_lat_mm;
    uint16_t dist_diagonal_right_mm;
    uint16_t dist_front_right_mm;
    uint16_t dist_front_left_mm;
    uint16_t dist_diagonal_left_mm;
    uint16_t dist_left_lat_mm;

    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    int32_t yaw_q16_deg;
} AppNavInput;

typedef struct
{
    int16_t right_motor_pwm;
    int16_t left_motor_pwm;

    bool maze_update_valid;
    uint8_t maze_x;
    uint8_t maze_y;
    uint8_t maze_cell_data;
    uint8_t maze_heading;
} AppNavOutput;

typedef struct
{
    uint16_t right_motor_base_speed;
    uint16_t left_motor_base_speed;
    uint16_t faster_motor_smooth_turn_speed;
    uint16_t slower_motor_smooth_turn_speed;

    uint16_t wall_threshold_mm_front;
    uint16_t wall_threshold_mm_braking_start;
    uint16_t wall_threshold_mm_diagonal;
    uint16_t wall_threshold_mm_side;
    uint16_t after_turn_wall_threshold_mm;
    uint16_t wall_target_mm;
    uint16_t wall_braking_target_mm;
    uint16_t tape_detection_threshold_adc;

    uint16_t turn_target_dps;
    uint16_t pivot_turn_target_dps;
} AppNavConfig;

typedef struct
{
    AppNavMode mode;
    AppNavState state;
    AppNavState previous_state;

    uint8_t last_transition_reason;
    uint8_t pending_transition_reason;
    uint8_t smooth_direction;
    uint8_t smooth_finish_reason;

    int16_t yaw_target_deg;
    int16_t pwm_right_cmd;
    int16_t pwm_left_cmd;
    uint16_t transition_sequence;
} AppNavDebug;

#endif /* INC_APP_NAV_TYPES_H_ */
