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
    APP_NAV_SMOOTH_TURN_LEFT = 0,
    APP_NAV_SMOOTH_TURN_RIGHT = 1
} AppNavSmoothTurnDirection;

typedef enum
{
    APP_NAV_SMOOTH_ACTION_IDLE = 0,
    APP_NAV_SMOOTH_ACTION_TURNING,
    APP_NAV_SMOOTH_ACTION_POST_YAW_SEEK_REAR_TAPE,
    APP_NAV_SMOOTH_ACTION_DONE_REAR_TAPE,
    APP_NAV_SMOOTH_ACTION_DONE_WALL,
    APP_NAV_SMOOTH_ACTION_DONE_POST_YAW_REAR_TAPE,
    APP_NAV_SMOOTH_ACTION_FRONT_WALL_SAFETY,
    APP_NAV_SMOOTH_ACTION_POST_YAW_TIMEOUT,
    APP_NAV_SMOOTH_ACTION_ERROR
} AppNavSmoothActionState;

typedef enum
{
    APP_NAV_SMOOTH_ACTION_LEFT = 0,
    APP_NAV_SMOOTH_ACTION_RIGHT
} AppNavSmoothActionType;

typedef enum
{
    APP_NAV_PIVOT_ACTION_IDLE = 0,
    APP_NAV_PIVOT_ACTION_RUNNING,
    APP_NAV_PIVOT_ACTION_DONE,
    APP_NAV_PIVOT_ACTION_TIMEOUT,
    APP_NAV_PIVOT_ACTION_ERROR
} AppNavPivotActionState;

typedef enum
{
    APP_NAV_PIVOT_LEFT_90 = 0,
    APP_NAV_PIVOT_RIGHT_90,
    APP_NAV_PIVOT_180_LEFT,
    APP_NAV_PIVOT_180_RIGHT
} AppNavPivotActionType;

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

typedef enum
{
    APP_NAV_ACTION_NONE = 0,
    APP_NAV_ACTION_GO_BACK,
    APP_NAV_ACTION_GO_FRONT_NAVIGATING,
    APP_NAV_ACTION_GO_FRONT_STRAIGHT,
    APP_NAV_ACTION_SMOOTH_LEFT,
    APP_NAV_ACTION_SMOOTH_RIGHT
} AppNavRecommendedAction;

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
    int16_t yaw_rate_dps;
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
    uint16_t wall_hysteresis_mm;
    uint16_t after_turn_wall_threshold_mm;
    uint16_t wall_target_mm;
    uint16_t wall_braking_target_mm;
    uint16_t tape_detection_threshold_adc;
    uint16_t tape_hysteresis_adc;

    uint16_t turn_target_dps;
    uint16_t pivot_turn_target_dps;
    uint16_t smooth_turn_completion_dead_zone_deg;
    uint16_t smooth_rear_tape_min_yaw_deg;
    uint16_t smooth_post_yaw_seek_timeout_ticks;

    int32_t advance_pid_kp_q16;
    int32_t advance_pid_ki_q16;
    int32_t advance_pid_kd_q16;
    int32_t advance_pid_output_limit_pwm;

    int32_t smooth_turn_pid_kp_q16;
    int32_t smooth_turn_pid_ki_q16;
    int32_t smooth_turn_pid_kd_q16;
    int32_t smooth_turn_pid_output_limit_pwm;

    int32_t pivot_turn_pid_kp_q16;
    int32_t pivot_turn_pid_ki_q16;
    int32_t pivot_turn_pid_kd_q16;
    int32_t pivot_turn_pid_output_limit_pwm;

    int32_t braking_pid_kp_q16;
    int32_t braking_pid_ki_q16;
    int32_t braking_pid_kd_q16;
    int32_t braking_pid_output_limit_pwm;
    int16_t braking_min_speed_pwm;
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

    uint8_t floor_front_black;
    uint8_t floor_rear_black;
    uint8_t wall_front;
    uint8_t wall_left;
    uint8_t wall_right;
    uint8_t wall_diag_left;
    uint8_t wall_diag_right;

    uint8_t available_options_mask;
    uint8_t valid_option_count;
    uint8_t last_recommended_action;

    uint16_t floor_front_adc;
    uint16_t floor_rear_adc;
    uint16_t dist_front_left_mm;
    uint16_t dist_front_right_mm;
    uint16_t dist_left_lat_mm;
    uint16_t dist_right_lat_mm;
    uint16_t dist_diagonal_left_mm;
    uint16_t dist_diagonal_right_mm;
} AppNavDebug;

#endif /* INC_APP_NAV_TYPES_H_ */
