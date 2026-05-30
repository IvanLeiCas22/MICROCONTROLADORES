#ifndef INC_APP_NAV_TYPES_H_
#define INC_APP_NAV_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#define APP_NAV_ADC_CHANNEL_COUNT 8U

typedef enum
{
    APP_NAV_SMOOTH_TURN_LEFT = 0,
    APP_NAV_SMOOTH_TURN_RIGHT = 1
} AppNavSmoothTurnDirection;

/*
 * Smooth action lifecycle:
 *
 * - TURNING:
 *   The robot is executing the curved part of the smooth turn.
 *
 * - POST_YAW_SEEK_REAR_TAPE:
 *   The curved part finished by yaw target or by diagonal/wall reference.
 *   The action is not complete yet; it keeps driving with yaw-hold until the
 *   rear floor sensor confirms the target cell boundary tape.
 *
 * - DONE_REAR_TAPE / DONE_POST_YAW_REAR_TAPE:
 *   Terminal states that confirm cell entry. The supervisor may update both
 *   logical heading and logical cell position.
 *
 * - DONE_WALL:
 *   Legacy terminal state. Normal smooth diagonal/wall detection should no
 *   longer generate this state; it should enter POST_YAW_SEEK_REAR_TAPE
 *   instead. If this state reaches the supervisor, it is treated defensively
 *   as a primitive error to avoid map desynchronization.
 */
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
    APP_NAV_ADVANCE_ACTION_IDLE = 0,
    APP_NAV_ADVANCE_ACTION_WAIT_LEAVE_REAR_TAPE,
    APP_NAV_ADVANCE_ACTION_RUNNING_WALL_FOLLOW,
    APP_NAV_ADVANCE_ACTION_RUNNING_YAW_HOLD,
    APP_NAV_ADVANCE_ACTION_DONE_REAR_TAPE,
    APP_NAV_ADVANCE_ACTION_TIMEOUT,
    APP_NAV_ADVANCE_ACTION_ERROR
} AppNavAdvanceActionState;

typedef enum
{
    APP_NAV_ADVANCE_ACTION_WALL_FOLLOW_AUTO_YAW_HOLD = 0
} AppNavAdvanceActionMode;

typedef enum
{
    APP_NAV_REAR_TAPE_PROFILE_NORMAL_CELL = 0,
    APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL,
    APP_NAV_REAR_TAPE_PROFILE_SPECIAL_CELL_AFTER_PIVOT
} AppNavRearTapeProfile;

typedef enum
{
    APP_NAV_APPROACH_FRONT_WALL_ACTION_IDLE = 0,
    APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_WALL_FOLLOW,
    APP_NAV_APPROACH_FRONT_WALL_ACTION_RUNNING_YAW_HOLD,
    APP_NAV_APPROACH_FRONT_WALL_ACTION_DONE_FRONT_WALL,
    APP_NAV_APPROACH_FRONT_WALL_ACTION_TIMEOUT,
    APP_NAV_APPROACH_FRONT_WALL_ACTION_ERROR
} AppNavApproachFrontWallActionState;

typedef enum
{
    APP_NAV_FRONT_TAPE_PROFILE_NORMAL_CELL = 0,
    APP_NAV_FRONT_TAPE_PROFILE_SPECIAL_CELL
} AppNavFrontTapeProfile;

typedef enum
{
    APP_NAV_CENTER_FRONT_TAPE_ACTION_IDLE = 0,
    APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_WALL_FOLLOW,
    APP_NAV_CENTER_FRONT_TAPE_ACTION_RUNNING_YAW_HOLD,
    APP_NAV_CENTER_FRONT_TAPE_ACTION_DONE_FRONT_TAPE,
    APP_NAV_CENTER_FRONT_TAPE_ACTION_TIMEOUT,
    APP_NAV_CENTER_FRONT_TAPE_ACTION_ERROR
} AppNavCenterFrontTapeActionState;

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

    uint8_t floor_front_black;
    uint8_t floor_rear_black;

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
    uint8_t floor_front_black;
    uint8_t floor_rear_black;

    uint8_t wall_front;
    uint8_t wall_left;
    uint8_t wall_right;
    uint8_t wall_diag_left;
    uint8_t wall_diag_right;

    uint16_t floor_front_adc;
    uint16_t floor_rear_adc;

    uint16_t dist_front_left_mm;
    uint16_t dist_front_right_mm;
    uint16_t dist_left_lat_mm;
    uint16_t dist_right_lat_mm;
    uint16_t dist_diagonal_left_mm;
    uint16_t dist_diagonal_right_mm;
} AppNavPerception;

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
    uint16_t approach_front_wall_target_mm;
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


#endif /* INC_APP_NAV_TYPES_H_ */