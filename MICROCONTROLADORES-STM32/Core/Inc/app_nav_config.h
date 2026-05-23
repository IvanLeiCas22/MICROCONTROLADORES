#ifndef INC_APP_NAV_CONFIG_H_
#define INC_APP_NAV_CONFIG_H_

#include "app_nav_types.h"

/*
 * Default navigation configuration for the portable navigation core.
 *
 * The current firmware still owns the live navigation variables in app_core.c.
 * These defaults are introduced as the future extraction boundary so that the
 * same navigation core can later be copied to the Qt simulator.
 */

#define APP_NAV_DEFAULT_RIGHT_MOTOR_BASE_SPEED 3575U
#define APP_NAV_DEFAULT_LEFT_MOTOR_BASE_SPEED 4550U
#define APP_NAV_DEFAULT_FASTER_SMOOTH_TURN_SPEED 6000U
#define APP_NAV_DEFAULT_SLOWER_SMOOTH_TURN_SPEED 2500U

#define APP_NAV_DEFAULT_WALL_THRESHOLD_FRONT_MM 70U
#define APP_NAV_DEFAULT_WALL_THRESHOLD_BRAKING_START_MM 40U
#define APP_NAV_DEFAULT_WALL_THRESHOLD_DIAGONAL_MM 130U
#define APP_NAV_DEFAULT_WALL_THRESHOLD_SIDE_MM 100U
#define APP_NAV_DEFAULT_WALL_HYSTERESIS_MM 15U
#define APP_NAV_DEFAULT_AFTER_TURN_WALL_THRESHOLD_MM 80U
#define APP_NAV_DEFAULT_WALL_TARGET_MM 55U
#define APP_NAV_DEFAULT_WALL_BRAKING_TARGET_MM 30U
#define APP_NAV_DEFAULT_TAPE_DETECTION_THRESHOLD_ADC 1500U
#define APP_NAV_DEFAULT_TAPE_HYSTERESIS_ADC 200U

#define APP_NAV_DEFAULT_TURN_TARGET_DPS 360U
#define APP_NAV_DEFAULT_PIVOT_TURN_TARGET_DPS 360U

static inline AppNavConfig App_Nav_DefaultConfig(void)
{
    AppNavConfig cfg;

    cfg.right_motor_base_speed = APP_NAV_DEFAULT_RIGHT_MOTOR_BASE_SPEED;
    cfg.left_motor_base_speed = APP_NAV_DEFAULT_LEFT_MOTOR_BASE_SPEED;
    cfg.faster_motor_smooth_turn_speed = APP_NAV_DEFAULT_FASTER_SMOOTH_TURN_SPEED;
    cfg.slower_motor_smooth_turn_speed = APP_NAV_DEFAULT_SLOWER_SMOOTH_TURN_SPEED;

    cfg.wall_threshold_mm_front = APP_NAV_DEFAULT_WALL_THRESHOLD_FRONT_MM;
    cfg.wall_threshold_mm_braking_start = APP_NAV_DEFAULT_WALL_THRESHOLD_BRAKING_START_MM;
    cfg.wall_threshold_mm_diagonal = APP_NAV_DEFAULT_WALL_THRESHOLD_DIAGONAL_MM;
    cfg.wall_threshold_mm_side = APP_NAV_DEFAULT_WALL_THRESHOLD_SIDE_MM;
    cfg.wall_hysteresis_mm = APP_NAV_DEFAULT_WALL_HYSTERESIS_MM;
    cfg.after_turn_wall_threshold_mm = APP_NAV_DEFAULT_AFTER_TURN_WALL_THRESHOLD_MM;
    cfg.wall_target_mm = APP_NAV_DEFAULT_WALL_TARGET_MM;
    cfg.wall_braking_target_mm = APP_NAV_DEFAULT_WALL_BRAKING_TARGET_MM;
    cfg.tape_detection_threshold_adc = APP_NAV_DEFAULT_TAPE_DETECTION_THRESHOLD_ADC;
    cfg.tape_hysteresis_adc = APP_NAV_DEFAULT_TAPE_HYSTERESIS_ADC;

    cfg.turn_target_dps = APP_NAV_DEFAULT_TURN_TARGET_DPS;
    cfg.pivot_turn_target_dps = APP_NAV_DEFAULT_PIVOT_TURN_TARGET_DPS;

    return cfg;
}

#endif /* INC_APP_NAV_CONFIG_H_ */
