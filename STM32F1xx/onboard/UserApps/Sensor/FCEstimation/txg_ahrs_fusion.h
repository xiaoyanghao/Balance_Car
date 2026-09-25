#ifndef __TXG_AHRS_FUSION_H
#define __TXG_AHRS_FUSION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * One consistent set of attitude angles published by the fusion task.
 * Consumers copy the whole structure in one assignment, so roll / pitch / yaw
 * always belong to the same fusion step.
 */
typedef struct
{
    double   roll_deg;      /**< 横滚角, 单位 deg */
    double   pitch_deg;     /**< 俯仰角, 单位 deg */
    double   yaw_deg;       /**< 偏航角, 单位 deg */
    uint32_t timestamp_ms;  /**< 本次姿态解算完成时刻, 单位 ms */
    bool     valid;         /**< 是否已经产生过有效姿态 */
} ahrs_attitude_t;

/* Attitude estimation task. Both functions only touch in-memory state, but the
   update step must still be called from the task layer so that it stays in the
   same execution context as the IMU sampling task. */
void ahrs_init_fusion(void);
void ahrs_update_fusion(void);

/* Published attitude access. */
bool ahrs_get_attitude(ahrs_attitude_t *attitude);
bool ahrs_attitude_is_fresh(uint32_t max_age_ms);
uint32_t ahrs_get_last_update_ms(void);

/* Euler angle getters, used by the telemetry encoder. */
double get_roll_deg(void);
double get_pitch_deg(void);
double get_yaw_deg(void);

#endif
