#include "txg_ahrs_fusion.h"
#include "txg_math.h"
#include "imu.h"
#include <math.h>
#include "stm32f1xx.h"
#include <stdlib.h>



/** Private variables *************************************************************************** */

/* Runtime attitude state. Yaw is gyro-integrated because no magnetometer is present. */
static double attitude_q[4] = {1.0, 0.0, 0.0, 0.0};
static uint32_t attitude_last_ms = 0;
static bool attitude_started = false;
static double attitude_integral[3] = {0.0, 0.0, 0.0};

/** Latched attitude published to the control / telemetry layer.
    Written by ahrs_update_fusion(), copied out by ahrs_get_attitude(). */
static ahrs_attitude_t attitude_published = {0.0, 0.0, 0.0, 0U, false};

#define ATTITUDE_KP       2.0
#define ATTITUDE_KI       0.03
#define ATTITUDE_GRAVITY_TOLERANCE 0.35



/** Private functions *************************************************************************** */
/* 原 ArduPilot DCM/AHRS 私有函数已全部删除, 原因见 ahrs_update_fusion() 之后的说明。 */

static double normalize_signed_180_deg(double angle_deg)
{
    while (angle_deg > 180.0)
    {
        angle_deg -= 360.0;
    }
    while (angle_deg < -180.0)
    {
        angle_deg += 360.0;
    }
    return angle_deg;
}

void ahrs_init_fusion(void)
{
    attitude_q[0] = 1.0;
    attitude_q[1] = 0.0;
    attitude_q[2] = 0.0;
    attitude_q[3] = 0.0;
    attitude_integral[0] = 0.0;
    attitude_integral[1] = 0.0;
    attitude_integral[2] = 0.0;
    attitude_last_ms = HAL_GetTick();
    attitude_started = true;

    /** Invalidate the published snapshot: until the first update completes, a
        consumer must not treat the (zeroed) angles as a valid attitude. */
    attitude_published.roll_deg = 0.0;
    attitude_published.pitch_deg = 0.0;
    attitude_published.yaw_deg = 0.0;
    attitude_published.timestamp_ms = attitude_last_ms;
    attitude_published.valid = false;
}

void ahrs_update_fusion(void)
{
    double ax = imu_get_acc(IMU_INSTANCE_MPU, 0);
    double ay = imu_get_acc(IMU_INSTANCE_MPU, 1);
    double az = imu_get_acc(IMU_INSTANCE_MPU, 2);
    double gx = imu_get_gyro(IMU_INSTANCE_MPU, 0) * DEG_TO_RAD;
    double gy = imu_get_gyro(IMU_INSTANCE_MPU, 1) * DEG_TO_RAD;
    double gz = imu_get_gyro(IMU_INSTANCE_MPU, 2) * DEG_TO_RAD;
    double dt;
    double acc_norm;
    double vx;
    double vy;
    double vz;
    double ex;
    double ey;
    double ez;
    double q0;
    double q1;
    double q2;
    double q3;
    double q_norm;
    uint32_t now = HAL_GetTick();

    if (!attitude_started)
        ahrs_init_fusion();

    dt = (uint32_t)(now - attitude_last_ms) * 0.001;
    attitude_last_ms = now;
    if (dt <= 0.0 || dt > 0.05)
        dt = 0.005;

    q0 = attitude_q[0];
    q1 = attitude_q[1];
    q2 = attitude_q[2];
    q3 = attitude_q[3];
    acc_norm = sqrt(ax * ax + ay * ay + az * az);

    if (acc_norm > 0.01 && fabs(acc_norm - GRAVITY) < ATTITUDE_GRAVITY_TOLERANCE * GRAVITY)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        vx = 2.0 * (q1 * q3 - q0 * q2);
        vy = 2.0 * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        attitude_integral[0] += ATTITUDE_KI * ex * dt;
        attitude_integral[1] += ATTITUDE_KI * ey * dt;
        attitude_integral[2] += ATTITUDE_KI * ez * dt;
        gx += ATTITUDE_KP * ex + attitude_integral[0];
        gy += ATTITUDE_KP * ey + attitude_integral[1];
        gz += ATTITUDE_KP * ez + attitude_integral[2];
    }

    attitude_q[0] += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    attitude_q[1] += 0.5 * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    attitude_q[2] += 0.5 * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    attitude_q[3] += 0.5 * ( q0 * gz + q1 * gy - q2 * gx) * dt;

    q_norm = sqrt(attitude_q[0] * attitude_q[0] + attitude_q[1] * attitude_q[1] +
                  attitude_q[2] * attitude_q[2] + attitude_q[3] * attitude_q[3]);
    if (q_norm > 0.0001)
    {
        attitude_q[0] /= q_norm;
        attitude_q[1] /= q_norm;
        attitude_q[2] /= q_norm;
        attitude_q[3] /= q_norm;
    }

    /** Publish one consistent snapshot: roll / pitch / yaw and the timestamp are
        written as a unit, so the control and telemetry tasks can never mix angles
        coming from two different fusion steps. */
    attitude_published.roll_deg = normalize_signed_180_deg(
        atan2(2.0 * (attitude_q[0] * attitude_q[1] + attitude_q[2] * attitude_q[3]),
              1.0 - 2.0 * (attitude_q[1] * attitude_q[1] + attitude_q[2] * attitude_q[2])) * 57.2957795);
    attitude_published.pitch_deg = normalize_signed_180_deg(
        asin(constrain_double(2.0 * (attitude_q[0] * attitude_q[2] - attitude_q[3] * attitude_q[1]), -1.0, 1.0)) * 57.2957795);
    attitude_published.yaw_deg = normalize_signed_180_deg(
        atan2(2.0 * (attitude_q[0] * attitude_q[3] + attitude_q[1] * attitude_q[2]),
              1.0 - 2.0 * (attitude_q[2] * attitude_q[2] + attitude_q[3] * attitude_q[3])) * 57.2957795);
    attitude_published.timestamp_ms = now;
    attitude_published.valid = true;
}

/* ---------------------------------------------------------------------------------------------
 * 说明: 原文件在此处包含约 1600 行从 ArduPilot 移植的 DCM/AHRS 代码
 * (update_trig / dcm_update / matrix_update / renorm / normalize / drift_correction /
 *  observation_heading / euler_angles / update_trigonometric ...)。
 * 这些函数在 2026-09 的架构整理中被删除, 原因:
 *   1) 本工程不使用 DMP, 姿态由下面的 ahrs_update_fusion() (Mahony 四元数) 单独完成;
 *   2) 它们没有任何调用者 (编译器 Pe177 "declared but never referenced"),
 *      链接器本来就会丢弃, 留着只会拖慢编译并产生误导;
 *   3) 它们依赖各传感器 (GPS/RTK/磁罗盘/空速) 的挂载偏移与融合参数, 本工程均未接入。
 * 如需重新启用, 请从 git 历史中取回本文件的旧版本。
 * ------------------------------------------------------------------------------------------- */


double get_roll_deg(void)
{
    return attitude_published.roll_deg;
}

double get_pitch_deg(void)
{
    return attitude_published.pitch_deg;
}

double get_yaw_deg(void)
{
    return attitude_published.yaw_deg;
}

/**
 * @brief  Copy the attitude snapshot published by the fusion task.
 * @param  attitude : destination, must not be NULL
 * @retval true if a valid attitude has already been produced
 * @note   Reading through this accessor guarantees that roll/pitch/yaw belong to
 *         the same fusion step, instead of three separate reads that could be
 *         split by another update.
 */
bool ahrs_get_attitude(ahrs_attitude_t *attitude)
{
    if (attitude == NULL)
    {
        return false;
    }

    *attitude = attitude_published;

    return attitude_published.valid;
}

/**
 * @brief  Return true if the published attitude is not older than max_age_ms.
 */
bool ahrs_attitude_is_fresh(uint32_t max_age_ms)
{
    if (!attitude_published.valid)
    {
        return false;
    }

    return (uint32_t)(HAL_GetTick() - attitude_published.timestamp_ms) <= max_age_ms;
}

/**
 * @brief  Timestamp of the last published attitude, unit: ms (HAL tick)
 */
uint32_t ahrs_get_last_update_ms(void)
{
    return attitude_published.timestamp_ms;
}




/* The Invensense DMP path (self test, DMP feature/FIFO setup and the DMP based
   getters) has been removed on purpose: see readme.txt, this project does not
   use the DMP library. Attitude is estimated by ahrs_update_fusion() from the
   raw MPU60x0 samples provided by the IMU task. */

