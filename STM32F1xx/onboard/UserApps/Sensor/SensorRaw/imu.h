#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include <stdbool.h>

/** Extern Micros definition ******************************************************************* */
/** Default acc range */
#define ACC_X_MAX		GRAVITY
#define ACC_X_MIN		-GRAVITY
#define ACC_Y_MAX		GRAVITY
#define ACC_Y_MIN		-GRAVITY
#define ACC_Z_MAX		GRAVITY
#define ACC_Z_MIN		-GRAVITY

/** Private micros definition ****************************************************************** */
#define IMU_INSTANCE_MPU		0
#define IMU_INSTANCE_NUM		1

#define delay_ms      HAL_Delay
/** Extern function type declaration *********************************************************** */
void imu_init(void);

/**
 * IMU task entry: read -> decode -> calibrate/filter exactly one sample.
 * Blocking software I2C is used internally, so this must only be called from the
 * task layer (never from an interrupt handler).
 */
void imu_update(void);

/* Sample publication: downstream stages detect new data through this sequence. */
uint32_t imu_get_sample_seq(void);
uint32_t imu_get_last_update_ms(void);
bool imu_is_fresh(uint32_t max_age_ms);

void imu_set_fcut_lpf_gyro_acc(uint8_t imu_inst, uint8_t fcut_gyro, uint8_t fcut_acc);
void imu_set_fcut_lpf_acc_for_inav(uint8_t imu_inst, uint8_t fcut_acc_for_inav);
void imu_set_acc_range(uint8_t imu_inst, uint8_t *min_max);

/**
 * Capture the gyro zero bias while the board is standing still.
 * The result is stored internally and every later gyro reading has the bias
 * removed automatically (see imu_get_gyro / imu_get_gyro_raw).
 * @param  imu_inst : imu instance
 * @param  samples  : number of samples to average (5ms apart)
 * @retval true if enough valid samples were collected and the bias is installed
 * @note   Blocking (uses the software I2C bus), so it runs at init time before
 *         the scheduler starts or from a task, never from an interrupt.
 */
bool imu_calib_gyro_bias(uint8_t imu_inst, uint16_t samples);

/** Measured gyro zero bias of one axis, unit: deg/s (0 while uncalibrated). */
double imu_get_gyro_bias_dps(uint8_t imu_inst, uint8_t axis);
/** True once imu_calib_gyro_bias() succeeded for this instance. */
bool imu_gyro_bias_is_calibrated(uint8_t imu_inst);

//1、传感器原始数据
double imu_get_gyro_rawest(uint8_t imu_inst, uint8_t axis);
double imu_get_acc_rawest(uint8_t imu_inst, uint8_t axis);

//2、传感器原始数据做平均值滤波数据
double imu_get_gyro_raw(uint8_t imu_inst, uint8_t axis);
double imu_get_acc_raw(uint8_t imu_inst, uint8_t axis);
//3、传感器平均值滤波数据再做低通滤波
double imu_get_gyro(uint8_t imu_inst, uint8_t axis);
double imu_get_acc(uint8_t imu_inst, uint8_t axis);
//4、传感器平均值滤波数据再做低通滤波数据 和上面的区别是滤波参数不一样
double imu_get_acc_for_inav(uint8_t imu_inst, uint8_t axis);

double imu_get_accz_rawest_mpu6000(void);

//5、健康-1
bool imu_get_health_status(uint8_t inst);
bool imu_get_init_status_gyro(void);
bool imu_get_init_status_acc(void);

bool imu_get_primary_status(void);
bool imu_get_backup_status(void);

//6 传感器温度
double imu_get_gyro_temperature(uint8_t axis);
double imu_get_acc_temperature(void);
double imu_get_gyro_before_filter(uint8_t imu_inst, uint8_t axis);


#endif
/** END OF FILE ******************************************************************************** */
