#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include <stdbool.h>

/** Extern Micros definition ******************************************************************* */
typedef struct
{
    uint8_t    en;
    uint8_t    valid_flag;
    double     bias[3];
    double     scale[3][3];
    double     misalign[3][3];
} imu_adv_cali_param_t;

/** Default acc range */
#define ACC_X_MAX		GRAVITY
#define ACC_X_MIN		-GRAVITY
#define ACC_Y_MAX		GRAVITY
#define ACC_Y_MIN		-GRAVITY
#define ACC_Z_MAX		GRAVITY
#define ACC_Z_MIN		-GRAVITY

#define IMU_ADV_CALI_PARAM_VALID 0xA5

/** Private micros definition ****************************************************************** */
#define IMU_INSTANCE_MPU		0
#define IMU_INSTANCE_NUM		1

#define delay_ms      HAL_Delay
/** Extern function type declaration *********************************************************** */
void imu_init(void);
void imu_update(void);
void imu_samples_pull(int unused);

void imu_set_fcut_lpf_gyro_acc(uint8_t imu_inst, uint8_t fcut_gyro, uint8_t fcut_acc);
void imu_set_fcut_lpf_acc_for_inav(uint8_t imu_inst, uint8_t fcut_acc_for_inav);
void imu_calib_gyro_bias(void);
void imu_set_acc_range(uint8_t imu_inst, uint8_t *min_max);
void imu_set_acc_6axis_calib_matrix(uint8_t imu_inst, uint8_t *data);

void get_gyro_calib_offset_from_eeprom(uint8_t imu_inst, int16_t *gyro_offset);

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
