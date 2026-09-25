#include "imu.h"
#include "periph_mpu60x0.h"
#include "txg_vector3.h"
#include "txg_low_pass_filter.h"
#include "txg_math.h"
#include "txg_ahrs_fusion.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/** Private structure definition *************************************************************** */
/* Struct def of imu read */
typedef struct
{
    double rawest_acc[3];          //rawest acc data
    double rawest_gyro[3];         //rawest gyro data
    double raw_acc[3];             //raw acc data with average filter
    double raw_gyro[3];            //raw gyro data with average filter
    double gyro_offset[3];         //gyro offset
    double acc[3];                 //filtered acc data with low pass filter
    double gyro[3];                //filtered gyro data with low pass filter
    bool   health_status;
    uint32_t last_update_ms;       //time of the last decoded sample, unit: ms
} imu_t;

/** Private variables declaration ************************************************************** */
static imu_t _imu[IMU_INSTANCE_NUM];

/** 2nd butterworth LPF structure */
static struct SecondOrderLowPass_t _lpf_gyro[IMU_INSTANCE_NUM][3];
static struct SecondOrderLowPass_t _lpf_acc[IMU_INSTANCE_NUM][3];
static struct SecondOrderLowPass_t _lpf_acc_for_inav[IMU_INSTANCE_NUM][3];

/** 4th butterworth LPF structure */
static struct Butterworth4LowPass_t _btw_gyro[IMU_INSTANCE_NUM][3];
static struct Butterworth4LowPass_t _btw_acc[IMU_INSTANCE_NUM][3];
static struct Butterworth4LowPass_t _btw_acc_for_inav[IMU_INSTANCE_NUM][3];

/** cut-off frequency */
static uint8_t _fcut_lpf_gyro[IMU_INSTANCE_NUM] = {50};
static uint8_t _fcut_lpf_acc[IMU_INSTANCE_NUM] = {30};
static uint8_t _fcut_lpf_acc_for_inav[IMU_INSTANCE_NUM] = {25};

/** IMU sensor init state */
static int8_t _init_result_mpu60x0 = 0;

/** Monotonic counter of decoded samples.
 *  Every increment means one new sample has passed the calibrate + LPF pipeline
 *  and can be consumed by the estimation / control layer. Downstream tasks use
 *  this instead of polling raw values, so one sample is processed exactly once. */
static volatile uint32_t _sample_seq = 0;

/** Gyro zero bias calibration: 200 samples * 5ms = 1s of stationary sampling. */
#define IMU_GYRO_CALIB_SAMPLES      200
static bool _gyro_bias_calibrated[IMU_INSTANCE_NUM] = {false};


/** acc for inav */
static double _acc_for_inav[IMU_INSTANCE_NUM][3] = {0.0, 0.0, GRAVITY};

/** for acc calibrate */
static double _acc_max[IMU_INSTANCE_NUM][3] = {{ACC_X_MAX, ACC_Y_MAX, ACC_Z_MAX}};
static double _acc_min[IMU_INSTANCE_NUM][3] = {{ACC_X_MIN, ACC_Y_MIN, ACC_Z_MIN}};

/** gyro (before filter) for rate control */
static double _imu_gyro_filter_before[2][3] = {0.0};

/** Private functions type declaration ********************************************************* */
static void imu_calib_acc(uint8_t imu_inst, double *acc_raw, double *acc_cali);
static void imu_calib_gyro(uint8_t imu_inst, double *gyro_raw, double *gyro_cali);

static void init_filter_ahrs_fusion(uint8_t imu_inst, uint8_t fcut_gyro, uint8_t fcut_acc);
static void init_filter_acc_for_inav(uint8_t imu_inst, uint8_t fcut_acc_for_inav);


/** Functions declaration ********************************************************************** */
/**
 * @brief: Init IMU sensor and get init state.
 */
void imu_init(void)
{
    _init_result_mpu60x0 = mpu60x0_init();
    _imu[IMU_INSTANCE_MPU].health_status = _init_result_mpu60x0 != 0;
    for (uint8_t i = 0; i < IMU_INSTANCE_NUM; i++)
    {
        init_filter_ahrs_fusion(i, _fcut_lpf_gyro[i], _fcut_lpf_acc[i]);
        init_filter_acc_for_inav(i, _fcut_lpf_acc_for_inav[i]);
    }

    /** 静止零偏标定: 上电时车体必须静止。标定后 imu_get_gyro/imu_get_gyro_raw
        自动扣除零偏, 姿态融合的积分漂移会小很多。失败时保持零偏为 0。 */
    (void)imu_calib_gyro_bias(IMU_INSTANCE_MPU, IMU_GYRO_CALIB_SAMPLES);
}

/**
 * @brief: Capture the gyro zero bias while the board is standing still.
 * @param imu_inst : imu instance
 * @param samples  : number of samples to average, 5ms apart
 * @retval true if the bias has been captured and installed
 * @note   This reads the sensor directly (not through imu_update()) on purpose:
 *         calibrating through the normal pipeline would push uncalibrated data
 *         into the low pass filters. Blocking, so task/init context only.
 */
bool imu_calib_gyro_bias(uint8_t imu_inst, uint16_t samples)
{
    double sum[3] = {0.0, 0.0, 0.0};
    uint16_t valid_samples = 0;

    if (imu_inst >= IMU_INSTANCE_NUM || samples == 0 || _init_result_mpu60x0 == 0)
    {
        return false;
    }

    for (uint16_t i = 0; i < samples; i++)
    {
        mpu60x0_transfer();
        if (!mpu60x0_update())
        {
            continue;
        }

        for (uint8_t axis = 0; axis < 3; axis++)
        {
            sum[axis] += mpu60x0_get_rawgyo_dps(axis);
        }
        valid_samples++;

        HAL_Delay(5);
    }

    /** Too many failed reads: keep the previous bias instead of a garbage one. */
    if (valid_samples < (samples / 2)) 
    {
        return false;
    }

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        /** imu_calib_gyro() adds this offset, so store -bias to cancel it out. */
        _imu[imu_inst].gyro_offset[axis] = -(sum[axis] / (double)valid_samples);
    }

    _gyro_bias_calibrated[imu_inst] = true;

    return true;
}

/**
 * @brief: Measured gyro zero bias of one axis, unit: deg/s.
 */
double imu_get_gyro_bias_dps(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst >= IMU_INSTANCE_NUM || axis > 2)
    {
        return 0.0;
    }

    return -_imu[imu_inst].gyro_offset[axis];
}

/**
 * @brief: True once the gyro zero bias calibration succeeded.
 */
bool imu_gyro_bias_is_calibrated(uint8_t imu_inst)
{
    if (imu_inst >= IMU_INSTANCE_NUM)
    {
        return false;
    }

    return _gyro_bias_calibrated[imu_inst];
}


/**
 * @brief: Set LPF f_cut of gyro and acc. Then it will be used to fusion ahrs.
 * @brief: NOTE: Filter should be re-init if f_cut changed.
 * @param fcut_gyro : f_cut of gyro
 * @param fcut_acc  : f_cut of acc
 */
void imu_set_fcut_lpf_gyro_acc(uint8_t imu_inst, uint8_t fcut_gyro, uint8_t fcut_acc)
{
    _fcut_lpf_gyro[imu_inst] = fcut_gyro;
    _fcut_lpf_acc[imu_inst]  = fcut_acc;

    init_filter_ahrs_fusion(imu_inst, _fcut_lpf_gyro[imu_inst], _fcut_lpf_acc[imu_inst]);
}

/**
 * @brief: Set LPF f_cut of acc used to fusion nav.
 * @brief: NOTE: Filter should be reinit if f_cut changed.
 * @param fcut_acc_for_inav : fcut of acc for nav.
 */
void imu_set_fcut_lpf_acc_for_inav(uint8_t imu_inst, uint8_t fcut_acc_for_inav)
{
    _fcut_lpf_acc_for_inav[imu_inst] = fcut_acc_for_inav;

    init_filter_acc_for_inav(imu_inst, _fcut_lpf_acc_for_inav[imu_inst]);
}

/**
 * @brief: Init gyro and acc filter for AHRS fusion
 * @param f_c_gyro : cutoff frequency of gyro
 * @param f_c_acc : cutoff frequency of acc
 */
static void init_filter_ahrs_fusion(uint8_t imu_inst, uint8_t fcut_gyro, uint8_t fcut_acc)
{
    /** TODO: If not auto sample, rate same as main loop */
    uint16_t smp_rate = 200;  //zhxw modify imu sample rate

    for (uint8_t i=0; i<3; i++)
    {
        init_second_order_low_pass(&_lpf_gyro[imu_inst][i], fcut_gyro, smp_rate, 0.707, _imu[imu_inst].raw_gyro[i]);
        init_second_order_low_pass(&_lpf_acc[imu_inst][i],  fcut_acc,  smp_rate, 0.707, _imu[imu_inst].raw_acc[i]);

        init_butterworth_4_low_pass(&_btw_gyro[imu_inst][i], fcut_gyro, smp_rate, _imu[imu_inst].raw_gyro[i]);
        init_butterworth_4_low_pass(&_btw_acc[imu_inst][i],  fcut_acc,  smp_rate, _imu[imu_inst].raw_acc[i]);
    }
}


/**
 * @brief: Init acc filter for nav fusion
 * @param f_c_acc_ef : cutoff frequency of acc for nav
 */
static void init_filter_acc_for_inav(uint8_t imu_inst, uint8_t fcut_acc_for_inav)
{
    /** TODO: If not auto sample, rate same as main loop */
    uint16_t smp_rate = 200;    //zhxw modify acc sample rate

    for (uint8_t i=0; i<3; i++)
    {
        init_second_order_low_pass(&_lpf_acc_for_inav[imu_inst][i], fcut_acc_for_inav, smp_rate, 0.707, _imu[imu_inst].raw_acc[i]);

        init_butterworth_4_low_pass(&_btw_acc_for_inav[imu_inst][i], fcut_acc_for_inav, smp_rate, _imu[imu_inst].raw_acc[i]);
    }
}


/**
 * @brief: Read one IMU sample from the sensor bus.
 * @note   The MPU60x0 is wired to a bit-banged (blocking) software I2C bus, so
 *         this function MUST only be called from the task layer. Calling it from
 *         an interrupt handler would hold the CPU inside an ISR for hundreds of
 *         microseconds and would race with the task that consumes the data.
 */
static void imu_pull_samples(void)
{
    /** Get MPU60x0 if init pass */
    if (_init_result_mpu60x0 != 0)
    {
        mpu60x0_transfer();
    }
}

/**
 * @brief: Calibrate and low pass filter the sample decoded by mpu60x0_update().
 * @retval true  : a new sample has been published (sequence incremented)
 * @retval false : sensor not ready, no new sample was produced
 * @note   Task context only.
 */
static bool imu_major_samples_pull(void)
{
    if (_init_result_mpu60x0 != 0)
    {
        //original data directly from mpu60x0
        _imu[IMU_INSTANCE_MPU].rawest_gyro[0] = (double)(mpu60x0_get_rawgyrox_dps());
        _imu[IMU_INSTANCE_MPU].rawest_gyro[1] = (double)(mpu60x0_get_rawgyroy_dps());
        _imu[IMU_INSTANCE_MPU].rawest_gyro[2] = (double)(mpu60x0_get_rawgyroz_dps());
        _imu[IMU_INSTANCE_MPU].rawest_acc[0] = (double)(mpu60x0_get_rawaccx_mss());
        _imu[IMU_INSTANCE_MPU].rawest_acc[1] = (double)(mpu60x0_get_rawaccy_mss());
        _imu[IMU_INSTANCE_MPU].rawest_acc[2] = (double)(mpu60x0_get_rawaccz_mss());

        //original data with average filter from mpu60x0
        _imu[IMU_INSTANCE_MPU].raw_gyro[0] = (double)(mpu60x0_get_rawgyo_dps(0));
        _imu[IMU_INSTANCE_MPU].raw_gyro[1] = (double)(mpu60x0_get_rawgyo_dps(1));
        _imu[IMU_INSTANCE_MPU].raw_gyro[2] = (double)(mpu60x0_get_rawgyo_dps(2));
        _imu[IMU_INSTANCE_MPU].raw_acc[0] = (double)(mpu60x0_get_rawacc_mss(0));
        _imu[IMU_INSTANCE_MPU].raw_acc[1] = (double)(mpu60x0_get_rawacc_mss(1));
        _imu[IMU_INSTANCE_MPU].raw_acc[2] = (double)(mpu60x0_get_rawacc_mss(2));
    }
    else
    {
        /** Sensor not ready: do not re-process the previous sample, otherwise the
            low pass filters would be fed with stale data and the sample sequence
            would report "new data" when there is none. */
        return false;
    }

    //*************************************Rotate yaw from FCU frame to BODY frame*************************************//
    float imu_rotate_yaw_deg = (float)(0 * -1.0f); //(float)(param_get_imu_dir_deg(0, 2)) * (-1.0f);
    imu_rotate_yaw_deg = constrain_double(imu_rotate_yaw_deg, -360.0, 360.0);

    double cos_yaw = cosf(imu_rotate_yaw_deg*DEG_TO_RAD);
    double sin_yaw = sinf(imu_rotate_yaw_deg*DEG_TO_RAD);

    double temp_gx_rawest = 0.0;
    double temp_gy_rawest = 0.0;
    double temp_ax_rawest = 0.0;
    double temp_ay_rawest = 0.0;

    double temp_gx_raw = 0.0;
    double temp_gy_raw = 0.0;
    double temp_ax_raw = 0.0;
    double temp_ay_raw = 0.0;

    for (uint8_t idx = 0; idx<IMU_INSTANCE_NUM; idx++)
    {
        //rotate rawest IMU data
        temp_gx_rawest =  _imu[idx].rawest_gyro[0] * cos_yaw + _imu[idx].rawest_gyro[1] * sin_yaw;
        temp_gy_rawest = -_imu[idx].rawest_gyro[0] * sin_yaw + _imu[idx].rawest_gyro[1] * cos_yaw;
        _imu[idx].rawest_gyro[0] = temp_gx_rawest;
        _imu[idx].rawest_gyro[1] = temp_gy_rawest;

        temp_ax_rawest =  _imu[idx].rawest_acc[0] * cos_yaw + _imu[idx].rawest_acc[1] * sin_yaw;
        temp_ay_rawest = -_imu[idx].rawest_acc[0] * sin_yaw + _imu[idx].rawest_acc[1] * cos_yaw;
        _imu[idx].rawest_acc[0] = temp_ax_rawest;
        _imu[idx].rawest_acc[1] = temp_ay_rawest;

        //rotate raw IMU data
        temp_gx_raw =  _imu[idx].raw_gyro[0] * cos_yaw + _imu[idx].raw_gyro[1] * sin_yaw;
        temp_gy_raw = -_imu[idx].raw_gyro[0] * sin_yaw + _imu[idx].raw_gyro[1] * cos_yaw;
        _imu[idx].raw_gyro[0] = temp_gx_raw;
        _imu[idx].raw_gyro[1] = temp_gy_raw;

        temp_ax_raw =  _imu[idx].raw_acc[0] * cos_yaw + _imu[idx].raw_acc[1] * sin_yaw;
        temp_ay_raw = -_imu[idx].raw_acc[0] * sin_yaw + _imu[idx].raw_acc[1] * cos_yaw;
        _imu[idx].raw_acc[0] = temp_ax_raw;
        _imu[idx].raw_acc[1] = temp_ay_raw;
    }

    // *****************************************Calibration of imu raw data******************************************* //

    /** Calibrate acc with acc range in m/s^2 */
    imu_calib_acc(IMU_INSTANCE_MPU, _imu[IMU_INSTANCE_MPU].raw_acc, _imu[IMU_INSTANCE_MPU].acc);

    /** calibrate gyro with offset in deg/s */
    imu_calib_gyro(IMU_INSTANCE_MPU, _imu[IMU_INSTANCE_MPU].raw_gyro, _imu[IMU_INSTANCE_MPU].gyro);

    // *********************************LPF of imu raw data for AHRS and INAV fusion********************************* //
    for (uint8_t i=0; i<3; i++)
    {
        _imu_gyro_filter_before[IMU_INSTANCE_MPU][i] = _imu[IMU_INSTANCE_MPU].gyro[i];
        _acc_for_inav[IMU_INSTANCE_MPU][i] = update_second_order_low_pass(&_lpf_acc_for_inav[IMU_INSTANCE_MPU][i], _imu[IMU_INSTANCE_MPU].acc[i]);
        _imu[IMU_INSTANCE_MPU].gyro[i] = update_second_order_low_pass(&_lpf_gyro[IMU_INSTANCE_MPU][i], _imu[IMU_INSTANCE_MPU].gyro[i]);
        _imu[IMU_INSTANCE_MPU].acc[i]  = update_second_order_low_pass(&_lpf_acc[IMU_INSTANCE_MPU][i], _imu[IMU_INSTANCE_MPU].acc[i]);
    }

    // publish the new sample: timestamp + monotonic sequence
    _imu[IMU_INSTANCE_MPU].last_update_ms = HAL_GetTick();
    _sample_seq++;

    return true;
}


/**
 * Try to read new imu samples from sensor.
 * @note This is the single entry point that touches the (blocking) IMU bus.
 *       It must be called from the task layer, never from an interrupt.
 */
void imu_update(void)
{
    imu_pull_samples();

    if (mpu60x0_update())
    {
        (void)imu_major_samples_pull();
    }
}

/**
 * @brief: Calibrate raw acc, remove bias and align scale.
 * @param acc_raw:  raw acc
 * @param acc_cali: final acc at m/s^2
 */
static void imu_calib_acc(uint8_t imu_inst, double *acc_raw, double *acc_cali)
{
    double acc_mid[3]   = {0.0, 0.0, 0.0};
    double acc_range[3] = {0.0, 0.0, 0.0};
    double acc_cali_temp[3] = {0.0, 0.0, 0.0};

    for (uint8_t i=0; i<3; i++)
    {
        acc_mid[i]   = (_acc_max[imu_inst][i] + _acc_min[imu_inst][i]) / 2.0f;
        acc_range[i] = (_acc_max[imu_inst][i] - _acc_min[imu_inst][i]) / 2.0f;

        acc_cali_temp[i] = ((acc_raw[i] - acc_mid[i]) / acc_range[i]) * GRAVITY;
        //acc_cali[i] = acc_raw[i];
    }

    memcpy(acc_cali, acc_cali_temp, sizeof(double) * 3);
}

/**
 * @brief: Calibrate gyro, remove zero offset. Error of cross axis may could not done.
 * @param gyro_raw:  raw gyro in deg/s
 * @param gyro_cali: gyro without bias
 */
static void imu_calib_gyro(uint8_t imu_inst, double *gyro_raw, double *gyro_cali)
{
    for (uint8_t i=0; i<3; i++)
    {
        gyro_cali[i] = gyro_raw[i] + _imu[imu_inst].gyro_offset[i];
    }
}

/**
 * @brief: Set acc range to calibration result.
 * @param min_max: acc min/max buffer
 */
void imu_set_acc_range(uint8_t imu_inst, uint8_t *min_max)
{
    uint8_t axis = 0;
    int16_t temp_acc_min[3] = {0};
    int16_t temp_acc_max[3] = {0};

    for (axis=0; axis<3; axis++)
    {
        memcpy(&temp_acc_min[axis], &min_max[axis*2], 2);
        memcpy(&temp_acc_max[axis], &min_max[axis*2 + 6], 2);

        if (abs(temp_acc_min[axis])>20000 || abs(temp_acc_max[axis])>20000)
        {
            return;
        }
    }

    for (axis=0; axis<3; axis++)
    {
        /** accel range stored at mm/s^2 in eeprom */
        _acc_min[imu_inst][axis] = (double)temp_acc_min[axis] * 0.001;
        _acc_max[imu_inst][axis] = (double)temp_acc_max[axis] * 0.001;

        if (fabs(_acc_min[imu_inst][axis] + GRAVITY) > 4.0)
        {
            _acc_min[imu_inst][axis] = -GRAVITY;
        }
        if (fabs(_acc_max[imu_inst][axis] - GRAVITY) > 4.0)
        {
            _acc_max[imu_inst][axis] = GRAVITY;
        }
    }
}

/** Function return of IMU ********************************************************************* */

//get rawest imu data
double imu_get_gyro_rawest(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].rawest_gyro[axis];
    }
    else
    {
        return 0.0f;
    }
}

double imu_get_acc_rawest(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].rawest_acc[axis];
    }
    else
    {
        return 0.0f;
    }
}


/**
 * @brief: Return raw gyro of specified axis of IMU, unit: deg/s
 * @param axis: axis
 */
double imu_get_gyro_raw(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].raw_gyro[axis];
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief: Return raw acc of specified axis of IMU, unit: m/s/s
 * @param axis: axis
 */
double imu_get_acc_raw(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].raw_acc[axis];
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief: Return filtered gyro of specified axis of IMU currently used, unit: deg/s
 * @param axis: axis
 */
double imu_get_gyro(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].gyro[axis];
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief: Return filtered acc of specified axis of IMU currently used, unit: m/s/s
 * @param axis: axis
 */
double imu_get_acc(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu[imu_inst].acc[axis];
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief: Return filtered acc for inav of specified axis of IMU, unit: m/s/s
 * @param axis: axis
 */
double imu_get_acc_for_inav(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _acc_for_inav[imu_inst][axis];
    }
    else
    {
        return 0.0f;
    }
}

double imu_get_accz_rawest_mpu60x0(void)
{
    return mpu60x0_get_rawaccz_mss();
}

/**
 * @brief: Return imu status which mainly represent status of gyro
 */
bool imu_get_health_status(uint8_t inst)
{
    bool ret = _imu[inst].health_status;
    return ret;
}

/**
 * @brief: Return init result of gyro
 */
bool imu_get_init_status_gyro(void)
{
    return _init_result_mpu60x0;//_init_result_gyro;
}

/**
 * @brief: Return init result of acc
 */
bool imu_get_init_status_acc(void)
{
    return _init_result_mpu60x0;//_init_result_acc;
}

/**
 * @brief: Return the decoded sample counter. Any change of this value means the
 *         task layer has produced a new sample that downstream stages (attitude
 *         fusion, control, telemetry) have not consumed yet.
 */
uint32_t imu_get_sample_seq(void)
{
    return _sample_seq;
}

/**
 * @brief: Timestamp of the last published sample, unit: ms (HAL tick)
 */
uint32_t imu_get_last_update_ms(void)
{
    return _imu[IMU_INSTANCE_MPU].last_update_ms;
}

/**
 * @brief: Return true if the newest sample is not older than max_age_ms.
 * @param max_age_ms : maximum accepted age in ms
 */
bool imu_is_fresh(uint32_t max_age_ms)
{
    if (_init_result_mpu60x0 == 0)
    {
        return false;
    }

    return (uint32_t)(HAL_GetTick() - _imu[IMU_INSTANCE_MPU].last_update_ms) <= max_age_ms;
}

bool imu_get_primary_status(void)
{
    return 0;
}

bool imu_get_backup_status(void)
{
    return 1;
}

double imu_get_gyro_temperature(uint8_t axis)
{
    if (_init_result_mpu60x0)
    {
        return mpu60x0_get_rawtemp_d();
    }
    else
    {
        return 0.0;
    }
}

double imu_get_acc_temperature(void)
{
    if (_init_result_mpu60x0)
    {
        return mpu60x0_get_rawtemp_d();
    }
    else
    {
        return 0.0;
    }
}

double imu_get_gyro_before_filter(uint8_t imu_inst, uint8_t axis)
{
    if (imu_inst < IMU_INSTANCE_NUM)
    {
        return _imu_gyro_filter_before[imu_inst][axis];
    }
    else
    {
        return 0.0f;
    }
}

/** END OF FILE ******************************************************************************** */
