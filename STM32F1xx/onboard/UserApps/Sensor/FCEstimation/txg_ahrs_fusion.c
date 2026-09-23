#include "txg_ahrs_fusion.h"
#include "txg_math.h"
#include "txg_vector3.h"
#include "txg_matrix3.h"
#include "txg_low_pass_filter.h"
#include "float.h"
#include "imu.h"
#include <math.h>
#include "stm32f1xx.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <stdlib.h>
#include <string.h>



/** Pravite variables defination **************************************************************** */

static struct vector3f_t ahrs_acc_ef = {0.0, 0.0, 0.0};
/** Standard deviation of acc xy */
static double acc_std_dev_xy = 0.0;
/** Standard deviation of acc z */
static double acc_std_dev_z = 0.0;

static double _gps_pos_offset[3] = {0.0, 0.0, 0.0};
static double _gps_vel_offset[3] = {0.0, 0.0, 0.0};

static double _rtk_pos_offset[3] = {0.0, 0.0, 0.0};
static double _rtk_vel_offset[3] = {0.0, 0.0, 0.0};

static double _ins_pos_offset[3] = {0.0, 0.0, 0.0};
static double _ins_vel_offset[3] = {0.0, 0.0, 0.0};

static double _imu_pos_offset[3] = {0.0, 2.0, -2.0};
static double _imu_vel_offset[3] = {0.0, 0.0, 0.0};

/** ahrs fusion flags */
static struct ahrs_flags_t  ahrs_flags;

/** Euler angle in rad */
static double dcm_roll, dcm_yaw, dcm_pitch;
/** Euler angle in deg */
static double dcm_roll_deg, dcm_yaw_deg, dcm_pitch_deg;

// primary representation of attitude of board used for all inertial calculations
static double _dcm_matrix[3][3] = {{1.0, 0.0, 0.0},{0.0, 1.0, 0}, {0.0, 0.0, 1.0}};
// primary representation of attitude of flight vehicle body
static double _body_dcm_matrix[3][3] = {{1.0, 0.0, 0.0},{0.0, 1.0, 0}, {0.0, 0.0, 1.0}};
// mean as name, hehe
static double _rot_vehicle_body_to_autopilot_body[3][3] = {{1.0, 0.0, 0.0},{0.0, 1.0, 0}, {0.0, 0.0, 1.0}};

/** Fusion parameters */
static double _omega_P[3]     = {0.0, 0.0, 0.0}; // accel Omega proportional correction
static double _omega_yaw_P[3] = {0.0, 0.0, 0.0}; // proportional yaw correction
static double _omega_I[3]     = {0.0, 0.0, 0.0}; // Omega Integrator correction
static double _omega_I_sum[3] = {0.0, 0.0, 0.0};
static double _omega[3]       = {0.0, 0.0, 0.0}; // Corrected Gyro_Vector data

/** Virtual wind speed estimation */
static double _wind[3] = {0.0, 0.0, 0.0};

/** trim angle in deg*100 */
static int16_t trim_angle_ef_cd[2] = {0, 0};

// Hstate to support status reporting
static double _omega_I_sum_time = 0.0;
static double _renorm_val_sum   = 0.0;
static double _renorm_val_count = 0.0;
static double _error_rp         = 0.0;
static double _error_yaw        = 0.0;
static double _error_rp_sum     = 0.0;
static double _error_rp_count   = 0.0;

/** sin and cos of euler angle */
static double _cos_roll = 1.0;
static double _cos_pitch = 1.0;
static double _cos_yaw = 1.0;
static double _sin_yaw = 0.0;
static double _sin_roll = 0.0;
static double _sin_pitch = 0.0;

// Hstate of accel drift correction
static double _ra_sum[3]        = {0.0, 0.0, 0.0};
static double _last_velocity[3] = {0.0, 0.0, 0.0};
static double _ra_deltat        = 0.0;
static uint64_t _ra_sum_start   = 0;

// the earths magnetic field
static double _current_compass_declination = 0.0;
//static double _last_declination = 0.0;
//static double _mag_earth[2]     = {1.0, 0.0};
static double _accel_ef[3]      = {0.0, 0.0, 0.0};

// time in microseconds of last compass update
//static uint32_t _compass_last_update;
//static uint8_t  _yaw_source = 0;

static double _last_airspeed = 0.0;

// these are public for ArduCopter
static double _kp_yaw  = 0.2;
static double _ki      = 0.0087;
static double _ki_yaw  = 0.01;
static double _kp      = 0.2;    // set to small is good use gyro?
static double gps_gain = 1.0;
static double _ra_delay_buffer[3];
// the limit of the gyro drift claimed by the sensors, in radians/s/s
static double _gyro_drift_limit = DEG_TO_RAD*0.5/60;

//
static bool heading_RTK_ante_inv = false;
static bool heading_divergence = false;
static float omega_scale = 1.0;

/** historical buffer length **/
#define AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS 8

/** historical fusion base store buffer */
static double hist_heading[AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS];
static int8_t heading_buff_index = 0, heading_buff_num = 0;
static int8_t historic_heading_counter = 0;



/** Pravite functions *************************************************************************** */
static void update_trig(void);
static void acc_noise_check(void);
static void dcm_update(void);
static void matrix_update(double _G_Dt);
static void dcm_reset(bool recover_eulers);
static void check_matrix(void);
static bool renorm(double a[3], double result[3]);
static void normalize(void);
//static double yaw_error_compass(void);
static double _P_gain(double spin_rate);
static double _yaw_gain(void);
static double calculate_heading(double dcm_matrix[3][3]);
static bool use_fast_gains(void);
static double observation_heading(float dt);
static void drift_correction_yaw(double dt);
static void ra_delayed(double ra[3], double out[3]);
static void drift_correction(double deltat);
static void euler_angles(void);
static void update_trigonometric(void);
static double dcm_compassNormalize(double heading);


void ahrs_init_fusion(void)
{
    //init heading failsafe status flags
    //init_heading_fs_flag_status();
}
// 200Hz
void ahrs_update_fusion(void)
{
    /// Use APM DCM alg to calculate roll and pitch
    dcm_update();

    // update DCM element
    update_trig();

    acc_noise_check();

    // update heading failsafe strategy at 400hz
    // txg_heading_fs_plan();
}

static void update_trig(void)
{
    static uint8_t install_error_count = 0;
    static uint8_t gps_install_error_index = 0;
    static uint8_t rtk_install_error_index = 0;
    static double gps_pos_offset_hist_buff[10][3] = {0.0};
    static double gps_vel_offset_hist_buff[10][3] = {0.0};
    static double rtk_pos_offset_hist_buff[10][3] = {0.0};
    static double rtk_vel_offset_hist_buff[10][3] = {0.0};

    double temp_acc_ef[3] = {0, 0, 0};
    struct matrix3f_t dcm = {{1.0,0.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0}};

    double sr = sinf(get_roll_deg()*DEG_TO_RAD);
    double cr = cosf(get_roll_deg()*DEG_TO_RAD);
    double cp = cosf(get_pitch_deg()*DEG_TO_RAD);
    double sp = sinf(get_pitch_deg()*DEG_TO_RAD);
    double sy = sinf(get_yaw_deg()*DEG_TO_RAD);
    double cy = cosf(get_yaw_deg()*DEG_TO_RAD);

    // DCM rotation from body to earth.
    dcm.a.x = cp * cy;
    dcm.a.y = (sr * sp * cy) - (cr * sy);
    dcm.a.z = (cr * sp * cy) + (sr * sy);
    dcm.b.x = cp * sy;
    dcm.b.y = (sr * sp * sy) + (cr * cy);
    dcm.b.z = (cr * sp * sy) - (sr * cy);
    dcm.c.x = -sp;
    dcm.c.y = sr * cp;
    dcm.c.z = cr * cp;

    double acc_bf[3] = {0.0, 0.0, 0.0};
    acc_bf[0] = imu_get_acc_for_inav(IMU_INSTANCE_MPU,0);
    acc_bf[1] = imu_get_acc_for_inav(IMU_INSTANCE_MPU,1);
    acc_bf[2] = imu_get_acc_for_inav(IMU_INSTANCE_MPU,2);

    // convert accel to earth frame
    temp_acc_ef[0] = dcm.a.x*acc_bf[0] + dcm.a.y*acc_bf[1] + dcm.a.z*acc_bf[2];
    temp_acc_ef[1] = dcm.b.x*acc_bf[0] + dcm.b.y*acc_bf[1] + dcm.b.z*acc_bf[2];
    temp_acc_ef[2] = dcm.c.x*acc_bf[0] + dcm.c.y*acc_bf[1] + dcm.c.z*acc_bf[2];
    temp_acc_ef[2] -= GRAVITY;

    ahrs_acc_ef.x = temp_acc_ef[0];
    ahrs_acc_ef.y = temp_acc_ef[1];
    ahrs_acc_ef.z = temp_acc_ef[2];

    // TODO: GPS pos and vel offset should be store at least 10Hz, to deal with GPS latency, 200ms
    // calculate GPS pos and vel offset based on current DCM, direction is NED
    double gps_pos_to_imu[3] = {0.0, 0.0, 0.0};
   // gps_pos_to_imu[0] = param_get_gps_pos_offset(0) - param_get_imu_pos_offset(0);
   // gps_pos_to_imu[1] = param_get_gps_pos_offset(1) - param_get_imu_pos_offset(1);
   // gps_pos_to_imu[2] = param_get_gps_pos_offset(2) - param_get_imu_pos_offset(2);

    double gps_pos_offset_ef[3] = {0.0, 0.0, 0.0};
    gps_pos_offset_ef[0] = dcm.a.x*gps_pos_to_imu[0] + dcm.a.y*gps_pos_to_imu[1] + dcm.a.z*(-gps_pos_to_imu[2]);
    gps_pos_offset_ef[1] = dcm.b.x*gps_pos_to_imu[0] + dcm.b.y*gps_pos_to_imu[1] + dcm.b.z*(-gps_pos_to_imu[2]);
    gps_pos_offset_ef[2] = dcm.c.x*gps_pos_to_imu[0] + dcm.c.y*gps_pos_to_imu[1] + dcm.c.z*(-gps_pos_to_imu[2]);

    double rtk_pos_to_imu[3] = {0.0, 0.0, 0.0};
    // rtk_pos_to_imu[0] = param_get_ins_pos_offset(0) - param_get_imu_pos_offset(0);
    // rtk_pos_to_imu[1] = param_get_ins_pos_offset(1) - param_get_imu_pos_offset(1);
    // rtk_pos_to_imu[2] = param_get_ins_pos_offset(2) - param_get_imu_pos_offset(2);
    double rtk_pos_offset_ef[3] = {0.0, 0.0, 0.0};
    rtk_pos_offset_ef[0] = dcm.a.x*rtk_pos_to_imu[0] + dcm.a.y*rtk_pos_to_imu[1] + dcm.a.z*(-rtk_pos_to_imu[2]);
    rtk_pos_offset_ef[1] = dcm.b.x*rtk_pos_to_imu[0] + dcm.b.y*rtk_pos_to_imu[1] + dcm.b.z*(-rtk_pos_to_imu[2]);
    rtk_pos_offset_ef[2] = dcm.c.x*rtk_pos_to_imu[0] + dcm.c.y*rtk_pos_to_imu[1] + dcm.c.z*(-rtk_pos_to_imu[2]);

   // _imu_pos_offset[0] = dcm.a.x*param_get_imu_pos_offset(0) + dcm.a.y*param_get_imu_pos_offset(1) + dcm.a.z*(-param_get_imu_pos_offset(2));
    //_imu_pos_offset[1] = dcm.b.x*param_get_imu_pos_offset(0) + dcm.b.y*param_get_imu_pos_offset(1) + dcm.b.z*(-param_get_imu_pos_offset(2));
    // _imu_pos_offset[2] = dcm.c.x*param_get_imu_pos_offset(0) + dcm.c.y*param_get_imu_pos_offset(1) + dcm.c.z*(-param_get_imu_pos_offset(2));

    double omega[3] = {0.0};
    omega[0] = imu_get_gyro(IMU_INSTANCE_MPU,0) * DEG_TO_RAD;
    omega[1] = imu_get_gyro(IMU_INSTANCE_MPU,1)  * DEG_TO_RAD;
    omega[2] = imu_get_gyro(IMU_INSTANCE_MPU,2)  * DEG_TO_RAD;

    // direction is FRD in bodyframe
    double gps_vel_offset_bf[3] = {0.0};
    gps_vel_offset_bf[0] = omega[1]*(-gps_pos_to_imu[2]) - omega[2]*  gps_pos_to_imu[1];
    gps_vel_offset_bf[1] = omega[2]*  gps_pos_to_imu[0]  - omega[0]*(-gps_pos_to_imu[2]);
    gps_vel_offset_bf[2] = omega[0]*  gps_pos_to_imu[1]  - omega[1]*  gps_pos_to_imu[0];
    // calculate vel in NED
    double gps_vel_offset_ef[3] = {0.0};
    gps_vel_offset_ef[0] = dcm.a.x*gps_vel_offset_bf[0] + dcm.a.y*gps_vel_offset_bf[1] + dcm.a.z*gps_vel_offset_bf[2];
    gps_vel_offset_ef[1] = dcm.b.x*gps_vel_offset_bf[0] + dcm.b.y*gps_vel_offset_bf[1] + dcm.b.z*gps_vel_offset_bf[2];
    gps_vel_offset_ef[2] = dcm.c.x*gps_vel_offset_bf[0] + dcm.c.y*gps_vel_offset_bf[1] + dcm.c.z*gps_vel_offset_bf[2];

    // direction is FRD in bodyframe
    double rtk_vel_offset_bf[3] = {0.0};
    rtk_vel_offset_bf[0] = omega[1]*(-rtk_pos_to_imu[2]) - omega[2]*  rtk_pos_to_imu[1];
    rtk_vel_offset_bf[1] = omega[2]*  rtk_pos_to_imu[0]  - omega[0]*(-rtk_pos_to_imu[2]);
    rtk_vel_offset_bf[2] = omega[0]*  rtk_pos_to_imu[1]  - omega[1]*  rtk_pos_to_imu[0];
    // calculate vel in NED
    double rtk_vel_offset_ef[3] = {0.0};
    rtk_vel_offset_ef[0] = dcm.a.x*rtk_vel_offset_bf[0] + dcm.a.y*rtk_vel_offset_bf[1] + dcm.a.z*rtk_vel_offset_bf[2];
    rtk_vel_offset_ef[1] = dcm.b.x*rtk_vel_offset_bf[0] + dcm.b.y*rtk_vel_offset_bf[1] + dcm.b.z*rtk_vel_offset_bf[2];
    rtk_vel_offset_ef[2] = dcm.c.x*rtk_vel_offset_bf[0] + dcm.c.y*rtk_vel_offset_bf[1] + dcm.c.z*rtk_vel_offset_bf[2];

    // direction is FRD in bodyframe
    double imu_vel_offset_bf[3] = {0.0};
//    imu_vel_offset_bf[0] = omega[1]*(-param_get_imu_pos_offset(2)) - omega[2]*  param_get_imu_pos_offset(1);
//    imu_vel_offset_bf[1] = omega[2]*  param_get_imu_pos_offset(0)  - omega[0]*(-param_get_imu_pos_offset(2));
//    imu_vel_offset_bf[2] = omega[0]*  param_get_imu_pos_offset(1)  - omega[1]*  param_get_imu_pos_offset(0);

    // calculate vel in NED
    _imu_vel_offset[0] = dcm.a.x*imu_vel_offset_bf[0] + dcm.a.y*imu_vel_offset_bf[1] + dcm.a.z*imu_vel_offset_bf[2];
    _imu_vel_offset[1] = dcm.b.x*imu_vel_offset_bf[0] + dcm.b.y*imu_vel_offset_bf[1] + dcm.b.z*imu_vel_offset_bf[2];
    _imu_vel_offset[2] = dcm.c.x*imu_vel_offset_bf[0] + dcm.c.y*imu_vel_offset_bf[1] + dcm.c.z*imu_vel_offset_bf[2];

    // store gps/rtk offset estimate for future use at 50hz(20ms/count)
    install_error_count++;

    if (install_error_count >= 4)
    {
        install_error_count = 0;

        gps_pos_offset_hist_buff[gps_install_error_index][0] = gps_pos_offset_ef[0];
        gps_pos_offset_hist_buff[gps_install_error_index][1] = gps_pos_offset_ef[1];
        gps_pos_offset_hist_buff[gps_install_error_index][2] = gps_pos_offset_ef[2];

        gps_vel_offset_hist_buff[gps_install_error_index][0] = gps_vel_offset_ef[0];
        gps_vel_offset_hist_buff[gps_install_error_index][1] = gps_vel_offset_ef[1];
        gps_vel_offset_hist_buff[gps_install_error_index][2] = gps_vel_offset_ef[2];

        rtk_pos_offset_hist_buff[rtk_install_error_index][0] = rtk_pos_offset_ef[0];
        rtk_pos_offset_hist_buff[rtk_install_error_index][1] = rtk_pos_offset_ef[1];
        rtk_pos_offset_hist_buff[rtk_install_error_index][2] = rtk_pos_offset_ef[2];

        rtk_vel_offset_hist_buff[rtk_install_error_index][0] = rtk_vel_offset_ef[0];
        rtk_vel_offset_hist_buff[rtk_install_error_index][1] = rtk_vel_offset_ef[1];
        rtk_vel_offset_hist_buff[rtk_install_error_index][2] = rtk_vel_offset_ef[2];

        // we store historical data at 50hz, so 5 iterations implies 100ms ago, and 10 iterations for 200ms ago
        // Note: actual delay is about 200ms fro rtk velocity, 100ms for gps velocity
        gps_install_error_index++;
        if(gps_install_error_index >= 5)
        {
            gps_install_error_index = 0;
        }

        rtk_install_error_index++;
        if (rtk_install_error_index >= 10)
        {
            rtk_install_error_index = 0;
        }

        _gps_pos_offset[0] = gps_pos_offset_hist_buff[gps_install_error_index][0];
        _gps_pos_offset[1] = gps_pos_offset_hist_buff[gps_install_error_index][1];
        _gps_pos_offset[2] = gps_pos_offset_hist_buff[gps_install_error_index][2];

        _gps_vel_offset[0] = gps_vel_offset_hist_buff[gps_install_error_index][0];
        _gps_vel_offset[1] = gps_vel_offset_hist_buff[gps_install_error_index][1];
        _gps_vel_offset[2] = gps_vel_offset_hist_buff[gps_install_error_index][2];

        _rtk_pos_offset[0] = rtk_pos_offset_hist_buff[rtk_install_error_index][0];
        _rtk_pos_offset[1] = rtk_pos_offset_hist_buff[rtk_install_error_index][1];
        _rtk_pos_offset[2] = rtk_pos_offset_hist_buff[rtk_install_error_index][2];

        _rtk_vel_offset[0] = rtk_vel_offset_hist_buff[rtk_install_error_index][0];
        _rtk_vel_offset[1] = rtk_vel_offset_hist_buff[rtk_install_error_index][1];
        _rtk_vel_offset[2] = rtk_vel_offset_hist_buff[rtk_install_error_index][2];
    }
}

static void acc_noise_check(void)
{
#define ACC_FLOOR_FCUT_5HZ      0.0728205f
#define ACC_RMS_FCUT_2HZ        0.030459021f

    static struct SecondOrderLowPass_t acc_ultra_lpf[3];

    static uint64_t noise_count = 0;
    static uint16_t bad_acc_check_counter = 0;

    static double acc_std_add[3] = {0.0, 0.0, 0.0};

    double accel_diff[3] = {0.0, 0.0, 0.0};
    double acc_mean[3]   = {0.0, 0.0, GRAVITY};

    double acc[3]       = {0.0, 0.0, 0.0};
    double acc_raw[3]   = {0.0, 0.0, 0.0};

    acc[0] = imu_get_acc(IMU_INSTANCE_MPU,0);
    acc[1] = imu_get_acc(IMU_INSTANCE_MPU,1);
    acc[2] = imu_get_acc(IMU_INSTANCE_MPU,2);

    acc_raw[0] = imu_get_acc_raw(IMU_INSTANCE_MPU,0);
    acc_raw[1] = imu_get_acc_raw(IMU_INSTANCE_MPU,1);
    //original imu accz data
    acc_raw[2] = imu_get_acc_raw(IMU_INSTANCE_MPU,2);

    noise_count++;
    if(noise_count<2)
    {
        for(uint8_t i=0; i<3; i++)
        {
            init_second_order_low_pass(&acc_ultra_lpf[i], 2, 200, 0.707, acc[i]); //200Hz
        }
    }

    // low pass filter
    for (uint8_t m=0; m<3; m++)
    {
        acc_mean[m] = update_second_order_low_pass(&acc_ultra_lpf[m], acc[m]);

        accel_diff[m] = (acc_raw[m] - acc_mean[m]);
        acc_std_add[m] += ACC_RMS_FCUT_2HZ*(accel_diff[m]*accel_diff[m] - acc_std_add[m]);
    }

    acc_std_dev_xy  = sqrt(acc_std_add[0]+acc_std_add[1])/GRAVITY;
    acc_std_dev_z   = sqrt(acc_std_add[2])/GRAVITY;

    /// for common good copter, this value always lower than 4.0*gravity
    if(acc_std_dev_z >= 8.0)
    {
        if(bad_acc_check_counter < 400)
        {
            bad_acc_check_counter++;
        }
        else
        {
           // notify_flags.bad_fusion = 1;
        }
    }
    else
    {
        bad_acc_check_counter = 0;
        //notify_flags.bad_fusion = 0;
    }

}


/** user functions ********************************************************** */
/**
 * reset the current gyro drift estimate
 * should be called if gyro offsets are recalculated  delete
 */
void reset_gyro_drift(void)
{
    _omega_I[0] = _omega_I[1] = _omega_I[2] = 0;
    _omega_I_sum[0] = _omega_I_sum[1] = _omega_I_sum[2] = 0;
    _omega_I_sum_time = 0;
}

/**
 * run a full DCM update round, suggest at least 200Hz
 */
static void dcm_update(void)
{
    static uint64_t last_dcm_timer = 0;

    // ask the IMU how much time this sensor reading represents
    double delta_t = (HAL_GetTick()- last_dcm_timer) * 0.001;
    if(delta_t < 0.0)
    {
        delta_t = (HAL_GetTick() - last_dcm_timer + 0xFFFFFFFF) * 0.000001;
    }
    last_dcm_timer = HAL_GetTick();

    // if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors
    if(delta_t > 0.2)
    {
        memset(&_ra_sum[0], 0, sizeof(_ra_sum));
        _ra_deltat = 0;
        return;
    }


    // Integrate the DCM matrix using gyro inputs
    matrix_update(delta_t);

    // Normalize the DCM matrix
    normalize();

    // Perform drift correction
    drift_correction(delta_t);

    // paranoid check for bad values in the DCM matrix
    check_matrix();

    // Calculate pitch, roll, yaw for stabilization and navigation
    euler_angles();

    // update trig values including _cos_roll, cos_pitch
    update_trigonometric();
}

/**
 * update the DCM matrix using only the gyros
 * @param _G_Dt : delta time from last imu data
 */
static void matrix_update(double _G_Dt)
{
    double temp_vector[3]   = {0.0, 0.0, 0.0};
    double curr_gyro[3]     = {0.0, 0.0, 0.0};

    // note that we do not include the P terms in _omega. This is
    // because the spin_rate is calculated from _omega.length(),
    // and including the P terms would give positive feedback into
    // the _P_gain() calculation, which can lead to a very large P
    // value
    vector_zero(_omega);

    curr_gyro[0] = imu_get_gyro(IMU_INSTANCE_MPU,0) * DEG_TO_RAD;
    curr_gyro[1] = imu_get_gyro(IMU_INSTANCE_MPU,1) * DEG_TO_RAD;
    curr_gyro[2] = imu_get_gyro(IMU_INSTANCE_MPU,2) * DEG_TO_RAD;

    /** _omega = curr_gyro */
    vector_add_equal(_omega, curr_gyro);
    /** _omega += _omega_I */
    vector_add_equal(_omega, _omega_I);
    /** temp_vector = _omega + _omega_P */
    vector_add(_omega, _omega_P, temp_vector);
    /** temp_vector2 = temp_vector + _omega_yaw_P */
    vector_add(temp_vector, _omega_yaw_P, temp_vector);
    /** temp_vector3 = temp_vector2 * _G_Dt */
    vector_product(temp_vector, _G_Dt, temp_vector);

    //_dcm_matrix: transition matrix from body-frame to ned-frame
    matrix_rotate(_dcm_matrix, temp_vector);
}


/**
 *  reset the DCM matrix and omega. Used on ground start, and on
 *  extreme errors in the matrix
 */
static void dcm_reset(bool recover_eulers)
{
    // reset the integration terms
    vector_zero(_omega_I);
    vector_zero(_omega_P);
    vector_zero(_omega_yaw_P);
    vector_zero(_omega);

    // if the caller wants us to try to recover to the current
    // attitude then calculate the dcm matrix from the current
    // roll/pitch/yaw values
    if (recover_eulers && !isnan(dcm_roll) && !isnan(dcm_pitch) && !isnan(dcm_yaw))
    {
        matrix_from_euler(_dcm_matrix,dcm_roll, dcm_pitch, dcm_yaw);
    }
    else
    {
        // otherwise make it flat
        matrix_from_euler(_dcm_matrix,0, 0, 0);
    }
}

/**
 *  check the DCM matrix for pathological values
 */
static void check_matrix(void)
{
    if (matrix_is_nan(_dcm_matrix))
    {
        //Serial.printf("ERROR: DCM matrix NAN\n");
        dcm_reset(true);
        return;
    }
    // some DCM matrix values can lead to an out of range error in
    // the pitch calculation via asin().  These NaN values can
    // feed back into the rest of the DCM matrix via the
    // error_course value.
    if (!(_dcm_matrix[2][0] < 1.0 && _dcm_matrix[2][0] > -1.0))
    {
        // We have an invalid matrix. Force a normalisation.
        normalize();

        if (matrix_is_nan(_dcm_matrix) || fabsf(_dcm_matrix[2][0]) > 10)
        {
            // normalisation didn't fix the problem! We're
            // in real trouble. All we can do is reset
            //Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
            //	   _dcm_matrix.c.x);
            dcm_reset(true);
        }
    }
}

/**
 * renormalise one vector component of the DCM matrix
 * this will return false if renormalization fails
 */
static bool renorm(double a[3], double result[3])
{
    double renorm_val;

    // numerical errors will slowly build up over time in DCM,
    // causing inaccuracies. We can keep ahead of those errors
    // using the renormalization technique from the DCM IMU paper
    // (see equations 18 to 21).

    // For APM we don't bother with the taylor expansion
    // optimisation from the paper as on our 2560 CPU the cost of
    // the sqrt() is 44 microseconds, and the small time saving of
    // the taylor expansion is not worth the potential of
    // additional error buildup.

    // Note that we can get significant renormalisation values
    // when we have a larger delta_t due to a glitch eleswhere in
    // APM, such as a I2c timeout or a set of EEPROM writes. While
    // we would like to avoid these if possible, if it does happen
    // we don't want to compound the error by making DCM less
    // accurate.

    renorm_val = 1.0 / vector_length(a);

    // keep the average for reporting
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0 && renorm_val > 0.5))
    {
        // this is larger than it should get - log it as a warning
        if (!(renorm_val < 1.0e6 && renorm_val > 1.0e-6))
        {
            // we are getting values which are way out of
            // range, we will reset the matrix and hope we
            // can recover our attitude using drift
            // correction before we hit the ground!
//            printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
//                   renorm_val);
            return false;
        }
    }

    vector_product(a, renorm_val, result);

    return true;
}

/**
 *  Direction Cosine Matrix IMU: Theory
 *  William Premerlani and Paul Bizard
 *
 *  Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
 *  to approximations rather than identities. In effect, the axes in the two frames of reference no
 *  longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
 *  simple matter to stay ahead of it.
 *  We call the process of enforcing the orthogonality conditions ÒrenormalizationÓ.
 */
static void normalize(void)
{
    double error;
    int8_t i;
    double t0[3], t1[3], t2[3];

    error = vector_dot_product(_dcm_matrix[0], _dcm_matrix[1]); // eq.18

    for (i=0; i<3; i++)
    {
        t0[i] = _dcm_matrix[0][i] - (_dcm_matrix[1][i]* (0.5 * error)); // eq.19
    }

    for (i=0; i<3; i++)
    {
        t1[i] = _dcm_matrix[1][i] - (_dcm_matrix[0][i]* (0.5 * error)); // eq.19
    }

    vector_cross_product(t0, t1, t2); // c= a x b // eq.20

    if (!renorm(t0, _dcm_matrix[0]) || !renorm(t1, _dcm_matrix[1]) ||
            !renorm(t2, _dcm_matrix[2]))
    {
        // Our solution is blowing up and we will force back
        // to last euler angles
        //_last_failure_ms = hal.scheduler->millis();
        dcm_reset(true);
    }
}


void compass_set_declination(double curr_dec)
{
    _current_compass_declination = curr_dec;
}

/**
 * the _P_gain raises the gain of the PI controller
 * when we are spinning fast. See the fastRotations
 * paper from Bill.
 * @param  spin_rate : rotation rate of all axis
 * @return           : p gain based on rotate rate
 */
static double _P_gain(double spin_rate)
{
    if (spin_rate < DEG_TO_RAD*50)
    {
        return 1.0;
    }
    if (spin_rate > DEG_TO_RAD*500)
    {
        return 10.0;
    }
    return spin_rate/(DEG_TO_RAD*50);
}

/**
 * _yaw_gain reduces the gain of the PI controller applied to heading errors
 * when observability from change of velocity is good (eg changing speed or turning)
 * This reduces unwanted roll and pitch coupling due to compass errors for planes.
 * High levels of noise on _accel_ef will cause the gain to drop and could lead to
 * increased heading drift during straight and level flight, however some gain is
 * always available. TODO check the necessity of adding adjustable acc threshold
 * and/or filtering accelerations before getting magnitude
 * @return  : yaw gain based on acc
 */
static double _yaw_gain(void)
{
    double VdotEFmag = sqrt(_accel_ef[0]*_accel_ef[0]+_accel_ef[1]*_accel_ef[1]);

    if (VdotEFmag <= 4.0)
    {
        return 0.2*(4.5 - VdotEFmag);
    }

    return 0.1;
}

/**
 * calculate a compass heading given the attitude from DCM and the mag_bf vector
 */
static double calculate_heading(double dcm_matrix[3][3])
{
    double cos_pitch_sq = 1.0-(dcm_matrix[2][0]*dcm_matrix[2][0]);

    double mag_bf[3] = {0.0, 0.0, 0.0};
    // mag_bf[0] = txg_std_sub_compass_mag_bf(0);
    // mag_bf[1] = txg_std_sub_compass_mag_bf(1);
    // mag_bf[2] = txg_std_sub_compass_mag_bf(2);

    // Tilt compensated magnetic field Y component:
    double headY = mag_bf[1] * dcm_matrix[2][2] - mag_bf[2] * dcm_matrix[2][1];

    // Tilt compensated magnetic field X component:
    double headX = mag_bf[0] * cos_pitch_sq - dcm_matrix[2][0] * (mag_bf[1] * dcm_matrix[2][1] + mag_bf[2] * dcm_matrix[2][2]);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    double heading = constrain_double(atan2(-headY,headX), -3.15, 3.15);

    // Declination correction
    heading = heading + (_current_compass_declination * DEG_TO_RAD);
    if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
        heading -= (2.0 * M_PI);
    else if (heading < -M_PI)
        heading += (2.0 * M_PI);
    return heading;
}


/**
 * if we should use fast gain to align to non imu sensor
 * @return  : true if use fast gain
 */
static bool use_fast_gains(void)
{
    return (HAL_GetTick()<15000u);
}

/**
 * The multiple heading sources are used to estimate the heading.
 * The main heading sources are magnetic heading, RTK heading and yaw rate.
 * The fault-tolerant course source is lost and the quality of course source becomes poor.
 */
// bit 1: compass on; bit 2: RTK on; bit 3: compass+RTK
uint8_t _heading_source_code = 0x00;
static double _heading_cps_obs = 0.0;
static double _heading_magnetic = 0.0;
static double _heading_diver_time = 0.0;
static double observation_heading(float dt)
{
    // // define states and measures
     static double Hstate[2] = {0.0};				// Hstate: heading, omega
    // static double Hstate_last[2] = {0.0};
    // static double z_car[3] = {0.0};
    // static double heading_last_analysis = 0.0;		// For error analysis
    // static double offset_Hc_to_Hr = 0.0;
    // //static bool   have_initial_hr = false;

    // /// source input: heading, update_time, covariance
    // // source 1: RTK heading
    // const double Hr_car = wrap_180_cd_double2(ins_get_heading()*0.01f);
    // const uint64_t Hr_update_ms = ins_get_last_hd_time_ms();
    // float Hr_covariance = txg_std_sub_ins_heading_covariance();
    // // source 2: compass heading
    // const double Hc_car = calculate_heading(_dcm_matrix)*RAD_TO_DEG;
    // const uint64_t Hc_update_us = txg_std_sub_compass_last_update_time_us();
    // float Hc_covariance = txg_std_sub_compass_covariance();
    // _heading_magnetic = Hc_car;
    // // source 3: omega yaw
    // const double omega[3] = {txg_std_sub_imu_gyro(0),txg_std_sub_imu_gyro(1),txg_std_sub_imu_gyro(2)};
    // const double Ho_car = _dcm_matrix[2][0]*omega[0]+_dcm_matrix[2][1]*omega[1]+_dcm_matrix[2][2]*omega[2];
    // const double Ho_covariance = 0.01;   // 0.1*0.1;

    // // date fusion
    // if(!ahrs_flags.have_initial_yaw)
    // {
    //     if(ins_is_head_good() && Hr_update_ms && (vector_length(_omega)<20.0*DEG_TO_RAD))
    //     {
    //         Hstate[0] = ins_get_heading()*0.01f;
    //         //have_initial_hr = true;
    //         ahrs_flags.have_initial_yaw = true;
    //         matrix_from_euler(_dcm_matrix, dcm_roll, dcm_pitch, radians(Hstate[0]));
    //     }
    //     else
    //     {
    //         if (txg_std_sub_compass_sensor_enabled() && Hc_update_us)
    //         {
    //             Hstate[0] = Hc_car;
    //             ahrs_flags.have_initial_yaw = true;
    //             matrix_from_euler(_dcm_matrix, dcm_roll, dcm_pitch, radians(Hstate[0]));
    //         }
    //         else
    //         {
    //             ahrs_flags.have_initial_yaw = false;
    //         }
    //     }
    //     heading_last_analysis = Hstate[0];
    // }
    // else
    // {
    //     // The initial heading value is introduced into RTK heading
    //     if(!txg_actuator_motors_armed_cmd())
    //     {

    //         if(ins_is_head_good() && Hr_update_ms && (vector_length(_omega)<20.0*DEG_TO_RAD))
    //         {
    //             const float ofs = wrap_180_cd_double2(Hr_car - Hc_car);
    //             if (fabsf(ofs) < 20.0)
    //             {
    //                 offset_Hc_to_Hr = ofs;
    //             }
    //             else
    //             {
    //                 // do nothing
    //             }
    //             //have_initial_hr = true;
    //         }
    //     }

    //     /**
    //     * ADMISSION JUDGEMENT
    //     * Determine whether the heading source is effective
    //     */
    //     bool rtk_hd_on	= false;
    //     // Reverse direction judgment of RTK measurement
    //     static bool check_RTK_ante_inv = false;
	// 	if(!check_RTK_ante_inv)
	// 	{
	//         double offset_cr = wrap_180_cd_double2(Hr_car - Hc_car);
	//         if(txg_std_sub_compass_sensor_enabled() && ins_is_head_good())
	//         {
	//         	if(fabsf(offset_cr)>135.0)
	//         	{
	// 	            // Only when RTK heading exists
	// 	            rtk_hd_on = false;
	// 	            _heading_source_code &= 0xFD;
	// 	            heading_RTK_ante_inv = true;
	//         	}
	// 			else
	// 			{
	// 				rtk_hd_on = true;
	// 	            _heading_source_code |= 0x02;
	// 	            heading_RTK_ante_inv = false;
	// 			}
	// 			check_RTK_ante_inv = true;
	//         }
	//         else
	//         {
	//             rtk_hd_on = true;
	//             _heading_source_code |= 0x02;
	//             heading_RTK_ante_inv = false;
	//         }
	// 	}

    //     /**
    //     * UPDATE MEASURES
    //     */
    //     // RTK heading access judgment
    //     if(txg_std_sub_ins_is_head_good())
    //     {
    //         rtk_hd_on = true;
    //     }
    //     else
    //     {
    //         rtk_hd_on = false;
    //     }
    //     // Using RTK course incremental updating method
    //     // The problems of jump, lag and the effect of large lag in rotation are considered.
    //     static double omega_bar = 0.0;
    //     static uint16_t omega_inc = 0;
    //     if(rtk_hd_on)
    //     {
    //         static uint64_t Hr_last_update_ms = 0;
    //         static double Hr_car_last = 0.0;
    //         // Update according to time stamp when quality is good
    //         if(Hr_last_update_ms != Hr_update_ms)
    //         {
    //             // initial
    //             if(!Hr_last_update_ms && Hr_update_ms)
    //             {
    //                 Hr_car_last = Hr_car;
    //                 Hstate_last[0] = Hstate[0];
    //             }

    //             // Observations update
    //             z_car[0] = Hr_car;

    //             // Calculate the difference
    //             const double delta_Hr_car  = wrap_180_cd_double2(Hr_car - Hr_car_last);
    //             const double delta_heading = wrap_180_cd_double2(Hstate[0] - Hstate_last[0]);
    //             const double delta_Hr_heading = wrap_180_cd_double2(delta_Hr_car - delta_heading);
    //             if(fabsf(delta_Hr_heading) < 10.0)
    //             {
    //                 if(fabsf(delta_Hr_heading) > 3.0)
    //                 {
    //                     Hr_covariance *= isq(delta_Hr_heading/3.0);
    //                 }
    //                 _heading_source_code |= 0x02;
    //             }
    //             else
    //             {
    //                 _heading_source_code &= 0xFD;
    //             }

    //             // update covariance due omega
    //             const float kc_due_omega = constrain_double(fabsf(omega_bar/20.0), 1.0, 3.0);
    //             Hr_covariance *= (kc_due_omega*kc_due_omega);
    //             omega_bar = 0.0;

    //             // update last value and time
    //             Hr_car_last = Hr_car;
    //             Hr_last_update_ms = Hr_update_ms;
    //         }
    //         else
    //         {
    //             omega_inc ++;
    //             omega_bar = (omega_bar*(omega_inc-1) + Ho_car)/omega_inc;
    //         }
    //     }
    //     else
    //     {
    //         omega_bar = 0.0;
    //         omega_inc = 0;
    //         // When the quality of data source is poor, the fusion result is used to replace the observation value,
    //         // and the covariance is enlarged
    //         z_car[0] = Hstate[0];
    //         Hr_covariance = 1e6;
    //         _heading_source_code &= 0xFD;
    //     }

    //     // Access judgment of magnetic heading
    //     // Update magnetic heading
    //     // The problem of magnetic heading measurement jump and deviation is considered.
    //     if (txg_std_sub_compass_sensor_enabled())
    //     {
    //         static uint64_t Hc_last_update_us = 0;
    //         static double Hc_car_last = 0.0;
    //         // Update according to time stamp when quality is good
    //         if (Hc_last_update_us != Hc_update_us)
    //         {
    //             // initialize
    //             if (!Hc_last_update_us && Hc_update_us)
    //             {
    //                 Hc_car_last = Hc_car;
    //                 Hstate_last[0] = Hstate[0];
    //             }
    //             // Observations update
    //             const double delta_Hc_car  = wrap_180_cd_double2(Hc_car - Hc_car_last);
    //             const double delta_heading = wrap_180_cd_double2(Hstate[0] - Hstate_last[0]);
    //             const double delta_Hc_heading = wrap_180_cd_double2(delta_Hc_car - delta_heading);
    //             if (fabsf(delta_Hc_heading) < 10.0)
    //             {
    //                 z_car[1] = wrap_180_cd_double2(Hc_car + offset_Hc_to_Hr);
    //                 _heading_source_code |= 0x01;
    //             }
    //             else
    //             {
    //                 z_car[1] = Hstate[0];
    //                 _heading_source_code &= 0xFE;
    //             }
    //             Hc_car_last = Hc_car;
    //             Hc_last_update_us = Hc_update_us;
    //         }
    //     }
    //     else
    //     {
    //         // When the quality of data source is poor, the fusion result is used to replace the observation value,
    //         // and the covariance is enlarged
    //         z_car[1] = Hstate[0];
    //         Hc_covariance = 1e6;
    //         _heading_source_code &= 0xFE;
    //     }

    //     // update kf heading history value
    //     Hstate_last[0] = Hstate[0];

    //     // Observations update
    //     z_car[2] = Ho_car;

    //     /**
    //     *  PREDICT and UPDATE
    //     *  x_ = F * x
    //     *  P_ = F * P * F' + Q
    //     *  y  = z^ - H * x_
    //     *  K  = P_ * H' * inv( H * P_* H' + R )
    //     *  x  = x_ + K * y
    //     *  P  = P_ - K * H * P_
    //     */
    //     // define matrix and vectors
    //     static double P[2][2] = {{1.0,0.0},{0.0,1.0}};
    //     const  double R[3][3] = {{Hr_covariance,0.0,0.0},{0.0,Hc_covariance,0.0},{0.0,0.0,Ho_covariance}};
    //     const  double H[3][2] = {{1.0, dt},{1.0, dt},{0.0, 1.0}};
    //     const  double F[2][2] = {{1.0, dt},{0.0, 1.0}};
    //     const  double Q[2][2] = {{0.5,0.0},{0.0, 0.1}};

    //     /// PREDICT states
    //     Hstate[0] = F[0][0]*Hstate[0] + F[0][1]*Hstate[1];
    //     Hstate[1] = F[1][0]*Hstate[0] + F[1][1]*Hstate[1];
    //     /// PREDICT: P
    //     const double FPF00 = P[0][0] + (P[1][0]+P[0][1])*dt + P[1][1]*dt*dt;
    //     const double FPF01 = P[0][1] + P[1][1]*dt;
    //     const double FPF10 = P[1][0] + P[1][1]*dt;
    //     const double FPF11 = P[1][1];
    //     P[0][0] = FPF00 + Q[0][0];
    //     P[0][1] = FPF01 + Q[0][1];
    //     P[1][0] = FPF10 + Q[1][0];
    //     P[1][1] = FPF11 + Q[1][1];
    //     /// UPDATE: Measurement deviation
    //     static double y[3] = {0.0};
    //     y[0] = z_car[0] - (H[0][0]*Hstate[0] + H[0][1]*Hstate[1]);
    //     y[1] = z_car[1] - (H[1][0]*Hstate[0] + H[1][1]*Hstate[1]);
    //     y[2] = z_car[2] - (H[2][0]*Hstate[0] + H[2][1]*Hstate[1]);
    //     y[0] = wrap_180_cd_double2(y[0]);
    //     y[1] = wrap_180_cd_double2(y[1]);
    //     /// UPDATE: kalman gain
    //     // H*P*H'
    //     const double HPH00 = P[0][0] + (P[1][0]+P[0][1])*dt + P[1][1]*dt*dt;
    //     const double HPH01 = HPH00;
    //     const double HPH02 = P[0][1]+P[1][1]*dt;
    //     const double HPH10 = HPH00;
    //     const double HPH11 = HPH00;
    //     const double HPH12 = HPH02;
    //     const double HPH20 = P[1][0]+P[1][1]*dt;
    //     const double HPH21 = HPH20;
    //     const double HPH22 = P[1][1];
    //     // H*P*H' + R
    //     const double HPH00R = HPH00 + R[0][0];
    //     const double HPH01R = HPH01;
    //     const double HPH02R = HPH02;
    //     const double HPH10R = HPH10;
    //     const double HPH11R = HPH11 + R[1][1];
    //     const double HPH12R = HPH12;
    //     const double HPH20R = HPH20;
    //     const double HPH21R = HPH21;
    //     const double HPH22R = HPH22 + R[2][2];
    //     const double HPHR[3][3] = {{HPH00R,HPH01R,HPH02R},{HPH10R,HPH11R,HPH12R},{HPH20R,HPH21R,HPH22R}};
    //     // (H*P*H'+R)^(-1)
    //     double HPHRI[3][3] = {0.0};
    //     matrix_inversion((double*)HPHR, 3, (double*)HPHRI);
    //     for (uint8_t i=0; i<3; i++)
    //     {
    //         for (uint8_t j=0; j<3; j++)
    //         {
    //             if (isnan(HPHRI[i][j]) || isinf(HPHRI[i][j]))
    //             {
    //                 HPHRI[i][j] = 0.0;
    //                 return Hstate[0];
    //             }
    //         }
    //     }
    //     // K
    //     double PHt[2][3] = {0.0};
    //     PHt[0][0] = P[0][0]+P[0][1]*dt;
    //     PHt[0][1] = PHt[0][0];
    //     PHt[0][2] = P[0][1];
    //     PHt[1][0] = P[1][0]+P[1][1]*dt;
    //     PHt[1][1] = PHt[1][0];
    //     PHt[1][2] = P[1][1];
    //     double K[2][3] = {0.0};
    //     matrix_multiply((double*)PHt, (double*)HPHRI, 2, 3, 3, (double*)K);
    //     /// UPDATE: states
    //     Hstate[0] += (K[0][0]*y[0] + K[0][1]*y[1] + K[0][2]*y[2]);
    //     Hstate[1] += (K[1][0]*y[0] + K[1][1]*y[1] + K[1][2]*y[2]);
    //     Hstate[0] = wrap_180_cd_double2(Hstate[0]);
    //     Hstate[1] = wrap_180_cd_double2(Hstate[1]);
    //     /// UPDATE: P = P-KHP
    //     double KH[2][2] = {0.0};
    //     KH[0][0] = K[0][0]+K[0][1];
    //     KH[0][1] = K[0][0]*dt+K[0][1]*dt+K[0][2];
    //     KH[1][0] = K[1][0]+K[1][1];
    //     KH[1][1] = K[1][0]*dt+K[1][1]*dt+K[1][2];
    //     double KHP[2][2] = {0.0};
    //     matrix_multiply((double*)KH, (double*)P, 2, 2, 2, (double*)KHP);
    //     P[0][0] -= KHP[0][0];
    //     P[0][1] -= KHP[0][1];
    //     P[1][0] -= KHP[1][0];
    //     P[1][1] -= KHP[1][1];

    //     /**
    //     * ERROR ANALYSIS
    //     */
    //     static double heading_inc_ori  = 0.0;
    //     static double heading_inc_diff = 0.0;
    //     static double heading_inc_time = 0.0;
    //     static double heading_inc_offset = 0.0;
    //     if (_heading_source_code == 0x00)
    //     {
    //         /// FULL LOAD METHOD
    //         const float delta_heading = heading_inc_diff - heading_inc_ori;
    //         if (fabsf(delta_heading)>10.0 && fabsf(heading_inc_diff)>90.0)
    //         {
    //             omega_scale = heading_inc_ori/delta_heading;
    //         }

    //         // Analyze the scale factor of calculation
    //         omega_scale = constrain_double(omega_scale, 0.9, 1.1);

    //         // Estimation of gyroscope bias
    //         double gyro_offset = 0.0;
    //         if (heading_inc_time < 1.0)
    //         {
    //             gyro_offset = 0.1;
    //         }
    //         else
    //         {
    //             gyro_offset = (heading_inc_ori-heading_inc_diff)/heading_inc_time;
    //         }
    //         gyro_offset = constrain_double(fabsf(gyro_offset), 0.02, 0.1);
    //         heading_inc_offset += gyro_offset*dt;
    //         _heading_diver_time = 10.0 / gyro_offset;
    //         if (fabsf(heading_inc_offset) > 10.0)
    //         {
    //             heading_divergence = true;
    //         }
    //     }
    //     else
    //     {
    //         if (!get_land_complete())
    //         {
    //             // Original integral
    //             heading_inc_ori += Hstate[1]*dt;

    //             // Differential integral calculus
    //             float heading_delta = wrap_180_cd_double2(Hstate[0]-heading_last_analysis);
    //             heading_last_analysis = Hstate[0];
    //             heading_inc_diff += heading_delta;

    //             // integration time
    //             heading_inc_time += dt;
    //         }

    //         // Notification status
    //         heading_divergence = false;
    //         heading_inc_offset = 0.0;
    //     }

    //     /// Fault tolerance: course source failure and recovery

    // }

    // // output
    // _heading_cps_obs = wrap_180_cd_double2(Hstate[0] - offset_Hc_to_Hr);

    // over
    return Hstate[0];
}


/**
 * Store fusion base of heading to historical buffer
 * @param pos_x : fusion heading base
 */
static void to_hist_heading_buff(double heading)
{
    hist_heading[heading_buff_index] = heading;

    heading_buff_index++;
    if (heading_buff_index >= AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS)
    {
        heading_buff_index = 0;
    }

    heading_buff_num++;
    if (heading_buff_num >= AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS)
    {
        heading_buff_num = AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS;
    }
}

/**
 * yaw drift correction using the compass or GPS
 * this function prodoces the _omega_yaw_P vector, and also
 * contributes to the _omega_I.z long term yaw drift estimate
 */
static void drift_correction_yaw(double dt)
{
    // // update heading
    // const double heading_rad = radians(observation_heading(dt));
    // double dcm_yaw_lag = 0.0;

    //  // store 3rd order estimate (i.e. horizontal position) for future use at40hz(25ms/count)
    // historic_heading_counter++;

    // // we store historical heading at 40hz so 8 iterations implies 200ms ago
    // if (historic_heading_counter >= MAIN_LOOP_RATE/40)
    // {
    //     historic_heading_counter = 0;
    //     to_hist_heading_buff(dcm_yaw);
    // }

    // if (txg_std_sub_ins_is_head_good() && heading_buff_num >= AP_INTERTIALNAV_GPS_LAG_IN_40HZ_INCREMENTS)
    // {
    //     dcm_yaw_lag = hist_heading[heading_buff_index];
    // }
    // else
    // {
    //     dcm_yaw_lag = dcm_yaw;
    // }

    // // compute error in rad
    // _error_yaw = wrap_PI(heading_rad - dcm_yaw_lag);
    // if (!get_yaw_source())
    // {
    //     vector_product_equal(_omega_yaw_P, 0.97);
    //     return;
    // }
    // // obtain dcm fusion error
    // double yaw_error = sinf(_error_yaw);

    // // convert the error vector to body frame
    // double error_z = _dcm_matrix[2][2] * yaw_error;

    // /**
    // * The object has a large moment of inertia. In order to get better fusion results, we need to reduce the gain
    // */
    // static double yaw_error_fd = 0.0;
    // yaw_error_fd = 0.8*yaw_error_fd + 0.2*yaw_error;

    // double err_yaw_limit = 0.7;
    // if (!txg_actuator_is_motors_armed())
    // {
    //     err_yaw_limit = 0.3;
    // }
    // else
    // {
    //     /// TODO: this should be reletive to spin rate which spin faster, lower limit.
    //     err_yaw_limit = 0.7;
    // }

    // // the spin rate changes the P gain, and disables the
    // // integration at higher rates
    // double spin_rate = vector_length(_omega);
    // if (spin_rate > DEG_TO_RAD*50)
    // {
    //     err_yaw_limit = 0.2;
    // }

    // double error_yaw_scale = 1.0 - sqrtf(constrain_double(_error_yaw, 0.0, err_yaw_limit));
    // yaw_error *= error_yaw_scale;

    // // sanity check _kp_yaw
    // if (_kp_yaw < AP_AHRS_YAW_P_MIN)
    // {
    //     _kp_yaw = AP_AHRS_YAW_P_MIN;
    // }

    // // update the proportional control to drag the
    // // yaw back to the right value. We use a gain
    // // that depends on the spin rate. See the fastRotations.pdf
    // // paper from Bill Premerlani
    // // We also adjust the gain depending on the rate of change of horizontal velocity which
    // // is proportional to how observable the heading is from the acceerations and GPS velocity
    // // The accelration derived heading will be more reliable in turns than compass or GPS
    // _omega_yaw_P[2] = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();

    // if (use_fast_gains())
    // {
    //     _omega_yaw_P[2] *= 8.0;
    // }

    // if (spin_rate < DEG_TO_RAD*SPIN_RATE_LIMIT)
    // {
    //     // also add to the I term
    //     _omega_I_sum[2] += (error_z * _ki_yaw * dt);
    // }
}

/**
 * return an accel vector delayed by AHRS_ACCEL_DELAY samples for a
 * specific accelerometer instance
 */
static void ra_delayed(double ra[3], double out[3])
{
    int8_t i = 0;

    // get the old element, and then fill it with the new element
    for (i=0; i<3; i++)
    {
        out[i] = _ra_delay_buffer[i];

    }
    for (i=0; i<3; i++)
    {
        _ra_delay_buffer[i] = ra[i];
    }

    if (is_zero(out))
    {
        // use the current vector if the previous vector is exactly
        // zero. This prevents an error on initialisation
        for (int8_t j=0; j<3; j++)
        {
            out[j] =  ra[j];
        }
    }
}

/**
 * perform drift correction. This function aims to update _omega_P and
 * _omega_I with our best estimate of the short term and long term
 * gyro error. The _omega_P value is what pulls our attitude solution
 * back towards the reference vector quickly. The _omega_I term is an
 * attempt to learn the long term drift rate of the gyros.
 * This drift correction implementation is based on a paper
 * by Bill Premerlani from here:
 * http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
 * @param deltat : delta time from last update
 */
static void drift_correction(double deltat)
{
    double velocity[3]            = {0.0, 0.0, 0.0};
    double temp[3]                = {0.0, 0.0, 0.0};
    double curr_acc[3]            = {0.0, 0.0, 0.0};
    uint64_t last_correction_time = 0;
    uint8_t index                 = 0;
    static bool _have_gps_lock    = false;

    // We only use GNSS aid if
    // 1# gps status good or RTK
    // 2# sat_num>8
    // 3# !failsafe gps
    // 4# gnss_ok
    //uint8_t gps_fixT = (txg_kernel_sub_gnss_status()>=4 && txg_kernel_sub_gnss_satnum()>8 && !failsafe_gps_get_status() && txg_kernel_sub_is_gnss_ok());	//marked by wgs
    uint8_t gps_fixT = 0;

    // perform yaw drift correction if we have a new yaw reference vector
    drift_correction_yaw(deltat);

    curr_acc[0] = - imu_get_acc(IMU_INSTANCE_MPU, 0);
    curr_acc[1] = - imu_get_acc(IMU_INSTANCE_MPU, 1);
    curr_acc[2] = - imu_get_acc(IMU_INSTANCE_MPU, 2);

    // rotate accelerometer values into the earth frame
    matrix_multi_vector(_dcm_matrix, curr_acc, _accel_ef);
    // integrate the accel vector in the earth frame between GPS readings
    for (int8_t i=0; i<3; i++)
    {
        _ra_sum[i] += _accel_ef[i] * deltat;
    }

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

        // increase to 1.0
        if (gps_gain < 1.0)
        {
            gps_gain += 0.0025;
        }
        gps_gain = fminf(gps_gain, 1.0f);

    /// if gps status<GPS_OK_FIX_3D || gps_sat_num<10 || no GPS, gps_fixT=0
    if (gps_fixT == 0)
    {
        // no GPS, or not a good lock. From experience we need at
        // least 6 satellites to get a really reliable velocity number
        // from the GPS.
        //
        // As a fallback we use the fixed wing acceleration correction
        // if we have an airspeed estimate (which we only have if
        // _fly_forward is set), otherwise no correction
        if (_ra_deltat < 0.2)
        {
            // not enough time has accumulated
            return;
        }
        double airspeed = _last_airspeed;

        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        colx_of(_dcm_matrix, temp);
        vector_product(temp, airspeed, velocity);

        /// add in wind estimate
        vector_add_equal(velocity, _wind);

        last_correction_time = HAL_GetTick();
        _have_gps_lock = false;
    }
    else
    {
        /// TODO: last good time or fix time??
        if (0 == _ra_sum_start)
        {
            // we don't have a new GPS fix - nothing more to do
            return;
        }

        // velocity[0] = txg_kernel_sub_gnss_vel_n()*0.01;
        // velocity[1] = txg_kernel_sub_gnss_vel_e()*0.01;
        // velocity[2] = txg_kernel_sub_gnss_vel_d()*0.01;

        // velocity[0] = txg_kernel_sub_iNav_vel_x()*0.01;
        // velocity[1] = txg_kernel_sub_iNav_vel_y()*0.01;
        // velocity[2] = -txg_kernel_sub_iNav_vel_z()*0.01;

        last_correction_time = 0;
        if (_have_gps_lock == false)
        {
            // if we didn't have GPS lock in the last drift
            // correction interval then set the velocities equal
            for (index=0; index<3; index++)
            {
                _last_velocity[index] = velocity[index];
            }
        }
        _have_gps_lock = true;

        // keep last airspeed estimate for dead-reckoning purposes
        double airspeed[3] = {0.0,0.0,0.0};
        for (index=0; index<3; index++)
        {
            airspeed[index] = velocity[index] - _wind[index];
        }
        airspeed[2] = 0;
        _last_airspeed = vector_length(airspeed);
    }

    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0)
    {
        _ra_sum_start = last_correction_time;
        for (index=0; index<3; index++)
        {
            _last_velocity[index] = velocity[index];
        }
        return;
    }

    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    double GA_e[3] = {0, 0, -1.0};

    bool using_gps_corrections = false;
    double ra_scale = 1.0/(_ra_deltat*GRAVITY);

    if (ahrs_flags.correct_centrifugal && _have_gps_lock)
    {
        double v_scale = gps_gain*ra_scale;		// here v_scale=gps_gain*ra_scale, gps_gain=1.0
        double vdelta[3] = {0, 0, 0};

        for (index = 0; index<3; index++)
        {
            vdelta[index] = (velocity[index] - _last_velocity[index]) * v_scale;
            GA_e[index] += vdelta[index];
        }

        vector_normalize(GA_e);

        if (vector_is_inf(GA_e))
        {
            // wait for some non-zero acceleration information
            return;
        }

        using_gps_corrections = true;
    }

    // calculate the error term in earth frame.
    // we do this for each available accelerometer then pick the
    // accelerometer that leads to the smallest error term. This takes
    // advantage of the different sample rates on different
    // accelerometers to dramatically reduce the impact of aliasing
    // due to harmonics of vibrations that match closely the sampling
    // rate of our accelerometers. On the Pixhawk we have the LSM303D
    // running at 800Hz and the MPU6000 running at 1kHz, by combining
    // the two the effects of aliasing are greatly reduced.
    double error[3] = {0.0, 0.0, 0.0};
    double error_dirn = 1.0;
    double best_error = 0.0;
    double GA_b[3] = {0.0, 0.0, 0.0};

    vector_product_equal(_ra_sum,ra_scale);

    // get the delayed ra_sum to match the GPS lag
    if (using_gps_corrections)
    {
        ra_delayed(_ra_sum,GA_b);
    }
    else
    {
        for (index = 0; index<3; index++)
        {
            GA_b[index] = _ra_sum[index];
        }
    }
    /** check is norm zero and normalized */
    if (!is_zero(GA_b))
    {
        vector_normalize(GA_b);
    }
    /// check inf

    /** error = GA_b % GA_e */
    vector_cross_product(GA_b, GA_e, error);
    // Take dot product to catch case vectors are opposite sign and parallel
    // error_dirn = GA_b * GA_e
    error_dirn = vector_dot_product(GA_b, GA_e);

    double error_length = vector_length(error);
    best_error = error_length;

    // Catch case where orientation is 180 degrees out
    if (error_dirn < 0.0)
    {
        best_error = 1.0;
    }

    _error_rp_sum += error_length;
    _error_rp_count++;

#define YAW_INDEPENDENT_DRIFT_CORRECTION    0
#if YAW_INDEPENDENT_DRIFT_CORRECTION
    // step 2 calculate earth_error_Z
    double earth_error_Z = error[2];

    // equation 10
    double tilt = sqrt(isq(GA_e[0]) + isq(GA_e[1]));

    // equation 11
    double theta = atan2f(GA_b[1], GA_b[0]);

    // equation 12
    double GA_e2[3] = {0.0, 0.0, 0.0};
    GA_e2[0] = cosf(theta)*tilt;
    GA_e2[1] = sinf(theta)*tilt;
    GA_e2[2] = GA_e[2];

    // step 6
    vector_cross_product(GA_b, GA_e2, error);
    error[2] = earth_error_Z;
#endif // YAW_INDEPENDENT_DRIFT_CORRECTION

    // to reduce the impact of two competing yaw controllers, we
    // reduce the impact of the gps/accelerometers on yaw when we are
    // flat, but still allow for yaw correction using the
    // accelerometers at high roll angles as long as we have a GPS
    if (gps_fixT != 0)
    {
        error[2] *= sinf(fabsf(dcm_roll));
    }
    else
    {
        error[2] = 0;
    }

    // if ins is unhealthy then stop attitude drift correction and
    // hope the gyros are OK for a while. Just slowly reduce _omega_P
    // to prevent previous bad accels from throwing us off
    // convert the error term to body frame
    matrix_mul_transpose(_dcm_matrix, error, error);

    if (vector_is_nan(error) || vector_is_inf(error))
    {
        // don't allow bad values
        check_matrix();
        return;
    }

    _error_rp = 0.8 * _error_rp + 0.2 * best_error;

    // base the P gain on the spin rate
    double spin_rate = vector_length(_omega);

    // sanity check _kp value
    if (_kp < AP_AHRS_RP_P_MIN)
    {
        _kp = AP_AHRS_RP_P_MIN;
    }

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    vector_product(error, _P_gain(spin_rate) * _kp, _omega_P);
    if (use_fast_gains())
    {
        _omega_P[0] *= 8.0;
        _omega_P[1] *= 8.0;
        _omega_P[2] *= 8.0;
    }

    // accumulate some integrator error
    if (spin_rate < DEG_TO_RAD * SPIN_RATE_LIMIT)
    {
        for (index=0; index<3; index++)
        {
            _omega_I_sum[index] += (error[index] * _ki * _ra_deltat);
        }
        _omega_I_sum_time += _ra_deltat;
    }

    if (_omega_I_sum_time >= 5)
    {
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        double change_limit = _gyro_drift_limit * _omega_I_sum_time;
        _omega_I_sum[0] = constrain_double(_omega_I_sum[0], -change_limit, change_limit);
        _omega_I_sum[1] = constrain_double(_omega_I_sum[1], -change_limit, change_limit);
        _omega_I_sum[2] = constrain_double(_omega_I_sum[2], -change_limit, change_limit);
        vector_add_equal(_omega_I, _omega_I_sum);
        vector_zero(_omega_I_sum);
        _omega_I_sum_time = 0;
    }

    // zero our accumulator ready for the next GPS step
    vector_zero(_ra_sum);
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    for (index=0; index<3; index++)
    {
        _last_velocity[index] = velocity[index];
    }
}

// set the correct centrifugal flag
// allows arducopter to disable corrections when disarmed
void set_correct_centrifugal(bool setting)
{
    ahrs_flags.correct_centrifugal = setting;
}

// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a
// positive vehicle rotation about that axis (ie a negative offset)
static void euler_angles(void)
{
    matrix_multi_matrix(_dcm_matrix, _rot_vehicle_body_to_autopilot_body, _body_dcm_matrix);

    matrix_to_euler(_body_dcm_matrix, &dcm_roll, &dcm_pitch, &dcm_yaw);

    dcm_roll_deg = dcm_roll * RAD_TO_DEG;
    dcm_pitch_deg = dcm_pitch * RAD_TO_DEG;
    dcm_yaw_deg = dcm_compassNormalize(dcm_yaw * RAD_TO_DEG);
}



/**
 * set angle ef trim value in cd.
 * @param trim_x : roll trim angle
 * @param trim_y : pitch trim angle
 */
void ahrs_set_trim(int16_t trim_x, int16_t trim_y)
{
    if (abs(trim_x) < 500)
    {
        trim_angle_ef_cd[0] = trim_x;
    }

    if (abs(trim_y) < 500)
    {
        trim_angle_ef_cd[1] = trim_y;
    }
}

/**
 * update_trigonometric - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
 * should be called after _dcm_matrix is updated
 */
static void update_trigonometric(void)
{
    static int16_t _last_trim[3] = {0, 0};
    double yaw_vector[3] = {0.0};

    if ((_last_trim[0]!=trim_angle_ef_cd[0]) || (_last_trim[1]!=trim_angle_ef_cd[1]))
    {
        _last_trim[0] = trim_angle_ef_cd[0];
        _last_trim[1] = trim_angle_ef_cd[1];
        matrix_from_euler(_rot_vehicle_body_to_autopilot_body, _last_trim[0]*DEGX100_TO_RAD, _last_trim[1]*DEGX100_TO_RAD, 0.0);
        matrix_transposed(_rot_vehicle_body_to_autopilot_body);
    }

    // sin_yaw, cos_yaw
    yaw_vector[0] = _body_dcm_matrix[0][0];
    yaw_vector[1] = _body_dcm_matrix[1][0];
    vector_normalize(yaw_vector);
    _sin_yaw = constrain(yaw_vector[1], -1.0f, 1.0f);
    _cos_yaw = constrain(yaw_vector[0], -1.0f, 1.0f);

    // cos_roll, cos_pitch
    float cx2 = _body_dcm_matrix[2][0] * _body_dcm_matrix[2][0];
    if (cx2 >= 1.0f)
    {
        _cos_pitch = 0;
        _cos_roll = 1.0f;
    }
    else
    {
        _cos_pitch = safe_sqrt(1 - cx2);
        _cos_roll = _body_dcm_matrix[2][2] / _cos_pitch;
    }
    _cos_pitch = constrain(_cos_pitch, 0, 1.0);
    _cos_roll = constrain(_cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing

    // sin_roll, sin_pitch
    _sin_pitch = -_body_dcm_matrix[2][0];
    if (is_zero_D(_cos_pitch))
    {
        _sin_roll = sinf(dcm_roll);
    }
    else
    {
        _sin_roll = _body_dcm_matrix[2][1] / _cos_pitch;
    }

    // sanity checks
    if (vector_is_nan(yaw_vector) || vector_is_inf(yaw_vector))
    {
        yaw_vector[0] = 0.0f;
        yaw_vector[1] = 0.0f;
        _sin_yaw = 0.0f;
        _cos_yaw = 1.0f;
    }

    if (isinf(_cos_roll) || isnan(_cos_roll))
    {
        _cos_roll = cosf(dcm_roll);
    }

    if (isinf(_sin_roll) || isnan(_sin_roll))
    {
        _sin_roll = sinf(dcm_roll);
    }
}

/**
 * convert body variables to earth frame.
 */
void convert_var_of_body_to_earth(double var_bf[3], double var_ef[3])
{
    matrix_multi_vector(_body_dcm_matrix, var_bf, var_ef);
}

/**
 * convert earth variables to body frame.
 */
void convert_var_of_earth_to_body(double var_ef[3], double var_bf[3])
{
    matrix_mul_transpose(_body_dcm_matrix, var_ef, var_bf);
}

/**
 * get euler angle of roll pitch yaw
 * @param  axis : axis
 * @return      : angle in degree
 */
double ahrs_get_angle_deg(uint8_t axis)
{
    double ret = 0.0;

    switch (axis)
    {
        case 0:
            ret = dcm_roll_deg;
            break;
        case 1:
            ret = dcm_pitch_deg;
            break;
        case 2:
            ret = dcm_yaw_deg;
            break;
    }

    return ret;
}


/**
 * normalize yaw heading to 0-360deg
 * @param  heading : current heading
 * @return         : nomalized heading
 */
static double dcm_compassNormalize(double heading)
{
    while (heading < 0.0)
        heading += 360.0;
    while (heading >= 360.0)
        heading -= 360.0;

    return heading;
}

double get_roll_deg(void)
{
  return  get_dmp_ahrs(1);
  //return ahrs_get_angle_deg(0);
}

double get_pitch_deg(void)
{
    return  get_dmp_ahrs(0);
      //  return ahrs_get_angle_deg(1);
}

double get_yaw_deg(void)
{
   return  get_dmp_ahrs(2);
   // return ahrs_get_angle_deg(2);
}

double get_magnetic_heading(void)
{
    double ret = _heading_magnetic;
    while (ret < 0.0)
        ret += 360.0;
    while (ret >= 360.0)
        ret -= 360.0;
    return ret;
}

/**
 * @brief   Return sin roll
 */
double get_sin_roll(void)
{
    return _sin_roll;
}

/**
 * @brief   Return cos roll
 */
double get_cos_roll(void)
{
    return _cos_roll;
}

/**
 * @brief   Return sin pitch
 */
double get_sin_pitch(void)
{
    return _sin_pitch;
}

/**
 * @brief   Return cos pitch
 */
double get_cos_pitch(void)
{
    return _cos_pitch;
}

/**
 * return sin yaw
 * @return  [description]
 */
double get_sin_yaw(void)
{
    return _sin_yaw;
}

/**
 * return cos_yaw
 * @return  [description]
 */
double get_cos_yaw(void)
{
    return _cos_yaw;
}

void get_acc_ef(struct vector3f_t *data)
{
    data->x = ahrs_acc_ef.x;
    data->y = ahrs_acc_ef.y;
    data->z = ahrs_acc_ef.z;
}

double get_acc_std_dev_xy(void)
{
    return acc_std_dev_xy*100;
}

double get_acc_std_dev_z(void)
{
    return acc_std_dev_z*100;
}

// ned frame
double get_gps_pos_offset(uint8_t axis)
{
    return _gps_pos_offset[axis];
}
// ned frame
double get_gps_vel_offset(uint8_t axis)
{
    return _gps_vel_offset[axis];
}
// ned frame
double get_rtk_pos_offset(uint8_t axis)
{
    return _rtk_pos_offset[axis];
}
// ned frame
double get_rtk_vel_offset(uint8_t axis)
{
    return _rtk_vel_offset[axis];
}
// ned frame
double get_ins_pos_offset(uint8_t axis)
{
    return _ins_pos_offset[axis];
}
// ned frame
double get_ins_vel_offset(uint8_t axis)
{
    return _ins_vel_offset[axis];
}
// ned frame
double get_imu_pos_offset(uint8_t axis)
{
    return (_imu_pos_offset[axis]);
}
// ned frame
double get_imu_vel_offset(uint8_t axis)
{
    return (_imu_vel_offset[axis]);
}

/**
 * average error_roll_pitch since last call
 * @return  : this could be a DCM Hstate
 */
double get_error_rp(void)
{
    return _error_rp;
}

/**
 * average error_yaw since last call
 * @return  : this could be a yaw DCM Hstate
 */
double get_error_yaw(void)
{
    return (_error_yaw * RAD_TO_DEG);
}

uint8_t get_yaw_source(void)
{
    return _heading_source_code;
}

bool get_yaw_cps_access(void)
{
    return (_heading_source_code&0x01 == 0x01);
}

/**
 * Obtain heading fusion divergence flag
 */
bool get_heading_divergence(void)
{
    return heading_divergence;
}

/**
* Get RTK install reverse flag
*/
bool get_heading_RTK_ante_inv(void)
{
    return heading_RTK_ante_inv;
}

double get_heading_diver_time(void)
{
    return _heading_diver_time;
}

double get_heading_cps_obs(void)
{
    return wrap_360_cd_double2(_heading_cps_obs - _current_compass_declination);
}


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

//MPU6050�Բ���
//����ֵ:0,����
//    ����,ʧ��
int result;
uint8_t run_self_test(void)
{
	
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static uint32_t dmp_last_update_ms = 0;
static bool dmp_data_valid = false;

//mpu6050,dmp��ʼ��
//����ֵ:0,����
//    ����,ʧ��
uint8_t mpu_dmp_init(void)
{
    uint8_t res=0;
    dmp_data_valid = false;
    dmp_last_update_ms = 0;
    imu_init();
    struct int_param_s int_param;//���ûʲô�ã�����Ϊ���ܸ���ʵ�ε�������		//��ʼ��IIC����
    if(mpu_init(&int_param)!=0)
      return 1;
    res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//��������Ҫ�Ĵ�����
    if(res)return 1; 
    res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//����FIFO
    if(res)return 2; 
    res=mpu_set_sample_rate(100);	//���ò�����
    if(res)return 3; 
    res=dmp_load_motion_driver_firmware();		//����dmp�̼�
    if(res)return 4; 
    res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//���������Ƿ���
    if(res)return 5; 
    res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//����dmp����
        DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
        DMP_FEATURE_GYRO_CAL);
    if(res)return 6; 
    res=dmp_set_fifo_rate(100);	//����DMP�������(��󲻳���200Hz)
    if(res)return 7;   
    res=run_self_test();		//�Լ�
    if(res)return 8;   
    res=mpu_set_dmp_state(1);	//ʹ��DMP
    if(res)return 9;     
	return 0;
}
//�õ�dmp�����������(ע��,��������Ҫ�Ƚ϶��ջ,�ֲ������е��)
//pitch:������ ����:0.1��   ��Χ:-90.0�� <---> +90.0��
//roll:�����  ����:0.1��   ��Χ:-180.0��<---> +180.0��
//yaw:�����   ����:0.1��   ��Χ:-180.0��<---> +180.0��
//����ֵ:0,����
//    ����,ʧ��
#define q30  1073741824.0f


struct MPU_DMP
{
  float gyro[3];
  float accel[3];
  float ahrs[3];
};
struct MPU_DMP mpu_dmp = {0};
void mpu_dmp_get_data(void)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	float pitch_arg;
	bool gyro_updated = false;
	bool attitude_updated = false;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))
          return;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	if (sensors & INV_XYZ_GYRO )
        {
          mpu_dmp.gyro[0] = gyro[0]/32768.0f*2000;
          mpu_dmp.gyro[1] = gyro[1]/32768.0f*2000;
          mpu_dmp.gyro[2] = gyro[2]/32768.0f*2000;
          gyro_updated = true;
        }
	
	if (sensors & INV_XYZ_ACCEL)
        {
          mpu_dmp.accel[0] = accel[0]/32768.0f*2*9.8;
          mpu_dmp.accel[1] = accel[1]/32768.0f*2*9.8;
          mpu_dmp.accel[2] = accel[2]/32768.0f*2*9.8;
        }                  
	
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30��ʽת��Ϊ������
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//����õ�������/�����/�����
		pitch_arg = -2.0f * q1 * q3 + 2.0f * q0 * q2;
		if (pitch_arg > 1.0f) pitch_arg = 1.0f;
		if (pitch_arg < -1.0f) pitch_arg = -1.0f;
		mpu_dmp.ahrs[0] = asinf(pitch_arg) * 57.2957795f;	// pitch
		mpu_dmp.ahrs[1] = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
		                          -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.2957795f;
		mpu_dmp.ahrs[2] = atan2f(2.0f * (q1 * q2 + q0 * q3),
		                          q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f;
                attitude_updated = true;
                
	}
	if (gyro_updated && attitude_updated) {
		dmp_last_update_ms = HAL_GetTick();
		dmp_data_valid = true;
	}
        //else return 2;
	//return 0;
}

bool mpu_dmp_data_is_fresh(uint32_t max_age_ms)
{
    return dmp_data_valid && ((uint32_t)(HAL_GetTick() - dmp_last_update_ms) <= max_age_ms);
}

uint32_t mpu_dmp_get_last_update_ms(void)
{
    return dmp_last_update_ms;
}

float get_dmp_gyro(uint8_t idx)
{ 
    return mpu_dmp.gyro[idx];
}

float get_dmp_accel(uint8_t idx)
{ 
    return mpu_dmp.accel[idx];
}

float get_dmp_ahrs(uint8_t idx)
{ 
    return mpu_dmp.ahrs[idx];
}

