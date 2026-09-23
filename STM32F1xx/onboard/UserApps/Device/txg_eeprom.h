#ifndef __TXG_EEPROM_H
#define __TXG_EEPROM_H

#include <stdint.h>



/** Extern micros declration ******************************************************************** */
#define TXG_PARAM_GROUP_PAGE_SIZE           128


/** Page 1  (0-127):    acc 6axis calib parameters ---------------------------------------------- */
#define EEPROM_PARAM_ACC_6AXIS_ADDR       (128*0)
#define EEPROM_ACC_SCA_6AXIS_ADDR           0
#define EEPROM_ACC_MPU_6AXIS_ADDR           24

// Length definition of acc 6axis calib parameters
#define EEPROM_ACC_SCA_6AXIS_LENGTH         24
#define EEPROM_ACC_MPU_6AXIS_LENGTH         24


/** Page 3  (256-383):  Copter parameters ------------------------------------------------------- */
#define EEPROM_PARAM_COPTER_BASE_ADDR       (128*2)
#define EEPROM_FIRMWARE_VER_ADDR            0       // 1: L=2, firmware version(Now we use only 2Bytes)
#define EEPROM_COPTER_TYPE_ADDR             2       // 2: L=1, Copter type such as heli/multi/fixwing
#define EEPROM_COPTER_LEN_ADDR              3       // 3: L=1, copter length level from 250-xxx(mm)
#define EEPROM_MOTOR_TYPE_ADDR				4		// 4: L=1, motor type define
#define EEPROM_COPTER_MODEL_ADDR            5       // 5: L=1, craft model define
#define EEPROM_COPTER_NUM_ADDR              6       // 5: L=1, craft model define

// Length definition of copter parameters
#define EEPROM_FIRMWARE_VER_LENGTH          2
#define EEPROM_COPTER_TYPE_LENGTH           1
#define EEPROM_COPTER_LEN_LENGTH            1
#define EEPROM_MOTOR_TYPE_LENGTH			1
#define EEPROM_COPTER_MODEL_LENGTH          1
#define EEPROM_COPTER_NUM_LENGTH            1

/** Page 4  (384-639):  Calibrate params -------------------------------------------------------- */
#define EEPROM_PARAM_CALIB_BASE_ADDR        (128*3)
// ESC
#define EEPROM_ESC_CALI_START_ADDR          0       // 1: L=1, ESC calibrate state flag.
// GPS/IMU mount offset
#define EEPROM_GPS_POS_OFFSET_ADDR          1       // 2: L=3, gps mount offset x/y/z in cm
#define EEPROM_IMU_POS_OFFSET_ADDR          4       // 3: L=3, imu mount offset x/y/z in cm
#define EEPROM_RTK_POS_OFFSET_ADDR          64      // 9: L=3, rtk mount offset x/y/z in cm
#define EEPROM_INS_POS_OFFSET_ADDR          51      // 13: L = 5, ins mount offset x/y/z in cm
// Compass
#define EEPROM_COMPASS_RANGE_ADDR           7       // 4: L=12, s16*6
#define EEPROM_COMPASS_RADIUS_ADDR          101
#define EEPROM_COMPASS1_RADIUS_ADDR         103
#define EEPROM_COMPASS2_RADIUS_ADDR         105

//IMU gyro offset
#define EEPROM_IMU1_GYRO_OFFSET_ADDR        107      // L=6, IMU1 gyro offset in dps
#define EEPROM_IMU2_GYRO_OFFSET_ADDR        113      // L=6, IMU1 gyro offset in dps

// Acc or lean angle
#define EEPROM_ACC_SCA_RANGE_ADDR           19      // 5: L=12, s16*6
#define EEPROM_ACC_MPU_RANGE_ADDR           31      // 6: L=12, s16*6
#define EEPROM_TRIM_SCA_XY_ADDR             43      // 7: L=4, s16*2, roll/pitch
#define EEPROM_TRIM_MPU_XY_ADDR             47      // 8: L=4, s16*2, roll/pitch

// Compass 1/2 offset
#define EEPROM_COMPASS1_RANGE_ADDR			70		// 10: L=12, s16*6
#define EEPROM_COMPASS2_RANGE_ADDR			82		// 11: L=12, s16*2
// IMU dir offset
#define EEPROM_IMU_DIR_DEG_ADDR             94      // 12: L=7

// Length of calibrate parameters
#define EEPROM_ESC_CALI_START_LENGTH        1
#define EEPROM_GPS_POS_OFFSET_LENGTH        3
#define EEPROM_IMU_POS_OFFSET_LENGTH        3
#define EEPROM_RTK_POS_OFFSET_LENGTH        5
#define EEPROM_INS_POS_OFFSET_LENGTH        8
#define EEPROM_COMPASS_RANGE_LENGTH         12
#define EEPROM_COMPASS1_RANGE_LENGTH	    12
#define EEPROM_COMPASS2_RANGE_LENGTH		12
#define EEPROM_ACC_SCA_RANGE_LENGTH         12
#define EEPROM_ACC_MPU_RANGE_LENGTH         12
#define EEPROM_TRIM_SCA_XY_LENGTH           4
#define EEPROM_TRIM_MPU_XY_LENGTH           4
#define EEPROM_IMU_DIR_DEG_LENGTH           7
#define EEPROM_COMPASS_RADIUS_LENGTH        2
#define EEPROM_COMPASS1_RADIUS_LENGTH       2
#define EEPROM_COMPASS2_RADIUS_LENGTH       2
#define EEPROM_IMU1_GYRO_OFFSET_LENGTH      6
#define EEPROM_IMU2_GYRO_OFFSET_LENGTH      6


// RC
#define EEPROM_PARAM_RC_CALIB_BASE_ADDR     (128*4)
#define EEPROM_RC_TYPE_ADDR			        0       // 1: L=1, RC receiver type, PPM/PWM/SBUS/XBUS
#define EEPROM_RCINPUT_REV_ADDR				1       // 2: L=2, RC channel reverse, CH1-8 with Bit0-7
#define EEPROM_RC_RANGE_ADDR				3       // 3: L=48, u16*8*(max/mid/min)

// Length of rc calibrate parameters
#define EEPROM_RC_TYPE_LENGTH               1
#define EEPROM_RCINPUT_REV_LENGTH           2
#define EEPROM_RC_RANGE_LENGTH              48

//
#define EEPROM_PARAM_CALIB2_BASE_ADDR       (128*5)
// Compass rotation direct, @degree
#define EEPROM_COMPASS_DIR_DEG_ADDR			0
#define EEPROM_COMPASS1_DIR_DEG_ADDR		7
#define EEPROM_COMPASS2_DIR_DEG_ADDR		14
#define EEPROM_ADVANCED_CALI_ACC_ADDR       21      // 21: L=49, flag + bias + scale + misalign
#define EEPROM_ADVANCED_CALI_GYRO_ADDR      70      // 70: L=49, flag + bias + scale + misalign

#define EEPROM_COMPASS_DIR_DEG_LENGTH		7
#define EEPROM_ADVANCED_CALI_GYRO_LENGTH    49
#define EEPROM_ADVANCED_CALI_ACC_LENGTH     49


/** Page 7  (768-895):  Control params ---------------------------------------------------------- */
#define EEPROM_CTRLPARAM_GP1_BASE_ADDR      (128*6)
// Fcut of LPF
#define EEPROM_LPF_FCUT_RATECTRL_ADDR       0       // 1: L=1, LPF f_cut of rate control
#define EEPROM_LPF_FCUT_ACCNAV_ADDR         1       // 2: L=1, LPF f_cut of acc for nav fusion
#define EEPROM_LPF_VELZ_ERR_ADDR            2       // 3: L=1, LPF f_cut of alt control velz error
#define EEPROM_LPF_ACCZ_ERR_ADDR            3       // 4: L=1, LPF f_cut of alt control accz error

// Length of control parameters group 1
#define EEPROM_LPF_FCUT_RATECTRL_LENGTH     1
#define EEPROM_LPF_FCUT_ACCNAV_LENGTH       1
#define EEPROM_LPF_VELZ_ERR_LENGTH          1
#define EEPROM_LPF_ACCZ_ERR_LENGTH          1


/** Page 8  (896-1023):  Control params --------------------------------------------------------- */
#define EEPROM_CTRLPARAM_GP2_BASE_ADDR      (128*7)
#define EEPROM_PILOT_RC_FEEL_RP_ADDR        0       // 1: L=1, pilot input rc stick feelling of roll/pitch
#define EEPROM_PILOT_RC_FEEL_YAW_ADDR       1       // 2: L=1, pilot input rc stick feeling of yaw
#define EEPROM_PID_ANGLE_RP_ADDR            2       // 3: L=24, float*3*2, angle roll/pitch PID value
#define EEPROM_PID_ANGLE_YAW_ADDR           26      // 4: L=12, float*3, angle yaw PID value
#define EEPROM_PID_RATE_RP_ADDR             38      // 5: L=24, float*3*2, rate roll/pitch PID value
#define EEPROM_PID_RATE_YAW_ADDR            62      // 6: L=12, float*3, rate yaw PID value

// Length of control params group 2
#define EEPROM_PILOT_RC_FEEL_RP_LENGTH      1
#define EEPROM_PILOT_RC_FEEL_YAW_LENGTH     1
#define EEPROM_PID_ANGLE_RP_LENGTH          24
#define EEPROM_PID_ANGLE_YAW_LENGTH         12
#define EEPROM_PID_RATE_RP_LENGTH           24
#define EEPROM_PID_RATE_YAW_LENGTH          12


/** Page 9  (1024-1151):  Control params -------------------------------------------------------- */
#define EEPROM_CTRLPARAM_GP3_BASE_ADDR      (128*8)
#define EEPROM_PID_POS_XY_ADDR              0       // 1: L=12, float*3, same pos x/y PID value
#define EEPROM_PID_POS_Z_ADDR               12      // 2: L=12, float*3, pos z PID value
#define EEPROM_PID_VEL_XY_ADDR              24      // 3: L=12, float*3, same vel x/y PID value
#define EEPROM_PID_VEL_Z_ADDR               36      // 4: L=12, float*3, vel z PID value
#define EEPROM_PID_ACC_XY_ADDR              48      // 5: L=12, float*3, acc x/y PID value
#define EEPROM_PID_ACC_Z_ADDR               60      // 6: L=12, float*3, acc z PID value

// Length of control params group 3
#define EEPROM_PID_POS_XY_LENGTH            12
#define EEPROM_PID_POS_Z_LENGTH             12
#define EEPROM_PID_VEL_XY_LENGTH            12
#define EEPROM_PID_VEL_Z_LENGTH             12
#define EEPROM_PID_ACC_XY_LENGTH            12
#define EEPROM_PID_ACC_Z_LENGTH             12


/** Page 10  (1152-1279):  Control params --------------------------------------------------------- */
#define EEPROM_CTRLPARAM_GP4_BASE_ADDR      (128*9)
#define EEPROM_MOTOR_IDLE_ADDR              0       // 1: L=1, from lowest to highest
#define EEPROM_RTL_ALT_ADDR                 1       // 2: L=2, default rtl alt
#define EEPROM_RTL_HEAD_STRATEGY_ADDR       3       // 2: L=1
// Flight time and active time
#define EEPROM_ACTIVE_TIME_ADDR             4       // 3: L=4, total time since active
#define EEPROM_FLT_TIME_SINCE_ACTIVE_ADDR   8       // 4: L=4, total flight time since active
#define EEPROM_FLT_TIME_SINCE_MAINT_ADDR    12      // 5: L=4, flight maintance period
#define EEPROM_FLT_MAINT_PERIOD_ADDR        16      // 6: L=4, flight time since last maintance
#define EEPROM_MAINT_STRATEGY_ADDR          20      // 7: L=1, maintance strategy
#define EEPROM_FLT_MAINT_ALARM_ADDR         21      // 8: L=4, flight time maintance alarm
#define EEPROM_MAINT_ALARM_STRATEGY_ADDR    25      // 9: L=1, maintance alarm strategy


// Length of control param group 4
#define EEPROM_MOTOR_IDLE_LENGTH            1
#define EEPROM_RTL_ALT_LENGTH               2
#define EEPROM_RTL_HEAD_STRATEGY_LENGTH     1
#define EEPROM_ACTIVE_TIME_LENGTH           4
#define EEPROM_FLT_TIME_SINCE_ACTIVE_LENGTH 4
#define EEPROM_FLT_TIME_SINCE_MAINT_LENGTH  4
#define EEPROM_FLT_MAINT_PERIOD_LENGTH      4
#define EEPROM_MAINT_STRATEGY_LENGTH        1
#define EEPROM_FLT_MAINT_ALARM_LENGTH		4
#define EEPROM_MAINT_ALARM_STRATEGY_LENGTH  1


/** Page 11  (1280-1407):  IAP params ----------------------------------------------------------- */
#define EEPROM_PARAM_IAP_BASE_ADDR          (128*10)
#define EEPROM_IAP_UPDATE_ADDR              0       // 1: L=1, IAP flag
#define EEPROM_IAP_UPDATE_OVER_ADDR         1       // 2: L=1, IAP success, maybe not use
#define EEPROM_IAP_APP_LEN_ADDR             2       // 3: L=4, IAP app total length
#define EEPROM_IAP_CRC_ADDR                 6       // 4: L=2, IAP app CRC

// Length of IAP param
#define EEPROM_IAP_UPDATE_LENGTH            1
#define EEPROM_IAP_UPDATE_OVER_LENGTH       1
#define EEPROM_IAP_APP_LEN_LENGTH           4
#define EEPROM_IAP_CRC_LENGTH               2


/** Page 14  (1664-1791):  Safe method ---------------------------------------------------------- */
#define EEPROM_PARAM_SAFE_BASE_ADDR         (128*13)
#define EEPROM_LOW_BATT_VOT_ADDR            0       // 1: L=5, 4 for lv trgg 1/2, 1 for lb method
#define EEPROM_RC_FS_ADDR                   5       // 2: L=1, rc lost failsafe method
#define EEPROM_API_FS_ADDR                  6       // 2: L=1, api lost failsafe method
#define EEPROM_FENCE_ALT_ADDR               7       // 3: L=2, max alt limit
#define EEPROM_FENCE_RADIUS_ADDR            9       // 4: L=2, 2 for top, 2 for bottom
#define EEPROM_MAX_VEL_XY_ADDR              13      // 5: L=2
#define EEPROM_MAX_VEL_Z_ADDR               15      // 6: L=4
#define EEPROM_MAX_ANGLE_RP_ADDR            19      // 7: L=4
#define EEPROM_MAX_RATE_ANGLE_ADDR          23      // 8: L=6
#define EEPROM_MAX_ACCEL_XY_ADDR            29      // 9: L=2
#define EEPROM_MAX_ACCEL_Z_ADDR             31      // 10: L=2
#define EEPROM_MAX_BRAKE_ANGLE_ADDR         33      // 11: L=1
#define EEPROM_MAX_BRAKE_RATE_ADDR          34      // 12: L=1
#define EEPROM_BRAKE_GAIN_ADDR              35      // 13: L=1
#define EEPROM_LOW_BATT_CAP_ADDR            40      // 14: L=5
#define EEPROM_LINK_FS_STRATEGY_ADDR        50      // 2: L=4, link lost failsafe strategy
#define EEPROM_FDR_LOG_MODE_ADDR            60      // L=1, fdr log mode switch strategy
#define EEPROM_OVERWEIGHT_DETECT            61      // L=3, overweight detection param setting

// Length of param safe
#define EEPROM_LOW_BATT_VOT_LENGTH          5
#define EEPROM_RC_FS_LENGTH                 1
#define EEPROM_API_FS_LENGTH                1
#define EEPROM_FENCE_ALT_LENGTH             2
#define EEPROM_FENCE_RADIUS_LENGTH          2
#define EEPROM_MAX_VEL_XY_LENGTH            2
#define EEPROM_MAX_VEL_Z_LENGTH             4
#define EEPROM_MAX_ANGLE_RP_LENGTH          4
#define EEPROM_MAX_RATE_ANGLE_LENGTH        6
#define EEPROM_MAX_ACCEL_XY_LENGTH          2
#define EEPROM_MAX_ACCEL_Z_LENGTH           2
#define EEPROM_MAX_BRAKE_ANGLE_LENGTH       1
#define EEPROM_MAX_BRAKE_RATE_LENGTH        1
#define EEPROM_BRAKE_GAIN_LENGTH            1
#define EEPROM_LOW_BATT_CAP_LENGTH          5
#define EEPROM_LINK_FS_STRATEGY_LENGTH		4
#define EEPROM_FDR_LOG_MODE_LENGTH          1
#define EEPROM_OVERWEIGHT_LENGTH 3


/** Page 15  (1792-1919):  Others --------------------------------------------------------------- */
#define EEPROM_PARAM_OTHERS_ADDR            (128*14)
#define EEPROM_DEVICE_ENABLE_ADDR           0
#define EEPROM_RADAR_SENS_UP_ADDR           40
#define EEPROM_RADAR_SENS_DOWN_ADDR         41
#define EEPROM_RADAR_VEL_UP_ADDR            42
#define EEPROM_RADAR_VEL_DOWN_ADDR          44
#define EEPROM_RADAR_WORK_ALT_ADDR          46
#define EEPROM_RADAR_TYPE_ADDR              48


// Length of other param
#define EEPROM_DEVICE_ENABLE_LENGTH         20
#define EEPROM_RADAR_SENS_UP_LENGTH         1
#define EEPROM_RADAR_SENS_DOWN_LENGTH       1
#define EEPROM_RADAR_VEL_UP_LENGTH          2
#define EEPROM_RADAR_VEL_DOWN_LENGTH        2
#define EEPROM_RADAR_WORK_ALT_LENGTH        2
#define EEPROM_RADAR_TYPE_LENGTH            1


// PAGE 16: sens params
#define EEPROM_SENS_PARAM_BASE_ADDR         (128*15)
#define EEPROM_SENS_PARAM_ADDR              0
#define EEPROM_SENS_ADD_PARAM_ADDR          16
#define EEPROM_G_MOMENT_PARAM_ADDR			32


// Length of sens param
#define EEPROM_SENS_PARAM_LENGTH            10
#define EEPROM_SENS_ADD_PARAM_LENGTH        13
#define EEPROM_G_MOMENT_PARAM_LENGTH		9

#define EEPROM_TELE_FREQ_PARAM_BASE_ADDR         (128*16)
#define EEPROM_API_PARAM_ADDR              0
#define EEPROM_GCS_PARAM_ADDR              64

// Length of sens param
#define EEPROM_API_PARAM_LENGTH            27
#define EEPROM_GCS_PARAM_LENGTH            27


/** Page 18  (2176-2303):  WP method ------------------------------------------------------------ */
#define EEPROM_PARAM_WP_BASE_ADDR			(128*17)
#define EEPROM_WP_TOTAL_ADDR				0
#define EEPROM_WP_REPEAT_CYCLE_ADDR         2
#define EEPROM_WPTURN_TYPE_ADDR				3
#define EEPROM_WP_LOST_STRATEGY_ADDR        4
#define EEPROM_WP_TRACK_INDEX_ADDR          5
#define EEPROM_WP_PACKET_INDEX_ADDR         6

// Length of wp params
#define EEPROM_WP_TOTAL_LENGTH              2
#define EEPROM_WP_REPEAT_CYCLE_LENGTH       1
#define EEPROM_WPTURN_TYPE_LENGTH           1
#define EEPROM_WP_LOST_STRATEGY_LENGTH      1
#define EEPROM_WP_TRACK_INDEX_LENGTH        1
#define EEPROM_WP_PACKET_INDEX_LENGTH       1


/** Start address of wp store aera ------------------------------------------------------------ */
#define EEPROM_WP_START_BYTE				(128*18)	// 4£º18B/WP, start with page 450, 128 WP take 18 pages, so we store from page 19 to 35

#define EEPROM_WP_SIZE                      32

/** End of definition *************************************************************************** */



/** Declaration of param structure ************************************************************** */
struct eeprom_store_sets_t
{
    const char *name;
    uint8_t  *value;
    uint16_t addr_start;
    uint8_t  addr_offset;
    uint8_t  len;
};

struct param_acc_6axis_t
{
    struct eeprom_store_sets_t acc_sca_6sides_calib;
    struct eeprom_store_sets_t acc_mpu_6sides_calib;
};

struct param_copter_t
{
    struct eeprom_store_sets_t firm_version;
    struct eeprom_store_sets_t craft_type;
    struct eeprom_store_sets_t rotor_base;
	struct eeprom_store_sets_t motor_type;
    struct eeprom_store_sets_t craft_model;
    struct eeprom_store_sets_t craft_num;
};

struct param_calib_t
{
    struct eeprom_store_sets_t esc_calib_start;
    struct eeprom_store_sets_t gps_mount_ofst;
    struct eeprom_store_sets_t imu_mount_ofst;
    struct eeprom_store_sets_t rtk_mount_ofst;
    struct eeprom_store_sets_t ins_mount_ofst;          //add for INS mount x y z
    struct eeprom_store_sets_t imu_dir_deg;
    struct eeprom_store_sets_t mag_range;
    struct eeprom_store_sets_t mag1_range;
    struct eeprom_store_sets_t mag2_range;
    struct eeprom_store_sets_t acc_sca_range;
    struct eeprom_store_sets_t acc_mpu_range;
    struct eeprom_store_sets_t trim_sca_angle_rp;
    struct eeprom_store_sets_t trim_mpu_angle_rp;
    struct eeprom_store_sets_t mag_radius;
    struct eeprom_store_sets_t mag1_radius;
    struct eeprom_store_sets_t mag2_radius;
    struct eeprom_store_sets_t rc_type;
    struct eeprom_store_sets_t rc_input_rev;
    struct eeprom_store_sets_t rc_range;
    struct eeprom_store_sets_t imu_vti_gyro_offset;     //add for VTI imu gyro calibration
    struct eeprom_store_sets_t imu_mpu_gyro_offset;     //add for MPU imu gyro calibration
};

struct param_calib2_t
{
    struct eeprom_store_sets_t cps_dir_deg;
    struct eeprom_store_sets_t cps_dir_deg1;
    struct eeprom_store_sets_t cps_dir_deg2;
    struct eeprom_store_sets_t acc_cali_para;
    struct eeprom_store_sets_t gyro_cali_para;
};

struct param_ctrl_gp1_t
{
    struct eeprom_store_sets_t lpf_fcut_rate_ctrl;
    struct eeprom_store_sets_t lpf_fcut_acc_nav;
    struct eeprom_store_sets_t lpf_fcut_velz_err;
    struct eeprom_store_sets_t lpf_fcut_accz_err;
};

struct param_ctrl_gp2_t
{
    struct eeprom_store_sets_t rc_feel_rp;
    struct eeprom_store_sets_t rc_feel_yaw;
    struct eeprom_store_sets_t pid_angle_rp;
    struct eeprom_store_sets_t pid_angle_yaw;
    struct eeprom_store_sets_t pid_rate_rp;
    struct eeprom_store_sets_t pid_rate_yaw;
};

struct param_ctrl_gp3_t
{
    struct eeprom_store_sets_t pid_pos_xy;
    struct eeprom_store_sets_t pid_pos_z;
    struct eeprom_store_sets_t pid_vel_xy;
    struct eeprom_store_sets_t pid_vel_z;
    struct eeprom_store_sets_t pid_acc_xy;
    struct eeprom_store_sets_t pid_acc_z;
};

struct param_ctrl_gp4_t
{
    struct eeprom_store_sets_t motor_idle;
    struct eeprom_store_sets_t rtl_alt;
    struct eeprom_store_sets_t rtl_head_strategy;
    struct eeprom_store_sets_t active_time;
    struct eeprom_store_sets_t flt_time_since_active;
};

struct param_iap_t
{
    struct eeprom_store_sets_t update_flag;
    struct eeprom_store_sets_t update_over_flag;
    struct eeprom_store_sets_t app_len;
    struct eeprom_store_sets_t app_crc;
};

struct param_safe_method_t
{
    struct eeprom_store_sets_t fs_batt_lv;
    struct eeprom_store_sets_t fs_batt_lc;
    struct eeprom_store_sets_t fs_rc;
    struct eeprom_store_sets_t fs_api;
    struct eeprom_store_sets_t fence_alt;
    struct eeprom_store_sets_t fence_radius;
    struct eeprom_store_sets_t max_vel_xy;
    struct eeprom_store_sets_t max_vel_z;
    struct eeprom_store_sets_t max_angle_rp;
    struct eeprom_store_sets_t max_rate_angle;
    struct eeprom_store_sets_t max_accel_xy;
    struct eeprom_store_sets_t max_accel_z;
    struct eeprom_store_sets_t max_brake_angle;
    struct eeprom_store_sets_t max_brake_rate;
    struct eeprom_store_sets_t brake_gain;
    struct eeprom_store_sets_t fs_link_strategy;
    struct eeprom_store_sets_t fdr_log_mode_switch;     //add for FCU FDR log mode switch
    struct eeprom_store_sets_t overweight_detect;
};

struct param_others_t
{
    struct eeprom_store_sets_t device_enable;
    struct eeprom_store_sets_t radar_sens_up;
    struct eeprom_store_sets_t radar_sens_down;
    struct eeprom_store_sets_t radar_vel_up;
    struct eeprom_store_sets_t radar_vel_down;
    struct eeprom_store_sets_t radar_work_alt;
	struct eeprom_store_sets_t radar_type;
    struct eeprom_store_sets_t sens_for_m2;
	struct eeprom_store_sets_t sens_additional;
	struct eeprom_store_sets_t gravity_moment;
    struct eeprom_store_sets_t tele_freq_api;
    struct eeprom_store_sets_t tele_freq_gcs;
};

struct param_wp_t
{
    struct eeprom_store_sets_t wp_total;
    struct eeprom_store_sets_t repeat_cycle;
    struct eeprom_store_sets_t wp_turn_type;
    struct eeprom_store_sets_t rc_lost_strategy;
    struct eeprom_store_sets_t track_index;
    struct eeprom_store_sets_t wp_packet_index;
    struct eeprom_store_sets_t wp_start;
};

struct param_group_t
{
    struct param_acc_6axis_t    acc_6axis;
    struct param_copter_t		copter;
    struct param_calib_t		calib;
    struct param_calib2_t		calib2;
    struct param_ctrl_gp1_t		ctrl_gp1;
    struct param_ctrl_gp2_t		ctrl_gp2;
    struct param_ctrl_gp3_t		ctrl_gp3;
    struct param_ctrl_gp4_t		ctrl_gp4;
    struct param_iap_t			iap;
    struct param_safe_method_t	safe_method;
    struct param_others_t		others;
    struct param_wp_t			wp;
};

/** End of param structure ********************************************************************** */

extern struct param_group_t param_g;

/** Extern functioon type delcaration *********************************************************** */
void eeprom_init(void);

void eeprom_read_data(uint16_t address, uint8_t length, uint8_t*data);
void eeprom_write_data(uint16_t address, uint8_t length, uint8_t*data);


#endif

/** END OF FILE ********************************************************************************* */
