#ifndef __TXG_AHRS_FUSION_H
#define __TXG_AHRS_FUSION_H

#include "txg_vector3.h"
#include <stdbool.h>
#include <stdint.h>

#define AP_AHRS_RP_P_MIN   	0.05        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  	0.05        // minimum value for AHRS_YAW_P parameter
#define SPIN_RATE_LIMIT 	20 // the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf

// flags structure
struct ahrs_flags_t
{
    uint8_t have_initial_yaw;    // whether the yaw value has been intialised with a reference
    uint8_t fast_ground_gains;   // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
    uint8_t correct_centrifugal; // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
} ;


void ahrs_init_fusion(void);
void ahrs_update_fusion(void);
void compass_set_declination(double curr_dec);
void set_correct_centrifugal(bool setting);
void ahrs_set_trim(int16_t trim_x, int16_t trim_y);

//void reset_gyro_drift(void);
//void convert_var_of_body_to_earth(double var_bf[3], double var_ef[3]);
//void convert_var_of_earth_to_body(double var_ef[3], double var_bf[3]);

double ahrs_get_angle_deg(uint8_t axis);

double get_roll_deg(void);
double get_pitch_deg(void);
double get_yaw_deg(void);
double get_magnetic_heading(void);

double get_sin_roll(void);
double get_cos_roll(void);
double get_sin_pitch(void);
double get_cos_pitch(void);
double get_sin_yaw(void);
double get_cos_yaw(void);

void get_acc_ef(struct vector3f_t *data);

double get_acc_std_dev_xy(void);
double get_acc_std_dev_z(void);

double get_gps_pos_offset(uint8_t axis);
double get_gps_vel_offset(uint8_t axis);
double get_rtk_pos_offset(uint8_t axis);
double get_rtk_vel_offset(uint8_t axis);
double get_ins_pos_offset(uint8_t axis);
double get_ins_vel_offset(uint8_t axis);
double get_imu_pos_offset(uint8_t axis);
double get_imu_vel_offset(uint8_t axis);

double get_error_rp(void);
double get_error_yaw(void);
uint8_t get_yaw_source(void);
bool get_yaw_cps_access(void);
bool get_heading_divergence(void);
bool get_heading_RTK_ante_inv(void);
double get_heading_diver_time(void);
double get_heading_cps_obs(void);
uint8_t mpu_dmp_init(void);
void mpu_dmp_get_data(void);
bool mpu_dmp_data_is_fresh(uint32_t max_age_ms);
uint32_t mpu_dmp_get_last_update_ms(void);

float get_dmp_gyro(uint8_t idx);
float get_dmp_accel(uint8_t idx);
float get_dmp_ahrs(uint8_t idx);

#endif
