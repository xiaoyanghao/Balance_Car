#include "control.h"
#include "periph_serial_motor.h"
#include "imu.h"
#include "txg_ahrs_fusion.h"
#include "txg_low_pass_filter.h"
#include <math.h>
#include <string.h>

#define BALANCE_ANGLE_DEFAULT       0.25f
#define BALANCE_KP                 (120.0f * 0.6f)
#define BALANCE_KD                 (12.55f * 0.6f)
#define VEL_KP                     140.0f
#define VEL_KI                     140.0f
#define TURN_KP                    2.4f

#define CONTROL_DT                 (1.0f / 200.0f)   /* 200Hz control loop = 5ms */
#define SPEED_LPF_FCUT             20.0f              /* speed filter cutoff */
#define SPEED_LPF_FS               200.0f            /* speed filter sample rate */

#define LIMIT_SPD_INTEGRAL         8000.0f
#define LIMIT_PWM                  6000.0f
#define TARGET_SPEED_LIMIT         200
#define TARGET_YAW_RATE_LIMIT      180.0f
#define FALL_ANGLE_DEG             30.0f
#define IMU_TIMEOUT_MS             30U
#define COMMAND_TIMEOUT_MS         300U
#define L_MOTOR_PWM_DEAD           180U
#define R_MOTOR_PWM_DEAD           180U
#define MOTOR_DIR_STOP             2U

static struct CONTROL _control;
static float _speed_integral;
static uint32_t _last_command_ms;
static struct SecondOrderLowPass_t _speed_lpf;
static float _prev_yaw_rate_error;
static float _balance_angle = BALANCE_ANGLE_DEFAULT;

static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static int16_t float_to_i16(float value)
{
  value = clamp_float(value, -32767.0f, 32767.0f);
  if (value >= 0.0f)
    return (int16_t)(value + 0.5f);
  else
    return (int16_t)(value - 0.5f);
}

static bool control_value_is_valid(float value)
{
  return isfinite(value) != 0;
}

static void control_reset_dynamic_state(void)
{
  _speed_integral = 0.0f;
  _control.encode_spd = 0.0f;
  _control.encode_spd_integ = 0;
  _prev_yaw_rate_error = 0.0f;
  init_second_order_low_pass(&_speed_lpf, SPEED_LPF_FCUT, SPEED_LPF_FS,
                             0.707, 0.0);
}

static void control_stop_motors(bool reset_state)
{
  _control.active = 0;
  _control.balance_pwm = 0;
  _control.vel_pwm = 0;
  _control.lturn_pwm = 0;
  _control.rturn_pwm = 0;
  _control.lfinal_pwm = 0;
  _control.rfinal_pwm = 0;
  _control.l_dircmd = MOTOR_DIR_STOP;
  _control.r_dircmd = MOTOR_DIR_STOP;
  Periph_Motor_Stop_All();
  if (reset_state) {
    control_reset_dynamic_state();
  }
}

static float control_balance(float pitch, float pitch_rate)
{
  return _control.balance_kp * (pitch - _balance_angle)
       + _control.balance_kd * pitch_rate;
}

static float control_velocity(void)
{
  float measured_speed;
  float speed_error;

  measured_speed = (_control.lencode_spd + _control.rencode_spd) * 0.5f;
  _control.encode_spd = (float)update_second_order_low_pass(&_speed_lpf, (double)measured_speed);
  speed_error = _control.encode_spd - (float)_control.taget_spd;

  _speed_integral += speed_error * CONTROL_DT;
  _speed_integral = clamp_float(_speed_integral,
                                -LIMIT_SPD_INTEGRAL,
                                LIMIT_SPD_INTEGRAL);
  _control.encode_spd_integ = float_to_i16(_speed_integral);

  return -(_control.vel_kp * speed_error + _control.vel_ki * _speed_integral);
}

static float control_turn(float target_yaw_rate)
{
  float yaw_rate_error = _control.yaw_gyro - target_yaw_rate;
  float yaw_rate_deriv = (yaw_rate_error - _prev_yaw_rate_error) / CONTROL_DT;
  _prev_yaw_rate_error = yaw_rate_error;
  return _control.turn_kp * yaw_rate_error + _control.turn_kd * yaw_rate_deriv;
}

static void control_apply_motor(uint8_t motor_id,
                                float signed_pwm,
                                uint16_t dead_zone,
                                uint8_t *direction_cmd)
{
  uint16_t output;
  float magnitude;

  if (signed_pwm == 0.0f) {
    *direction_cmd = MOTOR_DIR_STOP;
    Periph_Motor_Set_Out(motor_id, 0);
    Periph_Motor_Set_ST(motor_id);
    return;
  }

  if (signed_pwm < 0.0f) {
    Periph_Motor_Set_DF(motor_id);
    *direction_cmd = 0;
    magnitude = -signed_pwm;
  } else {
    Periph_Motor_Set_DB(motor_id);
    *direction_cmd = 1;
    magnitude = signed_pwm;
  }

  /* Smooth dead zone compensation: scale the offset proportionally
     to PWM magnitude to eliminate discontinuity at zero crossing */
  {
    float dz = (float)dead_zone;
    float ratio = fminf(magnitude / dz, 1.0f);
    magnitude = magnitude + dz * ratio;
  }
  magnitude = clamp_float(magnitude, 0.0f, (float)BSP_TIM1_PWM_PERIOD);
  output = (uint16_t)magnitude;
  Periph_Motor_Set_Out(motor_id, output);
}

static bool control_parameters_are_valid(const struct NET_RA *param)
{
  if (!control_value_is_valid(param->balance_kp) ||
      !control_value_is_valid(param->balance_kd) ||
      !control_value_is_valid(param->vel_kp) ||
      !control_value_is_valid(param->vel_ki) ||
      !control_value_is_valid(param->turn_kp) ||
      !control_value_is_valid(param->turn_kd) ||
      !control_value_is_valid(param->taget_yaw)) {
    return false;
  }

  return param->balance_kp >= 0.0f && param->balance_kp <= 1000.0f &&
         param->balance_kd >= 0.0f && param->balance_kd <= 100.0f &&
         param->vel_kp >= 0.0f && param->vel_kp <= 1000.0f &&
         param->vel_ki >= 0.0f && param->vel_ki <= 50.0f &&
         param->turn_kp >= 0.0f && param->turn_kp <= 100.0f &&
         param->turn_kd >= 0.0f && param->turn_kd <= 100.0f;
}

void Control_Init(void)
{
  memset(&_control, 0, sizeof(_control));
  _control.balance_kp = BALANCE_KP;
  _control.balance_kd = BALANCE_KD;
  _control.vel_kp = VEL_KP;
  _control.vel_ki = VEL_KI;
  _control.turn_kp = TURN_KP;
  _control.turn_kd = 0.0f;
  _control.enable_cmd = 0;
  _control.fault = CONTROL_FAULT_NONE;
  _last_command_ms = HAL_GetTick();
  control_stop_motors(true);
}

void Control_Spd_Updte(void)
{
  _control.lencode_spd = Periph_Motor_Get_Encoder(MOTORL_ID);
  _control.rencode_spd = Periph_Motor_Get_Encoder(MOTORR_ID);
  _control.l_dir = Periph_Motor_Get_Dir(MOTORL_ID);
  _control.r_dir = Periph_Motor_Get_Dir(MOTORR_ID);
}

void Control_Updte(void)
{
  float balance_pwm;
  float velocity_pwm;
  float turn_pwm;
  float left_pwm;
  float right_pwm;

  _control.tick = HAL_GetTick();

  if (!_control.enable_cmd) {
    _control.fault = CONTROL_FAULT_NONE;
    control_stop_motors(true);
    return;
  }

  if (_control.fault != CONTROL_FAULT_NONE) {
    control_stop_motors(true);
    return;
  }

  if (!mpu_dmp_data_is_fresh(IMU_TIMEOUT_MS)) {
    _control.fault = CONTROL_FAULT_IMU_TIMEOUT;
    control_stop_motors(true);
    return;
  }

  _control.pitch = (float)get_pitch_deg();
  _control.pitch_gyro = (float)imu_get_gyro(IMU_INSTANCE_MPU, 1);
  _control.yaw = (float)get_yaw_deg();
  _control.yaw_gyro = (float)imu_get_gyro(IMU_INSTANCE_MPU, 2);

  if (!control_value_is_valid(_control.pitch) ||
      !control_value_is_valid(_control.pitch_gyro) ||
      !control_value_is_valid(_control.yaw) ||
      !control_value_is_valid(_control.yaw_gyro)) {
    _control.fault = CONTROL_FAULT_NUMERIC;
    control_stop_motors(true);
    return;
  }

  if (fabsf(_control.pitch - _balance_angle) > FALL_ANGLE_DEG) {
    _control.fault = CONTROL_FAULT_TILT;
    control_stop_motors(true);
    return;
  }

  if ((uint32_t)(HAL_GetTick() - _last_command_ms) > COMMAND_TIMEOUT_MS) {
    _control.taget_spd = 0;
    _control.taget_yaw = 0.0f;
  }

  _control.active = 1;
  balance_pwm = control_balance(_control.pitch, _control.pitch_gyro);
  velocity_pwm = control_velocity();
  turn_pwm = control_turn(_control.taget_yaw);

  left_pwm = clamp_float(balance_pwm + velocity_pwm + turn_pwm,
                         -LIMIT_PWM, LIMIT_PWM);
  right_pwm = clamp_float(balance_pwm + velocity_pwm - turn_pwm,
                          -LIMIT_PWM, LIMIT_PWM);

  _control.balance_pwm = float_to_i16(balance_pwm);
  _control.vel_pwm = float_to_i16(velocity_pwm);
  _control.lturn_pwm = float_to_i16(turn_pwm);
  _control.rturn_pwm = float_to_i16(turn_pwm);
  _control.lfinal_pwm = float_to_i16(left_pwm);
  _control.rfinal_pwm = float_to_i16(right_pwm);

  control_apply_motor(MOTORL_ID, left_pwm, L_MOTOR_PWM_DEAD, &_control.l_dircmd);
  control_apply_motor(MOTORR_ID, right_pwm, R_MOTOR_PWM_DEAD, &_control.r_dircmd);
}

void Control_Set_Enable(uint8_t enable)
{
  if (!enable) {
    _control.enable_cmd = 0;
    _control.fault = CONTROL_FAULT_NONE;
    control_stop_motors(true);
  } else if (!_control.enable_cmd) {
    _control.fault = CONTROL_FAULT_NONE;
    _control.enable_cmd = 1;
    _last_command_ms = HAL_GetTick();
  }
}

void Control_Get_Pram(struct CONTROL *control_pram)
{
  if (control_pram != NULL) {
    memcpy(control_pram, &_control, sizeof(struct CONTROL));
  }
}

int8_t Control_Set_PramA(struct NET_RA *param)
{
  if (param == NULL || !control_parameters_are_valid(param)) {
    return -1;
  }

  _control.balance_kp = param->balance_kp;
  _control.balance_kd = param->balance_kd;
  _control.vel_kp = param->vel_kp;
  _control.vel_ki = param->vel_ki;
  _control.turn_kp = param->turn_kp;
  _control.turn_kd = param->turn_kd;
  _control.taget_spd = (int16_t)clamp_float((float)param->taget_spd,
                                            -TARGET_SPEED_LIMIT,
                                            TARGET_SPEED_LIMIT);
  _control.taget_yaw = clamp_float(param->taget_yaw,
                                    -TARGET_YAW_RATE_LIMIT,
                                    TARGET_YAW_RATE_LIMIT);
  _last_command_ms = HAL_GetTick();
  return 0;
}

void Control_Set_BalanceAngle(float angle)
{
  _balance_angle = clamp_float(angle, -10.0f, 10.0f);
}

float Control_Get_BalanceAngle(void)
{
  return _balance_angle;
}

int8_t Control_Set_PramB(struct NET_RB *param)
{
  if (param == NULL || !control_value_is_valid(param->taget_ryaw)) {
    return -1;
  }

  _control.taget_spd = (int16_t)clamp_float((float)param->taget_spd,
                                            -TARGET_SPEED_LIMIT,
                                            TARGET_SPEED_LIMIT);
  _control.taget_yaw = clamp_float(param->taget_ryaw,
                                    -TARGET_YAW_RATE_LIMIT,
                                    TARGET_YAW_RATE_LIMIT);
  _last_command_ms = HAL_GetTick();
  Control_Set_Enable(param->enable);
  return 0;
}
