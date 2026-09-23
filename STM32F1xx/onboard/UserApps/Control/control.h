/**
  ******************************************************************************
  * @file   control.h
  * @author  YZH
  * @brief   PID控制
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

/* Includes ------------------------------------------------------------------*/

#include<stdint.h>
#include "net_link.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum
{
  CONTROL_FAULT_NONE = 0,
  CONTROL_FAULT_IMU_TIMEOUT,
  CONTROL_FAULT_TILT,
  CONTROL_FAULT_NUMERIC
} CONTROL_FAULT;

/** @addtogroup 通信接口
 * @{
 */
/** @defgroup 网络通信
 * @brief
 * @{
 */

 /** @defgroup  网络通信外部枚举/结构体类型/宏
  * @{
  */ 
/**
 * @brief  控制类命令类参数
 */
struct  CONTROL 
{
  float balance_kp;           /**<直立环比例P*/
  float balance_kd;           /**<直立环微分D*/
  float vel_kp;               /**<速度比例P*/
  float vel_ki;               /**<速度积分I*/ 
  float turn_kp;              /**<转向比例P*/
  float turn_kd;              /**<转向微分D*/

  int16_t balance_pwm;         /**<直立环输出PWM*/
  int16_t vel_pwm;             /**<速度环输出PWM*/
  int16_t lturn_pwm;           /**<左轮转向环输出PWM*/
  int16_t lfinal_pwm;          /**<左轮最终输出PWM*/
  int16_t rturn_pwm;           /**<右轮转向环输出PWM*/
  int16_t rfinal_pwm;          /**<右轮最终输出PWM*/
  uint8_t l_dircmd;            /**<左轮方向命令*/       
  uint8_t r_dircmd;            /**<右轮方向命令*/ 

  int16_t taget_spd;           /**<目标速度*/ 
  int16_t lencode_spd;         /**<左轮编码器速度*/ 
  int16_t rencode_spd;         /**<右轮编码器速度*/ 
  uint8_t l_dir;               /**<左轮方向*/ 
  uint8_t r_dir;               /**<右轮方向*/
  float   encode_spd;          /**<编码器速度*/ 
  int16_t encode_spd_integ;    /**<编码器速度积分*/

  float   taget_yaw;           /**<目标偏航角*/

  float   pitch;               /**<俯仰角*/
  float   pitch_gyro;          /**<俯仰角速度*/
  float   yaw;                 /**<偏航角*/
  float   yaw_gyro;            /**<偏航角速度*/

  uint8_t enable_cmd;          /**< external motor enable command */
  uint8_t active;              /**< closed loop is driving motors */
  uint8_t fault;               /**< latched CONTROL_FAULT value */
  
  uint32_t tick;               /**<命令更新时间 ms*/  

};


/**
* @}
*/  

 /* Exported function ------------------------------------------------------------*/
/** @defgroup 网络通信外部函数 网络通信外部函数
  * @{
  */
void Control_Init(void);
void Control_Updte(void);
void Control_Spd_Updte(void);
void Control_Set_Enable(uint8_t enable);
void Control_Get_Pram(struct  CONTROL *control_pram);
int8_t Control_Set_PramA(struct NET_RA  *pramA);
int8_t Control_Set_PramB(struct NET_RB  *pramB);
void Control_Set_BalanceAngle(float angle);
float Control_Get_BalanceAngle(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif 

/*****************************END OF FILE**************************************/
