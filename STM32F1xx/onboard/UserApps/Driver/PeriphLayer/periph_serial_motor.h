/**
  ******************************************************************************
  * @file    periph_serial_motor.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the port
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_MOTOR_H
#define PERIPH_SERIAL_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif   
/* Includes ------------------------------------------------------------------*/
#include "bsp_timer_drv.h"   

/* Exported types ------------------------------------------------------------*/
/** @defgroup Motor Exported Types
  * @{
  */ 

 typedef enum
 {
                
   MOTORL_ID,
   MOTORR_ID,
   
}MOTOR_ID; 

 typedef enum
 {
   
  MOTOR_DF,           
  MOTOR_DB,         
   
}MOTOR_DIR;

 
 
/**
* @}
*/  

/* Peripheral Control functions  **********************************************/
void Periph_Motor_Init(void);
void Periph_Motor_Set_Out(uint8_t which_motor, uint16_t val);
void Periph_Motor_Set_DF(uint8_t which_motor);
void Periph_Motor_Set_DB(uint8_t which_motor);
void Periph_Motor_Set_ST(uint8_t which_motor);
void Periph_Motor_Stop_All(void);
/**
 * @brief  编码器采样: 一次调用同时得到速度与转动方向
 * @param[in]  which_motor : 电机ID
 * @param[out] delta       : 本采样周期内的净脉冲数(有符号), 允许传入 NULL
 * @param[out] dir         : 与 delta 符号一致的转动方向(>0 为 MOTOR_DF), 允许 NULL
 * @note   这是唯一读取编码器硬件的接口, 每个速度控制周期对每个电机只调用一次。
 *         速度单位是"编码器脉冲数 / 采样周期", 换算见 .c 中的说明。
 */
void Periph_Motor_Get_Encoder_Data(uint8_t which_motor, int16_t *delta, MOTOR_DIR *dir);
/** 最近一次采样得到的速度(脉冲/采样周期), 不访问硬件 */
int16_t Periph_Motor_Get_Encoder_Speed(uint8_t which_motor);
/** 最近一次采样得到的方向, 与速度符号一致 */
MOTOR_DIR Periph_Motor_Get_Dir(uint8_t which_motor);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
