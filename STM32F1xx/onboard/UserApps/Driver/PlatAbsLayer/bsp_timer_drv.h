/**
  ******************************************************************************
  * @file    bsp_timer_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the  timer
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_TIMER_DRV_H
#define BSP_TIMER_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/  
#include "stm32f1xx_hal.h"
#include <stdbool.h> 
   
/* Exported types ------------------------------------------------------------*/
/** @defgroup TIM Exported Types
  * @{
  */
 typedef enum
 {
   
   TIM3_ID,
   TIM5_ID,
   
 }TIM_ID;

 typedef enum
 {
   
   TIM1_CH1,
   TIM1_CH4,
   
   TIMX_MAX
 }TIMX_CHX;
 
 
 /**
* @}
*/
/* extern macro -------------------------------------------------------------*/
#define BSP_TIM1_PWM_PERIOD             7200U

/* Peripheral Control functions  **********************************************/

void Bsp_Tim1_Outinit(uint16_t ccr_init1,uint16_t ccr_init4);
void Bsp_Tim1_Chx_Set_Ccrout(uint8_t timx_chx, uint16_t ccr_output);
/** 立即装载 CCR 影子寄存器(不等下一个更新事件), 换向时保证 PWM 已真正为 0 */
void Bsp_Tim1_Chx_Force_Update(uint8_t timx_chx);
void Bsp_Timx_Encoderinit(uint8_t tim_id);
/** 编码器自上次调用以来的净脉冲数(有符号), 差值法读取, 不会丢计数 */
int16_t Bsp_Timx_Get_Encoder_Count(uint8_t timx_chx);
/** 编码器计数器的瞬时计数方向: 0 向下, 1 向上 */
uint8_t Bsp_Timx_Get_Encoder_Dir(uint8_t timx_chx);

/** Optional TIM2 IMU timebase. Its ISR only counts ticks, it never reads the
    sensor: IMU sample / attitude fusion run in the main task pipeline. */
void txg_timer_init_imu_sample_trigger(void);
uint32_t Bsp_Tim2_Get_Imu_Tick_Count(void);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
