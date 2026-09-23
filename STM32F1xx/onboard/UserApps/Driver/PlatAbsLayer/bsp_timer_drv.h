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
void Bsp_Timx_Encoderinit(uint8_t tim_id);
int16_t Bsp_Timx_Get_Encoder_Count(uint8_t timx_chx);
uint8_t Bsp_Timx_Get_Encoder_Dir(uint8_t timx_chx);//0 down 1 up
void txg_timer_init_imu_sample_trigger(void);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
