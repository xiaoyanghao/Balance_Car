/**
  ******************************************************************************
  * @file    bsp_adc_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the  timer
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_ADC_DRV_H
#define BSP_ADC_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/  
#include "stm32f1xx_hal.h"

   
/* Exported types ------------------------------------------------------------*/
/** @defgroup ADC Exported Types
  * @{
  */
 typedef enum
 {
   
   ADC_CH1,
   ADC_CH2,
   ADC_CH3,
   ADC_CH4,
   ADC_CH5, /*sensor temp*/
   
   ADC_CH_MAX
 }ADC_CHX;
 
 /**
  * @brief  gpio mapping
  * @note    
  FUNC         \     PIN    \  ADC_CH                 \
  ADC_CH1      \     PC0    \  ADC_CHANNEL_10         \
  ADC_CH2      \     PC2    \  ADC_CHANNEL_11         \
  ADC_CH3      \     PC3    \  ADC_CHANNEL_12         \
  ADC_CH4      \     PC4    \  ADC_CHANNEL_13         \
  ADC_CH5      \            \  ADC_CHANNEL_TEMPSENSOR \

  SINGLE CH COVPRIOR    (239.5 + 12.5) * (1/12) = 21US
*/ 
   
 /**
* @}
*/
/* extern macro -------------------------------------------------------------*/
void Bsp_Adc_Init(void);
uint16_t Bsp_Get_Adc_Value(uint8_t ADC_CH);
uint32_t Bsp_Get_Adc_Last_Ms(void);

/* Peripheral Control functions  **********************************************/


#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
