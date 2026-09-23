/**
  ******************************************************************************
  * @file     periph_serial_adc.h
  * @author   zhy
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_ADC_H
#define PERIPH_SERIAL_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_adc_drv.h"

/* Exported types ------------------------------------------------------------*/
/** @defgroup ADS8688 Exported Types
  * @{
  */  
   
   
   /**
  * @brief ADC MODULE
  */
   
 typedef enum
 {
   
   AD_V = 0x00,      /*!<  voltage acquisition module 1 */ 
   AD_I,             /*!<  current acquisition module */ 
   AD_S,             /*!<  CPU sonsor temp module */ 
   
 }ADC_MODULE;
 
/**
  * @brief  ADC chx
*/
 typedef enum
 {
   
   ADCX_CH1,
   ADCX_CH2,
   
   ADCX_CHMAX,
 }ADCX_CHX; 
   
 
/* Peripheral Control functions  ************************************************/
void Periph_Adc_Init(void);
void Periph_Adc_Update(void);
float Periph_Adc_Get_Val(uint8_t adc_module, uint8_t adc_chx);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
