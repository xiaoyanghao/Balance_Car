/**
  ******************************************************************************
  * @file    bsp_gpio_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the  gipo
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_DELAY_DRV_H
#define BSP_DELAY_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/  

   
/* Peripheral Control functions  ************************************************/
void Bsp_Delay_Us(unsigned long ulCount);
void Bsp_Delay_Ms(unsigned long ulCount);
void Bsp_Delay_S(unsigned long ulCount);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
