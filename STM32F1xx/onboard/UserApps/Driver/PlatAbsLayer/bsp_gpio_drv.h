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
#ifndef BSP_GPIO_DRV_H
#define BSP_GPIO_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/  

   
/* Peripheral Control functions  ************************************************/

void Bsp_Gpio_LED_Init(void);
void Bsp_Gpio_LED_ON(void);
void Bsp_Gpio_LED_OFF(void);
void Bsp_Gpio_LED_Toggle(void);
#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
