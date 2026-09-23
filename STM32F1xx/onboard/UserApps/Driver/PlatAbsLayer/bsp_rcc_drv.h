/**
  ******************************************************************************
  * @file    bsp_rcc_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the rcc clock
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_RCC_DRV_H
#define BSP_RCC_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* extern macro -------------------------------------------------------------*/
   
/* Peripheral Control functions  ************************************************/

void Bsp_Rcc_SystemClock_Config(void);
void Rcc_Dwt_Init(void);
void Rcc_Count_Start(void);
void Rcc_Count_Stop(void);
void Rcc_Cpu_Loadcal(void);
float Rcc_Get_Cpuusage(void);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
