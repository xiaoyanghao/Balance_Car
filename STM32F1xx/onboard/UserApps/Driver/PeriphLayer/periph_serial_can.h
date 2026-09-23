/**
  ******************************************************************************
  * @file    periph_serial_can.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the can
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_CAN_H
#define PERIPH_SERIAL_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_can_drv.h"
   
/* Exported types ------------------------------------------------------------*/ 
/** @defgroup CAN Exported Types
  * @{
  */   

      
/**
* @}
*/  
   
/* MACRO-----------------------------------------------------------------------*/ 
   
   
/* Peripheral Control functions  ************************************************/
void Periph_Can_Serial_Open(CAN_ID can_id);
void Periph_Can1_Send_Data_Pkt(uint32_t id, uint8_t ide, uint8_t *pkt, uint16_t len);
void Periph_Can2_Send_Data_Pkt(uint32_t id, uint8_t ide, uint8_t *pkt, uint16_t len);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
