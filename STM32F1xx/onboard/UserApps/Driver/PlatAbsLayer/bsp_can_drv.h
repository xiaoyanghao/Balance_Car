/**
  ******************************************************************************
  * @file    bsp_can_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the can
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_DRV_H
#define BSP_CAN_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdbool.h>
   
/* Exported types ------------------------------------------------------------*/
/** @defgroup CAN Exported Types
  * @{
  */
    
  /**
  * @brief  CAN sturct
  */   
 typedef enum
 {
   
   CAN1_ID,
   CAN2_ID,
   
 }CAN_ID;
 
   /**
  * @brief  CAN State
  */    
 typedef enum
 {
   
   CAN_OFF,
   CAN_ON 
     
 }CAN_STATUS;
 
/**
  * @brief  CAN CAN_IDE
  */    
 typedef enum
 {
   
   CAN_STD,
   CAN_EXD 
     
 }CAN_IDE;
 
/**
  * @brief  CAN sturct
  */ 
typedef struct
{
  
  CAN_HandleTypeDef    *Can_Handle;     
  uint8_t               Can_Id;
  
  IRQn_Type             CAN_TX_IRQN;
  IRQn_Type             CAN_RX_IRQN;
  
  CAN_STATUS            status; 
  volatile   bool       txComplete;
  
}CAN_T;

 /**
  * @brief  gpio mapping
  * @note    

CAN       \     PIN     \  
CAN1_TX   \     PB9    \  
CAN1_RX   \     PB8     \ 
   
CAN2_TX   \     PB6     \   
CAN2_RX   \     PB5     \   

*/  

 /**
  * @brief  CAN rx callback function
  */
typedef void canx_rx_it_handler_callback_t(uint32_t ,uint8_t *,uint8_t );
typedef void canx_tx_it_handler_callback_t(void);

 /**
* @}
*/

/* Peripheral Control functions  ************************************************/
void Bsp_Can_Serial_Open(CAN_T *can_t);
void Bsp_Can_Register_Rx_It_Handler_Cb(CAN_T *can_t,canx_rx_it_handler_callback_t *cb);
void Bsp_Can_Register_Tx_It_Handler_Cb(CAN_T *can_t,canx_tx_it_handler_callback_t *cb);
void Bsp_Can_Send_Buffer(CAN_T *can_t,uint32_t id, uint8_t can_ide, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
