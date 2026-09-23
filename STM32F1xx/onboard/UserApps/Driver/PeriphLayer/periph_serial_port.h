/**
  ******************************************************************************
  * @file    periph_serial_port.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the port
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_PORT_H
#define PERIPH_SERIAL_PORT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_uart_drv.h"
  
/* Exported types ------------------------------------------------------------*/
/** @defgroup  port acquisition Types
* @{
*/  

 /**
  * @brief  gpio mapping
  * @note    

FUNC               \     PIN    \  UART  \
422_RX            \     PD2     \  UART5 \
422_TX            \     PC12    \ 


  
*/ 
   

 /**
* @}
*/  
   
/* Peripheral Control functions  ************************************************/
   
void Periph_Port_Serial_Open(UART_T *uart, UART_ID uart_id,uint32_t bd_rate,\
                             uint8_t *dma_rx_buff, uint8_t *dma_tx_buff, \
                             uint16_t dma_rx_buff_size, uint16_t dma_tx_buff_size, \
                             uint8_t *rx_buff, uint8_t *tx_buff, uint16_t rx_buff_size,\
                             uint16_t tx_buff_size);
void Periph_Port_Send_Buffer(UART_T *uart, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
