/**
  ******************************************************************************
  * @file    bsp_uart_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the UART
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_UART_DRV_H
#define BSP_UART_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdbool.h>
   
/* Exported types ------------------------------------------------------------*/

/** @defgroup UART Exported Types
  * @{
  */
    
  /**
  * @brief  Uart sturct
  */
    
 typedef enum
 {
   UART5_ID,
   UART_MAXID,
 }UART_ID;
 
   /**
  * @brief  Uart State
  */
    
 typedef enum
 {
   UART_OFF,
   UART_ON 
 }UART_STATUS;
/**
  * @brief  Uart sturct
  */
   
typedef struct
{
  UART_HandleTypeDef  *Uart_Handle;        /*!< uart hanle*/
  uint8_t              Uart_Id;
  uint32_t             Baud_Rate;          /*!<  UART baud rate */
  
  DMA_HandleTypeDef   *DMA_TX_Handle;      /*!<  UART TX Handle */
  DMA_HandleTypeDef   *DMA_RX_Handle;      /*!<  UART RX Handle */
//  DMA_Stream_TypeDef  *Tx_DMA_STREAM;     /*!<  UART tx stream  */
//  DMA_Stream_TypeDef  *Rx_DMA_STREAM;     /*!<  UART tx stream  */
//  uint16_t             TX_DMA_REQUEST;     /*!<  Tx DMA request */
//  uint16_t             RX_DMA_REQUEST;     /*!<  Tx DMA request */ 
//  uint32_t             TX_DMA_FLAG_TCIF;     /*!<  Tx  DMA FLAG TCIF */
  
  IRQn_Type            UART_IRQn;           /*!<  UART IRQn */   
  IRQn_Type            UART_DMA_TX_IRQn;   /*!<  UART DMA TX IRQnn */   
  IRQn_Type            UART_DMA_RX_IRQn;   /*!<  UART DMA RX IRQnn */ 
  
  
  /*DMA Buf*/
  union Ptx_buf{
      uint8_t 	     *pTxDMABuf;          /*!<  DMA send buffer */
      uint8_t 	     *pTx_Buf;             /*!<  tx send buffer */
  }ptx_buf;
  union Prx_buf{
      uint8_t 	     *pRxDMABuf;          /*!<  DMA send buffer */
      uint8_t 	     *pRx_Buf;             /*!<  tx send buffer */
  }prx_buf;
  union PTx_bufsize{
      uint16_t 	     usTxDMABufSize;          /*!<  DMA send buffer */
      uint16_t 	     pTx_BufSize;             /*!<  tx send buffer */
  }pTx_bufsize;
  union PRx_bufsize{
      uint16_t 	     usRxDMABufSize;          /*!<  DMA send buffer */
      uint16_t 	     pRx_BufSize;             /*!<  tx send buffer */
  }pRx_bufsize;
  
  /* FIFO */
  uint8_t 	     *pTxBuf;		 /* Data send buffer */
  uint8_t 	     *pRxBuf;		/* Data receive buffer */
  uint16_t 	     usTxBufSize;	/* length of send buff */
  uint16_t 	     usRxBufSize;       /* length of receive buff */

  __IO uint16_t      usTxHead;   /*!< write index of send buffer */
  __IO uint16_t      usTxTail;   /*!<  read index of send buffer */
  __IO uint16_t      usTxCount;  /*!< count of data to be sent */
  bool                txFillBuff;
  bool                txOverflow;
 __IO  bool           txComplete;

  __IO uint16_t       usRxHead;		/*!<write index of re buff */
  __IO uint16_t       usRxTail;		/*!< read index of re buff */
  __IO uint16_t       usRxCount;       /*!< count of data to be read */
  bool                rxComplete;
  bool                rxOverflow;
  UART_STATUS         status;                   
}UART_T;
  
 /**
* @}
*/

/* Peripheral Control functions  ************************************************/

void Bsp_Uart_Serial_Open(UART_T *uart);
void Bsp_Uart_Serial_Close(UART_T *uart);
bool Bsp_Uart_Send_Trigger(UART_T *uart);
#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
