/**
  ******************************************************************************
  * @file     periph_serial_port.c
  * @author  zhy
  * @brief   port config.
  *          This is the common part of the sys periph initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The common HAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the HAL.
    [..]
    The HAL contains two APIs' categories:
         (+) Common HAL APIs
         (+) Services HAL APIs

  @endverbatim
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_port.h"

    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void periph_port_var_init(UART_T *uart, UART_ID uart_id,uint32_t bd_rate, \
                                 uint8_t *dma_rx_buff, uint8_t *dma_tx_buff, \
                                 uint16_t dma_rx_buff_size, uint16_t dma_tx_buff_size, \
                                 uint8_t *rx_buff, uint8_t *tx_buff, uint16_t rx_buff_size,\
                                 uint16_t tx_buff_size);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Open Port 
  * @param uart            : UART structure
  * @param baud             : baud rate
  * @param dma_rx_buff      : rx buffer of DMA
  * @param dma_tx_buff      : tx buffer of DMA
  * @param dma_rx_buff_size : max size of DMA rx buffer
  * @param dma_tx_buff_size : max size of DMA tx buffer
  * @param rx_buff          : rx buffer size of data
  * @param tx_buff          : tx buffer size of data
  * @param rx_buff_size     : max size of rx data buffer
  * @param tx_buff_size     : max size of tx data buffer
  * @retval None
  */
void Periph_Port_Serial_Open(UART_T *uart, UART_ID uart_id,uint32_t bd_rate,\
                             uint8_t *dma_rx_buff, uint8_t *dma_tx_buff, \
                             uint16_t dma_rx_buff_size, uint16_t dma_tx_buff_size, \
                             uint8_t *rx_buff, uint8_t *tx_buff, uint16_t rx_buff_size,\
                             uint16_t tx_buff_size)
{

    periph_port_var_init(uart,uart_id,bd_rate,dma_rx_buff,dma_tx_buff,dma_rx_buff_size,\
                        dma_tx_buff_size,rx_buff,tx_buff,rx_buff_size,tx_buff_size);

    Bsp_Uart_Serial_Open(uart);
}


/**
 * @brief Init usart use default parameters. 
 * @param uart            : UART structure
 * @param bd_rate         : baud rate
 * @param rx_buff      : rx buffer of DMA
 * @param tx_buff      : tx buffer of DMA
 * @param rx_buff_size : max size of DMA rx buffer
 * @param tx_buff_size : max size of DMA tx buffer
 * @param rx_fifo_buff          : rx buffer size of data
 * @param tx_fifo_buff          : tx buffer size of data
 * @param rx_fifo_buf_size      : max size of rx data buffer
 * @param tx_fifo_buf_size      : max size of tx data buffer
 * @retval None
 */
static void periph_port_var_init(UART_T *uart, UART_ID uart_id,uint32_t bd_rate, \
                                 uint8_t *rx_buff, uint8_t *tx_buff, \
                                 uint16_t rx_buff_size, uint16_t tx_buff_size, \
                                 uint8_t *rx_fifo_buff, uint8_t *tx_fifo_buff, uint16_t rx_fifo_buf_size,\
                                 uint16_t tx_fifo_buf_size)
{
  
  uart->Uart_Id = uart_id;
  uart->Baud_Rate = bd_rate;
  uart->ptx_buf.pTx_Buf    = tx_buff;     //  tx buffer
  uart->prx_buf.pRx_Buf    = rx_buff;     //  rx buffer
  uart->pTx_bufsize.pTx_BufSize     = tx_buff_size;
  uart->pRx_bufsize.pRx_BufSize     = rx_buff_size;
  
  /** FIFO */
  uart->pTxBuf             = tx_fifo_buff;	   // data tx buffer
  uart->pRxBuf             = rx_fifo_buff;       // data rx buffer
  uart->usTxBufSize        = tx_fifo_buf_size;
  uart->usRxBufSize        = rx_fifo_buf_size;
  
  uart->usTxHead           = 0;     // tx write index
  uart->usTxTail           = 0;     // tx read index
  uart->txFillBuff         = false;
  uart->txComplete         = true;
  uart->txOverflow         = false;
  uart->usRxHead           = 0;     // rx write index
  uart->usRxTail           = 0;     // rx read index
  uart->usRxCount          = 0;     // counter of data received
  uart->usTxCount          = 0;     // counter of data to tx
  uart->rxOverflow         = false;

}

/**
  * @brief  This function UART send data.  
  * @param  
  * @retval None    
  */
void Periph_Port_Send_Buffer(UART_T *uart, uint8_t *buf, uint16_t len)
{
  uint16_t i = 0;
  /** Indicate we are filling data to tx buffer now. Do not trigger DMA tx at this time */
  uart->txFillBuff = true;
 // 
  /** If tx buffer overflow, do not fill untill data in buffer has been sent */
  if(!uart->txOverflow){
    for( i=0; i<len; i++) {
      uart->pTxBuf[uart->usTxHead] = buf[i];
      
      /** Move tx head index to next */
      uart->usTxHead = (uart->usTxHead + 1) % uart->usTxBufSize;
      
      /** Check if tx buffer overflow. If so, break and trigger sending */
      if(uart->usTxCount < uart->usTxBufSize) {
        uart->usTxCount++;
      }
      else{
        uart->txOverflow = true;
        break;
      }
    }
  }
  
  uart->txFillBuff = false;
  
  /** Trigger USART dma transfer. */
  Bsp_Uart_Send_Trigger(uart);
}



/********************************END OF FILE***********************************/
