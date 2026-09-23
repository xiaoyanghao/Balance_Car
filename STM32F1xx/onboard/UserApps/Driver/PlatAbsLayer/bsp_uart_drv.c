/**
  ******************************************************************************
  * @file    bsp_uart_drv.c
  * @author  zhy
  * @brief   uart config.
  *          This is the common part of the sys hardware initialization
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
#include "bsp_uart_drv.h"

/* Private define ------------------------------------------------------------*/
 /* ------------------------------USART5--------------------------------------*/
 /* Definition for UARTx clock resources */
#define UART5_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define UART5_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART5_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

 /* Definition for USARTx Pins */
    
#define UART5_TX_PIN                    GPIO_PIN_9
#define UART5_TX_GPIO_PORT              GPIOA
#define UART5_RX_PIN                    GPIO_PIN_10
#define UART5_RX_GPIO_PORT              GPIOA
    
/* Definition for USARTx's NVIC */
#define USART1_IRQn                      USART1_IRQn
 
/* Private variables ---------------------------------------------------------*/
// UART  handle
static UART_HandleTypeDef uart5_handle;  
static UART_T* uart_t[UART_MAXID] = {NULL};

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void bsp_uart_init(UART_T *uart);
static void bsp_uart_nvic_config(UART_T *uart);


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  OPEN UART
                 uart gpio clock config 
                 uart gpio config
                 uart clock config 
                 uart dma interrupt config
                 uart interrupt config
  * @param  
  * @retval None
  */
void Bsp_Uart_Serial_Open(UART_T *uart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    switch (uart->Uart_Id){
    case UART5_ID:
             UART5_CLK_ENABLE();
             UART5_RX_GPIO_CLK_ENABLE();
             UART5_TX_GPIO_CLK_ENABLE();
             GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
             GPIO_InitStruct.Pin = UART5_TX_PIN;
             HAL_GPIO_Init(UART5_TX_GPIO_PORT, &GPIO_InitStruct);
             GPIO_InitStruct.Pin = UART5_RX_PIN;
             GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
             GPIO_InitStruct.Pull = GPIO_NOPULL;
             HAL_GPIO_Init(UART5_RX_GPIO_PORT, &GPIO_InitStruct);
             
             uart->Uart_Handle = &uart5_handle;
             uart->Uart_Handle->Instance = USART1;         
             uart->UART_IRQn = USART1_IRQn;
             uart->status = UART_ON;
             uart_t[UART5_ID] = uart;
         break; 
    default:
         break;    
  }
  
  bsp_uart_init(uart);
  bsp_uart_nvic_config(uart);
  HAL_UARTEx_ReceiveToIdle_IT(uart->Uart_Handle,(uint8_t*)&uart->prx_buf.pRx_Buf[0], uart->pRx_bufsize.pRx_BufSize);

}


/**
  * @brief  Uart Initialization
  * @param   
  * @retval None
  */
static void bsp_uart_init(UART_T *uart)
{

  uart->Uart_Handle->Init.BaudRate = uart->Baud_Rate;
  uart->Uart_Handle->Init.WordLength = UART_WORDLENGTH_8B;
  uart->Uart_Handle->Init.StopBits = UART_STOPBITS_1;
  uart->Uart_Handle->Init.Parity = UART_PARITY_NONE;
  uart->Uart_Handle->Init.Mode = UART_MODE_TX_RX;
  uart->Uart_Handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->Uart_Handle->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(uart->Uart_Handle);
  
}
  

/**
  * @brief  Uart NVIC  Configuration
  * @param  
  * @retval None
  */
static void bsp_uart_nvic_config(UART_T *uart)
{
  
  HAL_NVIC_SetPriority(uart->UART_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(uart->UART_IRQn);

}

/**
  * @brief  This function UART TX  send trigger.  
  * @param  UART_T Pointer 
  * @retval None    
  */
bool Bsp_Uart_Send_Trigger(UART_T *uart)
{
  bool tx_ret = false;
  int16_t tx_len = 0;
  uint16_t tx_head = 0,i = 0;
  
  /**
  * 1#: DMA last transfer complete.
  * 2#: DMA tx buffer not used.
  */
  if(uart->txComplete &&!uart->txFillBuff)
  {
    tx_head = uart->usTxHead;
    
    /** Check tx buff if there are data to send */
    tx_len = tx_head - uart->usTxTail;
    if(tx_len < 0 || uart->usTxBufSize == uart->usTxCount){
      tx_len += uart->usTxBufSize;
    } 
    /** One time tx length do not exceed half of max DMA tx buffer size */
   // tx_len = tmin(tx_len, uart->usTxDMABufSize/2);
    
    /** If there are data in tx buffer, copy to DMA buffer and send it */
    if(tx_len > 0){
      for(i = 0; i<tx_len; i++){
        uart->ptx_buf.pTx_Buf[i] = uart->pTxBuf[uart->usTxTail];
        /** Move send index to next */
        /** NOTE: this would never exceed usTxHead */
        uart->usTxTail = (uart->usTxTail + 1) % uart->usTxBufSize;
      }
      
      /** Dicrease tx buffer counter monitor */
      uart->usTxCount -= tx_len;
      if(uart->usTxCount < uart->usTxBufSize){
        uart->txOverflow = false;
      }
      
      /** Set tx buffer length and enable DMA tx */
      
     HAL_UART_Transmit_IT(uart->Uart_Handle,&uart->ptx_buf.pTx_Buf[0],tx_len);
    
      /** Clear DMA tx complete flag */
      uart->txComplete = false;
    }
    
    tx_ret = true;
  }
  
  return tx_ret;
}


/**
  * @brief  This function handles UART IDLE interrupt request.  
  * @param  UART_T Pointer 
  * @retval None    
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t rx_length = 0,i = 0;
  UART_T *uart = NULL;
  if(huart == &uart5_handle){
    uart = uart_t[UART5_ID];
  }
  
  rx_length = Size;
  for(i = 0; i<rx_length; i++){
    /** Copy data recieved in  buffer to rx fifo buffer */
    uart->pRxBuf[uart->usRxHead] = uart->prx_buf.pRx_Buf[i];
    /** Change rx head to next */
    uart->usRxHead = (uart->usRxHead + 1) % uart->usRxBufSize;
    /** new data in rx buffer counter, for overflow check */
    if(uart->usRxCount < uart->usRxBufSize){
      uart->usRxCount++;
    }
    else{
      uart->rxOverflow = true;
    }
  }
  uart->rxComplete = true;
  HAL_UART_AbortReceive_IT(uart->Uart_Handle);
  HAL_UARTEx_ReceiveToIdle_IT(uart->Uart_Handle,(uint8_t*)&uart->prx_buf.pRx_Buf[0], uart->pRx_bufsize.pRx_BufSize);
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UART_T *uart = NULL;
  if(huart == &uart5_handle){
    uart = uart_t[UART5_ID];
  }
  uart->txComplete = true;
  Bsp_Uart_Send_Trigger(uart);
}

/*---------------------------IT------------------------------------------------*/
/*---------------------------USART5--------------------------------------------*/
/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&uart5_handle);
}

/********************************END OF FILE***********************************/
