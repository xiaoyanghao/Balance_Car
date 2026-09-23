/**
  ******************************************************************************
  * @file    bsp_can_drv.c
  * @author  zhy
  * @brief   CAN config.
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
#include "bsp_can_drv.h"
   
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 /* Definition for CAN1 clock resources */
#define CAN1_CLK_ENABLE()                 __HAL_RCC_CAN1_CLK_ENABLE()      
#define CAN1_TX_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN1_RX_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
 
/* Definition for CAN1 Pins */   
#define CAN1_TX_PIN                    GPIO_PIN_9
#define CAN1_TX_GPIO_PORT              GPIOB
#define CAN1_RX_PIN                    GPIO_PIN_8
#define CAN1_RX_GPIO_PORT              GPIOB
  
/* Definition for CAN1's NVIC */
#define CAN1_TX_IRQ                    CAN1_TX_IRQn
#define CAN1_RX_IRQ                    CAN1_RX0_IRQn
#define CAN1_TX_IRQHandler             CAN1_TX_IRQHandler 
#define CAN1_RX_IRQHandler             CAN1_RX0_IRQHandler

 /* Definition for CAN2 clock resources */
#define CAN2_CLK_ENABLE()                 __HAL_RCC_CAN2_CLK_ENABLE()      
#define CAN2_TX_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN2_RX_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
      
/* Definition for CAN2 Pins */   
#define CAN2_TX_PIN                    GPIO_PIN_6
#define CAN2_TX_GPIO_PORT              GPIOB
#define CAN2_RX_PIN                    GPIO_PIN_5
#define CAN2_RX_GPIO_PORT              GPIOB
    
/* Definition for CAN2's NVIC */
#define CAN2_TX_IRQ                    CAN2_TX_IRQn
#define CAN2_RX_IRQ                    CAN2_RX1_IRQn
#define CAN2_TX_IRQHandler             CAN2_TX_IRQHandler 
#define CAN2_RX_IRQHandler             CAN2_RX1_IRQHandler
    
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t can_clock_management = 0;
// Can handle
static CAN_HandleTypeDef can1_handle;  
static CAN_HandleTypeDef can2_handle; 

//CAN RX IT CALLBACK FUNCTION
static bool can1_rx_it_register = false;
static bool can2_rx_it_register = false;

static bool can1_tx_it_register = false;
static bool can2_tx_it_register = false;

static canx_rx_it_handler_callback_t *can1_rx_it_handler_callback;
static canx_rx_it_handler_callback_t *can2_rx_it_handler_callback;

static canx_tx_it_handler_callback_t *can1_tx_it_handler_callback;
static canx_tx_it_handler_callback_t *can2_tx_it_handler_callback;
/* Private function prototypes -----------------------------------------------*/
static void bsp_can_init(CAN_T *can_t);
static void bsp_can_nvic_config(CAN_T *can_t);
static void bsp_can_filter_config(CAN_T *can_t);
static void bsp_can_it_config(CAN_T *can_t);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Open Can Bus
  * @retval None
  */
void Bsp_Can_Serial_Open(CAN_T *can_t)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  switch(can_t->Can_Id){
    case CAN1_ID :
        if(can_clock_management == 0){
            CAN1_CLK_ENABLE();
         } 
         CAN1_TX_GPIO_CLK_ENABLE();
         CAN1_RX_GPIO_CLK_ENABLE();
         GPIO_InitStruct.Pin = CAN1_RX_PIN;
         GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStruct);
         
         GPIO_InitStruct.Pin = CAN1_TX_PIN;
         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
         HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStruct); 
         
         __HAL_AFIO_REMAP_CAN1_2();
         
         can_t->Can_Handle = &can1_handle; 
         can_t->Can_Handle->Instance = CAN1;
         can_t->CAN_TX_IRQN = CAN1_TX_IRQ;
         can_t->CAN_RX_IRQN = CAN1_RX_IRQ;
         can_t->status = CAN_ON;
         
         can_clock_management++;
    break;
  
    case CAN2_ID :
         CAN2_CLK_ENABLE();
         if(can_clock_management == 0){
            CAN1_CLK_ENABLE();
         }
         CAN2_TX_GPIO_CLK_ENABLE();
         CAN2_RX_GPIO_CLK_ENABLE();
         GPIO_InitStruct.Pin = CAN2_RX_PIN;
         GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStruct);
         
         GPIO_InitStruct.Pin = CAN2_TX_PIN;
         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
         HAL_GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStruct); 
         
         __HAL_AFIO_REMAP_CAN2_ENABLE();
         
         can_t->Can_Handle = &can2_handle;
         can_t->Can_Handle->Instance = CAN2;
         
         can_t->CAN_TX_IRQN = CAN2_TX_IRQ;
         can_t->CAN_RX_IRQN = CAN2_RX_IRQ;
         can_t->status = CAN_ON;  
         
         can_clock_management++;
    break;
    
    default:
    
    break;
  }
  if(can_t->Can_Handle != NULL){
    bsp_can_init(can_t);
    bsp_can_nvic_config(can_t);
    bsp_can_filter_config(can_t);
    bsp_can_it_config(can_t);
    HAL_CAN_Start(can_t->Can_Handle);
  }
}

/**
  * @brief  Can Bus Init
  * @retval None
  */
static void bsp_can_init(CAN_T *can_t)
{
  
  can_t->Can_Handle->Init.Prescaler = 4;
  can_t->Can_Handle->Init.Mode = CAN_MODE_NORMAL;
  can_t->Can_Handle->Init.SyncJumpWidth = CAN_SJW_1TQ;
  can_t->Can_Handle->Init.TimeSeg1 = CAN_BS1_6TQ;
  can_t->Can_Handle->Init.TimeSeg2 = CAN_BS2_2TQ;
  can_t->Can_Handle->Init.TimeTriggeredMode = DISABLE;
  can_t->Can_Handle->Init.AutoBusOff = DISABLE;
  can_t->Can_Handle->Init.AutoWakeUp = DISABLE;
  can_t->Can_Handle->Init.AutoRetransmission = ENABLE;
  can_t->Can_Handle->Init.ReceiveFifoLocked = DISABLE;
  can_t->Can_Handle->Init.TransmitFifoPriority = DISABLE;
  
  HAL_CAN_Init(can_t->Can_Handle);
  
}

/**
  * @brief  Can Bus NVIC Config
  * @retval None
  */
static void bsp_can_nvic_config(CAN_T *can_t)
{
    HAL_NVIC_SetPriority(can_t->CAN_TX_IRQN, 0, 1);
    HAL_NVIC_EnableIRQ(can_t->CAN_TX_IRQN);
    HAL_NVIC_SetPriority(can_t->CAN_RX_IRQN, 0, 1);
    HAL_NVIC_EnableIRQ(can_t->CAN_RX_IRQN);
}

/**
  * @brief  Can Bus Receive Filter Config
  * @retval None
  */
static void bsp_can_filter_config(CAN_T *can_t)
{
  
  CAN_FilterTypeDef sFilterConfig;
 
  sFilterConfig.FilterIdHigh = 0x00;
  sFilterConfig.FilterIdLow = 0x00;
  sFilterConfig.FilterMaskIdHigh = 0x00;
  sFilterConfig.FilterMaskIdLow = 0x00;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  if(can_t->Can_Id == CAN1_ID){
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.SlaveStartFilterBank = 0;
  }
  else{
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    sFilterConfig.FilterBank = 14; 
    sFilterConfig.SlaveStartFilterBank = 14;
  }
  
  HAL_CAN_ConfigFilter(can_t->Can_Handle, &sFilterConfig);                    
	
}

/**
  * @brief  Can Bus IT Config
  * @retval None
  */
static void bsp_can_it_config(CAN_T *can_t)
{
  if(can_t->Can_Id == CAN1_ID){
    HAL_CAN_ActivateNotification(can_t->Can_Handle,CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  else{
    HAL_CAN_ActivateNotification(can_t->Can_Handle,CAN_IT_RX_FIFO1_MSG_PENDING);
  }
  HAL_CAN_ActivateNotification(can_t->Can_Handle,CAN_IT_TX_MAILBOX_EMPTY);

}

/**
  * @brief This function handles CAN1 TX interrupt.
  */
void CAN1_TX_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&can1_handle);
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&can1_handle);

}

/**
  * @brief This function handles CAN2 TX interrupt.
  */
void CAN2_TX_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&can2_handle);
  
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&can2_handle);

}


/**
 * @brief : Register cb function to CANx it handler.
 * @param cb   : cb function address.
 */
void Bsp_Can_Register_Tx_It_Handler_Cb(CAN_T *can_t,canx_tx_it_handler_callback_t *cb)
{
   switch(can_t->Can_Id){
     case CAN1_ID:
       can1_tx_it_register = true;
       can1_tx_it_handler_callback = cb;
     break;
 
     case CAN2_ID:
       can2_tx_it_register = true;
       can2_tx_it_handler_callback = cb;
     break;
     
     default:
    
     break;  
   }
  
}

/**
 * @brief : Register cb function to CANx it handler.
 * @param cb   : cb function address.
 */
void Bsp_Can_Register_Rx_It_Handler_Cb(CAN_T *can_t,canx_rx_it_handler_callback_t *cb)
{
   switch(can_t->Can_Id){
     case CAN1_ID:
       can1_rx_it_register = true;
       can1_rx_it_handler_callback = cb;
     break;
 
     case CAN2_ID:
       can2_rx_it_register = true;
       can2_rx_it_handler_callback = cb;
     break;
     
     default:
    
     break;  
   }
  
}

/**
  * @brief  Can Bus RxFifo0 Callback Function
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  uint8_t can_rxbuff[8] = {0};
  CAN_RxHeaderTypeDef pHeader;
  if(hcan == &can1_handle){
    HAL_CAN_GetRxMessage(&can1_handle, CAN_RX_FIFO0, &pHeader, can_rxbuff);
    if(can1_rx_it_register){
      if(pHeader.IDE == CAN_ID_STD){
       can1_rx_it_handler_callback(pHeader.StdId,can_rxbuff, \
                                                           (uint8_t)pHeader.DLC);
      }
      else{
       can1_rx_it_handler_callback(pHeader.ExtId,can_rxbuff, \
                                                           (uint8_t)pHeader.DLC);        
      }
    }
  }

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Can Bus RxFifo1 Callback Function
  * @retval None
  */

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t can_rxbuff[8] = {0};
  CAN_RxHeaderTypeDef pHeader;
  if(hcan == &can2_handle){
    HAL_CAN_GetRxMessage(&can2_handle, CAN_RX_FIFO1, &pHeader, can_rxbuff);
    if(can2_rx_it_register){
      if(pHeader.IDE == CAN_ID_STD){
       can2_rx_it_handler_callback(pHeader.StdId,can_rxbuff, \
                                                           (uint8_t)pHeader.DLC);
      }
      else{
       can2_rx_it_handler_callback(pHeader.ExtId,can_rxbuff, \
                                                           (uint8_t)pHeader.DLC);        
      }
    }
  }
}
/**
  * @brief  Can Bus TxMailbox0 Callback Function
  * @retval None
*/

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  if(hcan == &can1_handle){
    if(can1_tx_it_register){
      can1_tx_it_handler_callback();
    }
  }
  if(hcan == &can2_handle){
    if(can2_tx_it_register){
      can2_tx_it_handler_callback();
    }
  }
 
  
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Can Bus Send
  * @param  can_t : struct  CAN_T address
            can_ide : CAN_IDE
             buf   : send buff address
             len   : number of send buff  
  * @retval None
  * @note   len <= 8
  */
uint32_t TxMailBox = 0;
void Bsp_Can_Send_Buffer(CAN_T *can_t,uint32_t id, uint8_t can_ide, uint8_t *buf, uint8_t len)
{
   uint16_t tx_delay = 0;
   
   CAN_TxHeaderTypeDef txheader;
   
   
   if(can_ide == CAN_EXD){
     txheader.ExtId = id;
     txheader.IDE = CAN_ID_EXT;
   }
   else{
     txheader.StdId = id;
     txheader.IDE = CAN_STD;
   }
   txheader.RTR = CAN_RTR_DATA;
   txheader.DLC = len;
   txheader.TransmitGlobalTime = DISABLE;
   while(!can_t->txComplete && tx_delay < 100){
     tx_delay++;
   }
   HAL_CAN_AddTxMessage(can_t->Can_Handle, &txheader, buf, &TxMailBox);
   
   can_t->txComplete = false; 
}

/********************************END OF FILE***********************************/
