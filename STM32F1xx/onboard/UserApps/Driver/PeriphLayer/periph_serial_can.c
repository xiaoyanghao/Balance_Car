/**
  ******************************************************************************
  * @file     periph_serial_can.c
  * @author  zhy
  * @brief   can config.
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
#include "periph_serial_can.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/   
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_T can1_t;
static CAN_T can2_t;

/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void can1_rx_it_handler_callback(uint32_t extend_id,uint8_t *buf,uint8_t len);
static void can2_rx_it_handler_callback(uint32_t extend_id,uint8_t *buf,uint8_t len);
void can1_tx_it_handler_callback(void);
void can2_tx_it_handler_callback(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Open CAN 
  * @param can_t            : CAN_T structure address
  * @param can_id           : can_id
  * @retval None
  */
void Periph_Can_Serial_Open(CAN_ID can_id)
{
  if(can_id == CAN1_ID){
    can1_t.Can_Id = can_id;
    can1_t.txComplete = true;
    can1_t.status = CAN_OFF;
    Bsp_Can_Serial_Open(&can1_t);
    Bsp_Can_Register_Rx_It_Handler_Cb(&can1_t,can1_rx_it_handler_callback);
    Bsp_Can_Register_Tx_It_Handler_Cb(&can1_t,can1_tx_it_handler_callback);
   }
   if(can_id == CAN2_ID){
      can2_t.Can_Id = can_id;
      can2_t.txComplete = true;
      can2_t.status = CAN_OFF;
      Bsp_Can_Serial_Open(&can2_t);
      Bsp_Can_Register_Rx_It_Handler_Cb(&can2_t,can2_rx_it_handler_callback);
      Bsp_Can_Register_Tx_It_Handler_Cb(&can2_t,can2_tx_it_handler_callback);
   }
}


/*---------------------------callback function--------------------------------*/
/**
  * @brief  CAN 1 rx  callback function
  * @param  extend id
            buf  :  rx buffer address 
            len  :  number of rx 
  * @retval None
  */

static void can1_rx_it_handler_callback(uint32_t extend_id,uint8_t *buf,uint8_t len)
{

  
}

/**
  * @brief  CAN 2 rx  callback function
  * @param  extend id
            buf  :  rx buffer address 
            len  :  number of rx 
  * @retval None
  */

static void can2_rx_it_handler_callback(uint32_t extend_id,uint8_t *buf,uint8_t len)
{

  
}


/**
  * @brief  CAN 1 tx  callback function
  * @retval None
  */
void can1_tx_it_handler_callback(void)
{
   can1_t.txComplete = true;
}

/**
  * @brief  CAN 2 tx  callback function
  * @retval None
  */
void can2_tx_it_handler_callback(void)
{
   can2_t.txComplete = true; 
}

/**
  * @brief  CAN 1 send packet
  * @param can_t            : CAN_T structure address
  * @param id               : can_id
  * @param ide              : CAN_IDE
  * @param pkt              : address of send buff 
  * @param len              : len of send data
  * @note  len <= 255
  * @retval None
  */
void Periph_Can1_Send_Data_Pkt(uint32_t id, uint8_t ide, uint8_t *pkt, uint16_t len)
{
  uint8_t index = 0;
  while(len > 8){
    Bsp_Can_Send_Buffer(&can1_t, id, ide, &pkt[index*8],8);
    len -= 8;
    index++;
  }
  if(len > 0){
    Bsp_Can_Send_Buffer(&can1_t, id, ide ,&pkt[index*8],len);
  }
}

/**
  * @brief  CAN 2 send packet
  * @param can_t            : CAN_T structure address
  * @param id               : can_id
  * @param ide              : CAN_IDE
  * @param pkt              : address of send buff 
  * @param len              : len of send data
  * @note  len <= 255
  * @retval None
  */
void Periph_Can2_Send_Data_Pkt(uint32_t id, uint8_t ide, uint8_t *pkt, uint16_t len)
{
  uint8_t index = 0;
  while(len > 8){
    Bsp_Can_Send_Buffer(&can2_t, id, ide, &pkt[index*8],8);
    len -= 8;
    index++;
  }
  if(len > 0){
    Bsp_Can_Send_Buffer(&can2_t, id, ide ,&pkt[index*8],len);
  }
}


/********************************END OF FILE***********************************/
