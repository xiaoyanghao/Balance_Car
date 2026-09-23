/**
******************************************************************************
* @file    bsp_spi_drv.c
* @author  zhy
* @brief   spi config.
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
#include "bsp_spi_drv.h"
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/

/* ------------------------------SPI Parameter--------------------------------*/
#define SPI_BAUD_MASK		  (~(((uint32_t)0x07)<<3))

/* ------------------------------SPI1----------------------------------------*/
/* Definition for SPI1 clock resources */
#define SPI1_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPI1_DMA_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()

#define SPI1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for SPI1 Pins */
#define SPI1_GPIO_PORT                   GPIOA
#define SPI1_SCK_PIN                     GPIO_PIN_5
#define SPI1_MISO_PIN                    GPIO_PIN_6
#define SPI1_MOSI_PIN                    GPIO_PIN_7

/* Definition for SPI1's DMA */
#define SPI1_TX_DMA_CH                   DMA1_Channel3
#define SPI1_RX_DMA_CH                   DMA1_Channel2

/* Definition for SPI1's NVIC */
#define SPI1_DMA_TX_IRQn                 DMA1_Channel3_IRQn
#define SPI1_DMA_RX_IRQn                 DMA1_Channel2_IRQn

#define SPI1_DMA_TX_IRQHandler           DMA1_Channel3_IRQHandler
#define SPI1_DMA_RX_IRQHandler           DMA1_Channel2_IRQHandler

/* Private variables ---------------------------------------------------------*/
static SPI_T spi_t[SPI_MAX];

//spi handle
static SPI_HandleTypeDef spi1_handle;
static DMA_HandleTypeDef hdma1_tx_handle;
static DMA_HandleTypeDef hdma1_rx_handle;


/* Private function prototypes -----------------------------------------------*/
static void bsp_spi1_gpio_init(uint8_t spi_mode);

void bsp_spi_init(SPI_T *spi_t,uint32_t baud, uint8_t spi_mode);
void bsp_dma_config(SPI_T *spi_t);
static void bsp_spi_nvic_config(SPI_T *spi_t);

static void *bsp_aqCalloc(size_t count, size_t size);
static CS_Pin *bsp_cs_init(GPIO_TypeDef* port, const uint16_t pin);
static void bsp_spi_deselect(SpiClient_t *client);
static void bsp_spi_select(SpiClient_t *client);

static void bsp_spi_start_txn(SPI_T *spi_t);
static void bsp_spi_end_txn(SPI_T *spi_t);
static void bsp_spi_notify(SpiClient_t *client);

/* Private functions ---------------------------------------------------------*/

/**
* @brief Open SPI 
* @retval None
*/
SpiClient_t *Bsp_Spi_Serial_Open(uint8_t spi_id,uint32_t baud,uint8_t spi_mode, \
                                            GPIO_TypeDef *csPort, uint16_t csPin, \
                                volatile uint32_t *flag,spiCallback_t *callback)
{
  
  SpiClient_t * p_spiclient = NULL;
  switch(spi_id){
    case SPI1_ID:
    if(spi_t[spi_id].status == SPI_OFF){
      
      bsp_spi1_gpio_init(spi_mode);
      SPI1_CLK_ENABLE();
      SPI1_DMA_CLK_ENABLE();
      
      spi_t[spi_id].Spi_Handle = &spi1_handle;
      spi_t[spi_id].DMA_TX_Handle = &hdma1_tx_handle;
      spi_t[spi_id].DMA_RX_Handle = &hdma1_rx_handle;
      
      
      spi_t[spi_id].DMA_TX_Handle->Instance = SPI1_TX_DMA_CH;
      spi_t[spi_id].DMA_RX_Handle->Instance = SPI1_RX_DMA_CH;
      spi_t[spi_id].Spi_Handle->Instance = SPI1;
 
      spi_t[spi_id].SPI_DMA_TX_IRQn = SPI1_DMA_TX_IRQn;
      spi_t[spi_id].SPI_DMA_RX_IRQn = SPI1_DMA_RX_IRQn;
      
      p_spiclient = (SpiClient_t *)bsp_aqCalloc(1, sizeof(SpiClient_t));
      p_spiclient->cs = bsp_cs_init(csPort,csPin);
      p_spiclient->baud = baud;
      p_spiclient->flag = flag;
      p_spiclient->callback = callback;
      p_spiclient->spi_id = spi_id;
      
      bsp_spi_init(&spi_t[spi_id],baud,spi_mode);
      bsp_dma_config(&spi_t[spi_id]);
      bsp_spi_nvic_config(&spi_t[spi_id]);
      spi_t[spi_id].status = SPI_ON;
      
    }     
    break;

    default:
    
    break;
  }
  
  return p_spiclient;
}

/* ------------------------------Spi Gpio Init--------------------------------*/
/**
* @brief Spi 1 gpio Init
* @retval None
*/
static void bsp_spi1_gpio_init(uint8_t spi_mode)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI1_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = SPI1_SCK_PIN|SPI1_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  switch(spi_mode){
    case SPI_MO:
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    break;   
    case SPI_M1:
    case SPI_M2:
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    break; 
    case SPI_M3:
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    break; 
    default: 
    break;       
  }
  HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI1_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);
 
}


/* ------------------------------Spi Prepare Init------------------------------*/
/**
* @brief Spi  Init
* @retval None
*/
void bsp_spi_init(SPI_T *spi_t,uint32_t baud,uint8_t spi_mode)
{
  
  spi_t->Spi_Handle->Init.Mode = SPI_MODE_MASTER;
  spi_t->Spi_Handle->Init.Direction = SPI_DIRECTION_2LINES;
  spi_t->Spi_Handle->Init.DataSize = SPI_DATASIZE_8BIT;
  spi_t->Spi_Handle->Init.NSS = SPI_NSS_SOFT;
  spi_t->Spi_Handle->Init.BaudRatePrescaler = baud;
  spi_t->Spi_Handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_t->Spi_Handle->Init.TIMode = SPI_TIMODE_DISABLE;
  spi_t->Spi_Handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi_t->Spi_Handle->Init.CRCPolynomial = 10;
  switch(spi_mode){
    case SPI_MO:
    spi_t->Spi_Handle->Init.CLKPhase            = SPI_PHASE_1EDGE;
    spi_t->Spi_Handle->Init.CLKPolarity         = SPI_POLARITY_LOW;
    break;
    case SPI_M1:
    spi_t->Spi_Handle->Init.CLKPhase            = SPI_PHASE_2EDGE;
    spi_t->Spi_Handle->Init.CLKPolarity         = SPI_POLARITY_LOW;
    break;  
    case SPI_M2:
    spi_t->Spi_Handle->Init.CLKPhase            = SPI_PHASE_1EDGE;
    spi_t->Spi_Handle->Init.CLKPolarity         = SPI_POLARITY_HIGH;
    break;
    case SPI_M3:
    spi_t->Spi_Handle->Init.CLKPhase            = SPI_PHASE_2EDGE;
    spi_t->Spi_Handle->Init.CLKPolarity         = SPI_POLARITY_HIGH;
    break;
    default:
    
    break;       
  }
  HAL_SPI_Init(spi_t->Spi_Handle);
}

/**
* @brief Spi DMA Config
* @retval None
*/
void bsp_dma_config(SPI_T *spi_t)
{
  
  spi_t->DMA_RX_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
  spi_t->DMA_RX_Handle->Init.PeriphInc = DMA_PINC_DISABLE;
  spi_t->DMA_RX_Handle->Init.MemInc = DMA_MINC_ENABLE;
  spi_t->DMA_RX_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  spi_t->DMA_RX_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  spi_t->DMA_RX_Handle->Init.Mode = DMA_NORMAL;
  spi_t->DMA_RX_Handle->Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(spi_t->DMA_RX_Handle);
 
  
  __HAL_LINKDMA(spi_t->Spi_Handle,hdmarx,*(spi_t->DMA_RX_Handle));
  

  spi_t->DMA_TX_Handle->Init.Direction = DMA_MEMORY_TO_PERIPH;
  spi_t->DMA_TX_Handle->Init.PeriphInc = DMA_PINC_DISABLE;
  spi_t->DMA_TX_Handle->Init.MemInc = DMA_MINC_ENABLE;
  spi_t->DMA_TX_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  spi_t->DMA_TX_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  spi_t->DMA_TX_Handle->Init.Mode = DMA_NORMAL;
  spi_t->DMA_TX_Handle->Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(spi_t->DMA_TX_Handle);

  
  __HAL_LINKDMA(spi_t->Spi_Handle,hdmatx,*(spi_t->DMA_TX_Handle));  
}

/**
* @brief  SPI RX NVIC  Configuration
* @param  
* @retval None
*/
static void bsp_spi_nvic_config(SPI_T *spi_t)
{
  
  HAL_NVIC_SetPriority(spi_t->SPI_DMA_TX_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(spi_t->SPI_DMA_TX_IRQn);
  HAL_NVIC_SetPriority(spi_t->SPI_DMA_RX_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(spi_t->SPI_DMA_RX_IRQn); 
  
}

/**
* @brief	aq heap management
* @param	count:
* @param	size:
*/
uint32_t heapUsed, heapHighWater;
static void *bsp_aqCalloc(size_t count, size_t size) 
{
  char *addr = 0;
  
  if (count * size) 
  {
    addr = (char*)calloc(count, size);  
    heapUsed += count * size;
    if (heapUsed > heapHighWater)
    {
      heapHighWater = heapUsed;
    }
  } 
  return addr;
}

/* ------------------------------SPI CS---------------------------------------*/
/**
* @brief	Specified port and pin to init as GPIO
* @param	port: port of GPIO
* @param	pin: pin of GPIO
*/
static CS_Pin *bsp_cs_init(GPIO_TypeDef* port, const uint16_t pin) 
{
  
  CS_Pin *p;
  GPIO_InitTypeDef GPIO_InitStruct;
  p = (CS_Pin *)malloc(sizeof(CS_Pin));
  p->port = port;
  p->pin = pin; 
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
  return p;
  
}

/**
* @brief Spi Deslect
* @retval None
*/
static void bsp_spi_deselect(SpiClient_t *client)
{
  HAL_GPIO_WritePin(client->cs->port,client->cs->pin,GPIO_PIN_SET);
}

/**
* @brief Spi Slect
* @retval None
*/
static void bsp_spi_select(SpiClient_t *client)
{
  HAL_GPIO_WritePin(client->cs->port,client->cs->pin,GPIO_PIN_RESET);
}

/* ------------------------------SPI1 IT--------------------------------------*/

/**
* @brief This function handles SPI1 DMA TX interrupt.
*/
void SPI1_DMA_TX_IRQHandler(void)    
{
  HAL_DMA_IRQHandler(&hdma1_tx_handle);
}

/**
* @brief This function handles SPI1 DMA RX interrupt.
*/
void SPI1_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma1_rx_handle);
}

/**
* @brief  TxRx Transfer completed callback.
* @param  hspi: SPI handle
* @note   This example shows a simple way to report end of DMA TxRx transfer, and 
*         you can add your own implementation. 
* @retval None
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  
  if(hspi == &spi1_handle){
    bsp_spi_end_txn(&spi_t[SPI1_ID]);
  }
}

/* ------------------------------SPI DMA Tran----------------------------------*/
/**
* @brief This function handles SPI TX.
*/
static void bsp_spi_start_txn(SPI_T *spi_t)
{
  SpiSlot_t   *slot = &spi_t->slots[spi_t->tail];
  SpiClient_t *client = slot->client;
  uint32_t tmp = 0;
  
  uint8_t head = 0, tail = 0, txRunning = 0;
  
  // is the current txn taking too long?
  if (spi_t->txRunning != 0 && (HAL_GetTick() - spi_t->txnStart) > SPI_MAX_TXN_TIME){
    __HAL_DMA_DISABLE(spi_t->DMA_RX_Handle);
    __HAL_DMA_DISABLE(spi_t->DMA_TX_Handle);
    bsp_spi_deselect(client);
    
    spi_t->tail = (spi_t->tail + 1) % SPI_SLOTS;
    slot = &spi_t->slots[spi_t->tail];
    client = slot->client;
    
    spi_t->txRunning = 0;
    spi_t->txnTimeouts++;
  }
  
  head = spi_t->head;
  tail = spi_t->tail;
  txRunning = spi_t->txRunning;
  
  if (tail != head && (txRunning == 0)){ 
    spi_t->txRunning = 1;
    spi_t->txnStart = HAL_GetTick();
    
     //set baud rate
    tmp = spi_t->Spi_Handle->Instance->CR1 & SPI_BAUD_MASK;
    tmp |= client->baud;
    spi_t->Spi_Handle->Instance->CR1 = tmp;
    
    bsp_spi_select(client);
    if (client->flag){
      *client->flag = 0;
    }
    __HAL_DMA_ENABLE(spi_t->DMA_RX_Handle);
    __HAL_DMA_ENABLE(spi_t->DMA_TX_Handle);
    HAL_SPI_TransmitReceive_DMA(spi_t->Spi_Handle,(uint8_t *)slot->txBuf, \
                                            (uint8_t *)slot->rxBuf, slot->size);
    
  }
}

/**
* @brief This function handles SPI Rx.
*/
static void bsp_spi_end_txn(SPI_T *spi_t) 
{
  uint8_t tail = spi_t->tail;
  SpiClient_t *client = spi_t->slots[tail].client;
  uint32_t tmp;
  __HAL_DMA_DISABLE(spi_t->DMA_RX_Handle);
  __HAL_DMA_DISABLE(spi_t->DMA_TX_Handle);

  // record longest txn
  tmp = HAL_GetTick() - spi_t->txnStart;
  if (tmp > spi_t->txnMaxTime){
    spi_t->txnMaxTime = tmp;
  }
  bsp_spi_notify(client);
  
  tail = (tail + 1) % SPI_SLOTS;
  spi_t->tail = tail;
  bsp_spi_deselect(client);
  spi_t->txRunning = 0;
  
  bsp_spi_start_txn(spi_t);
}

/**
* @brief This function handles client.
*/
static void bsp_spi_notify(SpiClient_t *client)
{
  
  if (client->flag){
    *client->flag = HAL_GetTick();
  } 
  if (client->callback){
    client->callback(0);
  }
  
}

/* ------------------------------SPI  EX Function------------------------------*/
/**
* @brief This function spi tranction.
*/

void Bsp_Spi_Transaction(SpiClient_t *client, volatile void *rxBuf, void *txBuf, \
                         uint16_t size)
{
  
  spi_t[client->spi_id].slots[spi_t[client->spi_id].head].size = size;
  spi_t[client->spi_id].slots[spi_t[client->spi_id].head].rxBuf = (uint32_t)rxBuf;
  spi_t[client->spi_id].slots[spi_t[client->spi_id].head].txBuf = (uint32_t)txBuf;
  spi_t[client->spi_id].slots[spi_t[client->spi_id].head].client = client;
  
  spi_t[client->spi_id].head = (spi_t[client->spi_id].head + 1) % SPI_SLOTS;
  
  bsp_spi_start_txn(&spi_t[client->spi_id]);
}

/**
* @brief This function to change baud.
*/
void Bsp_Spi_Change_Baud(SpiClient_t *client, uint32_t baud)
{
  client->baud = baud; 
}

/**
* @brief This function to change callback.
*/
void Bsp_Spi_Change_Callback(SpiClient_t *client, spiCallback_t *callback)
{
  client->callback = callback;
}





