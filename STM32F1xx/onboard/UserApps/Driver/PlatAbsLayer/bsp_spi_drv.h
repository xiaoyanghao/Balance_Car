/**
  ******************************************************************************
  * @file    bsp_spi_drv.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the SPI
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_SPI_DRV_H
#define BSP_SPI_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
   
/* ------------------------------SPI Struct Parameter-------------------------*/
#define SPI_SLOTS                  5
#define SPI_MAX_TXN_TIME           3 // Ms  
 
/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI Exported Types
  * @{
  */
    
/**
  * @brief  SPI IT Callback 
  */    
typedef void spiCallback_t(int);  

/**
  * @brief  SPI STATUS
  */
 typedef enum
 {
   
   SPI_OFF,
   SPI_ON 
     
 }SPI_STATUS;
 
/**
  * @brief  SPI ID
  */
 typedef enum
 {
   
   SPI1_ID = 0x00,
   SPI_MAX,
   
 }SPI_ID;
 
/**
  * @brief  SPI MODE
  * @note   |mode   |   CLKPhase       |    CLKPolarity     |
            |SPI_MO | SPI_PHASE_1EDGE  |  SPI_POLARITY_LOW  |
            |SPI_M1 | SPI_PHASE_2EDGE  |  SPI_POLARITY_LOW  |
            |SPI_M2 | SPI_PHASE_1EDGE  |  SPI_POLARITY_HIGH  |
            |SPI_M3 | SPI_PHASE_2EDGE  | SPI_POLARITY_HIGH  |
  */
  typedef enum
 {
   
   SPI_MO, 
   SPI_M1,
   SPI_M2,
   SPI_M3,
  
 }SPI_MODE;

/**
  * @brief  SPI CS
  */
typedef struct 
{
    GPIO_TypeDef   *port;
    uint16_t       pin;
} CS_Pin;

/**
  * @brief  SPI Client
  */
typedef struct 
{
    CS_Pin             *cs;
    uint32_t           baud;
    spiCallback_t      *callback;
    volatile uint32_t  *flag;
    uint8_t            spi_id;
    
} SpiClient_t;

/**
  * @brief  SPI Slot
  */
typedef struct 
{
  
    uint16_t size;
    uint32_t  rxBuf;
    uint32_t  txBuf;
    SpiClient_t *client;
    
} SpiSlot_t;

/**
  * @brief  SPI Struct
  */
typedef struct 
{
  
    SpiSlot_t slots[SPI_SLOTS];
    SPI_HandleTypeDef *Spi_Handle;
    DMA_HandleTypeDef *DMA_TX_Handle;
    DMA_HandleTypeDef *DMA_RX_Handle;
    IRQn_Type            SPI_IRQn;           
    IRQn_Type            SPI_DMA_TX_IRQn;    
    IRQn_Type            SPI_DMA_RX_IRQn;         
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t txRunning;
    uint64_t txnStart;
    uint64_t txnTimeouts;
    uint64_t txnMaxTime;
    uint8_t  status;
    
} SPI_T;

 /**
* @}
*/

/* Peripheral Control functions  ************************************************/
SpiClient_t *Bsp_Spi_Serial_Open(uint8_t spi_id,uint32_t baud,uint8_t spi_mode,\
                                  GPIO_TypeDef *csPort, uint16_t csPin,\
                                  volatile uint32_t *flag,spiCallback_t *callback);
void Bsp_Spi_Serial_Close(SpiClient_t * client);
void Bsp_Spi_Transaction(SpiClient_t *client, volatile void *rxBuf, void *txBuf,\
                                                                  uint16_t size);
void Bsp_Spi_Change_Baud(SpiClient_t *client, uint32_t baud);
void Bsp_Spi_Change_Callback(SpiClient_t *client, spiCallback_t *callback);

#ifdef __cplusplus
}
#endif

#endif 

/********************************END OF FILE***********************************/
