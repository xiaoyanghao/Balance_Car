/**
  ******************************************************************************
  * @file    bsp_adc_drv.c
  * @author  zhy
  * @brief   adc config.
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
#include "bsp_adc_drv.h"
    
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define ADC_CH1234_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_CLK_ENABLE()              __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_DMA_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
/* Definition for ADC CH gpio Pins */   
#define ADC_CH1234_PORT                GPIOC

#define ADC_CH1_PIN                    GPIO_PIN_0
#define ADC_CH2_PIN                    GPIO_PIN_1
#define ADC_CH3_PIN                    GPIO_PIN_2
#define ADC_CH4_PIN                    GPIO_PIN_3

#define ADC_DMA_CH                     DMA1_Channel1
#define ADC_DMA_IRQ                    DMA1_Channel1_IRQn
#define ADC_DMA_IRQ_HANDLER            DMA1_Channel1_IRQHandler
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef h_adc;
static DMA_HandleTypeDef hdma_adc;
static uint16_t adc_buf[ADC_CH_MAX] = {0};
static uint32_t adc_lt_ms = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* ------------------------------ADC OUT FUNCTION----------------------------*/
/**
  * @brief  adc  Init
  * @param  None
  * @retval None
  */
void Bsp_Adc_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   ADC_ChannelConfTypeDef sConfig = {0};
   
   ADC_CH1234_CLK_ENABLE();
   ADC_CLK_ENABLE();
   ADC_DMA_CLK_ENABLE();
   
   GPIO_InitStruct.Pin = ADC_CH1_PIN|ADC_CH2_PIN|ADC_CH3_PIN|ADC_CH4_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   HAL_GPIO_Init(ADC_CH1234_PORT, &GPIO_InitStruct);
    
   hdma_adc.Instance = ADC_DMA_CH;
   hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
   hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
   hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
   hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
   hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
   hdma_adc.Init.Mode = DMA_CIRCULAR;
   hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
   HAL_DMA_Init(&hdma_adc);
   __HAL_LINKDMA(&h_adc,DMA_Handle,hdma_adc);
   
   h_adc.Instance = ADC1;
   h_adc.Init.ScanConvMode = ADC_SCAN_ENABLE;
   h_adc.Init.ContinuousConvMode = ENABLE;
   h_adc.Init.DiscontinuousConvMode = DISABLE;
   h_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
   h_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   h_adc.Init.NbrOfConversion = ADC_CH_MAX;
   HAL_ADC_Init(&h_adc);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&h_adc, &sConfig);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&h_adc, &sConfig);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&h_adc, &sConfig);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  HAL_ADC_ConfigChannel(&h_adc, &sConfig);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&h_adc, &sConfig);

  HAL_NVIC_SetPriority(ADC_DMA_IRQ, 0, 2);
  HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&h_adc);

  HAL_ADC_Start_DMA(&h_adc, (uint32_t *)adc_buf, 5);
  
}

/**
  * @brief  Get adc value
  * @param  ADC_CH
  * @retval None
  */
uint16_t Bsp_Get_Adc_Value(uint8_t ADC_CH)
{
  uint16_t val = 0;
   if(ADC_CH < ADC_CH_MAX){
     val = adc_buf[ADC_CH];
   }
   return val;
}

/**
  * @brief  Get adc last update time
  * @param  ADC_CH
  * @retval None
  */
uint32_t Bsp_Get_Adc_Last_Ms(void)
{
  return adc_lt_ms;
}

/* ------------------------------CONVCPLT CALLBACK FUNCTION-------------------*/
/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void ADC_DMA_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&hdma_adc);
}

/**
  * @brief  adc conv cp callback
  * @param  None
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &h_adc){
    adc_lt_ms = HAL_GetTick();
  }
}

/********************************END OF FILE***********************************/
