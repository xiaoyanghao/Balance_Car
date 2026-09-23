/**
  ******************************************************************************
  * @file    bsp_timer_drv.c
  * @author  zhy
  * @brief   timer config.
  *          This is the common part of the sys hardware initialization
  *
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_timer_drv.h"
#include "imu.h"
    
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* ------------------------------TIM1-----------------------------------------*/
 /* Definition for TIM1 CH gpio clock resources */
#define TIM1_CH14_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
    
/* Definition for TIM1 CH gpio Pins */   
#define TIM1_CH14_PORT                GPIOA

#define TIM1_CH1_PIN                    GPIO_PIN_8
#define TIM1_CH4_PIN                    GPIO_PIN_11

#define TIM1_PRESCALER                  (0)
#define TIM1_PERIOD                     BSP_TIM1_PWM_PERIOD /* about 10 kHz at 72 MHz */

/* ------------------------------TIM2-----------------------------------------*/
#define TIM2_PRESCALER                  (72-1)
#define TIM2_PERIOD                     (1000) 

/* ------------------------------TIM3-----------------------------------------*/
 /* Definition for TIM3 CH gpio clock resources */
#define TIM3_CH12_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

/* Definition for TIM2 CH gpio Pins */   
#define TIM3_CH12_PORT                   GPIOC

#define TIM3_CH1_PIN                    GPIO_PIN_6
#define TIM3_CH2_PIN                    GPIO_PIN_7

#define TIM3_PRESCALER                  (720-1)
#define TIM3_PERIOD                     (100) 

/* ------------------------------TIM5-----------------------------------------*/
 /* Definition for TIM5 CH gpio clock resources */
#define TIM5_CH12_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for TIM5 CH gpio Pins */   
#define TIM5_CH12_PORT                   GPIOA

#define TIM5_CH1_PIN                    GPIO_PIN_0
#define TIM5_CH2_PIN                    GPIO_PIN_1

#define TIM5_PRESCALER                  (72-1)
#define TIM5_PERIOD                     (500) 


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef    tim1_handle;
static TIM_HandleTypeDef    tim2_handle;
static TIM_HandleTypeDef    tim3_handle;
static TIM_HandleTypeDef    tim5_handle;

/* Private function prototypes -----------------------------------------------*/
static void bsp_tim1_gpio_outinit(void);
static void bsp_tim3_gpio_encoderinit(void);
static void bsp_tim5_gpio_encoderinit(void);
static void bsp_timx_nvic_config(IRQn_Type timex_irq);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Timer 1 gpio Init
  * @retval None
  */
static void bsp_tim1_gpio_outinit(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM1_CH14_CLK_ENABLE();
  
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin  = TIM1_CH1_PIN|TIM1_CH4_PIN;
  HAL_GPIO_Init(TIM1_CH14_PORT, &GPIO_InitStruct);
  
}

/**
  * @brief Timer 3 gpio Init
  * @retval None
  */
static void bsp_tim3_gpio_encoderinit(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM3_CH12_CLK_ENABLE();
  
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Pin   = TIM3_CH1_PIN|TIM3_CH2_PIN;
  HAL_GPIO_Init(TIM3_CH12_PORT, &GPIO_InitStruct);
  __HAL_AFIO_REMAP_TIM3_ENABLE();
}

/**
  * @brief Timer 5 gpio Init
  * @retval None
  */
static void bsp_tim5_gpio_encoderinit(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM5_CH12_CLK_ENABLE();
  
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Pin  = TIM5_CH1_PIN|TIM5_CH2_PIN;
  HAL_GPIO_Init(TIM5_CH12_PORT, &GPIO_InitStruct);
 
}

/* ------------------------------TIM OUT FUNCTION----------------------------*/
/**
  * @brief Timer  x  output Init
  * @param  tim_id TIMID
            ccr_init  CCR
  * @retval None
  */
void Bsp_Tim1_Outinit(uint16_t ccr_init1,uint16_t ccr_init4)
{
   TIM_OC_InitTypeDef sConfigOC = {0};
   TIM_HandleTypeDef  *timx_handle = NULL;

   bsp_tim1_gpio_outinit();
   __TIM1_CLK_ENABLE();
   timx_handle = &tim1_handle;
   timx_handle->Instance = TIM1;
   timx_handle->Init.Prescaler = TIM1_PRESCALER;
   timx_handle->Init.Period = TIM1_PERIOD;
            
   timx_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
   timx_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   timx_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
   HAL_TIM_PWM_Init(timx_handle);
   
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;
   
   sConfigOC.Pulse = ccr_init1;
   HAL_TIM_OC_ConfigChannel(timx_handle, &sConfigOC, TIM_CHANNEL_1);
   sConfigOC.Pulse = ccr_init4;
   HAL_TIM_OC_ConfigChannel(timx_handle, &sConfigOC, TIM_CHANNEL_4);  
   __HAL_TIM_ENABLE_OCxPRELOAD(timx_handle, TIM_CHANNEL_1);
   __HAL_TIM_ENABLE_OCxPRELOAD(timx_handle, TIM_CHANNEL_4);

   HAL_TIM_OC_Start(timx_handle, TIM_CHANNEL_1);
   HAL_TIM_OC_Start(timx_handle, TIM_CHANNEL_4);
      
}


/**
  * @brief Timer x Set CCR
  * @retval None
  *@note : crr_output = 0    the pin output low level
           crr_output > TIMER_PERIOD  the pin output high level
  */
                       
void Bsp_Tim1_Chx_Set_Ccrout(uint8_t timx_chx, uint16_t ccr_output)
{
 
  switch (timx_chx){
    
    case TIM1_CH1:
          __HAL_TIM_SET_COMPARE(&tim1_handle, TIM_CHANNEL_1, ccr_output);
    break;
    case TIM1_CH4:
         __HAL_TIM_SET_COMPARE(&tim1_handle, TIM_CHANNEL_4, ccr_output);
    break;
    
    default:
    break;
  }

}


/**
 * Initialize timer for IMU sample at 1000Hz or faster.
 */
void txg_timer_init_imu_sample_trigger(void)
{
  
   TIM_HandleTypeDef  *timx_handle = NULL;
    // Enable RCC clock
   __TIM2_CLK_ENABLE();
   timx_handle = &tim2_handle;
   timx_handle->Instance = TIM2;
   timx_handle->Init.Prescaler = TIM2_PRESCALER;
   timx_handle->Init.Period = TIM2_PERIOD;
            
   timx_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
   timx_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   timx_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
   bsp_timx_nvic_config(TIM2_IRQn);
   HAL_TIM_Base_Init(timx_handle);
   //HAL_TIM_Base_Start(timx_handle);
   HAL_TIM_Base_Start_IT(timx_handle);
  
   
}
/* ------------------------------TIM INPUT FUNCTION----------------------------*/
/**
  * @brief Timer x input Init
  * @param  tim_id TIMID
  * @retval None
  */
void Bsp_Timx_Encoderinit(uint8_t tim_id)
{
  
   TIM_Encoder_InitTypeDef sConfig = {0};
   TIM_HandleTypeDef     *timx_handle = NULL;
   IRQn_Type timex_irq;
   
   switch(tim_id){

     case  TIM3_ID:
             bsp_tim3_gpio_encoderinit();
            __TIM3_CLK_ENABLE();
            timx_handle = &tim3_handle;
            timx_handle->Instance = TIM3;
            timx_handle->Init.Prescaler = 0x00;
            timex_irq = TIM3_IRQn;
            
     break;

    case  TIM5_ID:
            bsp_tim5_gpio_encoderinit();
            __TIM5_CLK_ENABLE();
            timx_handle = &tim5_handle;
            timx_handle->Instance = TIM5;
            timx_handle->Init.Prescaler = 0x00;
            timex_irq = TIM5_IRQn;
            
     break;
     default:
           
     break;
     
   }
   if(timx_handle != NULL){
     
     timx_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
     timx_handle->Init.Period = 0xFFFF;
     timx_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
     timx_handle->Init.RepetitionCounter = 0;
     timx_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
     
     sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
     sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
     sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
     sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
     sConfig.IC1Filter = 4;
     sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
     sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
     sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
     sConfig.IC2Filter = 4;   
     
     HAL_TIM_Encoder_Init(timx_handle, &sConfig);
     

     HAL_TIM_Encoder_Start(timx_handle, TIM_CHANNEL_ALL);
     
     bsp_timx_nvic_config(timex_irq);
     
   }
}



/**
  * @brief Timer  nvic config
  * @retval None
  */
static void bsp_timx_nvic_config(IRQn_Type timex_irq)
{
  
  HAL_NVIC_SetPriority(timex_irq, 2, 1);
  HAL_NVIC_EnableIRQ(timex_irq);
  
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim3_handle);
}


void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2_handle);
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim5_handle);
}


/**
  * @brief  This function handles  TIM_PeriodElapsed.
  * @param  None
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &tim3_handle){
    
  }
  if(htim == &tim2_handle){
    /* ISR context already provides mutual exclusion against main loop;
       no need to globally disable all interrupts */
    imu_samples_pull(0);
  }
  if(htim == &tim5_handle){
    
  }
}

/**
  * @brief Timer get encoder count
  * @retval None
  */                     
int16_t Bsp_Timx_Get_Encoder_Count(uint8_t timx_chx)
{
  int16_t encoder = 0;
  if(timx_chx == TIM3_ID){
    encoder = (int16_t)__HAL_TIM_GET_COUNTER(&tim3_handle);
    __HAL_TIM_SetCounter(&tim3_handle, 0);
  }
  else if(timx_chx == TIM5_ID){
    encoder = (int16_t)__HAL_TIM_GET_COUNTER(&tim5_handle);
     __HAL_TIM_SetCounter(&tim5_handle, 0);
  }
  else{
  }
  return encoder;
}

/**
  * @brief Timer get encoder direct
  * @retval None
  */                     
uint8_t Bsp_Timx_Get_Encoder_Dir(uint8_t timx_chx)
{
  uint8_t dir = 0;
  if(timx_chx == TIM3_ID){
    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&tim3_handle);
  }
  else if(timx_chx == TIM5_ID){
    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&tim5_handle);;
  }
  else{
  }
  return dir;

}
/********************************END OF FILE***********************************/
