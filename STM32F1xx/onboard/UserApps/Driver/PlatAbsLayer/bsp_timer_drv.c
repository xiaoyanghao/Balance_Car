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

/** Free running count of TIM2 IMU timebase ticks, updated in ISR context only.
 *  It carries no data and is never used to trigger work: the ISR stays short and
 *  the sample / fusion / control pipeline runs exclusively in the task layer. */
static volatile uint32_t    tim2_imu_tick_count = 0;

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
  * @brief  立即把 CCR 影子寄存器装载到输出(不等下一次更新事件)
  * @param  timx_chx : TIM1_CH1 或 TIM1_CH4
  * @retval None
  * @note   这两路 PWM 开启了预装载(OCxPE), 直接写 CCR 要等下一个更新事件才生效,
  *         最长延迟一个 PWM 周期(10kHz 时 100us)。换向时必须先让 "PWM=0"
  *         真正输出, 再翻转方向引脚, 否则那 5us 死区形同虚设, 电机会在
  *         换向瞬间以旧占空比承受反向浪涌。此函数用于消除这个延迟。
  */
void Bsp_Tim1_Chx_Force_Update(uint8_t timx_chx)
{
  if ((timx_chx == TIM1_CH1) || (timx_chx == TIM1_CH4))
  {
    /* UG 事件: 立刻从影子寄存器装载 CCR/ARR/PSC, 并把计数器清零 */
    tim1_handle.Instance->EGR = TIM_EGR_UG;
  }
}


/**
 * Initialize the optional TIM2 IMU timebase (1kHz with the current prescaler).
 * @note  The interrupt only increments Bsp_Tim2_Get_Imu_Tick_Count(). It does NOT
 *        read the sensor: sampling is driven by the 200Hz IMU task in main.c, so
 *        the same sample buffers are never touched from two contexts.
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
  * @note   ISR rule for this project: an interrupt handler only advances
  *         counters/flags. All sensor I/O (the MPU60x0 sits on a bit-banged
  *         software I2C bus), attitude fusion and control work is done by the
  *         cooperative task pipeline in main.c. Running blocking I2C here used
  *         to add hundreds of microseconds to the ISR and raced with the task
  *         that consumed the very same sample buffers.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &tim2_handle){
    tim2_imu_tick_count++;
  }
  else if(htim == &tim3_handle){
    
  }
  else if(htim == &tim5_handle){
    
  }
}

/**
  * @brief  Get the TIM2 IMU timebase tick count (diagnostics only).
  * @retval number of ticks since startup
  */
uint32_t Bsp_Tim2_Get_Imu_Tick_Count(void)
{
  return tim2_imu_tick_count;
}

/** 上一次读到的计数器值: 用于把"读后清零"改成"差值累加", 见下方说明 */
static uint16_t tim3_encoder_last = 0;
static uint16_t tim5_encoder_last = 0;

/**
  * @brief  读取编码器自上次调用以来的净脉冲数(有符号)
  * @param  timx_chx : TIM3_ID 或 TIM5_ID
  * @retval 本采样周期内的净脉冲数(有符号), 符号跟随计数方向
  * @note   为什么不用"读 CNT 再写 0":
  *         1) 读和清零之间存在窗口, 该窗口内到达的脉冲会被丢掉, 而且是只少
  *            不多, 会让速度反馈系统性偏小并带来随机抖动;
  *         2) 差值法下 16 位计数器回绕(0x0000 <-> 0xFFFF)由有符号减法自动
  *            处理, 不需要清零即可长期连续工作;
  *         3) 前提是两次调用之间的净脉冲数小于 32768 —— 200Hz 采样、350rpm
  *            减速电机约 11 脉冲/周期, 余量极大。
  */
int16_t Bsp_Timx_Get_Encoder_Count(uint8_t timx_chx)
{
  int16_t delta = 0;

  if(timx_chx == TIM3_ID){
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&tim3_handle);
    delta = (int16_t)(now - tim3_encoder_last);
    tim3_encoder_last = now;
  }
  else if(timx_chx == TIM5_ID){
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&tim5_handle);
    delta = (int16_t)(now - tim5_encoder_last);
    tim5_encoder_last = now;
  }
  else{
  }

  return delta;
}

/**
  * @brief  读取编码器计数器的瞬时计数方向
  * @param  timx_chx : TIM3_ID 或 TIM5_ID
  * @retval 0: 计数器正在向下计数, 1: 正在向上计数
  * @note   这是"此刻"的计数方向, 不是本采样周期的转动方向(电机在周期内可能
  *         先减速反转)。需要与速度符号严格一致的方向, 请用
  *         Periph_Motor_Get_Encoder_Data()。
  */
uint8_t Bsp_Timx_Get_Encoder_Dir(uint8_t timx_chx)
{
  uint8_t dir = 0;
  if(timx_chx == TIM3_ID){
    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&tim3_handle);
  }
  else if(timx_chx == TIM5_ID){
    dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&tim5_handle);
  }
  else{
  }
  return dir;

}
/********************************END OF FILE***********************************/
