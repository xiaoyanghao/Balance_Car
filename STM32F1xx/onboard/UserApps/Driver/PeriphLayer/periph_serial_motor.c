/**
  ******************************************************************************
  * @file    periph_serial_motor.c
  * @author  zhy
  * @brief   motor config.
  *          This is the common part of the sys periph initialization
  *
  @verbatim
    轮子直径 60mm 
    GM25-370 电机
    供电电压：12V（额定电压，电机的所有参数都是在该电压测得）
            350RPM(11000RPM，减速比 1：34)
    编码器：配备 11 CPR 霍尔 AB 两相编码器，减速后输出单圈 374 个正交脉冲

    0.06*2*pi/374  =  米/单个脉冲
    速度 = 米/脉冲 * 总脉冲/秒 = 米/s
  @endverbatim
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_motor.h"
#include "bsp_delay_drv.h"
    
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TB6612_XNCLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
    
/* Definition for TIM2 CH gpio Pins */   
#define TB6612_PORT                  GPIOC

#define TB6612_AN1_PIN               GPIO_PIN_1
#define TB6612_AN2_PIN               GPIO_PIN_0
#define TB6612_BN1_PIN               GPIO_PIN_2
#define TB6612_BN2_PIN               GPIO_PIN_3

#define TB6612_AN1_H                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_AN1_PIN, GPIO_PIN_SET)  
#define TB6612_AN1_L                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_AN1_PIN, GPIO_PIN_RESET)  

#define TB6612_AN2_H                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_AN2_PIN, GPIO_PIN_SET)  
#define TB6612_AN2_L                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_AN2_PIN, GPIO_PIN_RESET)  

#define TB6612_BN1_H                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_BN1_PIN, GPIO_PIN_SET)  
#define TB6612_BN1_L                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_BN1_PIN, GPIO_PIN_RESET)  

#define TB6612_BN2_H                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_BN2_PIN, GPIO_PIN_SET)  
#define TB6612_BN2_L                 HAL_GPIO_WritePin(TB6612_PORT, TB6612_BN2_PIN, GPIO_PIN_RESET)  


/* Private variables ---------------------------------------------------------*/
/* Track current direction to detect changes and insert dead time */
static uint8_t motor_dir_prev[2] = {2, 2};  /* 0=DF, 1=DB, 2=stop/unknown */

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void periph_TB6612_gpio_init(void);

/**
  * @brief 初始化TB6612 GPIO控制引脚
  */
static void periph_TB6612_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TB6612_XNCLK_ENABLE();
  
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin  = TB6612_AN1_PIN|TB6612_AN2_PIN|TB6612_BN1_PIN|TB6612_BN2_PIN;
  HAL_GPIO_Init(TB6612_PORT, &GPIO_InitStruct);

}

/**
  * @brief   电机初始化 
  */
void Periph_Motor_Init(void)
{
  
  Bsp_Tim1_Outinit(0, 0);
  Bsp_Timx_Encoderinit(TIM3_ID);
  Bsp_Timx_Encoderinit(TIM5_ID);
  periph_TB6612_gpio_init();
  Periph_Motor_Set_ST(MOTORL_ID);
  Periph_Motor_Set_ST(MOTORR_ID);

}


/**
  * @brief   电机设置输出值 
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @param[in]  val: 电机输出值 0~100
  */
void Periph_Motor_Set_Out(uint8_t which_motor, uint16_t val)
{
  if (val > BSP_TIM1_PWM_PERIOD) {
    val = BSP_TIM1_PWM_PERIOD;
  }

  switch (which_motor) {
  case MOTORL_ID:
    Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH1, val);
    break;
   case MOTORR_ID:
    Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH4, val);
    break; 
  default:
    break;
  }

}

/**
  * @brief   设置电机向前转
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @note    方向切换时先归零PWM并插入5μs死区时间，防止H桥直通短路
  */
void Periph_Motor_Set_DF(uint8_t which_motor)
{
  switch (which_motor) {
  case MOTORL_ID:
      if (motor_dir_prev[0] != 0) {
        Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH1, 0);
        Bsp_Delay_Us(5);
      }
      TB6612_AN1_H;
      TB6612_AN2_L;
      motor_dir_prev[0] = 0;
    break;
   case MOTORR_ID:
      if (motor_dir_prev[1] != 0) {
        Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH4, 0);
        Bsp_Delay_Us(5);
      }
      TB6612_BN1_L;
      TB6612_BN2_H;
      motor_dir_prev[1] = 0;
    break; 
  default:
    break;
  } 
}

/**
  * @brief   设置电机向后转
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @note    方向切换时先归零PWM并插入5μs死区时间，防止H桥直通短路
  */
void Periph_Motor_Set_DB(uint8_t which_motor)
{

  switch (which_motor) {
  case MOTORL_ID:
      if (motor_dir_prev[0] != 1) {
        Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH1, 0);
        Bsp_Delay_Us(5);
      }
      TB6612_AN1_L;
      TB6612_AN2_H;
      motor_dir_prev[0] = 1;
    break;
   case MOTORR_ID:
      if (motor_dir_prev[1] != 1) {
        Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH4, 0);
        Bsp_Delay_Us(5);
      }
      TB6612_BN1_H;
      TB6612_BN2_L;
      motor_dir_prev[1] = 1;
    break; 
  default:
    break;
  } 
}

/**
  * @brief   设置电机停转
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  */
void Periph_Motor_Set_ST(uint8_t which_motor)
{

  switch (which_motor) {
  case MOTORL_ID:
      TB6612_AN1_L;
      TB6612_AN2_L;
      motor_dir_prev[0] = 2;
    break;
   case MOTORR_ID:
      TB6612_BN1_L;
      TB6612_BN2_L;
      motor_dir_prev[1] = 2;
    break; 
  default:
    break;
  } 

}

void Periph_Motor_Stop_All(void)
{
  Periph_Motor_Set_Out(MOTORL_ID, 0);
  Periph_Motor_Set_Out(MOTORR_ID, 0);
  Periph_Motor_Set_ST(MOTORL_ID);
  Periph_Motor_Set_ST(MOTORR_ID);
}

/**
  * @brief   获取编码器值
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @retval  编码器值
  */
int16_t Periph_Motor_Get_Encoder(uint8_t which_motor)
{
  int16_t cout = 0;
  switch (which_motor) {
  case MOTORL_ID:
    cout = -Bsp_Timx_Get_Encoder_Count(TIM5_ID);
    break;
   case MOTORR_ID:
    cout = Bsp_Timx_Get_Encoder_Count(TIM3_ID);
    break; 
  default:
    break;
  }
  return cout;
}

/**
  * @brief   获取电机旋转方向
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @retval  编码器值
  */
MOTOR_DIR Periph_Motor_Get_Dir(uint8_t which_motor)
{
  MOTOR_DIR motor_dir = MOTOR_DF;
  uint8_t encoder_dir = 0;
  switch (which_motor) {
  case MOTORL_ID:
    encoder_dir = Bsp_Timx_Get_Encoder_Dir(TIM5_ID);
    break;
   case MOTORR_ID:
    encoder_dir = !Bsp_Timx_Get_Encoder_Dir(TIM3_ID);  
    break; 
  default:
    break;
  }
  if(!encoder_dir){
    motor_dir = MOTOR_DB;
  }
  return motor_dir;
}


/********************************END OF FILE***********************************/
