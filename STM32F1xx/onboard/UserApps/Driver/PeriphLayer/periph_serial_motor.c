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

/* 换向死区时间: PWM 归零生效后、方向引脚翻转前的等待时间, 单位 us */
#define MOTOR_DIR_DEADTIME_US       5

    
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
  * @brief  换向前把 PWM 真正拉到 0, 并等待死区时间
  * @param[in]  which_motor: 电机ID
  * @note   PWM 通道带预装载, 只写 CCR=0 要等一个 PWM 周期才生效, 所以这里
  *         显式触发一次更新事件让 0 立即输出, 再等死区时间让 H 桥完全关断,
  *         之后才允许翻转方向引脚。
  */
static void periph_motor_prepare_direction_switch(uint8_t which_motor)
{
  if (which_motor == MOTORL_ID) {
    Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH1, 0);
    Bsp_Tim1_Chx_Force_Update(TIM1_CH1);
  } else if (which_motor == MOTORR_ID) {
    Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH4, 0);
    Bsp_Tim1_Chx_Force_Update(TIM1_CH4);
  } else {
    return;
  }

  Bsp_Delay_Us(MOTOR_DIR_DEADTIME_US);
}

/**
  * @brief   设置电机向前转
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @note    方向切换时先归零PWM并插入死区时间，防止H桥直通短路
  */
void Periph_Motor_Set_DF(uint8_t which_motor)
{
  switch (which_motor) {
  case MOTORL_ID:
      if (motor_dir_prev[0] != 0) {
        periph_motor_prepare_direction_switch(MOTORL_ID);
      }
      TB6612_AN1_H;
      TB6612_AN2_L;
      motor_dir_prev[0] = 0;
    break;
   case MOTORR_ID:
      if (motor_dir_prev[1] != 0) {
        periph_motor_prepare_direction_switch(MOTORR_ID);
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
  * @note    方向切换时先归零PWM并插入死区时间，防止H桥直通短路
  */
void Periph_Motor_Set_DB(uint8_t which_motor)
{

  switch (which_motor) {
  case MOTORL_ID:
      if (motor_dir_prev[0] != 1) {
        periph_motor_prepare_direction_switch(MOTORL_ID);
      }
      TB6612_AN1_L;
      TB6612_AN2_H;
      motor_dir_prev[0] = 1;
    break;
   case MOTORR_ID:
      if (motor_dir_prev[1] != 1) {
        periph_motor_prepare_direction_switch(MOTORR_ID);
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
  * @note    同时把 PWM 归零: "停转"必须是自包含的, 不能依赖调用方先调
  *          Periph_Motor_Set_Out(0), 否则残留占空比会在停止命令后继续驱动电机。
  */
void Periph_Motor_Set_ST(uint8_t which_motor)
{

  switch (which_motor) {
  case MOTORL_ID:
      Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH1, 0);
      TB6612_AN1_L;
      TB6612_AN2_L;
      motor_dir_prev[0] = 2;
    break;
   case MOTORR_ID:
      Bsp_Tim1_Chx_Set_Ccrout(TIM1_CH4, 0);
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

/* Private variables ---------------------------------------------------------*/
/**
 * 最近一次编码器采样的结果。
 * 速度取自"本采样周期内的净脉冲数", 方向直接由该值的符号给出, 因此两者
 * 永远自洽; 旧实现用计数器的瞬时计数方向判断, 与整周期净脉冲的符号可能
 * 相反(遥测上表现为"速度为负但方向显示前进")。
 */
static int16_t motor_speed_last[2] = {0, 0};
static MOTOR_DIR motor_dir_last[2] = {MOTOR_DF, MOTOR_DF};

/**
  * @brief   采样编码器, 同时得到速度与转动方向
  * @param[in]  which_motor: 电机ID
  *             @arg @ref MOTORL_ID  左电机
  *             @arg @ref MOTORR_ID  右电机
  * @param[out] delta: 本采样周期内的净脉冲数(有符号), 允许传入 NULL
  * @param[out] dir  : 与 delta 符号一致的转动方向, 允许传入 NULL
  * @note   速度单位是"编码器脉冲数 / 采样周期", 本工程采样周期 5ms(200Hz)。
  *
  *         换算成物理量的参数 (见本文件头部):
  *           编码器 11 线/圈, 减速比 34 => 输出轴 374 线/圈
  *           TIM 工作在 TI12(四倍频) => 1496 计数/圈
  *           轮径 60mm => 每计数 0.126mm
  *           1 脉冲/周期 = 200 计数/s = 25.2 mm/s
  *           满速 350rpm(输出轴) 约 44 脉冲/周期
  */
void Periph_Motor_Get_Encoder_Data(uint8_t which_motor, int16_t *delta, MOTOR_DIR *dir)
{
  int16_t value = 0;

  switch (which_motor) {
  case MOTORL_ID:
    value = -Bsp_Timx_Get_Encoder_Count(TIM5_ID);
    break;
   case MOTORR_ID:
    value = Bsp_Timx_Get_Encoder_Count(TIM3_ID);
    break; 
  default:
    return;
  }

  motor_speed_last[which_motor] = value;
  motor_dir_last[which_motor] = (value < 0) ? MOTOR_DB : MOTOR_DF;

  if (delta != 0) {
    *delta = value;
  }
  if (dir != 0) {
    *dir = motor_dir_last[which_motor];
  }
}

/**
  * @brief   返回最近一次采样得到的速度
  * @param[in]  which_motor: 电机ID
  * @retval  脉冲数/采样周期, 有符号; 尚未采样时返回 0
  * @note    不访问硬件
  */
int16_t Periph_Motor_Get_Encoder_Speed(uint8_t which_motor)
{
  switch (which_motor) {
  case MOTORL_ID:
    return motor_speed_last[0];
   case MOTORR_ID:
    return motor_speed_last[1];
  default:
    return 0;
  }
}

/**
  * @brief   返回最近一次采样得到的电机转动方向
  * @param[in]  which_motor: 电机ID
  * @retval  MOTOR_DF / MOTOR_DB, 与 Periph_Motor_Get_Encoder_Speed() 符号一致
  * @note    不访问硬件
  */
MOTOR_DIR Periph_Motor_Get_Dir(uint8_t which_motor)
{
  switch (which_motor) {
  case MOTORL_ID:
    return motor_dir_last[0];
   case MOTORR_ID:
    return motor_dir_last[1];
  default:
    return MOTOR_DF;
  }
}


/********************************END OF FILE***********************************/
