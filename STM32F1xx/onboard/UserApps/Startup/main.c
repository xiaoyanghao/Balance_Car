/**
  ******************************************************************************
  * @file    main.c
  * @author  zhy
  * @brief   mian task
  *
  @verbatim

  @endverbatim
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "bsp_rcc_drv.h"   
#include "bsp_gpio_drv.h"
#include "bsp_timer_drv.h"
#include "bsp_delay_drv.h"
#include "periph_serial_motor.h" 
#include "periph_serial_port.h"
#include "periph_mpu60x0.h"

#include "imu.h"
#include "txg_ahrs_fusion.h"
#include "txg_math.h"

#include "control.h"
#include "net_link.h"

#include <stdio.h>
#include <math.h>
  
/* Private define ------------------------------------------------------------*/  
#define SYS_TASK_NUM       7
#define SYS_TASK_1S        6

/* Private typedef -----------------------------------------------------------*/
struct TaskStruct{
  volatile uint16_t TaskTickNow;
  uint16_t TaskTickMax;
  volatile uint8_t  TaskStatus;
  void (*taskfunc)(void);
};

/* Private function prototypes -----------------------------------------------*/
/* static  function ----------------------------------------------------------*/
static void sys_task_timer_cb(void);
static void sys_task_run(void);

/* Private variables ---------------------------------------------------------*/
static struct TaskStruct systask[SYS_TASK_NUM] = { \
  {0, 500,  0, Bsp_Gpio_LED_Toggle},
  //{0, 1,    0, imu_samples_pull},
  //{0, 5,    0, imu_update},
  {0, 10,  0, mpu_dmp_get_data},      /* 100Hz — matches DMP FIFO output rate */
 // {0, 5,    0, ahrs_update_fusion},
  {0, 5,    0, Control_Spd_Updte},     /* 200Hz encoder read */
  {0, 5,   0, Control_Updte},           /* 200Hz control loop */
  {0, 20,   0, Net_Link_Decode},        /* 50Hz command decode */
  {0, 10,   0, Net_Link_Out},            /* 100Hz telemetry output (was 2ms, actual rate 10ms) */
  {0, 1000, 0, Rcc_Cpu_Loadcal},
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  main func
  *        
  */
void main(void)
{
  HAL_Init();
  
  Bsp_Rcc_SystemClock_Config();

  Rcc_Dwt_Init();

 // txg_timer_init_imu_sample_trigger();
  
  Bsp_Gpio_LED_Init();
  
  /* Put the motor bridge in a known stopped state before lengthy IMU init. */
  Periph_Motor_Init();
  Periph_Motor_Stop_All();

  HAL_Delay(1000);

  (void)mpu_dmp_init();
  
  Net_Link_Init();
  
  Control_Init();

  /* Start periodic application scheduling only after every module is ready. */
  Bsp_SysTick_Register_Maintask_Cb(sys_task_timer_cb);

  while(1){  
    sys_task_run();
    
  }
}

/**
  * @brief timer task cb
  * @retval None
  */
static void sys_task_timer_cb(void)
{
   uint8_t taskindex = 0;
   for(taskindex = 0; taskindex < SYS_TASK_NUM; taskindex++){
     if(!systask[taskindex].TaskStatus){
       if(++systask[taskindex].TaskTickNow >= systask[taskindex].TaskTickMax){
         systask[taskindex].TaskTickNow = 0;
          systask[taskindex].TaskStatus = 1;
       }
     }
   }
}

/**
  * @brief run sys task
  * @retval None
  */
static void sys_task_run(void)
{
   uint8_t taskindex = 0;
   for(taskindex = 0; taskindex < SYS_TASK_NUM; taskindex++){
     if(systask[taskindex].TaskStatus){
       if(taskindex != SYS_TASK_1S){
          Rcc_Count_Start();
       } 
       systask[taskindex].taskfunc();
       if(taskindex != SYS_TASK_1S){
          Rcc_Count_Stop();
       }
       systask[taskindex].TaskStatus = 0;
     }
   }
}

/********************************END OF FILE***********************************/
