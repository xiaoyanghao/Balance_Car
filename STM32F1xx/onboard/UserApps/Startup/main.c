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
#define SYS_TICK_MS        1U      /* SysTick period: scheduler time base */

/* Private typedef -----------------------------------------------------------*/
/** Task body / optional data dependency gate. */
typedef void    (*sys_task_fn_t)(void);
typedef uint8_t (*sys_task_gate_t)(void);

/**
 * @brief  Cooperative task descriptor.
 * @note   TaskTickNow / TaskStatus are only advanced by the 1kHz SysTick
 *         callback, while the task body always runs in the main loop. Blocking
 *         drivers (software I2C, HAL_Delay, ...) are therefore allowed inside a
 *         task function, but never inside an interrupt handler.
 */
struct TaskStruct{
  volatile uint16_t TaskTickNow;    /* elapsed ticks of the current period */
  uint16_t          TaskTickMax;    /* period, unit: ms (= ticks at 1kHz) */
  volatile uint8_t  TaskStatus;     /* 1: period elapsed, task is due */
  sys_task_fn_t     TaskFunc;       /* task body */
  uint8_t           Profile;        /* 1: include in the CPU load measurement */
  sys_task_gate_t   Gate;           /* 0: run on period only, else data gate */
};

/* Private function prototypes -----------------------------------------------*/
/* static  function ----------------------------------------------------------*/
static void sys_task_timer_cb(void);
static void sys_task_run(void);
static uint8_t sys_imu_sample_is_new(void);

/* Private variables ---------------------------------------------------------*/
/**
  * Task table. The order is the execution order of one scheduler pass, which is
  * what keeps the data flow consistent: sample -> fusion -> control.
  *
  *   ISR (SysTick / optional TIM2): 只推进计数, 不做 I/O, 不做姿态解算
  *        |
  *   sys_task_run() (main loop): 按顺序执行到期的任务
  *        |
  *        +-- imu_update()          5ms   软件 I2C 读取 + 解码 + 校准/低通滤波
  *        +-- ahrs_update_fusion()  5ms   仅在 IMU 产生新样本时解算姿态
  *        +-- Control_Spd_Updte()   5ms   编码器读取
  *        +-- Control_Updte()       5ms   平衡 / 速度 / 转向控制
  *        +-- Net_Link_Decode()    20ms   遥控指令解析
  *        +-- Net_Link_Out()       10ms   遥测上报
  *        +-- LED / CPU 负载   500ms / 1s
  */
static struct TaskStruct systask[] = { \
  /* TaskTickNow, Period, Status, TaskFunc,            Profile, Gate                  */
  {0, 5,    0, imu_update,                             1, 0                    },
  {0, 5,    0, ahrs_update_fusion,                     1, sys_imu_sample_is_new },
  {0, 5,    0, Control_Spd_Updte,                      1, 0                    },
  {0, 5,    0, Control_Updte,                          1, 0                    },
  {0, 20,   0, Net_Link_Decode,                        0, 0                    },
  {0, 10,   0, Net_Link_Out,                           0, 0                    },
  {0, 500,  0, Bsp_Gpio_LED_Toggle,                    0, 0                    },
  {0, 1000, 0, Rcc_Cpu_Loadcal,                        0, 0                    },
};

/** Task count, derived from the table itself so it can never run out of sync. */
#define SYS_TASK_NUM       ((uint8_t)(sizeof(systask) / sizeof(systask[0])))

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

  /* Optional hardware IMU timebase. It only counts ticks: the sampling itself
     is done by the imu_update() task below, so the ISR never touches the bus. */
  // txg_timer_init_imu_sample_trigger();
  
  Bsp_Gpio_LED_Init();
  
  /* Put the motor bridge in a known stopped state before lengthy IMU init. */
  Periph_Motor_Init();
  Periph_Motor_Stop_All();

  HAL_Delay(1000);

  /* Sensor layer: MPU60x0 init + calibration/filter setup (blocking I2C). */
  imu_init();

  /* Estimation layer: start from a known, invalid attitude state. */
  ahrs_init_fusion();
  
  Net_Link_Init();
  
  Control_Init();

  /* Start periodic application scheduling only after every module is ready. */
  Bsp_SysTick_Register_Maintask_Cb(sys_task_timer_cb);

  while(1){  
    sys_task_run();
    
  }
}

/**
  * @brief  SysTick callback, interrupt context, every SYS_TICK_MS ms.
  * @retval None
  * @note   This is the only ISR the pipeline depends on, and it does nothing but
  *         advance the software timers. No I/O and no attitude math happen here,
  *         so interrupt latency stays tiny and no task can be re-entered from an
  *         interrupt while the main loop is executing it.
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
  * @brief  Gate of the attitude fusion task: run it once per new IMU sample.
  * @retval 1 if imu_update() published a sample the fusion has not used yet.
  * @note   Without this gate a delayed scheduler pass would integrate the same
  *         gyro sample twice, which adds a fake rotation to the attitude.
  */
static uint8_t sys_imu_sample_is_new(void)
{
   static uint32_t consumed_seq = 0;
   uint32_t seq = imu_get_sample_seq();

   if(seq == consumed_seq){
      return 0;
   }

   consumed_seq = seq;

   return 1;
}

/**
  * @brief run sys task
  * @retval None
  */
static void sys_task_run(void)
{
   uint8_t taskindex = 0;
   for(taskindex = 0; taskindex < SYS_TASK_NUM; taskindex++){
     if(!systask[taskindex].TaskStatus){
       continue;
     }

     /* Data dependency: skip the task when its input is not ready. The period is
        restarted, so the data is picked up in the next scheduler pass. */
     if((systask[taskindex].Gate != 0) && (systask[taskindex].Gate() == 0)){
       systask[taskindex].TaskStatus = 0;
       continue;
     }

     /* Acknowledge the period before running the body: a tick that arrives while
        the task executes must schedule the next period, not be erased. */
     systask[taskindex].TaskStatus = 0;

     if(systask[taskindex].Profile){
        Rcc_Count_Start();
     }

     systask[taskindex].TaskFunc();

     if(systask[taskindex].Profile){
        Rcc_Count_Stop();
     }
   }
}

/********************************END OF FILE***********************************/
