/**
  ******************************************************************************
  * @file    bsp_rcc_drv.c
  * @author  zhy
  * @brief   sys clock config.
  *          This is the common part of the sys hardware initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
   * @note   |MODULE   |   CLK FREQUENCY (MHZ)   | 
             |HSE      |            25           |
             |CPU      |            72           |
             |TIME     |            72           |
  @endverbatim
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_rcc_drv.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define  DWT_CR      					*(__IO uint32_t *)0xE0001000
#define  DWT_CYCCNT  					*(__IO uint32_t *)0xE0001004
#define  DEM_CR      					*(__IO uint32_t *)0xE000EDFC


#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

//获取内核时钟频率
#define GET_CPU_ClkFreq()       		HAL_RCC_GetSysClockFreq()
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t DWT_tick1 = 0;
static uint32_t DWT_tick2 = 0;

static float CpuLoad = 0.0f;			   //负荷百分比,范围0~100
static uint32_t DWTLoadCount = 0;			//负荷计数值
/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void Bsp_Rcc_SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
  
}
 
/*在Cortex-M里面有一个外设叫DWT(Data Watchpoint and Trace)，
 该外设有一个32位的寄存器叫CYCCNT，它是一个向上的计数器，
 记录的是内核时钟运行的个数，最长能记录的时间为：
 10.74s=2的32次方/400000000
 (假设内核频率为400M，内核跳一次的时间大概为1/400M=2.5ns)
 当CYCCNT溢出之后，会清0重新开始向上计数。
 使能CYCCNT计数的操作步骤：
 1、先使能DWT外设，这个由另外内核调试寄存器DEMCR的位24控制，写1使能
 2、使能CYCCNT寄存器之前，先清0
 3、使能CYCCNT寄存器，这个由DWT_CTRL(代码上宏定义为DWT_CR)的位0控制，写1使能
 */
void Rcc_Dwt_Init(void)
{
    //使能DWT外设
    DEM_CR |= (uint32_t)DEM_CR_TRCENA;                

    //DWT CYCCNT寄存器计数清0
    DWT_CYCCNT = (uint32_t)0u;

    //使能Cortex-M DWT CYCCNT寄存器
    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

void Rcc_Count_Start(void)
{
	DWT_tick1 = ((uint32_t)DWT_CYCCNT);
}

void Rcc_Count_Stop(void)
{
	DWT_tick2 = ((uint32_t)DWT_CYCCNT);
	DWTLoadCount += (uint32_t)( DWT_tick2 - DWT_tick1);
}

/**
 * @brief  CPU负载计算
 * @note   调用周期1s
 */
void Rcc_Cpu_Loadcal(void)		
{
	CpuLoad = (float)DWTLoadCount / GET_CPU_ClkFreq() * 100.0f;
	DWTLoadCount = 0;
}

/**
 * @brief  获取CPU 利用率
 * @retval CPU利用率
 *  
 */
float Rcc_Get_Cpuusage(void)
{
 return CpuLoad;
}
/********************************END OF FILE***********************************/
