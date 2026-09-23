/**
  ******************************************************************************
  * @file    bsp_gpio_drv.c
  * @author  zhy
  * @brief   GPIO config.
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
#include "bsp_delay_drv.h"
#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/*120Mhz时钟时，当ulCount为1时，函数耗时3个时钟，延时=3*1/120us=1/40us*/
/*
  SystemCoreClock=120000000
  us级延时,延时n微秒
  SysCtlDelay(n*(SystemCoreClock/3000000));
  ms级延时,延时n毫秒
  SysCtlDelay(n*(SystemCoreClock/3000));
  m级延时,延时n秒
  SysCtlDelay(n*(SystemCoreClock/3));
*/
   
#if defined   (__CC_ARM)     /*!< ARM Compiler */
__asm void SysCtlDelay(unsigned long ulCount)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}

#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
void SysCtlDelay(unsigned long ulCount)
{
  __asm("    subs    r0, #1\n"
        "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}

#elif defined (__GNUC__) /*!< GNU Compiler */
void __attribute__((naked)) SysCtlDelay(unsigned long ulCount)
{
  __asm("    subs    r0, #1\n"
        "    bne     SysCtlDelay\n"
          "    bx      lr");
}

#elif defined  (__TASKING__) /*!< TASKING Compiler */                           


#endif /* __CC_ARM */

/**
  * @brief delay us
  * @retval None
  */
void Bsp_Delay_Us(unsigned long ulCount)
{
  SysCtlDelay(ulCount * (SystemCoreClock/3000000));
}

/**
  * @brief delay ms
  * @retval None
  */
void Bsp_Delay_Ms(unsigned long ulCount)
{
  SysCtlDelay(ulCount * (SystemCoreClock/3000));
}

/**
  * @brief delay ms
  * @retval None
  */
void Bsp_Delay_S(unsigned long ulCount)
{
  SysCtlDelay(ulCount * (SystemCoreClock/3));
}

/********************************END OF FILE***********************************/
