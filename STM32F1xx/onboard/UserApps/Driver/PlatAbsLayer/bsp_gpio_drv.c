/**
  ******************************************************************************
  * @file    bsp_gpio_drv.c
  * @author  zhy
  * @brief   GPIO config.
  *          This is the common part of the sys hardware initialization
  *
  ******************************************************************************
  * @attention None
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio_drv.h"
#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define GPIO_LED_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE() 
#define GPIO_LED_PORT                 GPIOD  
#define GPIO_LED_PIN                  GPIO_PIN_2


/* Private functions ---------------------------------------------------------*/

/**
  * @brief Gpio LED init
  * @retval None
  * @note  output Level  default low
  */
void Bsp_Gpio_LED_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_LED_CLK_ENABLE();
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  GPIO_InitStruct.Pin = GPIO_LED_PIN;
  HAL_GPIO_Init(GPIO_LED_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_SET);

}

/**
  * @brief Gpio LED ON
  * @retval None
  * @note  output Level  default low
  */
void Bsp_Gpio_LED_ON(void)
{
   HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_RESET);
}

/**
  * @brief Gpio LED OFF
  * @retval None
  * @note  output Level  default low
  */
void Bsp_Gpio_LED_OFF(void)
{
   HAL_GPIO_WritePin(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_PIN_SET);
}

/**
  * @brief Gpio LED1 toggle
  * @retval None
  * @note  output Level  default low
  */
void Bsp_Gpio_LED_Toggle(void)
{
   HAL_GPIO_TogglePin(GPIO_LED_PORT, GPIO_LED_PIN);
}

/********************************END OF FILE***********************************/
