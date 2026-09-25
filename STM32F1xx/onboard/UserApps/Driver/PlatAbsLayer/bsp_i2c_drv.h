/**
  ******************************************************************************
  * @file    bsp_i2c_drv.h
  * @author  zhy
  * @brief   I2C config.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_I2C_DRV_H
#define BSP_I2C_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/  
#include "stm32f1xx_hal.h"

/*  struct  ------------------------------------------------------------------*/  
struct bsp_ioi2c_t
{
    GPIO_TypeDef* scl_gpio_port;	// GPIO port of SCL
    GPIO_TypeDef* sda_gpio_port;	// GPIO port of SDA
    uint16_t      scl_gpio_pin;
    uint16_t      sda_gpio_pin;
};

/* Peripheral Control functions  ************************************************/
void Bsp_I2c_Init(struct bsp_ioi2c_t *ioi2c);
uint8_t Bsp_I2c_Write_Buffer(struct bsp_ioi2c_t *ioi2c,uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
uint8_t Bsp_I2c_Read_Buffer(struct bsp_ioi2c_t *ioi2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
int8_t Bsp_I2c_Multi_Read_Reg16(struct bsp_ioi2c_t *ioi2c, uint8_t SlaveAddress, \
                                uint16_t REG_Address, uint8_t size,uint8_t *ptChar);
int8_t Bsp_I2c_Multi_Write_Reg16(struct bsp_ioi2c_t *ioi2c, uint8_t slave_addr, \
                                 uint16_t reg, uint8_t length, uint8_t *data);                           
#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
