/**
  ******************************************************************************
  * @file    bsp_i2c_drv.c
  * @author  zhy
  * @brief   I2C config.
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
#include "bsp_i2c_drv.h"
#include "bsp_delay_drv.h"
#include <stdint.h>
    
/* Private define ------------------------------------------------------------*/
#define GPIO_I2C_CLK_ENABLE()         do { \
  __HAL_RCC_GPIOA_CLK_ENABLE(); \
  __HAL_RCC_GPIOB_CLK_ENABLE(); \
  __HAL_RCC_GPIOC_CLK_ENABLE(); \
} while (0)

#define I2C_SCL_H                     HAL_GPIO_WritePin(ioi2c->scl_gpio_port, ioi2c->scl_gpio_pin, GPIO_PIN_SET)
#define I2C_SCL_L                     HAL_GPIO_WritePin(ioi2c->scl_gpio_port, ioi2c->scl_gpio_pin, GPIO_PIN_RESET)
#define I2C_SDA_H                     HAL_GPIO_WritePin(ioi2c->sda_gpio_port, ioi2c->sda_gpio_pin, GPIO_PIN_SET)
#define I2C_SDA_L                     HAL_GPIO_WritePin(ioi2c->sda_gpio_port, ioi2c->sda_gpio_pin, GPIO_PIN_RESET)
#define i2c_delay_us(us)              Bsp_Delay_Us(us)  
    
/* Private functions ---------------------------------------------------------*/
static inline uint8_t bsp_i2c_pin_pos(uint16_t pin)
{
  uint8_t pos = 0;
  while ((pin & 1) == 0) { pin >>= 1; pos++; }
  return pos;
}

static void bsp_i2c_sda_out(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_sda_in(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_start(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_stop(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_ack(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_nack(struct bsp_ioi2c_t *ioi2c);
static uint8_t bsp_i2c_wait_ack(struct bsp_ioi2c_t *ioi2c);
static void bsp_i2c_send_byte(struct bsp_ioi2c_t *ioi2c, uint8_t txd);
static uint8_t bsp_i2c_read_byte(struct bsp_ioi2c_t *ioi2c);
static uint8_t bsp_i2c_clock_high(struct bsp_ioi2c_t *ioi2c);

/**
  * @brief Gpio I2C init
  * @retval None
  * @note  output Level  default HIHG
  */
void Bsp_I2c_Init(struct bsp_ioi2c_t *ioi2c)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_I2C_CLK_ENABLE();
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  GPIO_InitStruct.Pin = ioi2c->scl_gpio_pin;
  HAL_GPIO_Init(ioi2c->scl_gpio_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ioi2c->sda_gpio_pin;
  HAL_GPIO_Init(ioi2c->sda_gpio_port, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(ioi2c->scl_gpio_port, ioi2c->scl_gpio_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ioi2c->sda_gpio_port, ioi2c->sda_gpio_pin, GPIO_PIN_SET);
}

/**
  * @brief Gpio SDA OUT 
  * @retval None
  * @note  Direct register access instead of HAL_GPIO_Init for speed
  */
static void bsp_i2c_sda_out(struct bsp_ioi2c_t *ioi2c)
{
  uint8_t pos = bsp_i2c_pin_pos(ioi2c->sda_gpio_pin);
  volatile uint32_t *cr;
  uint32_t shift;

  if (pos < 8) {
    cr = &ioi2c->sda_gpio_port->CRL;
    shift = pos * 4;
  } else {
    cr = &ioi2c->sda_gpio_port->CRH;
    shift = (pos - 8) * 4;
  }
  /* Output open-drain 50MHz: MODE=11, CNF=01 -> 0x7. */
  *cr = (*cr & ~(0x0FUL << shift)) | (0x07UL << shift);
}

static uint8_t bsp_i2c_clock_high(struct bsp_ioi2c_t *ioi2c)
{
  uint16_t timeout = 100;

  I2C_SCL_H;
  while (HAL_GPIO_ReadPin(ioi2c->scl_gpio_port, ioi2c->scl_gpio_pin) == GPIO_PIN_RESET)
  {
    if (--timeout == 0)
        return 0;
  }
  i2c_delay_us(2);
  return 1;
}

/**
  * @brief Gpio SDA IN
  * @retval None
  * @note  Direct register access instead of HAL_GPIO_Init for speed
  */
static void bsp_i2c_sda_in(struct bsp_ioi2c_t *ioi2c)
{
  uint8_t pos = bsp_i2c_pin_pos(ioi2c->sda_gpio_pin);
  volatile uint32_t *cr;
  uint32_t shift;

  if (pos < 8) {
    cr = &ioi2c->sda_gpio_port->CRL;
    shift = pos * 4;
  } else {
    cr = &ioi2c->sda_gpio_port->CRH;
    shift = (pos - 8) * 4;
  }
  /* Input with pull-up: MODE=00, CNF=10 → 0x8 */
  *cr = (*cr & ~(0x0FUL << shift)) | (0x08UL << shift);
  /* Set ODR bit for pull-up */
  ioi2c->sda_gpio_port->BSRR = ioi2c->sda_gpio_pin;
}

/**
  * @brief i2c start
  * @retval None
  * @note  None
  */
static void bsp_i2c_start(struct bsp_ioi2c_t *ioi2c)
{
  bsp_i2c_sda_out(ioi2c);
  I2C_SDA_H;
  if (!bsp_i2c_clock_high(ioi2c))
      return;
  I2C_SDA_L;
  i2c_delay_us(4);
  I2C_SCL_L; 
}

/**
  * @brief i2c stop
  * @retval None
  * @note  None
  */
static void bsp_i2c_stop(struct bsp_ioi2c_t *ioi2c)
{
  bsp_i2c_sda_out(ioi2c);
  
  I2C_SCL_L;
  I2C_SDA_L;
  i2c_delay_us(4);
  bsp_i2c_clock_high(ioi2c);
  I2C_SDA_H;
  i2c_delay_us(4);
}

/**
  * @brief i2c ACK
  * @retval None
  * @note  None
  */
static void bsp_i2c_ack(struct bsp_ioi2c_t *ioi2c)
{
   I2C_SCL_L;
   bsp_i2c_sda_out(ioi2c);
   I2C_SDA_L;
   i2c_delay_us(2);
  bsp_i2c_clock_high(ioi2c);
   I2C_SCL_L;
}

/**
  * @brief i2c NACK
  * @retval None
  * @note  None
  */
static void bsp_i2c_nack(struct bsp_ioi2c_t *ioi2c)
{
   I2C_SCL_L;
   bsp_i2c_sda_out(ioi2c);
   I2C_SDA_H;
   i2c_delay_us(2);
  bsp_i2c_clock_high(ioi2c);
   I2C_SCL_L;
}

/**
  * @brief i2c NACK
  * @retval None
  * @note  None
  * @return 0 - success  1-failed
  */
static uint8_t bsp_i2c_wait_ack(struct bsp_ioi2c_t *ioi2c)
{
  uint8_t tempTime=0;
  
  bsp_i2c_sda_in(ioi2c);
  
  I2C_SDA_H;
  i2c_delay_us(1);
  if (!bsp_i2c_clock_high(ioi2c))
  {
    bsp_i2c_stop(ioi2c);
    return 1;
  }

  while(HAL_GPIO_ReadPin(ioi2c->sda_gpio_port, ioi2c->sda_gpio_pin) == GPIO_PIN_SET)
  {
    if (++tempTime > 10)
    {
      I2C_SCL_L;
      bsp_i2c_stop(ioi2c);
      return 1;
    }
    i2c_delay_us(1);
  }
  
  I2C_SCL_L;
  return 0;
}

/**
  * @brief i2c send byte
  * @retval None
  * @note None
  */
static void bsp_i2c_send_byte(struct bsp_ioi2c_t *ioi2c, uint8_t txd)
{
  uint8_t i=0;
  
  bsp_i2c_sda_out(ioi2c);
  I2C_SCL_L;
  
  for(i=0;i<8;i++)
  {
    if((txd&0x80)>0) 
      I2C_SDA_H;
    else
      I2C_SDA_L;
    
    txd<<=1;
    if (!bsp_i2c_clock_high(ioi2c))
      return;
    I2C_SCL_L;
    i2c_delay_us(2);
  }
}

/**
  * @brief i2c read byte
  * @retval None
  * @note None
  */
static uint8_t bsp_i2c_read_byte(struct bsp_ioi2c_t *ioi2c)
{
  uint8_t i = 0,receive = 0;
  
  bsp_i2c_sda_in(ioi2c);
  for(i=0; i<8; i++)
  {
    I2C_SCL_L;
    i2c_delay_us(2);
    bsp_i2c_clock_high(ioi2c);
    receive<<=1;
    if(HAL_GPIO_ReadPin(ioi2c->sda_gpio_port, ioi2c->sda_gpio_pin) == GPIO_PIN_SET){
      receive++;
    }
    i2c_delay_us(1);	
  }
  return receive;
}

/**
  * @brief i2c write buf
  * @retval None
  * @note None
  */
uint8_t Bsp_I2c_Write_Buffer(struct bsp_ioi2c_t *ioi2c,uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
  int i;
  bsp_i2c_start(ioi2c);
  bsp_i2c_send_byte(ioi2c, addr << 1 | 0);
  if (bsp_i2c_wait_ack(ioi2c)) {
    bsp_i2c_stop(ioi2c);
    return 0;
  }
  bsp_i2c_send_byte(ioi2c, reg);
  if (bsp_i2c_wait_ack(ioi2c)) {
    bsp_i2c_stop(ioi2c);
    return 0;
  }
  for (i = 0; i < len; i++) {
    bsp_i2c_send_byte(ioi2c,*data);
    if (bsp_i2c_wait_ack(ioi2c)) {
      bsp_i2c_stop(ioi2c);
      return 0;
    }
    data++;
  }
  bsp_i2c_stop(ioi2c);
  return 1;
}

/**
  * @brief i2c read buf
  * @retval None
  * @note None
  */
uint8_t Bsp_I2c_Read_Buffer(struct bsp_ioi2c_t *ioi2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    bsp_i2c_start(ioi2c);
    bsp_i2c_send_byte(ioi2c, addr << 1 | 0);
    if (bsp_i2c_wait_ack(ioi2c)){
        bsp_i2c_stop(ioi2c);
        return 0;
    }
    bsp_i2c_send_byte(ioi2c, reg);
    if (bsp_i2c_wait_ack(ioi2c)) {
      bsp_i2c_stop(ioi2c);
      return 0;
    }

    bsp_i2c_start(ioi2c);
    bsp_i2c_send_byte(ioi2c, addr << 1 | 1);
    if (bsp_i2c_wait_ack(ioi2c)) {
      bsp_i2c_stop(ioi2c);
      return 0;
    }
    while (len){
      *buf = bsp_i2c_read_byte(ioi2c);
      if (len == 1)
        bsp_i2c_nack(ioi2c);
      else
        bsp_i2c_ack(ioi2c);
      buf++;
      len--;
    }
    bsp_i2c_stop(ioi2c);
    return 1;
}


/**
 * Read several bytes from i2c device.
 * @param  ioi2c        : ioi2c struct.
 * @param  SlaveAddress : device address.
 * @param  REG_Address  : register address.
 * @param  size         : length to read.
 * @param  ptChar       : data buffer.
 * @return              : 1 if read ok.
 */
int8_t Bsp_I2c_Multi_Read_Reg16(struct bsp_ioi2c_t *ioi2c, uint8_t SlaveAddress, \
                                uint16_t REG_Address, uint8_t size,uint8_t *ptChar)
{
    uint8_t i;

    if (size < 1)
        return 0;

    bsp_i2c_start(ioi2c);

    bsp_i2c_send_byte(ioi2c, SlaveAddress);
    if (bsp_i2c_wait_ack(ioi2c))
    {
        bsp_i2c_stop(ioi2c);
        return 0;
    }

    bsp_i2c_send_byte(ioi2c, (uint8_t)(REG_Address >> 8));
    if (bsp_i2c_wait_ack(ioi2c)) goto read16_fail;
    bsp_i2c_send_byte(ioi2c, (uint8_t)REG_Address);
    if (bsp_i2c_wait_ack(ioi2c)) goto read16_fail;

    bsp_i2c_start(ioi2c);
    bsp_i2c_send_byte(ioi2c, (uint8_t)(SlaveAddress | 1U));
    if (bsp_i2c_wait_ack(ioi2c)) goto read16_fail;

    for (i=1; i<size; i++)
    {
        *ptChar++ = bsp_i2c_read_byte(ioi2c);
        bsp_i2c_ack(ioi2c);
    }
    *ptChar++ = bsp_i2c_read_byte(ioi2c);
    bsp_i2c_nack(ioi2c);
    bsp_i2c_stop(ioi2c);
    return 1;

  read16_fail:
    bsp_i2c_stop(ioi2c);
    return 0;
}

/**
 * Write multiple data via i2c
 * @param  ioi2c      : ioi2c structure
 * @param  slave_addr : slave address
 * @param  reg        : register address.
 * @param  length     : length to read
 * @param  data       : read data buffer.
 * @return            : 1 if read ok
 */
int8_t Bsp_I2c_Multi_Write_Reg16(struct bsp_ioi2c_t *ioi2c, uint8_t slave_addr, \
                                 uint16_t reg, uint8_t length, uint8_t *data)
{
    uint8_t address_h = 0;
    uint8_t address_l = 0;
    uint8_t count = 0;

    address_h = (uint8_t)(reg>>8);
    address_l = (uint8_t)(reg);

    bsp_i2c_start(ioi2c);

    bsp_i2c_send_byte(ioi2c, slave_addr);

    if (bsp_i2c_wait_ack(ioi2c))
    {
        bsp_i2c_stop(ioi2c);
        return 0;
    }

    bsp_i2c_send_byte(ioi2c, address_h);
    if (bsp_i2c_wait_ack(ioi2c))
    {
      bsp_i2c_stop(ioi2c);
      return 0;
    }
    bsp_i2c_send_byte(ioi2c, address_l);
    if (bsp_i2c_wait_ack(ioi2c))
    {
      bsp_i2c_stop(ioi2c);
      return 0;
    }

    for (count=0; count<length; count++)
    {
        bsp_i2c_send_byte(ioi2c, data[count]);
        if (bsp_i2c_wait_ack(ioi2c))
        {
          bsp_i2c_stop(ioi2c);
          return 0;
        }
    }
    bsp_i2c_stop(ioi2c);

    return 1;
}





/********************************END OF FILE***********************************/
