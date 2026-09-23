/**
  ******************************************************************************
  * @file    periph_serial_motor.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the port
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_MOTOR_H
#define PERIPH_SERIAL_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif   
/* Includes ------------------------------------------------------------------*/
#include "bsp_timer_drv.h"   

/* Exported types ------------------------------------------------------------*/
/** @defgroup Motor Exported Types
  * @{
  */ 

 typedef enum
 {
                
   MOTORL_ID,
   MOTORR_ID,
   
}MOTOR_ID; 

 typedef enum
 {
   
  MOTOR_DF,           
  MOTOR_DB,         
   
}MOTOR_DIR;

 
 
/**
* @}
*/  

/* Peripheral Control functions  **********************************************/
void Periph_Motor_Init(void);
void Periph_Motor_Set_Out(uint8_t which_motor, uint16_t val);
void Periph_Motor_Set_DF(uint8_t which_motor);
void Periph_Motor_Set_DB(uint8_t which_motor);
void Periph_Motor_Set_ST(uint8_t which_motor);
void Periph_Motor_Stop_All(void);
int16_t Periph_Motor_Get_Encoder(uint8_t which_motor);
MOTOR_DIR Periph_Motor_Get_Dir(uint8_t which_motor);

#ifdef __cplusplus
}
#endif



#endif 

/********************************END OF FILE***********************************/
