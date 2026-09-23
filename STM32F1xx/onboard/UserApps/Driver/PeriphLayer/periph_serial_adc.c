/**
******************************************************************************
* @file    periph_serial_adc.c
* @author  zhy
* @brief   This is the common part of the sys periph initialization
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
#include "periph_serial_adc.h"
#include "txg_math.h"

/* Private define ------------------------------------------------------------*/   
#define ADC_FILTER_NUM                5
#define ADC_VREF                      3.3
#define ADC_FULL_SCALE                4096.0 //2^12 -1
#define ADC_COEF                      3
#define ADC_V25                       1.43
#define ADC_AVG_SLOPE                 0.0043
/* -----------------------------AD Current Extern  Resistance(¦¸)--------------*/
#define ADC_I_EX_RESIS               250.0  

/* Private variables ---------------------------------------------------------*/
static float adc_sample_value[ADC_CH_MAX * ADC_FILTER_NUM] = {0};
static float adc_filter_value[ADC_CH_MAX] = {0};
static uint8_t adc_sample_order = 0;

/* Private function prototypes -----------------------------------------------*/
static void periph_serial_adc_sample(void);
static void periph_serial_adc_fliter(void);

/**
* @brief  ADC init.  
* @param  None  
* @retval None    
*/

void Periph_Adc_Init(void)
{
  Bsp_Adc_Init();
}

/**
  * @brief adc sample filter
  * @retval None
  * @note   10ms
            temp fuc https://blog.csdn.net/fukangwei_lite/article/details/117805934
  */
static void periph_serial_adc_sample(void)
{
  uint16_t temp_raw_value = 0; 
  temp_raw_value = Bsp_Get_Adc_Value(ADC_CH1);
  adc_sample_value[ADC_CH1 * ADC_FILTER_NUM + adc_sample_order] = \
                (float)(temp_raw_value*ADC_VREF*ADC_COEF/ADC_FULL_SCALE);
  temp_raw_value = Bsp_Get_Adc_Value(ADC_CH2);
  adc_sample_value[ADC_CH2 * ADC_FILTER_NUM + adc_sample_order] = \
                (float)(temp_raw_value*ADC_VREF*ADC_COEF/ADC_FULL_SCALE);
  temp_raw_value = Bsp_Get_Adc_Value(ADC_CH3);
  adc_sample_value[ADC_CH3 * ADC_FILTER_NUM + adc_sample_order] = \
                (float)(temp_raw_value*ADC_VREF*ADC_COEF/ADC_FULL_SCALE/ADC_I_EX_RESIS);
  temp_raw_value = Bsp_Get_Adc_Value(ADC_CH4);
  adc_sample_value[ADC_CH4 * ADC_FILTER_NUM + adc_sample_order] = \
                (float)(temp_raw_value*ADC_VREF*ADC_COEF/ADC_FULL_SCALE/ADC_I_EX_RESIS);
  temp_raw_value = Bsp_Get_Adc_Value(ADC_CH5);
  adc_sample_value[ADC_CH5 * ADC_FILTER_NUM + adc_sample_order] = \
                (float)((ADC_V25 - temp_raw_value*ADC_VREF/ADC_FULL_SCALE)/ADC_AVG_SLOPE + 25); 
  adc_sample_order++;
}

/**
  * @brief adc filter
  * @retval None
  * @note   100ms
  */
static void periph_serial_adc_fliter(void)
{
   uint8_t index = 0,adc_fliter_num = 0;
   adc_fliter_num = (adc_sample_order > ADC_FILTER_NUM)? ADC_FILTER_NUM:adc_sample_order;
   for(index = 0; index < ADC_CH_MAX; index++){
     adc_filter_value[index] = Float_Sum_Mean(&adc_sample_value[index * ADC_FILTER_NUM],adc_fliter_num);
   }
   adc_sample_order %= ADC_FILTER_NUM;
}


/* -------------------------------------data decode---------------------------*/
/**
* @brief  adc data Update.  
*/
void Periph_Adc_Update(void)
{
  periph_serial_adc_sample();
  periph_serial_adc_fliter();
}

/* -------------------------------------get data-------------------------------*/


/**
* @brief  get raw data. 
* @param  adc_module ADC_MODULE
          adc_chx    ADCX_CHX
* @return  Voltage unit V
*          Current unit A
*/

float Periph_Adc_Get_Val(uint8_t adc_module, uint8_t adc_chx)
{
  return adc_filter_value[adc_module * 2 + adc_chx];
}


/********************************END OF FILE***********************************/
