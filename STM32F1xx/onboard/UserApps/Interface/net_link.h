/**
  ******************************************************************************
  * @file    net_link.h
  * @author  YZH
  * @brief   TTL-WIFI
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NET_LINK_H
#define __NET_LINK_H

/* Includes ------------------------------------------------------------------*/

#include<stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup 通信接口
 * @{
 */
/** @defgroup 网络通信
 * @brief
 * @{
 */

 /** @defgroup  网络通信外部枚举/结构体类型/宏
  * @{
  */ 
/**
 * @brief  协议识别码
 */
typedef enum  {

    NCID_TA = 0x01,    /**<A帧数据发送 传感器数据*/
    NCID_TB,           /**<B帧数据发送 控制类参数*/
    NCID_TC,           /**<C帧数据发送 遥控数据参数*/
    NCID_TD,           /**<D帧数据发送 命令反馈参数*/

    NCID_RA = 0x11,    /**<A帧数据接收 控制类参数设置*/
    NCID_RB = 0x12,    /**<B帧数据接收 摇杆数据*/
    
}NET_IDCODE;


/**
 * @brief  接收A帧数据
 */
struct  NET_RA
{
  float balance_kp;           /**<直立环比例P*/
  float balance_kd;           /**<直立环微分D*/
  float vel_kp;               /**<速度比例P*/
  float vel_ki;               /**<速度积分I*/ 
  float turn_kp;              /**<速度比例P*/
  float turn_kd;              /**<速度积分I*/ 
  int16_t taget_spd;          /**<目标速度*/ 
  float   taget_yaw;          /**<目标偏航角*/
  uint8_t  ifsave;            /**<是否保存*/
};

/**
 * @brief  接收B帧数据
 */
struct  NET_RB
{

  int16_t taget_spd;          /**<目标速度*/ 
  float   taget_ryaw;         /**<目标偏航角速率*/
  uint8_t enable;             /**<0: disarm, 1: arm*/

};

/**
* @}
*/  

 /* Exported function ------------------------------------------------------------*/
/** @defgroup 网络通信外部函数 网络通信外部函数
  * @{
  */
void Net_Link_Init(void);
void Net_Link_Decode(void);
void Net_Link_Out(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif 

/*****************************END OF FILE**************************************/
