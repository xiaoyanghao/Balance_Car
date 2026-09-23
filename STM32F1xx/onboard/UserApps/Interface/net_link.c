/**
  ******************************************************************************
  * @file   net_link.c
  * @author  YZH
  * @brief   TTL-WIFI
  *
  @verbatim
  ==============================================================================
					 ##### TTL-WIFI���� #####
  ==============================================================================
[..]
�������ܲ���:
   (+) �ͺ�          ��GTDA1136
   (+) ��Ч����ת��   ��360��
   (+) �ֱ���        ��4096λ ��12λ�� 
   (+) �����ٶ�      ��0.6ms
   (+) ������ѹ      ��5V��10%
   (+) ����ź�      ��0~5V
   (+) ��������      ��14mA
   (+) �����¶�      ��-30~+80��
   (+) �����¶�      ��-30~+80��
   (+) ��ת����      ��<5mN*m  
   (+) ����˵��      ����ɫ��-VCC ��ɫ��-GND ��ɫ��-�����OUT
   (+) �ο��ļ�      ��WXXY������ӷǽӴ�ʽ�����Ƕȴ�����
  ��+���궨�ļ��ο�   ��
  @endverbatim
  ******************************************************************************
  * @attention
	<pre>
	</pre>
  *
  ******************************************************************************
  */
/** @addtogroup ͨ�Žӿ�
 * @{
 */
/** @defgroup ����ͨ��
 * @brief
 * @{
 */
/* Includes ------------------------------------------------------------------*/
#include "net_link.h"
#include "bsp_rcc_drv.h"
#include "periph_serial_port.h"
#include "shared.h"
#include "txg_math.h"
#include "imu.h"
#include "txg_ahrs_fusion.h"
#include "control.h"
#include <string.h>

/* Private type---------------------------------------------------------------*/

/* Private Macro -------------------------------------------------------------*/
#define  NETBUF_SIZE            256
#define  NET_MSG_RLEN           45
#define  NET_MSG_TLEN           96
#define  NET_MSG_CMDLEN         20
/* -------------------------extern variable-----------------------------------*/
/* -------------------------static variable-----------------------------------*/
static UART_T  net_uart = {0};
static uint8_t net_frxbuf[NETBUF_SIZE] = {0};
static uint8_t net_ftxbuf[NETBUF_SIZE] = {0};
static uint8_t net_rxbuf[NETBUF_SIZE]  = {0};
static uint8_t net_txbuf[NETBUF_SIZE]  = {0};
static uint8_t net_cmd_msg[NET_MSG_CMDLEN] = {0};

/* -------------------------static function----------------------------------*/
/** @defgroup ����ͨ�ž�̬����
 * @{
 */
static void net_link_tx(uint8_t *buf, uint16_t len);
static enum MSG_INPUT_RESULT netlink_try_to_catch_frame(uint8_t *buf);
static void netlink_TA_encode(uint8_t *buf);
static void netlink_TB_encode(uint8_t *buf);
static void netlink_TD_encode(uint8_t *buf);
static void netlink_RA_decode(uint8_t *buf);
static void netlink_RB_decode(uint8_t *buf);
/**
 * @brief   ���緢�ͺ���
 * @param[in] buf:���ݷ��ͻ�����
 * @param[in] len:���ݷ��ͳ���
 */
static void net_link_tx(uint8_t *buf, uint16_t len)
{
    Periph_Port_Send_Buffer(&net_uart, buf, len);
}

/**
 * @brief  Try to get a whole frame of data received.
 * @param  buf : buffer to store rx data.
 * @retval MSG_INPUT_SUCCESS if frame get and CRC check pass.
 */
static enum MSG_INPUT_RESULT netlink_try_to_catch_frame(uint8_t *buf)
{
    enum MSG_INPUT_RESULT net_frame_get_ret = MSG_INPUT_HALF;

    uint16_t next_idx = 0;

    int16_t this_rx_len = 0;
    uint16_t index = 0;
    uint8_t crc_check_rx = 0;

    const uint16_t curr_head = net_uart.usRxHead;

    // If mcu handle input interrupt now, do not processing
    if (net_uart.rxComplete){
        this_rx_len = curr_head - net_uart.usRxTail; 
        if (this_rx_len < 0) {
            this_rx_len += net_uart.usRxBufSize;
        }
    } 
    else  {
        goto net_catch_frame_ret;
    }
    // check if there are enough data to handle
    if (this_rx_len < NET_MSG_RLEN){
        net_frame_get_ret = MSG_INPUT_HALF;

        goto net_catch_frame_ret;
    }
    // Find until start byte is 0xEB and 0x90
    next_idx = (net_uart.usRxTail+1) % net_uart.usRxBufSize;

    while (net_frxbuf[net_uart.usRxTail]!=0xEB || net_frxbuf[next_idx]!=0x90){
        // Until current head, there are no 0xEB and 0x90 found, frame may be error
        if (next_idx == curr_head){
            net_frame_get_ret = MSG_INPUT_HALF;

            goto net_catch_frame_ret;
        } 
        else{
            net_uart.usRxTail = (net_uart.usRxTail+1) % net_uart.usRxBufSize;
            net_uart.usRxCount--;

            next_idx = (net_uart.usRxTail+1) % net_uart.usRxBufSize;
        }
    }

    // Recalculate length of this frame because rxTail maybe changed
    this_rx_len = curr_head - net_uart.usRxTail;
    
    if (this_rx_len < 0){
        this_rx_len += net_uart.usRxBufSize;
    }

    // Check if there are enough data to handle
    if (this_rx_len < NET_MSG_RLEN )  {
        net_frame_get_ret = MSG_INPUT_HALF;

        goto net_catch_frame_ret;
    } // TODO: if this_rx_len>usRxCount, maybe error

    // Copy new data to rx_pkt, rxTail and usCount should also be processed
    for (index = 0; index < NET_MSG_RLEN; index++)  {
        buf[index] = net_frxbuf[net_uart.usRxTail];

        net_uart.usRxTail = (net_uart.usRxTail + 1) % net_uart.usRxBufSize;

        net_uart.usRxCount--;
    }

    // Check CRC of this frame
    crc_check_rx = checksum_8((uint8_t*)&buf[2], NET_MSG_RLEN-3);

    // If CRC check pass, return true to handle it, or CRC_ERROR status
    if (crc_check_rx == buf[NET_MSG_RLEN-1]) {
        net_frame_get_ret = MSG_INPUT_SUCCESS;

        goto net_catch_frame_ret;
    } 
    else{
        net_frame_get_ret = MSG_INPUT_CRC_ERR;

        goto net_catch_frame_ret;
    }
net_catch_frame_ret:
    return net_frame_get_ret;
}

/**
  * @brief A֡���ݷ���
*/
static void netlink_TA_encode(uint8_t *buf)
{
  int16_t tmp_i16 = 0;
  int32_t tmp_i32 = 0;
  uint32_t tmp_u32 = 0;
  uint8_t idx = 0;

  buf[0] = 0xEB;
  buf[1] = 0x90;
  buf[2] = NCID_TA;

  /*����
  */
 
  buf[3] = imu_get_health_status(IMU_INSTANCE_MPU);

  tmp_i16 = (int16_t)(imu_get_acc_temperature()*100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[4]); 

  tmp_i32 = (int32_t)(imu_get_gyro_rawest(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[6]);
  tmp_i32 = (int32_t)(imu_get_gyro_rawest(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[10]);
  tmp_i32 = (int32_t)(imu_get_gyro_rawest(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[14]);

  tmp_i32 = (int32_t)(imu_get_gyro_raw(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[18]);
  tmp_i32 = (int32_t)(imu_get_gyro_raw(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[22]);
  tmp_i32 = (int32_t)(imu_get_gyro_raw(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[26]);

  tmp_i32 = (int32_t)(imu_get_gyro(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[30]);
  tmp_i32 = (int32_t)(imu_get_gyro(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[34]);
  tmp_i32 = (int32_t)(imu_get_gyro(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[38]);

  tmp_i32 = (int32_t)(imu_get_acc_rawest(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[42]);
  tmp_i32 = (int32_t)(imu_get_acc_rawest(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[46]);
  tmp_i32 = (int32_t)(imu_get_acc_rawest(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[50]);

  tmp_i32 = (int32_t)(imu_get_acc_raw(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[54]);
  tmp_i32 = (int32_t)(imu_get_acc_raw(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[58]);
  tmp_i32 = (int32_t)(imu_get_acc_raw(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[62]);

  tmp_i32 = (int32_t)(imu_get_acc(IMU_INSTANCE_MPU,0)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[66]);
  tmp_i32 = (int32_t)(imu_get_acc(IMU_INSTANCE_MPU,1)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[70]);
  tmp_i32 = (int32_t)(imu_get_acc(IMU_INSTANCE_MPU,2)*1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[74]);

  tmp_i16 = (int16_t)(get_roll_deg()*100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[78]); 
  tmp_i16 = (int16_t)(get_pitch_deg()*100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[80]); 
  tmp_i16 = (int16_t)(get_yaw_deg()*100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[82]); 

  tmp_u32 = HAL_GetTick();
  EncodeDW2Bytes('l', tmp_u32, (char *)&buf[84]);

  for(idx = 89;idx < (NET_MSG_TLEN-1);idx++){
    buf[idx] = 0;
  }
  buf[NET_MSG_TLEN-1] = checksum_8(&buf[2], NET_MSG_TLEN-2);
}

/**
  * @brief B֡���ݷ���
*/
static void netlink_TB_encode(uint8_t *buf)
{
  int16_t tmp_i16 = 0;
  int32_t tmp_i32 = 0;
  uint32_t tmp_u32 = 0;
  uint8_t idx = 0;
  struct  CONTROL control_pram;

  buf[0] = 0xEB;
  buf[1] = 0x90;
  buf[2] = NCID_TB;

  /*����
  */
  Control_Get_Pram(&control_pram);

  tmp_i32 = (int32_t)(control_pram.balance_kp * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[3]); 
  tmp_i32 = (int32_t)(control_pram.balance_kd * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[7]); 
  tmp_i32 = (int32_t)(control_pram.vel_kp * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[11]); 
  tmp_i32 = (int32_t)(control_pram.vel_ki * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[15]); 
  tmp_i32 = (int32_t)(control_pram.turn_kp * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[19]); 
  tmp_i32 = (int32_t)(control_pram.turn_kd * 1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[23]);  


  EncodeDWord2Bytes('l', control_pram.balance_pwm, (char *)&buf[27]); 
  EncodeDWord2Bytes('l', control_pram.vel_pwm, (char *)&buf[29]); 
  EncodeDWord2Bytes('l', control_pram.lturn_pwm, (char *)&buf[31]); 
  EncodeDWord2Bytes('l', control_pram.lfinal_pwm, (char *)&buf[33]); 
  EncodeDWord2Bytes('l', control_pram.rturn_pwm, (char *)&buf[35]); 
  EncodeDWord2Bytes('l', control_pram.rfinal_pwm, (char *)&buf[37]); 
  buf[39]  = control_pram.l_dircmd;
  buf[40]  = control_pram.r_dircmd;

  EncodeDWord2Bytes('l', control_pram.taget_spd, (char *)&buf[41]); 
  EncodeDWord2Bytes('l', control_pram.lencode_spd, (char *)&buf[43]); 
  EncodeDWord2Bytes('l', control_pram.rencode_spd, (char *)&buf[45]);
  buf[47]  = control_pram.l_dir;
  buf[48]  = control_pram.r_dir;
  tmp_i16 = (int16_t)(control_pram.encode_spd * 100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[50]); 
  EncodeDWord2Bytes('l', control_pram.encode_spd_integ, (char *)&buf[52]);

  tmp_i16 = (int16_t)(control_pram.taget_yaw * 100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[54]);

  tmp_i16 = (int16_t)(control_pram.pitch * 100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[56]);
  tmp_i32 = (int32_t)(control_pram.pitch_gyro *1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[58]);
  tmp_i16 = (int16_t)(control_pram.yaw * 100);
  EncodeDWord2Bytes('l', tmp_i16, (char *)&buf[62]);
  tmp_i32 = (int32_t)(control_pram.yaw_gyro *1000);
  EncodeW2Bytes('l', tmp_i32, (char *)&buf[64]);

  tmp_u32 = control_pram.tick;
  EncodeDW2Bytes('l', tmp_u32, (char *)&buf[68]);

  buf[72] = control_pram.enable_cmd;
  buf[73] = control_pram.active;
  buf[74] = control_pram.fault;
  for(idx = 75;idx < (NET_MSG_TLEN-1);idx++){
    buf[idx] = 0;
  }
  buf[NET_MSG_TLEN-1] = checksum_8(&buf[2], NET_MSG_TLEN-2);
}

/**
  * @brief D֡���ݷ���
*/
static void netlink_TD_encode(uint8_t *buf)
{
  uint8_t idx = 0;
  uint16_t tmp_u16 = 0;
  uint32_t tmp_u32 = 0;
  buf[0] = 0xEB;
  buf[1] = 0x90;
  buf[2] = NCID_TD;
 // memcpy(control_cmd_msg,"hello word i am fcc",sizeof("hello word i am fcc"));
  memcpy(&buf[3],net_cmd_msg,NET_MSG_CMDLEN);

  tmp_u16 = (uint16_t)(Rcc_Get_Cpuusage()* 100);
  EncodeWord2Bytes('l', tmp_u16, (char *)&buf[23]);
  tmp_u32 = HAL_GetTick();
  EncodeDW2Bytes('l', tmp_u32, (char *)&buf[25]);
  for(idx = 29;idx < (NET_MSG_TLEN-1);idx++){
    buf[idx] = 0;
  }
  buf[NET_MSG_TLEN-1] = checksum_8(&buf[2], NET_MSG_TLEN-2);
  
}

/**
  * @brief ����A֡���ݽ���
*/
static void netlink_RA_decode(uint8_t *buf)
{

    // ֱ��KP KD  �ٶ� KP KI ת�� KP kI Ŀ���ٶ� Ŀ�꺽��
    struct  NET_RA net_RA;
    int32_t tmp_int32;
    int16_t tmp_int16;
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[3]);
    net_RA.balance_kp = tmp_int32/1000.0f;
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[7]);
    net_RA.balance_kd = tmp_int32/1000.0f;
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[11]);
    net_RA.vel_kp = tmp_int32/1000.0f;
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[15]);
    net_RA.vel_ki = tmp_int32/1000.0f;  
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[19]);
    net_RA.turn_kp = tmp_int32/1000.0f;
    tmp_int32 = ExtractWFromByte('l', (char *)&buf[23]);
    net_RA.turn_kd = tmp_int32/1000.0f;  
    net_RA.taget_spd = (short)ExtractWordFromBytes('l', (char *)&buf[27]);
    tmp_int16 = (short)ExtractWordFromBytes('l', (char *)&buf[29]);
    net_RA.taget_yaw = tmp_int16/100.0f;
    net_RA.ifsave = buf[31];
    memset(net_cmd_msg,0x00,NET_MSG_CMDLEN);
    if(Control_Set_PramA(&net_RA) == 0){
      if(net_RA.ifsave){
        memcpy(net_cmd_msg,"Control Save Success",sizeof("Control Save Success"));
      }
      else{
        memcpy(net_cmd_msg,"Control Set Success",sizeof("Control Set Success"));
      }
    }
    else{
      memcpy(net_cmd_msg,"Control Save Failed",sizeof("Control Save Failed"));
    }

}

/**
  * @brief ����A֡���ݽ���
*/
static void netlink_RB_decode(uint8_t *buf)
{

    //  Ŀ���ٶ� Ŀ�꺽�������
    struct  NET_RB net_RB;
    int16_t tmp_int16; 
    net_RB.taget_spd = (short)ExtractWordFromBytes('l', (char *)&buf[3]);
    tmp_int16 = (short)ExtractWordFromBytes('l', (char *)&buf[5]);
    net_RB.taget_ryaw = tmp_int16/100.0f;
    net_RB.enable = (buf[7] != 0U) ? 1U : 0U;
    Control_Set_PramB(&net_RB);
    memset(net_cmd_msg,0x00,NET_MSG_CMDLEN);
    memcpy(net_cmd_msg,"joystick control Success",sizeof("joystick control Success")); 
    
}

/**
 * @}
 */

/** @defgroup ����ͨ���ⲿ����
 * @{
 */
/**
 * @brief  ����ͨ�ų�ʼ�� 
 */
void Net_Link_Init(void)
{
  Periph_Port_Serial_Open(&net_uart,UART5_ID,460800,net_rxbuf,net_txbuf,NETBUF_SIZE, \
                          NETBUF_SIZE,net_frxbuf,net_ftxbuf,NETBUF_SIZE,NETBUF_SIZE);
}



/**
 * @brief  ����ͨ�Ž���
 * @note   10ms task 
 */
void Net_Link_Decode(void)
{
  uint8_t buf[NET_MSG_RLEN] = {0};
  if(netlink_try_to_catch_frame(buf) == MSG_INPUT_SUCCESS){
    switch (buf[2])
    {
    case NCID_RA:
      netlink_RA_decode(&buf[0]);
    break;
    case NCID_RB:
      netlink_RB_decode(&buf[0]);
    break; 
    default:
      break;
    }
  }
}

/**
 * @brief  ����ͨ�����
 * @note   50ms task 
 */
void Net_Link_Out(void)
{
   uint8_t buf[NET_MSG_TLEN] = {0};
   netlink_TB_encode(buf);
   net_link_tx(buf, NET_MSG_TLEN);
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
/*************************end of file******************************************/
