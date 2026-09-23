/********************************************************************************
** Form generated from reading UI file 'ui_parm.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef CONSOLE_PROTOCOL_H
#define CONSOLE_PROTOCOL_H

#include <QByteArray>
#include <QString>

#define CONTROL_MSG_CMDLEN     (20)
#define CONSOLE_MAX_TAGSPD     (10)
#define CONSOLE_MAX_TAGRYAW    (10)

/**
 * @brief  协议识别码
 */
typedef enum  {

    PROTOCOL_RA = 0x01,    /**<A帧数据接收 传感器数据*/
    PROTOCOL_RB,           /**<B帧数据接收 控制类参数*/
    PROTOCOL_RC,           /**<C帧数据接收 遥控数据参数*/
    PROTOCOL_RD,           /**<D帧数据接收 命令反馈参数*/

    PROTOCOL_TA = 0x11,    /**<A帧数据发送*/
    PROTOCOL_TB = 0x12,    /**<B帧数据发送*/

}PROTOCOL_IDCODE;


/**
 * @brief A帧接收
 */
struct PROTOCOL_RA
{

  unsigned char imu_health_status;   /**<IMU 状态 1-好 0-坏 */
  float imu_tmp;                     /**<IMU温度  单位℃ 分辨率100 */
  float imu_gyro_rawest[3];          /**<IMU角速率原始数据 单位deg/s 分辨率1000 */
  float imu_gyro_raw[3];             /**<IMU角速率原始数据求平均值 单位deg/s 分辨率1000 */
  float imu_gyro[3];                 /**<IMU角速率低通滤波 单位deg/s 分辨率1000 */
  float imu_acc_rawest[3];           /**<IMU加速度原始数据 单位m/^2 分辨率1000 */
  float imu_acc_raw[3];              /**<IMU加速度原始数据求平均值 单位m/^2 分辨率1000 */
  float imu_acc[3];                  /**<IMU加速度低通滤波 单位m/^2 分辨率1000 */
  float imu_roll;                    /**<IMU滚转角 单位deg 分辨率100 */
  float imu_pitch;                   /**<IMU俯仰角 单位deg 分辨率100 */
  float imu_yaw;                     /**<IMU航向角 单位deg 分辨率100 */
  unsigned int tick;                 /**<发送时间  单位ms*/
};

/**
 * @brief B帧接收
 */
struct PROTOCOL_RB
{
  float balance_kp;           /**<直立环比例P 分辨率1000*/
  float balance_kd;           /**<直立环微分D 分辨率1000*/
  float vel_kp;               /**<速度比例P   分辨率1000*/
  float vel_ki;               /**<速度积分I   分辨率1000*/ 
  float turn_kp;              /**<转向比例P   分辨率1000*/
  float turn_kd;              /**<转向微分D   分辨率1000*/

  int16_t balance_pwm;         /**<直立环输出PWM*/
  int16_t vel_pwm;             /**<速度环输出PWM*/
  int16_t lturn_pwm;           /**<左轮转向环输出PWM*/
  int16_t lfinal_pwm;          /**<左轮最终输出PWM*/
  int16_t rturn_pwm;           /**<右轮转向环输出PWM*/
  int16_t rfinal_pwm;          /**<右轮最终输出PWM*/
  uint8_t l_dircmd;            /**<左轮方向命令*/       
  uint8_t r_dircmd;            /**<右轮方向命令*/ 

  int16_t taget_spd;           /**<目标速度*/ 
  int16_t lencode_spd;         /**<左轮编码器速度*/ 
  int16_t rencode_spd;         /**<右轮编码器速度*/ 
  uint8_t l_dir;               /**<左轮方向*/ 
  uint8_t r_dir;               /**<右轮方向*/
  float   encode_spd;          /**<编码器速度 分辨率100*/ 
  int16_t encode_spd_integ;    /**<编码器速度积分*/

  float   taget_yaw;           /**<目标偏航角*/

  float   pitch;               /**<俯仰角 分辨率100*/
  float   pitch_gyro;          /**<俯仰角速度 分辨率1000*/
  float   yaw;                 /**<偏航角 分辨率100*/
  float   yaw_gyro;            /**<偏航角速度 分辨率1000*/
     
  unsigned int tick;           /**<命令更新时间 ms*/  

};

/**
 * @brief D帧接收
 */
struct PROTOCOL_RD
{
  unsigned char msg_cmd[CONTROL_MSG_CMDLEN];           /**<命令反馈*/
  float cpu_usage;                            /**<CPU利用率*/
  unsigned int tick;                          /**<发送时间 时间 ms*/

};

/**
 * @brief 发送A帧数据
 */
struct  PROTOCO_TA
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
 * @brief 发送A帧数据
 */
struct  PROTOCO_TB
{

  int16_t taget_spd;           /**<目标速度*/
  float   taget_ryaw;          /**<目标偏航角速率*/

};

/**
* @}
*/

class CONSOLE_PROTOCAL
{
 public:
    struct PROTOCOL_RA        protocol_rA;
    struct PROTOCOL_RB        protocol_rB;
    struct PROTOCOL_RD        protocol_rD;
    struct PROTOCO_TA         protocol_tA;
    struct PROTOCO_TB         protocol_tB;
    int  data_freq;
    int  data_freq_count;
    QByteArray readbuf;
    QByteArray writebuf;

    unsigned char checksum_8(const unsigned char *pBuffer, unsigned char length);
    unsigned short ExtractWordFromBytes(unsigned char endianness, const char * buf);
    void EncodeDWord2Bytes(unsigned char endianness, short word,char * buf_start);
    unsigned int ExtractDWFromByte(unsigned char endianness, const char *buf);
    int  ExtractWFromByte(unsigned char endianness, const char *buf);
    void EncodeW2Bytes(unsigned char endianness,  int dword,char * buf_start);

    void protocol_decode(void);
    void protocol_RA_decode(void);
    void protocol_RB_decode(void);
    void protocol_RD_decode(void);
    
    void protocol_TA_encode(void);
    void protocol_TB_encode(void);

};

#endif // UI_UI_PARM_H
