#include "console_protocol.h"
#include <QDebug>

#define CONSOLE_TX_LEN         (45)
#define CONSOLE_RX_LEN         (96)


static union
{
    char c[4];
    int l;
}endian_test = { { 'l', '?', '?', 'b' } };

#define ENDIANNESS ((char)endian_test.l)

void CONSOLE_PROTOCAL:: protocol_decode(void)
{

    uint8_t crc = 0x00;
    while(readbuf.length() >= CONSOLE_RX_LEN)
    {
      if((uint8_t)readbuf.at(0) == (uint8_t)0xeb && (uint8_t)readbuf.at(1) == (uint8_t)0x90)
      {
          crc = checksum_8((uint8_t*)readbuf.data()+2,CONSOLE_RX_LEN-3);
          if(crc == (uint8_t)readbuf.at(CONSOLE_RX_LEN-1))
          {

              switch ((uint8_t)readbuf.at(2))
              {
                  case PROTOCOL_IDCODE::PROTOCOL_RA:      protocol_RA_decode(); data_freq_count++;  break;
                  case PROTOCOL_IDCODE::PROTOCOL_RB:      protocol_RB_decode(); data_freq_count++;  break;
                  case PROTOCOL_IDCODE::PROTOCOL_RD:      protocol_RD_decode(); data_freq_count++;  break;
                  default:                             break;

              }
          }
          readbuf.remove(0,CONSOLE_RX_LEN);
      }
      else{
        readbuf.remove(0,1);
      }
    }

}

void CONSOLE_PROTOCAL:: protocol_RA_decode(void)
{

    short tmp_short;
    unsigned int tmp_uint;
    int tmp_int;

    protocol_rA.imu_health_status =  *(readbuf.data()+3);
    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+4);
    protocol_rA.imu_tmp = tmp_short/100.0f;

    tmp_int = ExtractWFromByte('l',readbuf.data()+6);
    protocol_rA.imu_gyro_rawest[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+10);
    protocol_rA.imu_gyro_rawest[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+14);
    protocol_rA.imu_gyro_rawest[2] = tmp_int/1000.0f;


    tmp_int = ExtractWFromByte('l',readbuf.data()+18);
    protocol_rA.imu_gyro_raw[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+22);
    protocol_rA.imu_gyro_raw[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+26);
    protocol_rA.imu_gyro_raw[2] = tmp_int/1000.0f;

    tmp_int = ExtractWFromByte('l',readbuf.data()+30);
    protocol_rA.imu_gyro[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+34);
    protocol_rA.imu_gyro[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+38);
    protocol_rA.imu_gyro[2] = tmp_int/1000.0f;

    tmp_int = ExtractWFromByte('l',readbuf.data()+42);
    protocol_rA.imu_acc_rawest[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+46);
    protocol_rA.imu_acc_rawest[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+50);
    protocol_rA.imu_acc_rawest[2] = tmp_int/1000.0f;


    tmp_int = ExtractWFromByte('l',readbuf.data()+54);
    protocol_rA.imu_acc_raw[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+58);
    protocol_rA.imu_acc_raw[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+62);
    protocol_rA.imu_acc_raw[2] = tmp_int/1000.0f;

    tmp_int = ExtractWFromByte('l',readbuf.data()+66);
    protocol_rA.imu_acc[0] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+70);
    protocol_rA.imu_acc[1] = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+74);
    protocol_rA.imu_acc[2] = tmp_int/1000.0f;

    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+78);
    protocol_rA.imu_roll = tmp_short/100.0f;
    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+80);
    protocol_rA.imu_pitch = tmp_short/100.0f;
    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+82);
    protocol_rA.imu_yaw = tmp_short/100.0f;

    tmp_uint = ExtractDWFromByte('l',readbuf.data()+84);
    protocol_rA.tick = tmp_uint;

}


void CONSOLE_PROTOCAL:: protocol_RB_decode(void)
{

    short tmp_short;
    int tmp_int;
    unsigned int tmp_uint;

    tmp_int = ExtractWFromByte('l',readbuf.data()+3);
    protocol_rB.balance_kp = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+7);
    protocol_rB.balance_kd = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+11);
    protocol_rB.vel_kp = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+15);
    protocol_rB.vel_ki = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+19);  
    protocol_rB.turn_kp = tmp_int/1000.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+23);
    protocol_rB.turn_kd = tmp_int/1000.0f;

    protocol_rB.balance_pwm  =   (short)ExtractWordFromBytes('l',readbuf.data()+27);
    protocol_rB.vel_pwm      =   (short)ExtractWordFromBytes('l',readbuf.data()+29);
    protocol_rB.lturn_pwm    =   (short)ExtractWordFromBytes('l',readbuf.data()+31);
    protocol_rB.lfinal_pwm   =   (short)ExtractWordFromBytes('l',readbuf.data()+33);
    protocol_rB.rturn_pwm    =   (short)ExtractWordFromBytes('l',readbuf.data()+35);
    protocol_rB.rfinal_pwm   =   (short)ExtractWordFromBytes('l',readbuf.data()+37);
    protocol_rB.l_dircmd = *(readbuf.data()+39);
    protocol_rB.r_dircmd = *(readbuf.data()+40);

    protocol_rB.taget_spd     =   (short)ExtractWordFromBytes('l',readbuf.data()+41);
    protocol_rB.lencode_spd   =   (short)ExtractWordFromBytes('l',readbuf.data()+43);
    protocol_rB.rencode_spd   =   (short)ExtractWordFromBytes('l',readbuf.data()+45);
    protocol_rB.l_dir         =   *(readbuf.data()+47);
    protocol_rB.r_dir         =   *(readbuf.data()+48);
    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+50);
    protocol_rB.encode_spd = tmp_short/100.0f;
    protocol_rB.encode_spd_integ   =  (short)ExtractWordFromBytes('l',readbuf.data()+52);  

    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+54);
    protocol_rB.taget_yaw = tmp_short/100.0f;

    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+56);
    protocol_rB.pitch = tmp_short/100.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+58);
    protocol_rB.pitch_gyro = tmp_int/1000.0f;
    tmp_short =   (short)ExtractWordFromBytes('l',readbuf.data()+62);
    protocol_rB.yaw = tmp_short/100.0f;
    tmp_int = ExtractWFromByte('l',readbuf.data()+64);
    protocol_rB.yaw_gyro = tmp_int/1000.0f;

    tmp_uint = ExtractDWFromByte('l',readbuf.data()+68);
    protocol_rB.tick= tmp_uint;


}

void CONSOLE_PROTOCAL:: protocol_RD_decode(void)
{
   unsigned char index = 0;
   unsigned short tmp_ushort;
   unsigned int tmp_uint;
   memset(&protocol_rD.msg_cmd[0],0x00,CONTROL_MSG_CMDLEN);
   for(index = 0; index < CONTROL_MSG_CMDLEN; index++){
       protocol_rD.msg_cmd[index] = *(readbuf.data()+3+index);
   }
    tmp_ushort = ExtractWordFromBytes('l',readbuf.data()+23);
    protocol_rD.cpu_usage = tmp_ushort/100.0f;
    tmp_uint = ExtractDWFromByte('l',readbuf.data()+25);
    protocol_rD.tick= tmp_uint;     
}


void CONSOLE_PROTOCAL::protocol_TA_encode(void)
{

    unsigned char crc;
    int tmp_int;
    short tmp_short;
    char buf[4] = {0};
    writebuf.clear();
    writebuf.append((char)0xeb);
    writebuf.append((char)0x90);
    writebuf.append((char)PROTOCOL_IDCODE::PROTOCOL_TA);
    tmp_int = (int)(protocol_tA.balance_kp * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);
    tmp_int = (int)(protocol_tA.balance_kd * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);

    tmp_int = (int)(protocol_tA.vel_kp * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);
    tmp_int = (int)(protocol_tA.vel_ki * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);

    tmp_int = (int)(protocol_tA.turn_kp * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);
    tmp_int = (int)(protocol_tA.turn_kd * 1000);
    EncodeW2Bytes('l', tmp_int, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)buf[2]);
    writebuf.append((char)buf[3]);

    EncodeDWord2Bytes('l', protocol_tA.taget_spd, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    tmp_short = protocol_tA.taget_yaw * 100;
    EncodeDWord2Bytes('l', tmp_short, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.append((char)protocol_tA.ifsave);

    writebuf.resize(CONSOLE_TX_LEN-1);
   // for(idx = 32; idx < CONSOLE_TX_LEN-1;idx++){
  //      writebuf.append((char)0x00);
  //  }
    crc = checksum_8((unsigned char *)(writebuf.data()+2),CONSOLE_TX_LEN-3);
    writebuf.append((char)crc);

}


void CONSOLE_PROTOCAL::protocol_TB_encode(void)
{

    unsigned char crc;
    short tmp_short;
    char buf[4] = {0};
    writebuf.clear();
    writebuf.append((char)0xeb);
    writebuf.append((char)0x90);
    writebuf.append((char)PROTOCOL_IDCODE::PROTOCOL_TB);

    EncodeDWord2Bytes('l', protocol_tB.taget_spd, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    tmp_short = protocol_tB.taget_ryaw * 100;
    EncodeDWord2Bytes('l', tmp_short, &buf[0]);
    writebuf.append((char)buf[0]);
    writebuf.append((char)buf[1]);
    writebuf.resize(CONSOLE_TX_LEN-1);

    crc = checksum_8((unsigned char *)(writebuf.data()+2),CONSOLE_TX_LEN-3);
    writebuf.append((char)crc);

}

/**
 * @brief 8bit check sum
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 */
unsigned char  CONSOLE_PROTOCAL:: checksum_8(const unsigned char *pBuffer, unsigned char length)
{
    const unsigned char *p = (const unsigned char *)pBuffer;
    unsigned long  sum = 0;
    while (length--){
        sum += *p++;
    }
    return (sum & 0x00ff);
}

/**
 * @brief extract unsigned short data form buffer
 *
 * @param buf start address of buf pointer
 * @param endianness the data store endian type 'l' or 'b', default as 'l'
 * @return unsigned short
 */
unsigned short CONSOLE_PROTOCAL:: ExtractWordFromBytes(unsigned char endianness, const char * buf)
{
    union { unsigned char B[2]; unsigned short W;} src ;

    if(endianness != 'l' && endianness != 'b')
    {
        endianness = 'l';//é»˜è?¤å¾…å¤„ç†æ•°æ®ä¸ºå°ç«?æ ¼å¼æ•°æ®
    }

    if(ENDIANNESS == endianness)
    {
        src.B[0] = buf[0];
        src.B[1] = buf[1];
    }
    else
    {
        src.B[0] = buf[1];
        src.B[1] = buf[0];
    }
    return src.W;
}

/* @brief encode  short data to buffer
 *
 * @param word input data to encode
  * @param endianness the data store endian type 'l' or 'b', default as 'l'
 * @param buf_start start address of buffer
 */
void CONSOLE_PROTOCAL:: EncodeDWord2Bytes(unsigned char endianness, short word,char * buf_start)
{
    union { unsigned char B[2]; short W;} src;

    if(endianness != 'l' && endianness != 'b')
    {
        endianness = 'l';//é»˜è?¤å‘é€æ•°æ?ä¸ºå°ç«?æ ¼å¼æ•°æ®
    }

    src.W = word;
    if(ENDIANNESS == endianness)
    {
        buf_start[0] = src.B[0];
        buf_start[1] = src.B[1];
    }
    else
    {
        buf_start[0] = src.B[1];
        buf_start[1] = src.B[0];
    }
}
/**
 * @brief encode unsigned int data to buffer
 * 
 * @param dword data to encode
 * @param buf_start start address of buffer
 * @param endianness the data store endian type 'l' or 'b', default as 'l'
 */
void CONSOLE_PROTOCAL::EncodeW2Bytes(unsigned char endianness,  int dword,char * buf_start)
{
	union { unsigned char B[4]; unsigned short W[2]; int DW;} src = {0};
	
	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//Ä¬ÈÏ·¢ËÍÊý¾ÝÎªÐ¡¶Ë¸ñÊ½Êý¾Ý
	}

	src.DW = dword;
	if(ENDIANNESS == endianness)
	{
		buf_start[0] = src.B[0];
		buf_start[1] = src.B[1];
		buf_start[2] = src.B[2];
		buf_start[3] = src.B[3];
	}
	else if(ENDIANNESS == 'b')
	{
		buf_start[0] = src.B[3];
		buf_start[1] = src.B[2];
		buf_start[2] = src.B[1];
		buf_start[3] = src.B[0];
	}
}
/**
 * @brief extract int32 data from buffer
 *
 * @param buf start address of buffer
 * @return uint32
 */
unsigned int CONSOLE_PROTOCAL:: ExtractDWFromByte(unsigned char endianness, const char *buf)
{
    union { unsigned char B[4]; unsigned short W[2];unsigned int DW;} src;

    if(endianness != 'l' && endianness != 'b')
    {
        endianness = 'l';//é»˜è?¤å¾…å¤„ç†æ•°æ®ä¸ºå°ç«?æ ¼å¼æ•°æ®
    }

    if(ENDIANNESS == endianness)
    {
        src.B[0] = buf[0];
        src.B[1] = buf[1];
        src.B[2] = buf[2];
        src.B[3] = buf[3];
    }
    else
    {
        src.B[0] = buf[3];
        src.B[1] = buf[2];
        src.B[2] = buf[1];
        src.B[3] = buf[0];
    }
    return src.DW;
}

/**
 * @brief extract int32 data from buffer
 *
 * @param buf start address of buffer
 * @return uint32
 */
int CONSOLE_PROTOCAL:: ExtractWFromByte(unsigned char endianness, const char *buf)
{
    union { unsigned char B[4]; unsigned short W[2]; int DW;} src;

    if(endianness != 'l' && endianness != 'b')
    {
        endianness = 'l';//é»˜è?¤å¾…å¤„ç†æ•°æ®ä¸ºå°ç«?æ ¼å¼æ•°æ®
    }

    if(ENDIANNESS == endianness)
    {
        src.B[0] = buf[0];
        src.B[1] = buf[1];
        src.B[2] = buf[2];
        src.B[3] = buf[3];
    }
    else
    {
        src.B[0] = buf[3];
        src.B[1] = buf[2];
        src.B[2] = buf[1];
        src.B[3] = buf[0];
    }
    return src.DW;
}
