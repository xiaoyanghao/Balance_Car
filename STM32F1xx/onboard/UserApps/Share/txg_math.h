#ifndef __TXG_MATH_H
#define __TXG_MATH_H

#include <stdint.h>
#include <stdbool.h>

/* const variables definition */
#define M_PI				3.14159265358979f

#define RAD_TO_DEG			57.2957795130823208768f
#define DEG_TO_RAD			0.01745329252
#define RAD_TO_DEGX100		        5729.57795130823208768
#define DEGX100_TO_RAD		        0.0001745329252

#define GRAVITY				9.80665     // m/s^2
#define G_Ref                           9.7939f

#define mps2Kmph         3.6f
//#define EPS              10000.0f

/** scaling factor from 1e-7 degrees to cent-meters at equator */
/** == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH */
#define REDUCE_SCALE_E_6         0.0000001f              //uint32_t -> double
#define REDUCE_SCALE_E_2         0.01f              //uint32_t -> double

#define LATLON_TO_CM 	        1.113195431531511       //radius of earth is 6378140m
#define LATLON_TO_CM_INVERSE 	0.8983148615910334      //inverse of LATLON_TO_CM

/** Channel index */
#define CH_1                        0
#define CH_2                        1
#define CH_3                        3       // switch CH3 and CH4, means change CH3 to yaw channel
#define CH_4                        2
#define CH_5                        4
#define CH_6                        5
#define CH_7                        6
#define CH_8                        7
#define CH_9                        8
#define CH_10                       9
#define CH_11                       10
#define CH_12                       11
#define CH_13                       12
#define CH_14                       13
#define CH_15                       14
#define CH_16                       15
#define CH_NUM                      16

/** Axis name in body frame. */
#define ROLL                        CH_1
#define PITCH                       CH_2
#define YAW                         CH_4    // NOTE: Channel index has been changed
#define THROTTLE                    CH_3

#define constrain(amt,low,high)  ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))
#define tmax(high,low)            ((high)>(low)?(high):(low))
#define tmin(low,high)            ((low)<(high)?(low):(high))
#define sabs(a)                  ((a) < 0 ? -(a) : (a))


/** POS LLH struct */
struct pos_llh_t
{
    int32_t lat;
    int32_t lng;
    int32_t alt;
};

/** Function type declaration ******************************************************************* */
void crc_accumulate(uint8_t data, uint16_t *crcAccum);
uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
void crc_accumulate_buffer(uint16_t *crcAccum, const uint8_t *pBuffer, uint8_t length);
uint16_t  checksum_16(const uint8_t *pBuffer, uint8_t length);

double wrap_360_cd_double(double angle_cd);
double wrap_360_cd_double2(double angle_cd);
double wrap_180_cd_double(double angle_cd);
double wrap_180_cd_double2(double angle_cd);
double wrap_PI(double angle_rad);

double isq(double x);
double safe_sqrt(double x);

bool is_equal_D(double var1, double var2);
bool is_zero_D(double var);

double constrain_double(double amt, double low, double high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);

double radians(double angle_deg);

double sign_d(double var);

double get_lpf_filer_alpha(uint8_t hz, double dt);

double txg_sqrt_controller(double error, double p, double second_ord_lim);

double norm_double2(double var1, double var2);

double fsg(double x, double d);
double fhan(double x1, double x2, double r, double h0);

double longitude_scale(int32_t lat);

void insert_sort(int16_t *array, uint16_t n);
bool hysteresis(bool *curr_stt, double al, double ah, double val);

float Float_Sum_Mean(float ArrayFloat[],uint8_t tmpArrayLen);  
uint8_t checksum_8(const uint8_t *pBuffer, uint8_t length);
uint8_t constrain_uint8(uint8_t amt, uint8_t low, uint8_t high);
unsigned short ExtractWordFromBytes(unsigned char endianness, const char * buf);
void EncodeWord2Bytes(unsigned char endianness, unsigned short word,char * buf_start);
void EncodeDWord2Bytes(unsigned char endianness, short word,char * buf_start);
void EncodeDW2Bytes(unsigned char endianness, unsigned int dword,char * buf_start);
void EncodeW2Bytes(unsigned char endianness,  int dword,char * buf_start);
int ExtractWFromByte(unsigned char endianness, const char *buf);

void SmoothLinear(float total, float *var, float unit);
#endif

/** END OF FILE ********************************************************************************* */
