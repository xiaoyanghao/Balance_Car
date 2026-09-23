#include "txg_math.h"
#include <float.h>
#include <math.h>

/** Private variable definition ***************************************************************** */
static union
{
    char c[4];
    int l;
}endian_test = { { 'l', '?', '?', 'b' } };

#define ENDIANNESS ((char)endian_test.l)

// --------------------------------------------------------------
// CRC-16: x^16 + x^15 + x^2 + x^0                 (0x8005)
// --------------------------------------------------------------
const uint16_t wCRC_Table[16] =
{
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

/** Function declaration ************************************************************************ */
/**
 * Calculate CRC use CRC table
 * @param data     : new data to cal
 * @param crcAccum : CRC result
 */
inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
    uint16_t tmp;

    tmp = wCRC_Table[(data ^ *crcAccum) & 15] ^ (*crcAccum >> 4);
    *crcAccum = wCRC_Table[((data >> 4) ^ tmp) & 15] ^ (tmp >> 4);
}

/**
 * Calculate CRC of u8 buffer with specified length
 * @param  pBuffer : data buffer to CRC
 * @param  length  : length to calculate
 * @return         : CRC result
 */
inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{

    uint16_t crcTmp;
    crcTmp  = 0xFFFF;
    while (length--)
    {
        crc_accumulate(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}

/**
 * @brief Accumulate the X.25 CRC by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 */
inline void crc_accumulate_buffer(uint16_t *crcAccum, const uint8_t *pBuffer, uint8_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    while (length--)
    {
        crc_accumulate(*p++, crcAccum);
    }
}

/**
 * @brief 16bit check sum
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 */
uint16_t  checksum_16(const uint8_t *pBuffer, uint8_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    uint32_t  sum = 0;
    while (length--)
    {
        sum += *p++;
    }
    return (sum & 0xffff);
}

/** End of CRC method --------------------------------------------------------------------------- */

/**
 * Wrap angle in centi-dgress to [0, 36000]
 * @param  angle_cd : raw angle in centi-dgree
 * @return          : angle in [0, 36000]
 */
double wrap_360_cd_double(double angle_cd)
{
    /** for larger number use fmodulus */
    if (angle_cd >= 72000.0 || angle_cd < -36000.0)
    {
        angle_cd = fmod(angle_cd, 36000.0);
    }

    if (angle_cd >= 36000.0)
    {
        angle_cd -= 36000.0;
    }
    else if (angle_cd < 0.0)
    {
        angle_cd += 36000.0;
    }

    return angle_cd;
}

/**
 * Wrap angle in centi-dgress to [0, 360]
 * @param  angle_cd : raw angle in centi-dgree
 * @return          : angle in [0, 360]
 */
double wrap_360_cd_double2(double angle_cd)
{
    /** for larger number use fmodulus */
    if (angle_cd >= 720.0 || angle_cd < -360.0)
    {
        angle_cd = fmod(angle_cd, 360.0);
    }

    if (angle_cd >= 360.0)
    {
        angle_cd -= 360.0;
    }
    else if (angle_cd < 0.0)
    {
        angle_cd += 360.0;
    }

    return angle_cd;
}

/**
 * Wrap angle in centi-dgree to [-18000, 18000]
 * @param  angle_cd : raw angle in centi-dgree
 * @return          : angle cd in [-18000, 18000]
 */
double wrap_180_cd_double(double angle_cd)
{
    if (angle_cd > 54000.0 || angle_cd < -54000.0)
    {
        angle_cd = fmod(angle_cd,36000.0);	// for large numbers use modulus
    }

    if (angle_cd > 18000.0)
    {
        angle_cd -= 36000.0;
    }
    else if (angle_cd < -18000.0)
    {
        angle_cd += 36000.0;
    }

    return angle_cd;
}

/**
 * Wrap angle in centi-dgree to [-180, 180]
 * @param  angle_cd : raw angle in centi-dgree
 * @return          : angle cd in [-180, 180]
 */
double wrap_180_cd_double2(double angle_cd)
{
    if (angle_cd > 540.0 || angle_cd < -540.0)
    {
        angle_cd = fmod(angle_cd,360.0);	// for large numbers use modulus
    }

    if (angle_cd > 180.0)
    {
        angle_cd -= 360.0;
    }
    else if (angle_cd < -180.0)
    {
        angle_cd += 360.0;
    }

    return angle_cd;
}


/**
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 * @param  angle_rad : raw angle in rad
 * @return           : angle in [-PI, PI]
 */
double wrap_PI(double angle_rad)
{
    if (angle_rad > 10*M_PI || angle_rad < -10*M_PI)
    {
        angle_rad = fmod(angle_rad, 2*M_PI);	// for very large numbers use modulus
    }

    while (angle_rad > M_PI)
    {
        angle_rad -= 2*M_PI;
    }
    while (angle_rad < -M_PI)
    {
        angle_rad += 2*M_PI;
    }

    return angle_rad;
}

/**
 * Return square of var.
 * @param  x : raw var
 * @return   : square of var double.
 */
inline double isq(double x)
{
    return (x*x);
}

/**
 * Return sqrt of var with safe check.
 * @param  x : raw variable
 * @return   : sqrt of var.
 */
double safe_sqrt(double x)
{
    return (x<0.0 ? 0.0 : sqrt(x));
}

/**
 * Check if two double var is equal.
 * @param  var1 : double var 1
 * @param  var2 : double var 2
 * @return      : true if equal.
 */
bool is_equal_D(double var1, double var2)
{
    return (fabs(var1-var2) < FLT_EPSILON ? true : false);
}

/**
 * Check if a double var is ZERO.
 * @param  var : double var.
 * @return     : true if equal to zero.
 */
bool is_zero_D(double var)
{
    return (fabs(var) < FLT_EPSILON ? true : false);
}

/**
 * constrain a double value between low and high.
 * @param  amt  : var to limit.
 * @param  low  : low limit.
 * @param  high : high limit.
 * @return      : var within limit.
 */
double constrain_double(double amt, double low, double high)
{
    // the check for NaN as a double prevents propogation of
    // doubleing point errors through any function that uses
    // constrain_double(). The normal double semantics already handle -Inf
    // and +Inf
    if (isnan(amt))
    {
        return (low+high)*0.5;
    }

    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * constrain a int16_t value between low and high.
 * @param  amt  : var to limit.
 * @param  low  : low limit.
 * @param  high : high limit.
 * @return      : var within limit.
 */
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * constrain a int32_t value between low and high.
 * @param  amt  : var to limit.
 * @param  low  : low limit.
 * @param  high : high limit.
 * @return      : var within limit.
 */
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * Convert deg to rad
 * @param  angle_deg : angle in degree
 * @return           : angle in rad.
 */
double radians(double angle_deg)
{
    return angle_deg*DEG_TO_RAD;
}

/**
 * Check if double variable larger than ZERO.
 * @param  var : var double.
 * @return     : true if larger than zero.
 */
double sign_d(double var)
{
    return (var>0 ? 1.0 : -1.0);
}

/**
 * Calculate first order LPF param with input of F_cut and Sample dt.
 * @param  hz : cutoff frequence.
 * @param  dt : delta time between two sample steps.
 * @return    : param of LPF.
 */
double get_lpf_filer_alpha(uint8_t hz, double dt)
{
    double temp = 1.0/(2.0*M_PI*hz);

    return (dt/(dt + temp));
}

/**
 * response based on the sqrt of the error instead of the more common linear response
 * @param  error          [current angle error]
 * @param  p              [smoothing gain which refer to response speed of stick]
 * @param  second_ord_lim [gyro rate accel limit]
 * @return                [smoothed desired rate based on angle error]
 */
double txg_sqrt_controller(double error, double p, double second_ord_lim)
{
    if (fabs(second_ord_lim) < 0.0001 || fabs(p) <= 0.0001)
    {
        return error*p;
    }

    double linear_dist = second_ord_lim/(p * p);

    if (error > linear_dist)
    {
        return safe_sqrt(2.0*second_ord_lim*(error-(linear_dist/2.0)));
    }
    else if (error < -linear_dist)
    {
        return -safe_sqrt(2.0*second_ord_lim*(-error-(linear_dist/2.0)));
    }
    else
    {
        return error*p;
    }
}

/**
 * Calculate norm of two var.
 * @param  var1 : var 1
 * @param  var2 : var 2
 * @return      : norm length.
 */
double norm_double2(double var1, double var2)
{
    return (safe_sqrt(isq(var1) + isq(var2)));
}

/** Just a specified function, DONOT change anything of this */
double fsg(double x, double d)
{
    return ((sign_d(x+d) - sign_d(x-d))*0.5);
}

/** Just a specified function, DONOT change anything of this */
double fhan(double x1, double x2, double r, double h0)
{
    double d   = r * isq(h0);    // d>0
    double a0  = h0 * x2;
    double y   = x1 + a0;
    double a1  = sqrt(d * (d + 8.0 * fabs(y)));
    double a2  = a0 + sign_d(y) * (a1 - d) * 0.5;
    double a   = (a0 + y) * fsg(y, d) + a2 * (1.0 - fsg(y,d));

    double ret = -r * (a / d) * fsg(a, d) - r * sign_d(a) * (1.0 - fsg(a,d));

    return ret;
}

/**
 * Longitude to meters scale based on lattitude.
 * @param  lat : lattitude.
 * @return     : scale to meters.
 */
double longitude_scale(int32_t lat)
{
    static int32_t last_lat;
    static double scale = 1.0;

    if (sabs(last_lat - lat) < 100000)
    {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }

    scale	 = cosf((sabs(lat)/10000000.0) * DEG_TO_RAD);
    last_lat = lat;

    return scale;
}

/**
 * Insert sort.
 * @param  array
 * @param  n
 */
void insert_sort(int16_t *array, uint16_t n)
{
    for (uint16_t i=1; i<n; i++)
    {
        int16_t temp = *(array+i);
        uint16_t j;

        for (j=i; (j>0) && (*(array+j-1)>temp); j--)
        {
            *(array+j) = *(array+j-1);
        }

        *(array+j) = temp;
    }
}

/**
 * Hysteresis characteristics
 * param @al, @ah: two thresholds of hysteresis
 * param @curr_pos:
 * param @curr_stt: which direction should the current data fall into, 0-low, 1-high
 * return:new_stt
 */
bool hysteresis(bool *curr_stt, double al, double ah, double val)
{
    #define LOW_POSITION  false
    #define HIGH_POSITION true

    bool new_stt = *curr_stt;
    if (*curr_stt)
    {
        if (val < al)
        {
            new_stt = LOW_POSITION;
        }
    }
    else
    {
        if (val > ah)
        {
            new_stt = HIGH_POSITION;
        }
    }
    return new_stt;
}

/**
* @brief  crc mean
* @retval None
*/
float Float_Sum_Mean(float ArrayFloat[],uint8_t tmpArrayLen)
{
    float sum = 0.0;
    uint8_t i = 0;
    float ret = 0;
    
    for(i = 0;i < tmpArrayLen; i++)
    sum += ArrayFloat[i];   

    ret = sum*1.0/tmpArrayLen;
    
    return ret;
}

/**
 * @brief 8bit check sum
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 */
uint8_t checksum_8(const uint8_t *pBuffer, uint8_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    uint64_t  sum = 0;
    while (length--){
        sum += *p++;
    }
    return (sum & 0x00ff);
}

/**
 * constrain a uint8_t value between low and high.
 * @param  amt  : var to limit.
 * @param  low  : low limit.
 * @param  high : high limit.
 * @return      : var within limit.
 */
uint8_t constrain_uint8(uint8_t amt, uint8_t low, uint8_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**
 * @brief extract unsigned short data form buffer
 * 
 * @param buf start address of buf pointer
 * @param endianness the data store endian type 'l' or 'b', default as 'l'
 * @return unsigned short 
 */
unsigned short ExtractWordFromBytes(unsigned char endianness, const char * buf)
{
	union { unsigned char B[2]; unsigned short W;} src = {0};

	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//默认待处理数据为小端格式数据
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

/**
 * @brief encode unsigned short data to buffer
 * 
 * @param word input data to encode
  * @param endianness the data store endian type 'l' or 'b', default as 'l'
 * @param buf_start start address of buffer
 */
void EncodeWord2Bytes(unsigned char endianness, unsigned short word,char * buf_start)
{
	union { unsigned char B[2]; unsigned short W;} src = {0};

	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//默认发送数据为小端格式数据
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

/* @brief encode  short data to buffer
 * 
 * @param word input data to encode
  * @param endianness the data store endian type 'l' or 'b', default as 'l'
 * @param buf_start start address of buffer
 */
void EncodeDWord2Bytes(unsigned char endianness, short word,char * buf_start)
{
	union { unsigned char B[2]; short W;} src = {0};

	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//默认发送数据为小端格式数据
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
void EncodeDW2Bytes(unsigned char endianness, unsigned int dword,char * buf_start)
{
	union { unsigned char B[4]; unsigned short W[2];unsigned int DW;} src = {0};
	
	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//默认发送数据为小端格式数据
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
 * @brief encode unsigned int data to buffer
 * 
 * @param dword data to encode
 * @param buf_start start address of buffer
 * @param endianness the data store endian type 'l' or 'b', default as 'l'
 */
void EncodeW2Bytes(unsigned char endianness,  int dword,char * buf_start)
{
	union { unsigned char B[4]; unsigned short W[2]; int DW;} src = {0};
	
	if(endianness != 'l' && endianness != 'b')
	{
		endianness = 'l';//默认发送数据为小端格式数据
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
int ExtractWFromByte(unsigned char endianness, const char *buf)
{
    union { unsigned char B[4]; unsigned short W[2]; int DW;} src;

    if(endianness != 'l' && endianness != 'b')
    {
        endianness = 'l';
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

void SmoothLinear(float total, float *var, float unit)
{
    float err = 0;
    err = total - *var;

    if(err >= unit)
    {
        *var = *var + unit;
        if(*var>=total) 
        {
        *var=total;
        
        }
    }
    else if(err <= -unit)
    {
        *var = *var - unit;
        if(*var<=total) *var=total;
    }
    else
    {
        *var = total;
    }
}
/** END OF FILE ********************************************************************************* */
