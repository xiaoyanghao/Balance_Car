#include "periph_mpu60x0.h"
#include "bsp_i2c_drv.h"

/* MPU60x0 sample state. */
typedef struct
{
    /** DMA method vars */

    uint8_t rxBuf[MPU60x0_BYTES];
    uint8_t sample_ready;
    float  rawTemp;
    double rawAcc[3];
    double rawGyo[3];
    float  temp;

    uint32_t lastUpdate;
    float  accSign[3];
    float  gyoSign[3];
    uint8_t readReg;
    uint8_t enabled;

} mpu60x0Struct_t;


/* Private macro -------------------------------------------------------------*/
#define MPU60x0_SCL_PORT                 GPIOB  
#define MPU60x0_SDA_PORT                 GPIOB  
#define MPU60x0_SCL_PIN                  GPIO_PIN_13
#define MPU60x0_SDA_PIN                  GPIO_PIN_12

/** Private variables *************************************************************************** */
static struct bsp_ioi2c_t mpu60x0_ioi2c_t = {MPU60x0_SCL_PORT,MPU60x0_SDA_PORT,MPU60x0_SCL_PIN,MPU60x0_SDA_PIN};

mpu60x0Struct_t _mpu60x0Data;
uint8_t _bad_mpu60x0 = 0;

static double _mpu60x0_accx_raw = 0.0;
static double _mpu60x0_accy_raw = 0.0;
static double _mpu60x0_accz_raw = 0.0;

static double _mpu60x0_gyrox_raw = 0.0;
static double _mpu60x0_gyroy_raw = 0.0;
static double _mpu60x0_gyroz_raw = 0.0;

/** Function type definations ******************************************************************* */

static void mpu60x0SetReg(uint8_t reg, uint8_t val);
static uint8_t mpu60x0GetReg(uint8_t reg);
static void mpu60x0ReliablySetReg(uint8_t reg, uint8_t val);
static int8_t mpu60x0_config(void);
static void mpu60x0ScaleAcc(double *in, double *out, double divisor);
static void mpu60x0ScaleGyo(double *in, double *out, double divisor);
static void mpu60x0Decode(void);

/** Function declaration ************************************************************************ */
static void mpu60x0SetReg(uint8_t reg, uint8_t val)
{  
    Bsp_I2c_Write_Buffer(&mpu60x0_ioi2c_t,MPU60x0_ADDR,reg,1,&val);
}

static uint8_t mpu60x0GetReg(uint8_t reg)
{
    uint8_t val = 0;
    Bsp_I2c_Read_Buffer(&mpu60x0_ioi2c_t,MPU60x0_ADDR,reg,1,&val);
    return val;
}

static void mpu60x0ReliablySetReg(uint8_t reg, uint8_t val)
{
    int8_t dec = 20;

    do
    {
        HAL_Delay(10);
        mpu60x0SetReg(reg, val);
        HAL_Delay(10);

        dec--;
    }
    while (mpu60x0GetReg(reg) != val && (dec > 0));

    /// if SPI commu failed, this should be set
    if (dec <= 0)
    {
        _bad_mpu60x0 = 2;
    }
}

static int8_t mpu60x0_config(void)
{
    /** WhoAmI register test */
    static uint8_t whoAmI = 0x00;
    int8_t try_c = 20;

    /** Asign gyro/acc direction default */
    _mpu60x0Data.accSign[0] = 1.0;
    _mpu60x0Data.accSign[1] = -1.0;
    _mpu60x0Data.accSign[2] = 1.0;
    _mpu60x0Data.gyoSign[0] = -1.0;
    _mpu60x0Data.gyoSign[1] = 1.0;
    _mpu60x0Data.gyoSign[2] = -1.0;

    // reset
    mpu60x0SetReg(107, 0x80);
    HAL_Delay(100);

    // wake up w/ Z axis clock reg
    mpu60x0ReliablySetReg(107, 0x03);

    // enable I2C interface
    mpu60x0ReliablySetReg(106, 0x00);

    // wait for a valid response
    
    do
    {
        whoAmI = mpu60x0GetReg(117);
        HAL_Delay(10);
        try_c--;
    }
    while (whoAmI != 0x68 && (try_c > 0));

    if (try_c <= 0)
    {
        // TODO: log an error of MPU sensor failure
        _bad_mpu60x0 = 3;

        return 0;
    }

    // GYO scale 
    mpu60x0ReliablySetReg(27, 0x10);

    // ACC scale
    mpu60x0ReliablySetReg(28, 0x18);

    // Sample rate=Gyroscope ODR/(1+SMPLRT_DIV)
    mpu60x0ReliablySetReg(25, 0x00);

    // LPF, 2 is 94-98Hz, 3 is 44-48Hz, gyro ODR is 1KHz
    mpu60x0ReliablySetReg(26, 0x02);

    // Interrupt setup
    mpu60x0ReliablySetReg(55, 0x01<<4);
    mpu60x0ReliablySetReg(56, 0x01);

    return 1;
}

/**
 * Scale acc raw to m/s^2
 * @param in      : summed raw  acc in LSB
 * @param out     : output acc in m/s^2
 * @param divisor : how many samples summed
 */
static void mpu60x0ScaleAcc(double *in, double *out, double divisor)
{
    double scale;

    scale = (MPU60x0_ACC_SCALE * 2.0) / 65536 * divisor;

    for (int8_t i=0; i<3; i++)
    {
        out[i] = _mpu60x0Data.accSign[i] * in[i] * scale * 9.80665;
    }
}

/**
 * Scale gyro raw to deg/s.
 * @param in      : summed sample data in LSB
 * @param out     : gyro in deg/s
 * @param divisor : how many samples summed
 */
static void mpu60x0ScaleGyo(double *in, double *out, double divisor)
{
    double scale;

    scale = (MPU60x0_GYO_SCALE * 2.0) / 65536 * divisor;

    for (int8_t i=0; i<3; i++)
    {
        out[i] = _mpu60x0Data.gyoSign[i] * in[i] * scale;
    }
}

/**
 * decode mpu60x0 new data. Convert LSB to deg/s or m/s^2, and do calibration.
 */
static void mpu60x0Decode(void)
{
    volatile uint8_t *d = _mpu60x0Data.rxBuf;
    int32_t temp = 0;
    double acc_sum[3] = {0.0, 0.0, 0.0};
    double gyo_sum[3] = {0.0, 0.0, 0.0};
    double divisor = 1.0;
    uint8_t i = 0;

    double accx_raw = 0.0;
    double accy_raw = 0.0;
    double accz_raw = 0.0;

    double gyrox_raw = 0.0;
    double gyroy_raw = 0.0;
    double gyroz_raw = 0.0;


    for (i = 0; i < 3; i++)
    {
        acc_sum[i] = 0;
        gyo_sum[i] = 0;
    }
    temp = 0;

    divisor = 1.0;
    for (i = 0; i < 1; i++)
    {
        uint8_t j = 0;

        for (int8_t k=0; k<3; k++)
        {
            acc_sum[k] += (int16_t)__rev16(*(uint16_t *)&d[j+2*k+0]);
            gyo_sum[k] += (int16_t)__rev16(*(uint16_t *)&d[j+2*k+8]);
        }

        //ADD RAWEST DATA FOR LOG
        accx_raw = (int16_t)__rev16(*(uint16_t *)&d[j+0]);
        accy_raw = (int16_t)__rev16(*(uint16_t *)&d[j+2]);
        accz_raw = (int16_t)__rev16(*(uint16_t *)&d[j+4]);

        gyrox_raw = (int16_t)__rev16(*(uint16_t *)&d[j+8]);
        gyroy_raw = (int16_t)__rev16(*(uint16_t *)&d[j+10]);
        gyroz_raw = (int16_t)__rev16(*(uint16_t *)&d[j+12]);

        temp += (int16_t)__rev16(*(uint16_t *)&d[j+6]);
    }

    divisor = 1.0 / divisor;

    /// MPU60x0 temp
    _mpu60x0Data.rawTemp = temp * divisor * (1.0 / 340.0) + 36.53;
    _mpu60x0Data.temp = _mpu60x0Data.temp*0.97 + _mpu60x0Data.rawTemp*0.03;

    /// MPU60x0 acc_sum in m/s^2
    mpu60x0ScaleAcc(acc_sum, _mpu60x0Data.rawAcc, divisor);

    //ADD RAWEST DATA FOR LOG
    _mpu60x0_accx_raw = _mpu60x0Data.accSign[0] * accx_raw * (MPU60x0_ACC_SCALE * 2.0) / 65536 * 9.80665;
    _mpu60x0_accy_raw = _mpu60x0Data.accSign[1] * accy_raw * (MPU60x0_ACC_SCALE * 2.0) / 65536 * 9.80665;
    _mpu60x0_accz_raw = _mpu60x0Data.accSign[2] * accz_raw * (MPU60x0_ACC_SCALE * 2.0) / 65536 * 9.80665;

    _mpu60x0_gyrox_raw = _mpu60x0Data.gyoSign[0] * gyrox_raw * (MPU60x0_GYO_SCALE * 2.0) / 65536;
    _mpu60x0_gyroy_raw = _mpu60x0Data.gyoSign[1] * gyroy_raw * (MPU60x0_GYO_SCALE * 2.0) / 65536;
    _mpu60x0_gyroz_raw = _mpu60x0Data.gyoSign[2] * gyroz_raw * (MPU60x0_GYO_SCALE * 2.0) / 65536;

    /// MPU60x0 gyro in deg/s
    mpu60x0ScaleGyo(gyo_sum, _mpu60x0Data.rawGyo, divisor);

    /// record update time
    _mpu60x0Data.lastUpdate = HAL_GetTick();
}


/**
 * Init mpu60x0 and return init state.
 * @return  : 1 if init success.
 */
int8_t mpu60x0_init(void)
{
    Bsp_I2c_Init(&mpu60x0_ioi2c_t);
    _mpu60x0Data.enabled = 0;
    _mpu60x0Data.sample_ready = 0;
    if (!mpu60x0_config())
        return 0;

    _mpu60x0Data.enabled = 1;
    return 1;
}


void mpu60x0_transfer(void)
{

    if (_mpu60x0Data.enabled)
    {
                _mpu60x0Data.sample_ready = Bsp_I2c_Read_Buffer(
                        &mpu60x0_ioi2c_t, MPU60x0_ADDR, 0x3B, MPU60x0_BYTES,
                        _mpu60x0Data.rxBuf);
    }
   
}

/**
 * Decode the raw buffer filled by mpu60x0_transfer().
 * @return : true if a new sample has been decoded, false if there was nothing new.
 * @note   : Task context only.
 */
bool mpu60x0_update(void)
{
    if (!_mpu60x0Data.sample_ready)
    {
        return false;
    }

    mpu60x0Decode();
    _mpu60x0Data.sample_ready = 0;

    return true;
}




double mpu60x0_get_rawgyo_dps(uint8_t axis)
{
    return _mpu60x0Data.rawGyo[axis];
}

double mpu60x0_get_rawacc_mss(uint8_t axis)
{
    return _mpu60x0Data.rawAcc[axis];
}


double mpu60x0_get_rawaccx_mss(void)
{
    return _mpu60x0_accx_raw;
}

double mpu60x0_get_rawaccy_mss(void)
{
    return _mpu60x0_accy_raw;
}

double mpu60x0_get_rawaccz_mss(void)
{
    return _mpu60x0_accz_raw;
}

double mpu60x0_get_rawgyrox_dps(void)
{
    return _mpu60x0_gyrox_raw;
}

double mpu60x0_get_rawgyroy_dps(void)
{
    return _mpu60x0_gyroy_raw;
}

double mpu60x0_get_rawgyroz_dps(void)
{
    return _mpu60x0_gyroz_raw;
}


double mpu60x0_get_rawtemp_d(void)
{
    return _mpu60x0Data.rawTemp;
}

int mpu60x0write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{  
    if(Bsp_I2c_Write_Buffer(&mpu60x0_ioi2c_t,addr,reg,len,data))
    {
        return 0;
    }
    else
    {
        return -1;
    }  
}

int mpu60x0read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    if(Bsp_I2c_Read_Buffer(&mpu60x0_ioi2c_t,addr,reg,len,data))
    {
        return 0;
    }
    else
    {
        return -1;
    }
        
}






