#include "periph_at24c512b.h"
#include "bsp_i2c_drv.h"
#include "bsp_delay_drv.h"

/** *********************************************************************************************
 * EEPROM initialize. Just need to initialize I2C for EEPROM AT24C512, which has 512 pages and 128
 * Bytes per page.
 * Support eeprom read/write with buffer.
 * Note: AT24Cxx need to delay more than 2.5ms if write cross page, or for twice write operation.
 * ********************************************************************************************** */

#define EEPROM_WRITE_PERIOD_MS         5u

__packed struct eeprom_flag_t
{
    uint64_t last_write_ms;
    uint64_t last_read_ms;
    int32_t  wait_ms_to_write;
};

/** Private variables ***************************S************************************************ */
/** I2C port struct init */
struct bsp_ioi2c_t _ioi2c_at24 = {IOI2C_SCL_PORT_AT24XX, IOI2C_SDA_PORT_AT24XX, \
                                  IOI2C_SCL_PIN_AT24XX, IOI2C_SDA_PIN_AT24XX
};

struct eeprom_flag_t _eeprom_flag = {0, 0, 0};

/** Private functions *************************************************************************** */
static int16_t eeprom_wait_ms_to_write(void);

/** Functions declaration *********************************************************************** */
/**
 * Init I2C for AT24C512.
 * @return  : Now always return true.
 */
bool at24cxx_init(void)
{
    Bsp_I2c_Init(&_ioi2c_at24);

    return true;
}

/**
 * @brief	Read specified length of data in specified address of EEPROM
 * @param	address: specified add to read
 * @param	length: length of data to be read in byte
 * @param	data: buffer to store read data
 */
void at24cxx_read_data(uint16_t address, uint8_t length, uint8_t *data)
{
    uint8_t last_space = 0;

    last_space = AT24C512B_ONE_PAGE_SIZE - address%AT24C512B_ONE_PAGE_SIZE;

    /** 1#: read length<=last size of this page, just read it.
     *  2#: read length>last size, seperate it to 2 pages read */
    if (last_space>=length)
    {
        /// If last space longer than length, just read it.
        Bsp_I2c_Multi_Read_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address, length, data);
    }
    else
    {
        /// Read the last space data into data_buffer
        Bsp_I2c_Multi_Read_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address, last_space, data);

        length -= last_space;	// last length to read

        Bsp_I2c_Multi_Read_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address+last_space, length, data+last_space);
    }

    _eeprom_flag.last_read_ms = HAL_GetTick();
}

/**
 * @brief	Write buffer data to specified address
 * @param	address: which position to be written
 * @param	length: data length to be written
 * @param	data: buffer which stored data
 */
uint8_t at24cxx_write_data(uint16_t address, uint8_t length, uint8_t *data)
{
    uint8_t last_space = 0;
    uint8_t ret = 0;

    if (eeprom_wait_ms_to_write() != 0)
    {
        HAL_Delay(eeprom_wait_ms_to_write());
    }

    last_space	= AT24C512B_ONE_PAGE_SIZE-address%AT24C512B_ONE_PAGE_SIZE;	// Calculate last space of this page
    if (last_space >= length)
    {
        // if last space in this page is enough to write, just do it
        ret = Bsp_I2c_Multi_Write_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address, length, data);
    }
    else
    {
        /// We should write the last space in this page
        ret = Bsp_I2c_Multi_Write_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address, last_space, data);
        HAL_Delay(3);

        length -= last_space;	// the last length to write

        ret = Bsp_I2c_Multi_Write_Reg16(&_ioi2c_at24, AT24C512B_ADDRESS, address+last_space, length, data+last_space);
    }

    _eeprom_flag.last_write_ms = HAL_GetTick();

    return ret;
}

/**
 * How long time should we need to do next write operation.
 * @return  : us to wait
 */
static int16_t eeprom_wait_ms_to_write(void)
{
    if (HAL_GetTick() - _eeprom_flag.last_write_ms >= EEPROM_WRITE_PERIOD_MS)
    {
        return 0;
    }
    else
    {
        int16_t wait_ms = (_eeprom_flag.last_write_ms+EEPROM_WRITE_PERIOD_MS-HAL_GetTick());

        if (wait_ms<0 || wait_ms>EEPROM_WRITE_PERIOD_MS)
        {
            wait_ms = EEPROM_WRITE_PERIOD_MS;
        }

        return wait_ms;
    }
}

/** TEST function ******************************************************************************* */
#define AT24CXX_TEST_ADDRESS		        125
#define AT24CXX_TEST_LEN			10
void at24cxx_write_test(void)
{
    uint8_t wr_char[AT24CXX_TEST_LEN] = {'H','e','l','l','o',',','T','X','G','!'};

    at24cxx_write_data(AT24CXX_TEST_ADDRESS, AT24CXX_TEST_LEN, &wr_char[0]);
    HAL_Delay(2);
}

void at24cxx_read_test(void)
{
    static uint8_t rd_char[10] = {0};

    at24cxx_read_data(AT24CXX_TEST_ADDRESS, AT24CXX_TEST_LEN, &rd_char[0]);
}

void at24cxx_reset_test(void)
{
    uint8_t wr_char[AT24CXX_TEST_LEN] = {0xFF};

    for (uint8_t i=0; i<AT24CXX_TEST_LEN; i++)
    {
        wr_char[i] = 0xFF;
    }

    at24cxx_write_data(AT24CXX_TEST_ADDRESS, AT24CXX_TEST_LEN, &wr_char[0]);
    HAL_Delay(2);
}

/** END OF FILE ********************************************************************************* */
