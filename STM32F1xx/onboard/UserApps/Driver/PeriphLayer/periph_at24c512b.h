#ifndef __PERIPH_AT24C512B_H
#define __PERIPH_AT24C512B_H

#include <stdint.h>
#include <stdbool.h>

/** Micros definition *************************************************************************** */
/** I2C port and pin config */
#define IOI2C_SCL_PORT_AT24XX			GPIOC
#define IOI2C_SCL_PIN_AT24XX			GPIO_PIN_4
#define IOI2C_SDA_PORT_AT24XX			GPIOC
#define IOI2C_SDA_PIN_AT24XX			GPIO_PIN_5

/* AT24C512B param definition */
#define AT24C512B_ADDRESS       	0xA0	// IIC address
#define AT24C512B_SIZE           	512		//
#define AT24C512B_ONE_PAGE_SIZE  	128		// Bytes of one page

/** extern public functions type declaration **************************************************** */
bool at24cxx_init(void);

void at24cxx_read_data(uint16_t address, uint8_t length, uint8_t*data);
uint8_t at24cxx_write_data(uint16_t address, uint8_t length, uint8_t*data);

void at24cxx_write_test(void);
void at24cxx_read_test(void);
void at24cxx_reset_test(void);

#endif
/** END OF FILE ********************************************************************************* */
