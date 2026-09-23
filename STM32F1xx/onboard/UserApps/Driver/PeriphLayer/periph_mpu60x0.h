#ifndef __PERIPH_MPU60x0_H
#define __PERIPH_MPU60x0_H

#include "stm32f1xx.h"

#ifndef __rev16
#define __rev16 __REV16
#endif

/** MPU60x0 ADDR */
#define MPU60x0_ADDR            (0x68)

#define MPU60x0_BYTES		    14
#define MPU60x0_SLOT_SIZE	    ((MPU60x0_BYTES+sizeof(int)-1) / sizeof(int) * sizeof(int))

#define MPU60x0_SLOTS	    	    10				// 100Hz bandwidth

/* MPU60x0 gyro and accel scale */
#define MPU60x0_ACC_SCALE      	16    // g      (2, 4, 8, 16)
#define MPU60x0_GYO_SCALE      	1000  // deg/s  (250, 500, 1000, 2000)

int8_t mpu60x0_init(void);
void mpu60x0_transfer(void);
void mpu60x0_update(void);


double mpu60x0_get_rawgyo_dps(uint8_t axis);
double mpu60x0_get_rawacc_mss(uint8_t axis);

double mpu60x0_get_rawaccx_mss(void);
double mpu60x0_get_rawaccy_mss(void);
double mpu60x0_get_rawaccz_mss(void);
double mpu60x0_get_rawgyrox_dps(void);
double mpu60x0_get_rawgyroy_dps(void);
double mpu60x0_get_rawgyroz_dps(void);

double mpu60x0_get_rawtemp_d(void);
int mpu60x0write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int mpu60x0read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);

#endif
