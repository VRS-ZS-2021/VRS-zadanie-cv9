#include "main.h"
#include "i2c.h"


#define LPS25HB_DEVICE_ADDRESS 0xBB //0xB8

#define LPS25HB_WHO_AM_I_ADDRES	0x0F
#define LPS25HB_WHO_AM_I_VALUE	0xBD

#define HTS221_DEVICE_ADDRESS 0xBE

#define HTS221_WHO_AM_I_ADDRES	0x0F
#define HTS221_WHO_AM_I_VALUE	0xBC

#define LSM6DS0_DEVICE_ADDRESS 0xD6 //0xD4

#define LSM6DS0_WHO_AM_I_VALUE 0x68
#define	LSM6DS0_WHO_AM_I_ADDRES	0x0F

uint8_t lps25hb_init(void);
uint8_t lps25hb_read_byte(uint8_t reg_addr);
uint8_t lsm6ds0_read_byte(uint8_t reg_addr);
uint8_t hts221_read_byte(uint8_t reg_addr);
