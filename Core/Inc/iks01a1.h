#include "main.h"
#include "i2c.h"


#define LPS25HB_DEVICE_ADDRESS 0xBB //0xB8

#define LPS25HB_WHO_AM_I_ADDRES	0x0F
#define LPS25HB_WHO_AM_I_VALUE	0xBD

#define HTS221_DEVICE_ADDRESS 0xBE

#define HTS221_WHO_AM_I_ADDRES	0x0F
#define HTS221_WHO_AM_I_VALUE	0xBC
#define HTS221_CTRL_REG1	0x20
#define HTS221_CTRL_REG2	0x21
#define HTS221_STATUS_REG	0x27
#define HTS221_HUMIDITY_ADDR	0x28
#define HTS221_TEMPERATURE_ADDR	0x2A
#define HTS221_T0_OUT	0x3E
#define HTS221_T1_OUT	0x3C
#define HTS221_T0_DEGC	0x32
#define HTS221_T1_DEGC	0x33
#define HTS221_T0_T1_DEGC_MSB	0x35
#define HTS221_H0_T0_OUT	0x36
#define HTS221_H1_T0_OUT	0x3A
#define HTS221_H0_RH	0x30
#define HTS221_H1_RH	0x31

#define LSM6DS0_DEVICE_ADDRESS 0xD6 //0xD4

#define LSM6DS0_WHO_AM_I_VALUE 0x68
#define	LSM6DS0_WHO_AM_I_ADDRES	0x0F

uint8_t iks01a1_init(void);
uint8_t lps25hb_read_byte(uint8_t reg_addr);

uint8_t lsm6ds0_read_byte(uint8_t reg_addr);
void lsm6ds0_write_byte(uint8_t reg_addr, uint8_t value);
void lsm6ds0_readArray(uint8_t * data, uint8_t reg, uint8_t length);

uint8_t hts221_read_byte(uint8_t reg_addr);
void hts221_write_byte(uint8_t reg_addr, uint8_t value);
void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length);
void hts221_start_measurement(void);
void hts221_get_humidity(float* out);
void hts221_get_temperature(float* out);

