#include "iks01a1.h"

uint8_t data = 0;

uint8_t lps25hb_init(void){
	LL_mDelay(100);
	uint8_t val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRES);
	//val = lsm6ds0_read_byte(LSM6DS0_WHO_AM_I_ADDRES);

	if(!(val == LPS25HB_WHO_AM_I_VALUE))
	{
		return 0; //bad who am I value
	}

	LL_mDelay(100);
	val = hts221_read_byte(HTS221_WHO_AM_I_ADDRES);

	if(!(val == HTS221_WHO_AM_I_VALUE))
	{
		return 0; //bad who am I value
	}
	return 1;
}

uint8_t lps25hb_read_byte(uint8_t reg_addr) {
	data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, LPS25HB_DEVICE_ADDRESS, 0));
}

uint8_t hts221_read_byte(uint8_t reg_addr) {
	data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, HTS221_DEVICE_ADDRESS, 0));
}

uint8_t lsm6ds0_read_byte(uint8_t reg_addr)
{
	data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, LSM6DS0_DEVICE_ADDRESS, 0));
}
