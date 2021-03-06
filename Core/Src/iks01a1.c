#include "iks01a1.h"

uint8_t data = 0;

uint8_t iks01a1_init(void){
	uint8_t ctrl;
	LL_mDelay(500);
	uint8_t val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRES);
	//val = lsm6ds0_read_byte(LSM6DS0_WHO_AM_I_ADDRES);

	if(!(val == LPS25HB_WHO_AM_I_VALUE))
	{
		return 0; //bad who am I value
	}

	ctrl = hts221_read_byte(LPS25HB_CTRL_REG1);
	ctrl |= 1 << 7;
	lps25hb_write_byte(LPS25HB_CTRL_REG1, ctrl); //active mode


	LL_mDelay(500);
	val = hts221_read_byte(HTS221_WHO_AM_I_ADDRES);

	if(!(val == HTS221_WHO_AM_I_VALUE))
	{
		return 0; //bad who am I value
	}

	ctrl = hts221_read_byte(HTS221_CTRL_REG1);
	ctrl |= 1 << 7;
	hts221_write_byte(HTS221_CTRL_REG1, ctrl); //active mode
	//uint8_t ctrltest = hts221_read_byte(HTS221_CTRL_REG1);

	//ctrl = ctrltest;

	return 1;
}

void hts221_start_measurement(void) {
	uint8_t ctrl = hts221_read_byte(LPS25HB_CTRL_REG2);
	ctrl |= 0x1;
	lps25hb_write_byte(LPS25HB_CTRL_REG2, ctrl); //activate measurement
}

void lps25hb_start_measurement(void) {
	uint8_t ctrl = hts221_read_byte(HTS221_CTRL_REG2);
	ctrl |= 0x1;
	hts221_write_byte(HTS221_CTRL_REG2, ctrl); //activate measurement
}

void hts221_get_humidity(float* out) { //humidity measurement (%)
	uint8_t data[2], h0_rh, h1_rh;
	int16_t h_out, h0_t0_out, h1_t0_out;



	uint8_t availability = 0;
	availability = hts221_read_byte(HTS221_STATUS_REG);
	availability &= (uint8_t)(1<<1) >> 1;

	if (availability==0) { //waiting for reading availability
		hts221_start_measurement();
		availability = hts221_read_byte(HTS221_STATUS_REG);
		availability &= (uint8_t)(0x1<<1);
	}

	h0_rh = hts221_read_byte(HTS221_H0_RH)/2;
	h1_rh = hts221_read_byte(HTS221_H1_RH)/2;

	hts221_readArray(data, HTS221_H0_T0_OUT, 2);
	h0_t0_out = ((uint16_t)data[1]) << 8 | data[0];

	hts221_readArray(data, HTS221_H1_T0_OUT, 2);
	h1_t0_out = ((uint16_t)data[1]) << 8 | data[0];

	hts221_readArray(data, HTS221_HUMIDITY_ADDR, 2);

	h_out = ((uint16_t)data[1]) << 8 | data[0];
	*out = (float)((h1_rh-h0_rh)*(h_out-h0_t0_out)/(float)(h1_t0_out-h0_t0_out)+h0_rh);
}

void hts221_get_temperature(float* out) { //temperature measurement (??C)
	uint8_t data[2];
	int16_t t_out, t0_out, t1_out, t0_degc, t1_degc;

	uint8_t availability = 0;
	availability = hts221_read_byte(HTS221_STATUS_REG);
	availability &= (uint8_t)(1);

	if (availability==0) { //waiting for reading availability
		hts221_start_measurement();
		availability = hts221_read_byte(HTS221_STATUS_REG);
		availability &= (uint8_t)(1);
	}

	uint8_t tmp = hts221_read_byte(HTS221_T0_T1_DEGC_MSB);

	t0_degc = hts221_read_byte(HTS221_T0_DEGC);
	t1_degc = hts221_read_byte(HTS221_T1_DEGC);

	t0_degc += ((tmp & 0x3)<<8);
	t0_degc /= 8;

	t1_degc += ((tmp & ((0x3)<<2))<<6);
	t1_degc /= 8;

	hts221_readArray(data, HTS221_T0_OUT, 2);
	t0_out = (((uint16_t)data[1]) << 8) | data[0];
	hts221_readArray(data, HTS221_T1_OUT, 2);
	t1_out = (((uint16_t)data[1]) << 8) | data[0];

	hts221_readArray(data, HTS221_TEMPERATURE_ADDR, 2);

	t_out = ((uint16_t)data[1]) << 8 | data[0];
	*out = (float)((t1_degc - t0_degc)*(t_out-t0_out)/(float)(t1_out-t0_out)+t0_degc);
}

void lps25hb_get_pressure(float* out) { //pressure measurement (hPa = mBAR)
	uint8_t data[3];
	uint32_t pressure;// ref_pressure;

	uint8_t availability = 0;
	availability = lps25hb_read_byte(LPS25HB_STATUS_REG);
	availability &= (uint8_t)(1<<1)>>1;

	if (availability==0) { //waiting for reading availability
		lps25hb_start_measurement();
		availability = lps25hb_read_byte(LPS25HB_STATUS_REG);
		availability &= (uint8_t)(1<<1);
	}

	lps25hb_readArray(data, LPS25HB_PRESSURE_ADDR, 3);
	pressure = ((uint32_t)data[2]) << 16 | ((uint16_t)data[1]) << 8 | data[0];

	/*lps25hb_readArray(data, LPS25HB_REF_PRESSURE_ADDR, 3);
	ref_pressure = ((uint32_t)data[2]) << 16 | ((uint16_t)data[1]) << 8 | data[0];*/

	*out =  (float)(pressure)/(float)(4096);
}

void lps25hb_get_altitude(float* out) { //altitude measurement (m.n.m./AMSL)
	uint8_t data[3];
	uint32_t pressure;

	uint8_t availability = 0;
	availability = lps25hb_read_byte(LPS25HB_STATUS_REG);
	availability &= (uint8_t)(1<<1)>>1;

	if (availability==0) { //waiting for reading availability
		lps25hb_start_measurement();
		availability = lps25hb_read_byte(LPS25HB_STATUS_REG);
		availability &= (uint8_t)(1<<1);
	}

	lps25hb_readArray(data, LPS25HB_PRESSURE_ADDR, 3);
	pressure = ((uint32_t)data[2]) << 16 | ((uint16_t)data[1]) << 8 | data[0];

	*out = 44330*(1-pow((float)(pressure)/(float)(4096*1013.25f),(1/5.255f)));
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

void lsm6ds0_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, LSM6DS0_DEVICE_ADDRESS, 1);
}

void hts221_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, HTS221_DEVICE_ADDRESS, 1);
}

void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, LPS25HB_DEVICE_ADDRESS, 1);
}

void lsm6ds0_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, LSM6DS0_DEVICE_ADDRESS, 0);
}

void hts221_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, HTS221_DEVICE_ADDRESS, 0);
}

void lps25hb_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, LPS25HB_DEVICE_ADDRESS, 0);
}
