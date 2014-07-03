/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file lsm330dlc_driver.c
*
* This file is the driver for the integrated 3axis gyroscope and accelerometer LSM330DLC
*/


#include "lsm330dlc_driver.h"
#include "gpio.h"
#include "i2c_driver_int.h"
#include "print_util.h"

static volatile lsm_gyro_data_t lsm_gyro_outputs;		///< Declare an object containing the gyroscope's data
static volatile lsm_acc_data_t  lsm_acc_outputs;		///< Declare an object containing the accelerometer's data

uint8_t lsm_read_register(uint8_t device, unsigned char address) 
{
	int16_t result;
	twim_write(&AVR32_TWIM0, (uint8_t*) &address, 1, device, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(result), 1, device, false);
	return result;
}

uint8_t lsm_write_register(uint8_t device, unsigned char address, uint8_t value) 
{
	// int16_t result;
	twim_write(&AVR32_TWIM0, (uint8_t*) &address, 1, device, false);
	twim_write(&AVR32_TWIM0, (uint8_t*) &value, 1, device, false);
}



void init_lsm330_acc(void) 
{	
	twim_write(&AVR32_TWIM0, (uint8_t*)&lsm_acc_default_config, sizeof(lsm_acc_default_config), LSM330_ACC_SLAVE_ADDRESS, false);
	twim_write(&AVR32_TWIM0, (uint8_t*) &fifo_config, 2, LSM330_ACC_SLAVE_ADDRESS, false);
}

void init_lsm330_gyro(void) 
{
	//gpio_configure_pin(LSM_GYRO_DEN_PIN, GPIO_DIR_OUTPUT);	
	//gpio_set_pin_high(LSM_GYRO_DEN_PIN);
	twim_write(&AVR32_TWIM0, (uint8_t*)&lsm_gyro_default_config, sizeof(lsm_gyro_default_config), LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_write(&AVR32_TWIM0, (uint8_t*) &fifo_config, 2, LSM330_GYRO_SLAVE_ADDRESS, false);
}

lsm_get_acc_config() 
{
	int i;
	uint8_t data_register_address=lsm_acc_default_config.start_address | LSM_AUTO_INCREMENT;
	uint8_t readbuffer[5];
	twim_transfer_t preamble;
	twim_transfer_t result;
	
	preamble.buffer=&data_register_address;
	preamble.chip=LSM330_ACC_SLAVE_ADDRESS;
	preamble.length=1;
	preamble.read=0;
	result.chip=LSM330_ACC_SLAVE_ADDRESS;
	result.buffer=&readbuffer;
	result.length=sizeof(readbuffer);
	result.read=1;
	
	twim_write(&AVR32_TWIM0, (uint8_t*) &data_register_address, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&readbuffer, sizeof(readbuffer), LSM330_ACC_SLAVE_ADDRESS, false);
	
	print_util_dbg_print("lsm acc config:\n");
	for (i=0; i<sizeof(readbuffer); i++) 
	{
		print_util_dbg_print_num(readbuffer[i], 16); print_util_dbg_print(" (");
		print_util_dbg_print_num(lsm_acc_default_config.ctrl_reg_a[i], 16); print_util_dbg_print(")\n");
	}
}

lsm_get_gyro_config() 
{
	int i;
	uint8_t data_register_address=lsm_acc_default_config.start_address | LSM_AUTO_INCREMENT;
	uint8_t readbuffer[5];
	twim_transfer_t preamble;
	twim_transfer_t result;
	
	preamble.buffer=&data_register_address;
	preamble.chip=LSM330_GYRO_SLAVE_ADDRESS;
	preamble.length=1;
	preamble.read=0;
	result.chip=LSM330_GYRO_SLAVE_ADDRESS;
	result.buffer=&readbuffer;
	result.length=sizeof(readbuffer);
	result.read=1;
	
	twim_write(&AVR32_TWIM0, (uint8_t*) &data_register_address, 1, LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&readbuffer, sizeof(readbuffer), LSM330_GYRO_SLAVE_ADDRESS, false);
	
	print_util_dbg_print("lsm gyro config:\n");
	for (i=0; i<sizeof(readbuffer); i++) 
	{
		print_util_dbg_print_num(readbuffer[i], 16); print_util_dbg_print(" (");
		print_util_dbg_print_num(lsm_gyro_default_config.ctrl_reg_g[i], 16); print_util_dbg_print(")\n");
	}
}

void lsm330dlc_driver_init(void) 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz=64000000,
		.speed = 400000,
		.chip = LSM330_ACC_SLAVE_ADDRESS,
		.smbus=false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	init_lsm330_acc();
	init_lsm330_gyro();
	lsm_get_acc_config();
	lsm_get_gyro_config();
}

lsm_acc_data_t* lsm330dlc_driver_get_acc_data(void) 
{
	uint8_t data_register_address=LSM_ACC_OUT_ADDRESS| LSM_AUTO_INCREMENT;
	int twim_return;
	lsm_acc_fifo_t fifo_values;
	int32_t axes[3]={0,0,0};
	int i;
	uint8_t fifo_fill=1;
	
	///< read number of bytes in fifo
	//fifo_fill=lsm_read_register(LSM330_ACC_SLAVE_ADDRESS, LSM_ACC_FIFO_SRC_ADDRESS) & 0x0f - 1;
	
	if (fifo_fill==0) 
	{
		fifo_fill=1; 
		//return &lsm_acc_outputs;
	}
	if (fifo_fill>6) 
	{
		fifo_fill=6;
	}
	
	twim_return=twim_write(&AVR32_TWIM0, (uint8_t*) &data_register_address, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_return=twim_read(&AVR32_TWIM0, (uint8_t*)&fifo_values.axes, 6*fifo_fill, LSM330_ACC_SLAVE_ADDRESS, false);

	for (i=0; i<fifo_fill; i++) 
	{
		axes[0]+=(int32_t)fifo_values.axes[6*i+1]*256 + fifo_values.axes[6*i];
		axes[1]+=(int32_t)fifo_values.axes[6*i+3]*256 + fifo_values.axes[6*i+2];
		axes[2]+=(int32_t)fifo_values.axes[6*i+5]*256 + fifo_values.axes[6*i+4];
	}
	lsm_acc_outputs.axes[0]=(int16_t)(axes[0]/fifo_fill);
	lsm_acc_outputs.axes[1]=(int16_t)(axes[1]/fifo_fill);
	lsm_acc_outputs.axes[2]=(int16_t)(axes[2]/fifo_fill);
	
	return &lsm_acc_outputs;
}

lsm_gyro_data_t* lsm330dlc_driver_get_gyro_data(void) 
{
	uint8_t data_register_address=LSM_GYRO_OUT_ADDRESS| LSM_AUTO_INCREMENT;
	int twim_return;
	lsm_gyro_fifo_t fifo_values;
	int32_t axes[3]={0,0,0};
	int i;
	///< read number of bytes in fifo
	int8_t fifo_fill=(int8_t)(lsm_read_register(LSM330_GYRO_SLAVE_ADDRESS, LSM_ACC_FIFO_SRC_ADDRESS) & 0x0f);
	
	if (fifo_fill<=0) 
	{
		return &lsm_gyro_outputs;
	}
	//if (fifo_fill>6) fifo_fill=6;
	
	fifo_fill=1;
	twim_return=twim_write(&AVR32_TWIM0, (uint8_t*)&data_register_address, 1, LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_return=twim_read(&AVR32_TWIM0, (uint8_t*)&fifo_values, 2+6*fifo_fill, LSM330_GYRO_SLAVE_ADDRESS, false);
	
	for (i=0; i<fifo_fill; i++) 
	{
		axes[0]+=fifo_values.axes[3*i];
		axes[1]+=fifo_values.axes[3*i+1];
		axes[2]+=fifo_values.axes[3*i+2];
	}
	
	lsm_gyro_outputs.temperature=fifo_values.temperature;
	lsm_gyro_outputs.axes[0]=(int16_t)(axes[0]/fifo_fill);
	lsm_gyro_outputs.axes[1]=(int16_t)(axes[1]/fifo_fill);
	lsm_gyro_outputs.axes[2]=(int16_t)(axes[2]/fifo_fill);
	
	return &lsm_gyro_outputs;
}