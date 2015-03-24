/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file lsm330dlc.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 *   
 * \brief This file is the driver for the integrated 3axis gyroscope and 
 * accelerometer LSM330DLC
 * 
 ******************************************************************************/


#include "lsm330dlc.h"
#include "gpio.h"
#include "i2c_driver_int.h"
#include "print_util.h"

#define LSM330_ACC_SLAVE_ADDRESS	0b0011000	///< Define the Accelerometer Address, as a slave on the i2c bus
#define LSM330_GYRO_SLAVE_ADDRESS	0b1101010	///< Define the Gyroscope Address, as a slave on the i2c bus

#define LSM_GYRO_DEN_PIN AVR32_PIN_PD23			///< Define the microcontroller pin to enable the gyroscope's data (Data EN)

///< CTRL_REG_A_1
#define LSM_ACC_DATARATE_OFF			0x00	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1HZ			0x10	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_10Hz			0x20	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_25Hz			0x30	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_50Hz			0x40	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_100Hz			0x50	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_200Hz			0x60	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_400Hz			0x70	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1620Hz			0x80	///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1344_5376Hz	0x90	///< Define the frequency of the Accelerometer data rate

#define LSM_ACC_X_EN					0x01	///< Enable the Accelerometer X_axis
#define LSM_ACC_Y_EN					0x02	///< Enable the Accelerometer Y_axis
#define LSM_ACC_Z_EN					0x04	///< Enable the Accelerometer Z_axis
#define LSM_ACC_ALL_EN					0x07	///< Enable the Accelerometer 3 axis

#define LSM_ACC_LOW_POWER_EN			0x08	///< Enable the acclerometer low power mode

///< CTRL_REG_A_2
#define LSM_ACC_HPIS1					0x01	///< Define the Accelerometer control register A2
#define LSM_ACC_HPIS2					0x02	///< Define the Accelerometer control register A2
#define LSM_ACC_HPCLICK					0x04	///< Define the Accelerometer control register A2
#define LSM_ACC_FDS						0x08	///< Define the Accelerometer control register A2
#define LSM_ACC_HPCF1					0x10	///< Define the Accelerometer control register A2
#define LSM_ACC_HPCF2					0x20	///< Define the Accelerometer control register A2
#define LSM_ACC_HPM0					0x40	///< Define the Accelerometer control register A2
#define LSM_ACC_HPM1					0x80	///< Define the Accelerometer control register A2

///< CTRL_REG_A_3
#define LSM_ACC_OVERRUN_INT				0x02	///< Define the Accelerometer control register A3
#define LSM_ACC_FIFO_WM_INT				0x04	///< Define the Accelerometer control register A3
#define LSM_ACC_DRDY2_INT				0x08	///< Define the Accelerometer control register A3
#define LSM_ACC_DRDY1_INT				0x10	///< Define the Accelerometer control register A3
#define LSM_ACC_AOI_INT					0x40	///< Define the Accelerometer control register A3
#define LSM_ACC_CLICK_INT				0x80	///< Define the Accelerometer control register A3

///< CTRL_REG_A_4
#define LSM_ACC_SPI_MODE				0x01	///< Define the mode (I2C/SPI) of the accelerometer sensor
#define LSM_ACC_HIGH_RES				0x08	///< Define the Accelerometer control register A4

#define LSM_ACC_FULL_SCALE_2G			0x00	///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_4G			0x10	///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_8G			0x20	///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_16G			0x30	///< Define the Accelerometer control register A4

#define LSM_ACC_BIG_ENDIAN				0x40	///< Define the Accelerometer control register A4

///< CTRL_REG_A_5
#define LSM_ACC_D4D_INT					0x04	///< Define the Accelerometer control register A5
#define LSM_ACC_LIR_INT					0x08	///< Define the Accelerometer control register A5
#define LSM_ACC_FIFO_EN					0x40	///< Define the Accelerometer control register A5
#define LSM_ACC_BOOT					0x80	///< Define the Accelerometer control register A5

///< CTRL_REG1_G
#define LSM_GYRO_DATARATE_95HZ			0x00	///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_190HZ			0x40	///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_380Hz			0x80	///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_760Hz			0xC0	///< Define the frequency of the gyroscope data rate

///< note: actual bandwidth depends on datarate - specified for 380Hz. Consult Datasheet.
#define LSM_GYRO_BANDWIDTH_20Hz			0x40	///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_25Hz			0x50	///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_50Hz			0x60	///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_100Hz		0x70	///< Define the gyroscope bandwith corresponding to a given data rate

#define LSM_GYRO_X_EN					0x01	///< Enable the gyroscope X_axis
#define LSM_GYRO_Y_EN					0x02	///< Enable the gyroscope Y_axis
#define LSM_GYRO_Z_EN					0x04	///< Enable the gyroscope Z_axis
#define LSM_GYRO_ALL_EN					0x07	///< Enable the gyroscope 3 axis
#define LSM_GYRO_POWER_ON				0x08	///< Power on the gyroscope X_axis

///< CTRL_REG2_G
#define LSM_GYRO_HPCF0					0x01	///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF1					0x02	///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF2					0x04	///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF3					0x08	///< Define the gyroscope control register G2
#define LSM_GYRO_HPM0					0x10	///< Define the gyroscope control register G2
#define LSM_GYRO_HPM1					0x20	///< Define the gyroscope control register G2
#define LSM_GYRO_LVL_EN					0x40	///< Enable the gyroscope control register G2
#define LSM_GYRO_EXTREN					0x80	///< Define the gyroscope control register G2

///< CTRL_REG3_G
#define LSM_GYRO_FIFO_EMPTY_INT			0x01	///< Define the gyroscope control register G3
#define LSM_GYRO_FIFO_OVRUN_INT			0x02	///< Define the gyroscope control register G3
#define LSM_GYRO_FIFO_WM_INT			0x04	///< Define the gyroscope control register G3
#define LSM_GYRO_DRDY_INT				0x08	///< Define the gyroscope control register G3
#define LSM_GYRO_PP_OD					0x10	///< Define the gyroscope control register G3
#define LSM_GYRO_H_L_ACT				0x20	///< Define the gyroscope control register G3
#define LSM_GYRO_I1_BOOT				0x40	///< Define the gyroscope control register G3
#define LSM_GYRO_I1_INT1				0x80	///< Define the gyroscope control register G3

///< CTRL_REG4_G
#define LSM_GYRO_SPI_MODE				0x01	///< Define the mode (I2C/SPI)of the gyroscope sensor

#define LSM_GYRO_FULL_SCALE_250			0x00	///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_500			0x10	///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_2000_1		0x20	///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_2000_2		0x30	///< Define the range of the gyroscope sensor

#define LSM_GYRO_BIG_ENDIAN				0x40	///< Define the gyroscope sensor endianness
#define LSM_GYRO_BLOCK_DATA				0x80	///< Define the block transmission mode of the gyroscope

///< CTRL_REG5_G
#define LSM_OUT_SEL0					0x01	///< Define the gyroscope control register G5
#define LSM_OUT_SEL1					0x02	///< Define the gyroscope control register G5
#define LSM_INT_SEL0					0x04	///< Define the gyroscope control register G5
#define LSM_INT_SEL1					0x08	///< Define the gyroscope control register G5
#define LSM_GYRO_HP_EN					0x10	///< Define the gyroscope control register G5
#define LSM_GYRO_FIFO_EN				0x40	///< Enable the FIFO of the gyroscope, part of control register G5
#define LSM_GYRO_BOOT					0x80	///< Define the gyroscope control register G5

#define LSM_ACC_CTRL_REG1_ADDRESS		0x20	///< Define the first control register address of the accelerometer

#define LSM_ACC_OUT_ADDRESS				0x27	///< Define the writing address of the accelerometer

#define LSM_ACC_FIFO_CTRL_ADDRESS		0x2E	///< Define the address of the FIFO control register, for the accelerometer
#define LSM_ACC_FIFO_SRC_ADDRESS		0x2F	///< Define the address of the FIFO source(?) register, for the accelerometer

#define LSM_GYRO_CTRL_REG1_ADDRESS		0x20	///< Define the first control register address of the gyroscope

#define LSM_GYRO_OUT_ADDRESS			0x26	///< Define the writing address of the gyroscope

#define LSM_GYRO_FIFO_CTRL_ADDRESS		0x2E	///< Define the address of the FIFO control register, for the gyroscope
#define LSM_GYRO_FIFO_SRC_ADDRESS		0x2F	///< Define the address of the FIFO source(?) register, for the gyroscope

#define LSM_AUTO_INCREMENT				0x80	///< Define the auto incrementation of the LSM330DLC sensor


/**
 * \brief Structure containing the configuration data of the accelerometer sensor. WARNING: start_address must 8-bits and the FIRST element. 
*/
typedef struct
{
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
	uint8_t ctrl_reg_a[5];		///< Define an array containing the Control register
} lsm330dlc_acc_write_conf_t;

/**
 * \brief Structure containing the configuration data of the accelerometer sensor. WARNING: start_address must 8-bits and the LAST element. 
*/
typedef struct
{
	uint8_t ctrl_reg_a[5];		///< Define an array containing the Control register
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
} lsm330dlc_acc_read_conf_t;

/**
 * \brief Structure containing the configuration data of the accelerometer sensor. WARNING: start_address must 8-bits and the FIRST element.  
*/
typedef struct
{
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
	uint8_t	ctrl_reg_g[5];		///< Define an array containing the Control register
} lsm330dlc_gyro_write_conf_t;

/**
 * \brief Structure containing the configuration data of the accelerometer sensor. WARNING: start_address must 8-bits and the LAST element.
*/
typedef struct
{
	uint8_t	ctrl_reg_g[5];		///< Define an array containing the Control register
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
} lsm330dlc_gyro_read_conf_t;

/**
 * \brief	Structure containing the accelerometer FIFO's data. WARNING: start_address must 8-bits and the LAST element.
 */
typedef struct
{
	uint8_t status_register;	///< Define the status register of the FIFO of the accelerometer
	int16_t axes[3];			///< Define an array containing the 3 axis of the accelerometer in the FIFO
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
} lsm_acc_read_t;

/**
 * \brief	Structure containing the gyroscope FIFO's data. WARNING: start_address must 8-bits and the LAST element.
 */
typedef struct
{
	int8_t temperature;			///< Define the temperature of the sensor
	uint8_t status_register;	///< Define the status register of the FIFO of the gyroscope
	int16_t axes[3];			///< Define an array containing the 3 axis of the gyroscope in the FIFO
	uint8_t start_address;		///< Define the start Address of the accelerometer sensor
} lsm_gyro_read_t;

/**
 * \brief	Structure containing filling of the FIFO. WARNING: start_address must 8-bits and the LAST element.
 */
typedef struct
{
	uint8_t fifo_fill;			///< Define the filling of the FIFO
	uint8_t start_address;		///< Define the start Address of the FIFO register
} lsm_read_fifo_fill_t;


/**
 * \brief	Define the configuration of the accelerometer
*/
static const lsm330dlc_acc_write_conf_t lsm_acc_default_config=
{
	.start_address = LSM_ACC_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
	.ctrl_reg_a=
	{	
		LSM_ACC_DATARATE_400Hz | LSM_ACC_ALL_EN ,						///< CTRL_REG_G_1
		0,																///< CTRL_REG_G_2
		0,																///< CTRL_REG_G_3
		LSM_ACC_HIGH_RES | LSM_ACC_FULL_SCALE_8G | LSM_ACC_BIG_ENDIAN,	///< CTRL_REG_G_4
		LSM_ACC_FIFO_EN													///< CTRL_REG_G_5
	}
};

/**
 * \brief	Define the configuration of the gyroscope
*/
static const lsm330dlc_gyro_write_conf_t lsm_gyro_default_config=
{
	.start_address = LSM_GYRO_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
	.ctrl_reg_g=
	{	
		LSM_GYRO_POWER_ON | LSM_GYRO_DATARATE_760Hz | LSM_GYRO_BANDWIDTH_50Hz | LSM_GYRO_ALL_EN,	///< CTRL_REG_A_1
		0,																							///< CTRL_REG_A_2
		0,																							///< CTRL_REG_A_3
		LSM_GYRO_FULL_SCALE_2000_2 | LSM_GYRO_BIG_ENDIAN,												///< CTRL_REG_A_4
		LSM_GYRO_FIFO_EN																			///< CTRL_REG_G_5
	}
};

/**
 * \brief	Declare the configuration of the FIFO
*/
static const uint8_t fifo_config[2] = {LSM_ACC_FIFO_CTRL_ADDRESS, 0x80};


/**
 * \brief			Write data to the LSM330 accelerometer  
 *
 * \param buffer	Address of the structure where data are send to the accelerometer including Device register address
 * \param nbytes	Number of bytes to send to the sensor, NOT including the device register address		
 */
static void	lsm330dlc_acc_write_register(uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Read data from the LSM330 gyroscope 
 *
 * \param address	Device register address to read the first data, with possible auto-increment
 * \param buffer	Address of the structure where data are read from the accelerometer
 * \param nbytes	Number of bytes to read in the sensor, NOT including the device register address		
 */
static void lsm330dlc_acc_read_register(uint8_t* address, uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Write data to the LSM330 gyroscope  
 *
 * \param buffer	Address of the structure where data are send to the gyroscope including Device register address
 * \param nbytes	Number of bytes to send to the sensor, NOT including the device register address		
 */
static void	lsm330dlc_gyro_write_register(uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Read data from the LSM330 gyroscope 
 *
 * \param address	Device register address to read the first data, with possible auto-increment
 * \param buffer	Address of the structure where data are read from the gyroscope
 * \param nbytes	Number of bytes to read in the sensor, NOT including the device register address		
 */
static void lsm330dlc_gyro_read_register(uint8_t* address, uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Initialize the LSM330 accelerometer sensor
 */
static void	lsm330dlc_acc_init(void);

/**
 * \brief			Initialize the LSM330 gyroscope sensor
 */
static void	lsm330dlc_gyro_init(void);

/**
 * \brief			Get the LSM330 accelerometer sensor config
 */
static void	lsm330dlc_get_acc_config(void);

/**
 * \brief			Get the LSM330 gyroscope sensor config
 */
static void	lsm330dlc_get_gyro_config(void);


static void lsm330dlc_acc_write_register(uint8_t* buffer, uint32_t nbytes) 
{
	twim_write(&AVR32_TWIM0, buffer, nbytes+1, LSM330_ACC_SLAVE_ADDRESS, false);
}

static void lsm330dlc_acc_read_register(uint8_t* address, uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, address, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, buffer, nbytes, LSM330_ACC_SLAVE_ADDRESS, false);
}

static void lsm330dlc_gyro_write_register(uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, buffer, nbytes+1, LSM330_GYRO_SLAVE_ADDRESS, false);
}

static void lsm330dlc_gyro_read_register(uint8_t* address, uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, address, 1, LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, buffer, nbytes, LSM330_GYRO_SLAVE_ADDRESS, false);
}

static void lsm330dlc_acc_init(void) 
{	
	lsm330dlc_acc_write_register((uint8_t*) &lsm_acc_default_config, 5);
	lsm330dlc_acc_write_register((uint8_t*) &fifo_config, 1);
}

static void lsm330dlc_gyro_init(void) 
{
	lsm330dlc_gyro_write_register((uint8_t*) &lsm_gyro_default_config, 5);
	lsm330dlc_gyro_write_register((uint8_t*) &fifo_config, 1);
}

static void lsm330dlc_get_acc_config(void)
{
	lsm330dlc_acc_read_conf_t lsm_acc_get_config;
	lsm_acc_get_config.start_address = LSM_ACC_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT;
	
	lsm330dlc_acc_read_register((uint8_t*)&lsm_acc_get_config.start_address, (uint8_t*)&lsm_acc_get_config.ctrl_reg_a[0], 5);
}

static void lsm330dlc_get_gyro_config(void) 
{
	lsm330dlc_gyro_read_conf_t lsm_gyro_get_config;
	lsm_gyro_get_config.start_address = LSM_GYRO_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT;
	
	lsm330dlc_gyro_read_register((uint8_t*)&lsm_gyro_get_config.start_address, (uint8_t*)&lsm_gyro_get_config.ctrl_reg_g[0], 5);
}

void lsm330dlc_init(void) 
{
	if(twim_probe(&AVR32_TWIM0, LSM330_ACC_SLAVE_ADDRESS) == STATUS_OK)
	{
		print_util_dbg_print("LSM330 sensor found (0x18) \r\n");
	}
	else
	{
		print_util_dbg_print("LSM330 sensor not responding (0x18) \r\n");
		return;
	} 
	
	lsm330dlc_acc_init();
	lsm330dlc_gyro_init();
	lsm330dlc_get_acc_config();
	lsm330dlc_get_gyro_config();
}

void lsm330dlc_acc_update(accelerometer_t *lsm_acc_outputs) 
{
	lsm_acc_read_t fifo_values;
	float axes[3] = {0.0f, 0.0f, 0.0f};
	
	fifo_values.start_address = LSM_ACC_OUT_ADDRESS | LSM_AUTO_INCREMENT ;

	///< read number of bytes in fifo
	lsm_read_fifo_fill_t read_fifo =
	{
		.fifo_fill = 1,
		.start_address = LSM_ACC_FIFO_SRC_ADDRESS
	};
	
	// Problem here: BUG with the accelerometer (attitude extremely non stable) ! ==> Skip the FIFO (set 1 in fifo_fill)
	lsm330dlc_acc_read_register((uint8_t*)&read_fifo.start_address, (uint8_t*)&read_fifo.fifo_fill, 1);
	
	if (read_fifo.fifo_fill > 0)
	{
		if (read_fifo.fifo_fill > 1)
		{
			read_fifo.fifo_fill = 1;
		}
		
		for (uint8_t i = 0; i < read_fifo.fifo_fill; i++)
		{
			lsm330dlc_acc_read_register((uint8_t*)&fifo_values.start_address, (uint8_t*)&fifo_values.status_register, 7);

			axes[0] += (float)((int16_t)fifo_values.axes[0]);
			axes[1] += (float)((int16_t)fifo_values.axes[1]);
			axes[2] += (float)((int16_t)fifo_values.axes[2]);
		}
		
		lsm_acc_outputs->data[0] = axes[0] / read_fifo.fifo_fill;
		lsm_acc_outputs->data[1] = axes[1] / read_fifo.fifo_fill;
		lsm_acc_outputs->data[2] = axes[2] / read_fifo.fifo_fill;
	}
}

void lsm330dlc_gyro_update(gyroscope_t *lsm_gyro_outputs) 
{
	lsm_gyro_read_t fifo_values;
	int16_t temperature = 0;
	float axes[3] = {0.0f, 0.0f, 0.0f};
	
	fifo_values.start_address = LSM_GYRO_OUT_ADDRESS | LSM_AUTO_INCREMENT;
	
	///< read number of bytes in fifo
	lsm_read_fifo_fill_t read_fifo = 
	{
		.fifo_fill = 1,
		.start_address = LSM_GYRO_FIFO_SRC_ADDRESS
	};
	
	// Problem here: BUG with the gyro ! ==> Skip the FIFO (set 1 in fifo_fill)
	lsm330dlc_gyro_read_register((uint8_t*)&read_fifo.start_address, (uint8_t*)&read_fifo.fifo_fill, 1);
	
	if (read_fifo.fifo_fill > 0)
	{
		if (read_fifo.fifo_fill > 1)
		{
			read_fifo.fifo_fill = 1;
		}
		
		for (uint8_t i = 0; i < read_fifo.fifo_fill; i++)
		{
			lsm330dlc_gyro_read_register((uint8_t*)&fifo_values.start_address, (uint8_t*)&fifo_values.temperature, 8);
			
			temperature += (int16_t)fifo_values.temperature;
			axes[0] += (float)((int16_t)fifo_values.axes[0]);
			axes[1] += (float)((int16_t)fifo_values.axes[1]);
			axes[2] += (float)((int16_t)fifo_values.axes[2]);
		}
		
		lsm_gyro_outputs->temperature = (int8_t)(temperature / read_fifo.fifo_fill);
		lsm_gyro_outputs->data[0] = axes[0] / read_fifo.fifo_fill;
		lsm_gyro_outputs->data[1] = axes[1] / read_fifo.fifo_fill;
		lsm_gyro_outputs->data[2] = axes[2] / read_fifo.fifo_fill;
	}
}
