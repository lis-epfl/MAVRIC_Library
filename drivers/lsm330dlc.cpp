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


#include "lsm330dlc.hpp"

extern "C"
{
	#include "twim.h"
	#include "print_util.h"
}

const uint8_t LSM330_ACC_SLAVE_ADDRESS		= 	0b0011000;	///< Define the Accelerometer Address, as a slave on the i2c bus
const uint8_t LSM330_GYRO_SLAVE_ADDRESS		= 	0b1101010;	///< Define the Gyroscope Address, as a slave on the i2c bus

const uint8_t LSM_GYRO_DEN_PIN 				= 	AVR32_PIN_PD23;			///< Define the microcontroller pin to enable the gyroscope's data (Data EN)

enum
{
	LSM_ACC_DATARATE_OFF,			
	LSM_ACC_DATARATE_1HZ,			
	LSM_ACC_DATARATE_10Hz,			
	LSM_ACC_DATARATE_25Hz,			
	LSM_ACC_DATARATE_50Hz,			
	LSM_ACC_DATARATE_100Hz,		
	LSM_ACC_DATARATE_200Hz,		
	LSM_ACC_DATARATE_400Hz,		
	LSM_ACC_DATARATE_1620Hz,		
	LSM_ACC_DATARATE_1344_5376Hz	
};

enum
{
	LSM_ACC_FULL_SCALE_2G,	
	LSM_ACC_FULL_SCALE_4G,	
	LSM_ACC_FULL_SCALE_8G,	
	LSM_ACC_FULL_SCALE_16G
};

enum
{
	LSM_GYRO_FULL_SCALE_250,
	LSM_GYRO_FULL_SCALE_500,	
	LSM_GYRO_FULL_SCALE_2000_1,
	LSM_GYRO_FULL_SCALE_2000_2
};

enum
{
	LSM_GYRO_BANDWIDTH_20Hz,
	LSM_GYRO_BANDWIDTH_25Hz,
	LSM_GYRO_BANDWIDTH_50Hz,
	LSM_GYRO_BANDWIDTH_100Hz
};

///< CTRL_REG_A_1
#define LSM_ACC_DATARATE 	(LSM_ACC_DATARATE_400Hz		<< 4)		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_FULL_SCALE 	(LSM_ACC_FULL_SCALE_8G 		<< 4)		///< Define the Accelerometer control register A4
#define LSM_GYRO_FULL_SCALE (LSM_GYRO_FULL_SCALE_2000_2 << 4)		///< Define the range of the gyroscope sensor
#define LSM_GYRO_BANDWIDTH 	(LSM_GYRO_BANDWIDTH_50Hz 	<< 6)		///< note: actual bandwidth depends on datarate - specified for 380Hz. Consult Datasheet.

const uint8_t LSM_ALL_EN					=	0x07;	///< Enable the accelerometer and gyroscope 3 axis
const uint8_t LSM_POWER_EN					=	0x08;	///< Enable the acclerometer and gyroscope power
const uint8_t LSM_BIG_ENDIAN				=	0x40;	///< Define the Accelerometer and Gyroscope control register A4
const uint8_t LSM_SPI_MODE					=	0x01;	///< Define the mode (I2C/SPI) of the accelerometer sensor
const uint8_t LSM_FIFO_EN					=	0x40;	///< Define the Accelerometer control register A5
const uint8_t LSM_CTRL_REG1_ADDRESS			=	0x20;	///< Define the first control register address
const uint8_t LSM_OUT_ADDRESS				=	0x27;	///< Define the writing address
const uint8_t LSM_FIFO_CTRL_ADDRESS			=	0x2E;	///< Define the address of the FIFO control register
const uint8_t LSM_FIFO_SRC_ADDRESS			=	0x2F;	///< Define the address of the FIFO source(?) register


const uint8_t LSM_AUTO_INCREMENT			=	0x80;	///< Define the auto incrementation of the LSM330DLC sensor

///< CTRL_REG_A_2
const uint8_t LSM_ACC_HPIS1					=	0x01;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPIS2					=	0x02;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPCLICK				=	0x04;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_FDS					=	0x08;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPCF1					=	0x10;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPCF2					=	0x20;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPM0					=	0x40;	///< Define the Accelerometer control register A2
const uint8_t LSM_ACC_HPM1					=	0x80;	///< Define the Accelerometer control register A2

///< CTRL_REG_A_3
const uint8_t LSM_ACC_OVERRUN_INT			=	0x02;	///< Define the Accelerometer control register A3
const uint8_t LSM_ACC_FIFO_WM_INT			=	0x04;	///< Define the Accelerometer control register A3
const uint8_t LSM_ACC_DRDY2_INT				=	0x08;	///< Define the Accelerometer control register A3
const uint8_t LSM_ACC_DRDY1_INT				=	0x10;	///< Define the Accelerometer control register A3
const uint8_t LSM_ACC_AOI_INT				=	0x40;	///< Define the Accelerometer control register A3
const uint8_t LSM_ACC_CLICK_INT				=	0x80;	///< Define the Accelerometer control register A3

///< CTRL_REG_A_4
const uint8_t LSM_ACC_HIGH_RES				=	0x08;	///< Define the Accelerometer control register A4

///< CTRL_REG_A_5
const uint8_t LSM_ACC_D4D_INT				=	0x04;	///< Define the Accelerometer control register A5
const uint8_t LSM_ACC_LIR_INT				=	0x08;	///< Define the Accelerometer control register A5
const uint8_t LSM_ACC_BOOT					=	0x80;	///< Define the Accelerometer control register A5

///< CTRL_REG1_G
const uint8_t LSM_GYRO_DATARATE_95HZ		=	0x00;	///< Define the frequency of the gyroscope data rate
const uint8_t LSM_GYRO_DATARATE_190HZ		=	0x40;	///< Define the frequency of the gyroscope data rate
const uint8_t LSM_GYRO_DATARATE_380Hz		=	0x80;	///< Define the frequency of the gyroscope data rate
const uint8_t LSM_GYRO_DATARATE_760Hz		=	0xC0;	///< Define the frequency of the gyroscope data rate


///< CTRL_REG2_G
const uint8_t LSM_GYRO_HPCF0				=	0x01;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_HPCF1				=	0x02;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_HPCF2				=	0x04;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_HPCF3				=	0x08;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_HPM0					=	0x10;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_HPM1					=	0x20;	///< Define the gyroscope control register G2
const uint8_t LSM_GYRO_LVL_EN				=	0x40;	///< Enable the gyroscope control register G2
const uint8_t LSM_GYRO_EXTREN				=	0x80;	///< Define the gyroscope control register G2

///< CTRL_REG3_G
const uint8_t LSM_GYRO_FIFO_EMPTY_INT		=	0x01;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_FIFO_OVRUN_INT		=	0x02;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_FIFO_WM_INT			=	0x04;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_DRDY_INT				=	0x08;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_PP_OD				=	0x10;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_H_L_ACT				=	0x20;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_I1_BOOT				=	0x40;	///< Define the gyroscope control register G3
const uint8_t LSM_GYRO_I1_INT1				=	0x80;	///< Define the gyroscope control register G3

///< CTRL_REG4_G
const uint8_t LSM_GYRO_BLOCK_DATA			=	0x80;	///< Define the block transmission mode of the gyroscope

///< CTRL_REG5_G
const uint8_t LSM_OUT_SEL0					=	0x01;	///< Define the gyroscope control register G5
const uint8_t LSM_OUT_SEL1					=	0x02;	///< Define the gyroscope control register G5
const uint8_t LSM_INT_SEL0					=	0x04;	///< Define the gyroscope control register G5
const uint8_t LSM_INT_SEL1					=	0x08;	///< Define the gyroscope control register G5
const uint8_t LSM_GYRO_HP_EN				=	0x10;	///< Define the gyroscope control register G5
const uint8_t LSM_GYRO_BOOT					=	0x80;	///< Define the gyroscope control register G5

const uint8_t LSM_ACC_DATA_BEGIN			=	0xA7;	///< Define the begin address of the accelerometer
const uint8_t LSM_GYRO_DATA_BEGIN			=	0xA6;	///< Define the begin address of the gyroscope

/**
 * \brief	Structure containing filling of the FIFO. WARNING: start_address must 8-bits and the LAST element.
 */
typedef struct
{
	uint8_t fifo_fill;			///< Define the filling of the FIFO
	uint8_t start_address;		///< Define the start Address of the FIFO register
} lsm_read_fifo_fill_t;


/**
 * \brief	Declare the configuration of the FIFO
*/
static const uint8_t fifo_config[2] = {LSM_FIFO_CTRL_ADDRESS, 0x80};

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Lsm330dlc::Lsm330dlc(I2c& i2c, accelerometer_t& accel_data, gyroscope_t& gyro_data):
	i2c_(i2c),
	accel_data_(accel_data),
	gyro_data_(gyro_data)
{}

bool Lsm330dlc::init(void) 
{
	bool res = true;

	// fifo_fill if the sensor if here
	res &= i2c_.probe(LSM330_ACC_SLAVE_ADDRESS);
	res &= i2c_.probe(LSM330_GYRO_SLAVE_ADDRESS);

	if(res == true)
	{
		print_util_dbg_print("LSM330 sensor found (0x18) \r\n");
	}
	else
	{
		print_util_dbg_print("LSM330 sensor not responding (0x18) \r\n");
		return res;
	} 
	
	//Define the configuration of the accelerometer
	static const uint8_t lsm_acc_default_config[6] =
	{
		LSM_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
		LSM_ACC_DATARATE | LSM_ALL_EN ,									///< CTRL_REG_G_1
		0,																///< CTRL_REG_G_2
		0,																///< CTRL_REG_G_3
		LSM_ACC_HIGH_RES | LSM_ACC_FULL_SCALE | LSM_BIG_ENDIAN,			///< CTRL_REG_G_4
		LSM_FIFO_EN														///< CTRL_REG_G_5
	};

	// Write configuration to the sensor
	res &= i2c_.write(lsm_acc_default_config, 6, LSM330_ACC_SLAVE_ADDRESS);
	res &= i2c_.write(fifo_config, 2, LSM330_ACC_SLAVE_ADDRESS);

	//brief	Define the configuration of the gyroscope
	static const uint8_t lsm_gyro_default_config[6] =
	{
		LSM_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
		LSM_POWER_EN | LSM_GYRO_DATARATE_760Hz | LSM_GYRO_BANDWIDTH | LSM_ALL_EN,		///< CTRL_REG_A_1
		0,																				///< CTRL_REG_A_2
		0,																				///< CTRL_REG_A_3
		LSM_GYRO_FULL_SCALE | LSM_BIG_ENDIAN,											///< CTRL_REG_A_4
		LSM_FIFO_EN
	};
	
	// Write configuration to the sensor
	res &= i2c_.write(lsm_gyro_default_config, 6, LSM330_GYRO_SLAVE_ADDRESS);
	res &= i2c_.write(fifo_config, 2, LSM330_GYRO_SLAVE_ADDRESS);

	return res;
}

bool Lsm330dlc::update(void) 
{
	bool res 				= true;	
	
	uint8_t fifo_fill 		= 1;
	uint8_t accel_data[7]	= {0, 0, 0, 0, 0, 0, 0};
	uint16_t gyro_data[4] 	= {0, 0, 0, 0};

	res &= i2c_.write( &LSM_FIFO_SRC_ADDRESS, 1, LSM330_ACC_SLAVE_ADDRESS );
	res &= i2c_.read( (uint8_t*)&fifo_fill, 1, LSM330_ACC_SLAVE_ADDRESS );

	if (fifo_fill > 0)
	{
		res &= i2c_.write( &LSM_ACC_DATA_BEGIN, 1, LSM330_ACC_SLAVE_ADDRESS );
		res &= i2c_.read( (uint8_t*)accel_data, 7, LSM330_ACC_SLAVE_ADDRESS );
		//First Byte is the status register
		accel_data_.data[0] = (float)((int16_t)(accel_data[2] << 8 | accel_data[1]));
		accel_data_.data[1] = (float)((int16_t)(accel_data[4] << 8 | accel_data[3]));
		accel_data_.data[2] = (float)((int16_t)(accel_data[6] << 8 | accel_data[5]));

		res &= i2c_.write( &LSM_GYRO_DATA_BEGIN, 1, LSM330_GYRO_SLAVE_ADDRESS );
		res &= i2c_.read( (uint8_t*)gyro_data, 8, LSM330_GYRO_SLAVE_ADDRESS );
		//First Byte is the sensor temperature and Second Byte is the status register
		gyro_data_.data[0] = (float)((int16_t)gyro_data[1]);
		gyro_data_.data[1] = (float)((int16_t)gyro_data[2]);
		gyro_data_.data[2] = (float)((int16_t)gyro_data[3]);
	}

	return res;
}
