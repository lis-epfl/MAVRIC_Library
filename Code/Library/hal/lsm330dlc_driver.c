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

#define LSM_ACC_OUT_ADDRESS				0x28	///< Define the writing address of the accelerometer

#define LSM_ACC_FIFO_CTRL_ADDRESS		0x2E	///< Define the address of the FIFO control register, for the accelerometer
#define LSM_ACC_FIFO_SRC_ADDRESS		0x2F	///< Define the address of the FIFO source(?) register, for the accelerometer

#define LSM_GYRO_CTRL_REG1_ADDRESS		0x20	///< Define the first control register address of the gyroscope

#define LSM_GYRO_OUT_ADDRESS			0x26	///< Define the writing address of the gyroscope

#define LSM_GYRO_FIFO_CTRL_ADDRESS		0x2E	///< Define the address of the FIFO control register, for the gyroscope
#define LSM_GYRO_FIFO_SRC_ADDRESS		0x2F	///< Define the address of the FIFO source(?) register, for the gyroscope

#define LSM_AUTO_INCREMENT				0x80	///< Define the auto incrementation of the LSM330DLC sensor


/**
 * \brief	Structure containing the accelerometer FIFO's data
*/
typedef struct
{
	uint8_t start_address;			///< Define the start Address of the accelerometer sensor
	int16_t axes[3];			///< Define an array containing the 3 axis of the accelerometer in the FIFO
	//uint8_t axes[32 * 6];			///< Define an array containing the 3 axis of the accelerometer in the FIFO
} lsm_acc_fifo_t;


/**
 * \brief	Structure containing the gyroscope FIFO's data
*/
typedef struct
{
	uint8_t start_address;			///< Define the start Address of the accelerometer sensor
	int8_t temperature;				///< Define the temperature of the sensor
	uint8_t status_register;		///< Define the status register of the FIFO of the gyroscope
	int16_t axes[3];			///< Define an array containing the 3 axis of the gyroscope in the FIFO
} lsm_gyro_fifo_t;


/**
 * \brief	Structure containing filling of the FIFO
*/
typedef struct
{
	uint8_t start_address;			///< Define the start Address of the FIFO register
	uint8_t fifo_fill;				///< Define the filling of the FIFO
} lsm_fifo_fill_t;


/**
 * \brief	Define the configuration of the accelerometer
*/
static const lsm330dlc_acc_conf_t lsm_acc_default_config=
{
	.start_address = LSM_ACC_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
	.ctrl_reg_a=
	{	
		LSM_ACC_DATARATE_400Hz | LSM_ACC_ALL_EN ,						///< CTRL_REG_G_1
		0,																///< CTRL_REG_G_2
		0,																///< CTRL_REG_G_3
		LSM_ACC_HIGH_RES | LSM_ACC_FULL_SCALE_8G  |LSM_ACC_BIG_ENDIAN,	///< CTRL_REG_G_4
		LSM_ACC_FIFO_EN													///< CTRL_REG_G_5
	}
};

/**
 * \brief	Define the configuration of the gyroscope
*/
static const lsm330dlc_gyro_conf_t lsm_gyro_default_config=
{
	.start_address = LSM_GYRO_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT,
	.ctrl_reg_g=
	{	
		LSM_GYRO_POWER_ON | LSM_GYRO_DATARATE_760Hz | LSM_GYRO_BANDWIDTH_50Hz | LSM_GYRO_ALL_EN,	///< CTRL_REG_A_1
		0,																							///< CTRL_REG_A_2
		0,																							///< CTRL_REG_A_3
		LSM_GYRO_FULL_SCALE_2000_2|LSM_GYRO_BIG_ENDIAN,												///< CTRL_REG_A_4
		0																							///< LSM_GYRO_FIFO_EN
	}
};

/**
 * \brief	Declare the configuration of the FIFO
*/
static const uint8_t fifo_config[2] = {LSM_ACC_FIFO_CTRL_ADDRESS, 0x80};


/**
 * \brief			Write data to one device of the LSM330, accelerometer or gyroscope  
 *
 * \param device	.
 * \param address	.
 * \param value		Value to write to the device
 */
static void	lsm330dlc_acc_write_register(uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Read data from one device of the LSM330, accelerometer or gyroscope 
 *
 * \param device	.
 * \param address	.
 *
 * \return			Value read from the device
 */
static void lsm330dlc_acc_read_register(uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Write data to one device of the LSM330, accelerometer or gyroscope  
 *
 * \param device	.
 * \param address	.
 * \param value		Value to write to the device
 */
static void	lsm330dlc_gyro_write_register(uint8_t* buffer, uint32_t nbytes);

/**
 * \brief			Read data from one device of the LSM330, accelerometer or gyroscope 
 *
 * \param device	.
 * \param address	.
 *
 * \return			Value read from the device
 */
static void lsm330dlc_gyro_read_register(uint8_t* buffer, uint32_t nbytes);

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
	twim_write(&AVR32_TWIM0, buffer, nbytes, LSM330_ACC_SLAVE_ADDRESS, false);
}

static void lsm330dlc_acc_read_register(uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, buffer, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)(buffer+1), nbytes-1, LSM330_ACC_SLAVE_ADDRESS, false);
}

static void lsm330dlc_gyro_write_register(uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, buffer, nbytes, LSM330_GYRO_SLAVE_ADDRESS, false);
}

static void lsm330dlc_gyro_read_register(uint8_t* buffer, uint32_t nbytes)
{
	twim_write(&AVR32_TWIM0, buffer, 1, LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)(buffer+1), nbytes-1, LSM330_GYRO_SLAVE_ADDRESS, false);
}

static void lsm330dlc_acc_init(void) 
{	
	lsm330dlc_acc_write_register((uint8_t*) &lsm_acc_default_config, sizeof(lsm_acc_default_config));
	lsm330dlc_acc_write_register((uint8_t*) &fifo_config, sizeof(fifo_config));
}

static void lsm330dlc_gyro_init(void) 
{
	lsm330dlc_gyro_write_register((uint8_t*) &lsm_gyro_default_config, sizeof(lsm_gyro_default_config));
	lsm330dlc_gyro_write_register((uint8_t*) &fifo_config, sizeof(fifo_config));
}

static void lsm330dlc_get_acc_config(void)
{
	int32_t i;
	/*
	uint8_t data_register_address = lsm_acc_default_config.start_address | LSM_AUTO_INCREMENT;
	uint8_t readbuffer[5];
	twim_transfer_t preamble;
	twim_transfer_t result;
	
	preamble.buffer = &data_register_address;
	preamble.chip = LSM330_ACC_SLAVE_ADDRESS;
	preamble.length = 1;
	preamble.read = 0;
	result.chip = LSM330_ACC_SLAVE_ADDRESS;
	result.buffer = &readbuffer;
	result.length = sizeof(readbuffer);
	result.read = 1;
	
	twim_write(&AVR32_TWIM0, &data_register_address, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, readbuffer, sizeof(readbuffer), LSM330_ACC_SLAVE_ADDRESS, false);
	*/
	lsm330dlc_acc_conf_t lsm_acc_get_config;
	lsm_acc_get_config.start_address = LSM_ACC_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT;
	
	lsm330dlc_acc_read_register((uint8_t*)&lsm_acc_get_config, sizeof(lsm_acc_get_config));
	
	print_util_dbg_print("lsm acc config:\n");
	for (i = 0; i < (sizeof(lsm_acc_get_config) - 1); i++) 
	{
		print_util_dbg_print_num(lsm_acc_get_config.ctrl_reg_a[i], 16); print_util_dbg_print(" (");
		print_util_dbg_print_num(lsm_acc_default_config.ctrl_reg_a[i], 16); print_util_dbg_print(")\n");
	}
}

static void lsm330dlc_get_gyro_config(void) 
{
	int32_t i;
	/*
	uint8_t data_register_address = lsm_acc_default_config.start_address | LSM_AUTO_INCREMENT;
	uint8_t readbuffer[5];
	twim_transfer_t preamble;
	twim_transfer_t result;
	
	preamble.buffer = &data_register_address;
	preamble.chip = LSM330_GYRO_SLAVE_ADDRESS;
	preamble.length = 1;
	preamble.read = 0;
	result.chip = LSM330_GYRO_SLAVE_ADDRESS;
	result.buffer = &readbuffer;
	result.length = sizeof(readbuffer);
	result.read = 1;
	
	twim_write(&AVR32_TWIM0, &data_register_address, 1, LSM330_GYRO_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, readbuffer, sizeof(readbuffer), LSM330_GYRO_SLAVE_ADDRESS, false);
	*/
	lsm330dlc_gyro_conf_t lsm_gyro_get_config;
	lsm_gyro_get_config.start_address = LSM_GYRO_CTRL_REG1_ADDRESS | LSM_AUTO_INCREMENT;
	
	lsm330dlc_gyro_read_register((uint8_t*)&lsm_gyro_get_config, sizeof(lsm_gyro_get_config));
	
	print_util_dbg_print("lsm gyro config:\n");
	for (i = 0; i < (sizeof(lsm_gyro_get_config) - 1); i++)
	{
		print_util_dbg_print_num(lsm_gyro_get_config.ctrl_reg_g[i], 16); print_util_dbg_print(" (");
		print_util_dbg_print_num(lsm_gyro_default_config.ctrl_reg_g[i], 16); print_util_dbg_print(")\n");
	}
}

void lsm330dlc_driver_init(void) 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz = 64000000,
		.speed = 400000,
		.chip = LSM330_ACC_SLAVE_ADDRESS,
		.smbus = false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	lsm330dlc_acc_init();
	lsm330dlc_gyro_init();
	lsm330dlc_get_acc_config();
	lsm330dlc_get_gyro_config();
}

void lsm330dlc_acc_update(accelero_data_t *lsm_acc_outputs) 
{
	
	lsm_acc_fifo_t fifo_values = { .start_address = LSM_ACC_OUT_ADDRESS | LSM_AUTO_INCREMENT };
	int32_t axes[3] = {0,0,0};
	uint8_t i;
	
	///< read number of bytes in fifo
	lsm_fifo_fill_t read_fifo = {LSM_ACC_FIFO_SRC_ADDRESS, 0};
	lsm330dlc_acc_read_register((uint8_t*)&read_fifo, sizeof(read_fifo));
	
	if (read_fifo.fifo_fill > 6)
	{
		read_fifo.fifo_fill = 6;
	}
	else if (read_fifo.fifo_fill > 0)
	{
		for (i = 0; i < read_fifo.fifo_fill; i++)
		{
			lsm330dlc_acc_read_register((uint8_t*)&fifo_values, sizeof(fifo_values));
			axes[0]+=fifo_values.axes[0];
			axes[1]+=fifo_values.axes[1];
			axes[2]+=fifo_values.axes[2];
		}
		
		lsm_acc_outputs->raw_data[0] = (int16_t)(axes[0] / read_fifo.fifo_fill);
		lsm_acc_outputs->raw_data[1] = (int16_t)(axes[1] / read_fifo.fifo_fill);
		lsm_acc_outputs->raw_data[2] = (int16_t)(axes[2] / read_fifo.fifo_fill);
	}
	
	/* // OLD
	uint8_t data_register_address = LSM_ACC_OUT_ADDRESS | LSM_AUTO_INCREMENT;
	int32_t twim_return;
	lsm_acc_fifo_t fifo_values;
	int32_t axes[3] = {0,0,0};
	int32_t i;
	uint8_t fifo_fill = 1;
	
	///< read number of bytes in fifo
	//fifo_fill = lsm_read_register(LSM330_ACC_SLAVE_ADDRESS, LSM_ACC_FIFO_SRC_ADDRESS) & 0x0f - 1;
	
	if (fifo_fill == 0) 
	{
		fifo_fill = 1; 
		//return &lsm_acc_outputs;
	}
	if (fifo_fill > 6) 
	{
		fifo_fill = 6;
	}
	
	twim_return = twim_write(&AVR32_TWIM0, &data_register_address, 1, LSM330_ACC_SLAVE_ADDRESS, false);
	twim_return = twim_read(&AVR32_TWIM0, (uint8_t*)&fifo_values.axes, 6 * fifo_fill, LSM330_ACC_SLAVE_ADDRESS, false);

	for (i = 0; i < fifo_fill; i++) 
	{
		axes[0]+=fifo_values.axes[3 * i];
		axes[1]+=fifo_values.axes[3 * i + 1];
		axes[2]+=fifo_values.axes[3 * i + 2];
		//axes[0]+=(int32_t)fifo_values.axes[6 * i + 1] * 256 + fifo_values.axes[6 * i];
		//axes[1]+=(int32_t)fifo_values.axes[6 * i + 3] * 256 + fifo_values.axes[6 * i + 2];
		//axes[2]+=(int32_t)fifo_values.axes[6 * i + 5] * 256 + fifo_values.axes[6 * i + 4];
	}
	
	lsm_acc_outputs.axes[0] = (int16_t)(axes[0] / fifo_fill);
	lsm_acc_outputs.axes[1] = (int16_t)(axes[1] / fifo_fill);
	lsm_acc_outputs.axes[2] = (int16_t)(axes[2] / fifo_fill);
	*/
}

void lsm330dlc_gyro_update(gyro_data_t *lsm_gyro_outputs) 
{
	lsm_gyro_fifo_t fifo_values = { .start_address = LSM_GYRO_OUT_ADDRESS | LSM_AUTO_INCREMENT };
	int32_t temperature = 0;
	int32_t axes[3] = {0,0,0};
	uint8_t i;
	///< read number of bytes in fifo
	lsm_fifo_fill_t read_fifo = {LSM_GYRO_FIFO_SRC_ADDRESS, 0};
	
	lsm330dlc_gyro_read_register((uint8_t*)&read_fifo, sizeof(read_fifo));
	
	if (read_fifo.fifo_fill > 6)
	{
		read_fifo.fifo_fill = 6;
	}
	else if (read_fifo.fifo_fill > 0) 
	{
		for (i = 0; i < read_fifo.fifo_fill; i++)
		{
			lsm330dlc_gyro_read_register((uint8_t*)&fifo_values, sizeof(fifo_values));
			temperature+=fifo_values.temperature;
			axes[0]+=fifo_values.axes[0];
			axes[1]+=fifo_values.axes[1];
			axes[2]+=fifo_values.axes[2];
		}
		
		lsm_gyro_outputs->temperature = (int16_t)(temperature / read_fifo.fifo_fill);
		lsm_gyro_outputs->raw_data[0] = (int16_t)(axes[0] / read_fifo.fifo_fill);
		lsm_gyro_outputs->raw_data[1] = (int16_t)(axes[1] / read_fifo.fifo_fill);
		lsm_gyro_outputs->raw_data[2] = (int16_t)(axes[2] / read_fifo.fifo_fill);
	}
}