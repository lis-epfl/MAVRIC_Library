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
* \file bmp085.c
*
* This file is the driver for the barometer module: BMP085
*/


#include "bmp085.h"

#include "twim.h"
#include "delay.h"

#include "math.h"
#include "maths.h"
#include "time_keeper.h"
#include "print_util.h"
#include "mavlink_communication.h"


#define BARO_ALT_LPF 0.95f						///< low pass filter factor for altitude measured by the barometer
#define VARIO_LPF 0.95f							///< low pass filter factor for the Vario altitude speed

#define BMP085_SLAVE_ADDRESS 0x77				///< Address of the barometer sensor on the i2c bus

#define BMP085_ULTRALOWPOWER 0					///< Ultra low power mode
#define BMP085_STANDARD 1						///< standard mode
#define BMP085_HIGHRES 2						///< high resolution mode
#define BMP085_ULTRAHIGHRES 3					///< ultra high resolution mode

#define BMP085_CAL_AC1 0xAA						///< R Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC						///< R Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE						///< R Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0						///< R Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2						///< R Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4						///< R Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6						///< R Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8						///< R Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA						///< R Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC						///< R Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE						///< R Calibration data (16 bits)

#define BMP085_CONTROL 0xF4						///< Control register of the barometer sensor 
#define BMP085_TEMPDATA 0xF6					///< Temperature register of the barometer sensor 
#define BMP085_PRESSUREDATA 0xF6				///< Pressure Data register of the barometer sensor 
#define BMP085_READTEMPCMD 0x2E					///< Read temperature Command register of the barometer sensor 
#define BMP085_READPRESSURECMD 0x34				///< Read Pressure Command register of the barometer sensor 

#define BMP085_OVERSAMPLING_MODE BMP085_HIGHRES	///< Set oversampling mode of the barometer sensor to high resolution mode

///< Declare configuration values for the barometer, given by the datasheet of the sensor
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;		
uint16_t ac4, ac5, ac6;


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

static int16_t bmp085_read_int(uint8_t address);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static int16_t bmp085_read_int(uint8_t address) 
{
	int16_t result;

	twim_write(&AVR32_TWIM0, &address, 1, BMP085_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(result), 2, BMP085_SLAVE_ADDRESS, false);
	
	return result;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void bmp085_init(barometer_t *baro, const mavlink_stream_t* mavlink_stream)
{
	baro->mavlink_stream = mavlink_stream;
	baro->altitude_offset = 0.0f;

	for (int32_t i = 0; i < 3; i++) 
	{
		baro->last_altitudes[i] = 0.0f;
	}
	
	baro->vario_vz = 0.0f;
	bmp085_init_slow();
}


void bmp085_init_slow()
{	
	if (twim_probe(&AVR32_TWIM0, BMP085_SLAVE_ADDRESS) == STATUS_OK)
	{
		print_util_dbg_print("BMP85 pressure sensor found (0x77) \r");
	}
	else
	{
		print_util_dbg_print("BMP85 pressure sensor not responding (0x77) \r");
		return;
	}
	
	///< Configure with datasheet values
	ac1 = 408;
	ac2 = -72;
	ac3 = -14383;
	ac4 = 32741;
	ac5 = 32757;
	ac6 = 23153;
	mc = -8711;
	md = 2868;
	b1 = 6190;
	b2 = 4;
	
	///< Try to update with Eprom values of the sensor
	ac1 = bmp085_read_int(0xAA);
	ac2 = bmp085_read_int(0xAC);
	ac3 = bmp085_read_int(0xAE);
	ac4 = bmp085_read_int(0xB0);
	ac5 = bmp085_read_int(0xB2);
	ac6 = bmp085_read_int(0xB4);
	b1 = bmp085_read_int(0xB6);
	b2 = bmp085_read_int(0xB8);
	mb = bmp085_read_int(0xBA);
	mc = bmp085_read_int(0xBC);
	md = bmp085_read_int(0xBE);
	
	print_util_dbg_print("BMP085 initialised \r");
}


void bmp085_reset_origin_altitude(barometer_t* baro, float origin_altitude)
{
	baro->altitude_offset = - ( baro->altitude - baro->altitude_offset - origin_altitude );
}

 
void bmp085_update(barometer_t *baro) 
{
	int32_t i;
	float altitude, vertical_speed;
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
		
	uint8_t start_address;
	
	uint8_t start_command_temp [] = 
	{ 
		BMP085_CONTROL, 
		BMP085_READTEMPCMD 
	};
	
	uint8_t start_command_pressure [] = 
	{ 
		BMP085_CONTROL, 
		BMP085_READPRESSURECMD + (BMP085_OVERSAMPLING_MODE << 6) 
	};

	int32_t sealevelPressure=101325;
	float dt;

	switch (baro->state) 
	{
		case IDLE:
			twim_write(&AVR32_TWIM0, (uint8_t*) &start_command_temp, 2, BMP085_SLAVE_ADDRESS, false);
			baro->state=GET_TEMP;
			break;

		case GET_TEMP:
			start_address = BMP085_TEMPDATA;
			twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS, false);
			twim_read(&AVR32_TWIM0, (uint8_t*)&(baro->raw_temperature), 2, BMP085_SLAVE_ADDRESS, false);
	
			twim_write(&AVR32_TWIM0, (uint8_t*) &start_command_pressure, 2, BMP085_SLAVE_ADDRESS, false);
			baro->state=GET_PRESSURE;
			break;

		case GET_PRESSURE:
			start_address = BMP085_PRESSUREDATA;
			twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS, false);
			twim_read(&AVR32_TWIM0, (uint8_t*)&(baro->raw_pressure), 3, BMP085_SLAVE_ADDRESS, false);
	
			UP= ((uint32_t)baro->raw_pressure[0] << 16 |(uint32_t)baro->raw_pressure[1] << 8 | (uint32_t)baro->raw_pressure[2]) >> (8 - BMP085_OVERSAMPLING_MODE);

			UT = baro->raw_temperature[0] << 8 |baro->raw_temperature[1];
			
			///< step 1
			X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
			X2 = ((int32_t)mc * pow(2,11)) / (X1 + (int32_t)md);
			B5 = X1 + X2;
			baro->temperature = (B5 + 8) / pow(2,4);
			baro->temperature /= 10;

			///< do pressure calcs
			B6 = B5 - 4000;
			X1 = ((int32_t)b2 * ( (B6 * B6) >> 12 )) >> 11;
			X2 = ((int32_t)ac2 * B6) >> 11;
			X3 = X1 + X2;
			B3 = ((((int32_t)ac1 * 4 + X3) << BMP085_OVERSAMPLING_MODE) + 2) / 4;


			X1 = ((int32_t)ac3 * B6) >> 13;
			X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
			X3 = ((X1 + X2) + 2) >> 2;
			B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
			B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> BMP085_OVERSAMPLING_MODE );


			if (B7 < 0x80000000) 
			{
				p = (B7 * 2) / B4;
			} 
			else 
			{
				p = (B7 / B4) * 2;
			}
		
			X1 = (p >> 8) * (p >> 8);
			X1 = (X1 * 3038) >> 16;
			X2 = ( - 7357 * p) >> 16;

			p = p + ((X1 + X2 + (int32_t)3791) >> 4);

			baro->pressure = p;
	
			vertical_speed = baro->altitude;
			altitude = 44330.0f * (1.0f - pow(baro->pressure /sealevelPressure,0.190295f)) + baro->altitude_offset;
		
			for (i = 0; i < 2; i++) 
			{
				baro->last_altitudes[i] = baro->last_altitudes[i + 1];
			}
			baro->last_altitudes[2] = altitude;
			altitude = maths_median_filter_3x(baro->last_altitudes[0], baro->last_altitudes[1], baro->last_altitudes[2]);
		
			if (maths_f_abs(altitude-baro->altitude) < 15.0f) 
			{
				baro->altitude = (BARO_ALT_LPF * baro->altitude) + (1.0f - BARO_ALT_LPF) * altitude;
			}
			else 
			{
				baro->altitude = altitude;
			}
		
			dt = (time_keeper_get_micros()-baro->last_update) / 1000000.0f;
			baro->dt = dt;
			vertical_speed = -(baro->altitude-vertical_speed) / dt;
		
			if (abs(vertical_speed) > 20) 
			{
				vertical_speed = 0.0f;
			}

			baro->vario_vz = (VARIO_LPF) * baro->vario_vz + (1.0f - VARIO_LPF) * (vertical_speed);
		
			baro->last_update = time_keeper_get_micros();
			baro->state = IDLE;
			break;
	}

	baro->last_state_update = time_keeper_get_micros();
}


task_return_t bmp085_send_pressure(barometer_t* baro)
{
	mavlink_message_t msg;
	
	mavlink_msg_scaled_pressure_pack(	baro->mavlink_stream->sysid,
										baro->mavlink_stream->compid,
										&msg,
										time_keeper_get_millis(),
										baro->pressure / 100.0f,
										baro->vario_vz,
										baro->temperature * 100.0f);
	
	mavlink_stream_send(baro->mavlink_stream,&msg);

	// mavlink_msg_named_value_float_pack(	baro->mavlink_stream->sysid,
	// 									baro->mavlink_stream->compid,
	// 									&msg,
	// 									time_keeper_get_millis(),
	// 									"pressAlt",
	// 									baro->altitude);

	// mavlink_stream_send(baro->mavlink_stream,&msg);

	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"lastAlt",
										//baro->last_altitudes[0]);

	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"baro_dt",
										//baro->dt);
	return TASK_RUN_SUCCESS;
}