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
#include <stdbool.h>
#include "print_util.h"

pressure_data pressure_outputs;		///< declare an object containing the barometer's data

int16_t bmp085_read_int(unsigned char address) {
	int16_t result;
	twim_write(&AVR32_TWIM0, (uint8_t*) &address, 1, BMP085_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&(result), 2, BMP085_SLAVE_ADDRESS, false);
	return result;
}

void init_bmp085()
{
	pressure_outputs.altitude_offset = 0.0f;
	for (int i = 0; i < 3; i++) 
	{
		pressure_outputs.last_altitudes[i] = 0.0f;
	}
	pressure_outputs.vario_vz = 0.0f;
	init_bmp085_slow();
}

///< Declare configuration values for the barometer, given by the datasheet of the sensor
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;		
uint16_t ac4, ac5, ac6;

void init_bmp085_slow(){
	static twim_options_t twi_opt= 
	{
		.pba_hz = 64000000,
		.speed = 400000,
		.chip = BMP085_SLAVE_ADDRESS,
		.smbus = false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);

	if (twim_probe(&AVR32_TWIM0, BMP085_SLAVE_ADDRESS) == STATUS_OK) 
	{
		dbg_print("BMP85/180 pressure sensor found (0x77)\n");
	} 
	else 
	{
		dbg_print("BMP85/180 pressure sensor not responding (0x77)\n");
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
}

 


pressure_data* get_pressure_data_slow(float offset) 
{
		int i;
		float altitude, vertical_speed;
		int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
		uint32_t B4, B7;
			
		uint8_t start_address;
		
		uint8_t start_command_temp [] = {BMP085_CONTROL, BMP085_READTEMPCMD};
		uint8_t start_command_pressure [] = {BMP085_CONTROL, BMP085_READPRESSURECMD+ (BMP085_OVERSAMPLING_MODE << 6)};
		int32_t sealevelPressure=101325;
		float dt;
		///< calibration: use datasheet numbers!

		switch (pressure_outputs.state) 
		{
			case IDLE:
				twim_write(&AVR32_TWIM0, (uint8_t*) &start_command_temp, 2, BMP085_SLAVE_ADDRESS, false);
				//delay_ms(7);
				pressure_outputs.state=GET_TEMP;
				break;
			case GET_TEMP:
				start_address = BMP085_TEMPDATA;
				twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS, false);
				twim_read(&AVR32_TWIM0, (uint8_t*)&(pressure_outputs.raw_temperature), 2, BMP085_SLAVE_ADDRESS, false);
		
				twim_write(&AVR32_TWIM0, (uint8_t*) &start_command_pressure, 2, BMP085_SLAVE_ADDRESS, false);
				pressure_outputs.state=GET_PRESSURE;
				//delay_ms(15);
				break;
			case GET_PRESSURE:
				start_address = BMP085_PRESSUREDATA;
				twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS, false);
				twim_read(&AVR32_TWIM0, (uint8_t*)&(pressure_outputs.raw_pressure), 3, BMP085_SLAVE_ADDRESS, false);
		
				UP= ((uint32_t)pressure_outputs.raw_pressure[0] << 16 |(uint32_t)pressure_outputs.raw_pressure[1] << 8 | (uint32_t)pressure_outputs.raw_pressure[2]) >> (8 - BMP085_OVERSAMPLING_MODE);
 
				UT = pressure_outputs.raw_temperature[0] << 8 |pressure_outputs.raw_temperature[1];
				///< step 1
				X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
				X2 = ((int32_t)mc * pow(2,11)) / (X1 + (int32_t)md);
				B5 = X1 + X2;
				pressure_outputs.temperature = (B5 + 8) / pow(2,4);
				pressure_outputs.temperature /= 10;
	
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

				pressure_outputs.pressure=p;
		
				vertical_speed = pressure_outputs.altitude;
				altitude=44330.0f * (1.0f - pow(pressure_outputs.pressure /sealevelPressure,0.190295f)) + pressure_outputs.altitude_offset;
			
				for (i = 0; i < 2; i++) 
				{
					pressure_outputs.last_altitudes[i] = pressure_outputs.last_altitudes[i + 1];
				}
				pressure_outputs.last_altitudes[2] = altitude;
				altitude=median_filter_3x(pressure_outputs.last_altitudes[0], pressure_outputs.last_altitudes[1], pressure_outputs.last_altitudes[2]);
			
				if (f_abs(altitude-pressure_outputs.altitude) < 15.0f) 
				{
					pressure_outputs.altitude = (BARO_ALT_LPF * pressure_outputs.altitude) + (1.0f - BARO_ALT_LPF) * altitude;
				}
				else 
				{
					pressure_outputs.altitude = altitude;
				}
			
				dt = (get_micros() - pressure_outputs.last_update) / 1000000.0f;
				pressure_outputs.dt = dt;
				vertical_speed = -(pressure_outputs.altitude-vertical_speed) / dt;
			
				if (abs(vertical_speed) > 20) 
				{
					vertical_speed = 0.0f;
				}
				pressure_outputs.vario_vz = (VARIO_LPF) * pressure_outputs.vario_vz + (1.0f - VARIO_LPF) * (vertical_speed);
			
				pressure_outputs.last_update = get_micros();
				pressure_outputs.state = IDLE;
				break;
		}
		pressure_outputs.last_state_update=get_micros();
		pressure_outputs.altitude_offset = offset;
		
		return &pressure_outputs;
}

bool newValidBarometer(uint32_t *timePrevBarometer)
{
	if (*timePrevBarometer < pressure_outputs.last_update) 
	{
		*timePrevBarometer = pressure_outputs.last_update;
		return true;
	}
	else
	{
		return false;
	}
	
}