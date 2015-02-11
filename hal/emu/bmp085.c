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
 * \file bmp085.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the barometer module: BMP085
 * 
 ******************************************************************************/


#include "bmp085.h"

// #include "twim.h"
#include "time_keeper.h"

#include "math.h"
#include "maths.h"
#include "time_keeper.h"
#include "print_util.h"


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


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void bmp085_init(barometer_t *bmp085)
{
	bmp085->altitude_offset = 0.0f;

	for (int32_t i = 0; i < 3; i++) 
	{
		bmp085->last_altitudes[i] = 0.0f;
	}
	
	bmp085->vario_vz = 0.0f;
	bmp085_init_slow();
}


void bmp085_init_slow()
{	
	print_util_dbg_print("[BMP085] Not implemented in emulation \r\n");
}


void bmp085_reset_origin_altitude(barometer_t* bmp085, float origin_altitude)
{
	bmp085->altitude_offset = - ( bmp085->altitude - bmp085->altitude_offset - origin_altitude );
}

 
void bmp085_update(barometer_t *bmp085) 
{
	;
}