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
 * \file curvace.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief Driver for the cylindrical curvace
 *
 ******************************************************************************/

 #include "curvace.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

static void curvace_read_spi(curvace_t* curvace);


static void curvace_derotate_all(curvace_t* curvace);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void curvace_read_spi(curvace_t* curvace)
{
	// Left hemisphere
	for (uint8_t i = 0; i < CURVACE_NB_OF / 2; ++i)
	{
		curvace->raw_of.left_hemisphere[i].x =  0;  // spi_read_16_bits(...)
		curvace->raw_of.left_hemisphere[i].y =  0;  // spi_read_16_bits(...)
	}

	// Right hemisphere
	for (uint8_t i = 0; i < CURVACE_NB_OF / 2; ++i)
	{
		curvace->raw_of.right_hemisphere[i].x =  0;  // spi_read_16_bits(...)
		curvace->raw_of.right_hemisphere[i].y =  0;  // spi_read_16_bits(...)
	}
}


static void curvace_derotate_all(curvace_t* curvace)
{
	uint32_t j = 0;
	for (uint8_t i = 0; i < 2 * CURVACE_NB_OF; ++i)
	{
		curvace->of.data[i] = 	curvace->scale_factor * curvace->raw_of.data[i] 
								- curvace->calib.data[j] * curvace->ahrs->angular_speed[0]
								- curvace->calib.data[j + 1] * curvace->ahrs->angular_speed[1]
								- curvace->calib.data[j + 2] * curvace->ahrs->angular_speed[2];
		j += 3;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void curvace_init(curvace_t* curvace, const ahrs_t* ahrs)
{
	// Init dependencies
	curvace->ahrs = ahrs;

	// Init scale factor
	float range = 32768;			// if OF vectors are encoded using full int16_t: -1..1 maps to -32767..32768  
									// TODO: check this

	float inter_ommatidia = 4.2;	// 4.2 degrees between each ommatidia
	curvace->scale_factor = 200 * ( inter_ommatidia * 3.14 / 180)  / range;

	// Init calib
	for (uint8_t i = 0; i < CURVACE_NB_OF / 2; ++i)
	{
		curvace->calib.left_hemisphere[i].Arx = 1.0f;
		curvace->calib.left_hemisphere[i].Apx = 2.0f;
		curvace->calib.left_hemisphere[i].Ayx = 3.0f;
		curvace->calib.left_hemisphere[i].Ary = 4.0f;
		curvace->calib.left_hemisphere[i].Apy = 5.0f;
		curvace->calib.left_hemisphere[i].Ayy = 6.0f;		// TODO: correct this 
															// and do not forget to multiply each term by scale_factor

		curvace->calib.right_hemisphere[i].Arx = 1.0f;
		curvace->calib.right_hemisphere[i].Apx = 2.0f;
		curvace->calib.right_hemisphere[i].Ayx = 3.0f;
		curvace->calib.right_hemisphere[i].Ary = 4.0f;
		curvace->calib.right_hemisphere[i].Apy = 5.0f;
		curvace->calib.right_hemisphere[i].Ayy = 6.0f;		// TODO: correct this
	}
}


void curvace_update(curvace_t* curvace)
{
	// curvace_read_spi(curvace);

	curvace_derotate_all(curvace);
}