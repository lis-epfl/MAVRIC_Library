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
#include "maths.h"
 #include "quick_trig.h"

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
	curvace->scale_factor = 200 * maths_deg_to_rad(inter_ommatidia) / range;

	// Init viewing directions
	curvace_pixel_coordinates_t pix_coord[CURVACE_NB_OF] =
	{
		// Left hemisphere
		// Column 2
		{ .x = 2, .y = 2 },
		{ .x = 2, .y = 4 },
		{ .x = 2, .y = 6 },
		{ .x = 2, .y = 8 },
		{ .x = 2, .y = 10 },
		{ .x = 2, .y = 12 },

		// Column 4
		{ .x = 4, .y = 2 },
		{ .x = 4, .y = 4 },
		{ .x = 4, .y = 6 },
		{ .x = 4, .y = 8 },
		{ .x = 4, .y = 10 },
		{ .x = 4, .y = 12 },

		// Column 6
		{ .x = 6, .y = 2 },
		{ .x = 6, .y = 4 },
		{ .x = 6, .y = 6 },
		{ .x = 6, .y = 8 },
		{ .x = 6, .y = 10 },
		{ .x = 6, .y = 12 },

		// Column 8
		{ .x = 8, .y = 2 },
		{ .x = 8, .y = 4 },
		{ .x = 8, .y = 6 },
		{ .x = 8, .y = 8 },
		{ .x = 8, .y = 10 },
		{ .x = 8, .y = 12 },

		// Column 10
		{ .x = 10, .y = 2 },
		{ .x = 10, .y = 4 },
		{ .x = 10, .y = 6 },
		{ .x = 10, .y = 8 },
		{ .x = 10, .y = 10 },
		{ .x = 10, .y = 12 },

		// Column 12
		{ .x = 12, .y = 2 },
		{ .x = 12, .y = 4 },
		{ .x = 12, .y = 6 },
		{ .x = 12, .y = 8 },
		{ .x = 12, .y = 10 },
		{ .x = 12, .y = 12 },

		// Column 14
		{ .x = 14, .y = 2 },
		{ .x = 14, .y = 4 },
		{ .x = 14, .y = 6 },
		{ .x = 14, .y = 8 },
		{ .x = 14, .y = 10 },
		{ .x = 14, .y = 12 },

		// Column 16
		{ .x = 16, .y = 2 },
		{ .x = 16, .y = 4 },
		{ .x = 16, .y = 6 },
		{ .x = 16, .y = 8 },
		{ .x = 16, .y = 10 },
		{ .x = 16, .y = 12 },

		// Column 18
		{ .x = 18, .y = 2 },
		{ .x = 18, .y = 4 },
		{ .x = 18, .y = 6 },
		{ .x = 18, .y = 8 },
		{ .x = 18, .y = 10 },
		{ .x = 18, .y = 12 },

		// Right hemisphere
		// Column 3 (right hemisphere starts at pixel 20)
		{ .x = 23, .y = 2 },
		{ .x = 23, .y = 4 },
		{ .x = 23, .y = 6 },
		{ .x = 23, .y = 8 },
		{ .x = 23, .y = 10 },
		{ .x = 23, .y = 12 },

		// Column 5 (right hemisphere starts at pixel 20)
		{ .x = 25, .y = 2 },
		{ .x = 25, .y = 4 },
		{ .x = 25, .y = 6 },
		{ .x = 25, .y = 8 },
		{ .x = 25, .y = 10 },
		{ .x = 25, .y = 12 },

		// Column 7 (right hemisphere starts at pixel 20)
		{ .x = 27, .y = 2 },
		{ .x = 27, .y = 4 },
		{ .x = 27, .y = 6 },
		{ .x = 27, .y = 8 },
		{ .x = 27, .y = 10 },
		{ .x = 27, .y = 12 },

		// Column 9 (right hemisphere starts at pixel 20)
		{ .x = 29, .y = 2 },
		{ .x = 29, .y = 4 },
		{ .x = 29, .y = 6 },
		{ .x = 29, .y = 8 },
		{ .x = 29, .y = 10 },
		{ .x = 29, .y = 12 },

		// Column 11 (right hemisphere starts at pixel 20)
		{ .x = 31, .y = 2 },
		{ .x = 31, .y = 4 },
		{ .x = 31, .y = 6 },
		{ .x = 31, .y = 8 },
		{ .x = 31, .y = 10 },
		{ .x = 31, .y = 12 },

		// Column 13 (right hemisphere starts at pixel 20)
		{ .x = 33, .y = 2 },
		{ .x = 33, .y = 4 },
		{ .x = 33, .y = 6 },
		{ .x = 33, .y = 8 },
		{ .x = 33, .y = 10 },
		{ .x = 33, .y = 12 },

		// Column 15 (right hemisphere starts at pixel 20)
		{ .x = 35, .y = 2 },
		{ .x = 35, .y = 4 },
		{ .x = 35, .y = 6 },
		{ .x = 35, .y = 8 },
		{ .x = 35, .y = 10 },
		{ .x = 35, .y = 12 },

		// Column 17 (right hemisphere starts at pixel 20)
		{ .x = 37, .y = 2 },
		{ .x = 37, .y = 4 },
		{ .x = 37, .y = 6 },
		{ .x = 37, .y = 8 },
		{ .x = 37, .y = 10 },
		{ .x = 37, .y = 12 },

		// Column 19 (right hemisphere starts at pixel 20)
		{ .x = 39, .y = 2 },
		{ .x = 39, .y = 4 },
		{ .x = 39, .y = 6 },
		{ .x = 39, .y = 8 },
		{ .x = 39, .y = 10 },
		{ .x = 39, .y = 12 },
	};
	for (uint8_t i = 0; i < CURVACE_NB_OF; ++i)
	{
		curvace->roi_coord.all[i].azimuth 		= maths_deg_to_rad( -180 + inter_ommatidia * pix_coord[i].x );
		curvace->roi_coord.all[i].elevation 	= maths_deg_to_rad( + 30 - inter_ommatidia * pix_coord[i].y );	// +30 because we consider positive elevation for positive pitch in NED
	}

	// Init calib
	for (uint8_t i = 0; i < CURVACE_NB_OF; ++i)
	{
		float azimuth = curvace->roi_coord.all[i].azimuth;
		float elevation = curvace->roi_coord.all[i].elevation;

		curvace->calib.all[i].Arx = curvace->scale_factor * ( - quick_trig_cos(azimuth	) 	* quick_trig_sin(elevation) );
		curvace->calib.all[i].Apx = curvace->scale_factor * ( - quick_trig_sin(elevation) 	* quick_trig_sin(azimuth)	);
		curvace->calib.all[i].Ayx = curvace->scale_factor * ( - quick_trig_cos(elevation)	);
		curvace->calib.all[i].Ary = curvace->scale_factor * (   quick_trig_sin(azimuth	) 	);
		curvace->calib.all[i].Apy = curvace->scale_factor * ( - quick_trig_cos(azimuth	)	);
		curvace->calib.all[i].Ayy = 0.0f; 
	}
}


void curvace_update(curvace_t* curvace)
{
	curvace_read_spi(curvace);

	curvace_derotate_all(curvace);
}