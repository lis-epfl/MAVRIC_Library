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
 * \author Julien Lecoeur
 * \author Geraud L'Eplattenier
 *
 * \brief Driver for the cylindrical curvace
 *
 ******************************************************************************/

#include "curvace.h"
#include "maths.h"
#include "quick_trig.h"
#include "time_keeper.h"
#include "gpio.h"
#include "spi.h"
#include "sysclk.h"
#include "delay.h"

#define slaveSelectTop AVR32_PIN_PC11
#define slaveSelectBot AVR32_PIN_PC12

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * @brief  Initializes the SPI and GPIOs for curvace
 */
static void curvace_init_spi(void);


/**
 * @brief  	Quick implementation of SPI comm
 * 
 * @param 	data 	Data to send
 *
 * @return 			Data received
 */
static uint16_t curvace_spi_low_level(uint16_t data);


/**
 * @brief 	Read pixel data from both hemispheres
 * 
 * @param 	curvace 	Pointer to data structure
 */
static void curvace_read_spi(curvace_t* curvace);


/**
 * @brief  		Send start message to the camera
 */
static void curvace_start(void);


/**
 * @brief 	Filters, derotate and scale newly received raw data
 * 
 * @param 	curvace 	Pointer to data structure
 */
static void curvace_process_all(curvace_t* curvace);


// Not implemented yet
// static void curvace_stop(void);
// static void curvace_derotate_all(curvace_t* curvace);
// static void curvace_scale_all(curvace_t* curvace);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void curvace_init_spi()
{
	// Init GPIO for Chip Select of the SPI communication
	gpio_enable_gpio_pin(slaveSelectTop);
	gpio_set_gpio_pin(slaveSelectTop);
	gpio_enable_gpio_pin(slaveSelectBot);
	gpio_set_gpio_pin(slaveSelectBot);
	
	// Init SPI communication
	static const gpio_map_t SPI_GPIO_MAP =
	{
		{AVR32_SPI0_SCK_0_0_PIN, AVR32_SPI0_SCK_0_0_FUNCTION }, //SCK
		{AVR32_SPI0_MISO_0_0_PIN, AVR32_SPI0_MISO_0_0_FUNCTION}, //MISO
		{AVR32_SPI0_MOSI_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION} //MOSI
	}; 
	
	spi_options_t spiOptions =
	{
		.reg          = 0,			// CS0
		.baudrate     = 5000000,	// 5MHz
		.bits         = 16,			// Bits!
		.spck_delay   = 0,			// # clocks to delay.
		.trans_delay  = 0,			// ?
		.stay_act     = 0,			// auto-unselect...?
		.spi_mode     = SPI_MODE_1,	// active high, low level idle -> mode 0
		.modfdis      = 1			// ...?
	};
	
	// Assign I/Os to SPI.
	gpio_enable_module(SPI_GPIO_MAP, sizeof(SPI_GPIO_MAP) / sizeof(SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster((&AVR32_SPI0), &spiOptions);

	// Set selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode((&AVR32_SPI0), 0, 0, 0);

	//Set how we're talking to the chip. (Bits!  et al)
	spi_setupChipReg((&AVR32_SPI0), &spiOptions, sysclk_get_pba_hz()); //very important!

	// Enable SPI.
	spi_enable(&AVR32_SPI0);
	
	// Configure Chip Select even if not used.
	spi_selectChip(&AVR32_SPI0, 0);
}


static uint16_t curvace_spi_low_level(uint16_t data)
{
	//wait until
	while (!spi_is_tx_ready(&AVR32_SPI0));
	
	spi_put(&AVR32_SPI0, data);
	
	//wait until
	while(!spi_is_rx_full(&AVR32_SPI0));
	
	return spi_get(&AVR32_SPI0);
}


static void curvace_read_spi(curvace_t* curvace)
{
	// Left hemisphere
	
	//CS ON
	gpio_clr_gpio_pin(slaveSelectBot);
	
	for (uint8_t i = 0; i < CURVACE_NB_OF / 2; ++i)
	{
		curvace->raw_of.left_hemisphere[i].x = curvace_spi_low_level(100);
		curvace->raw_of.left_hemisphere[i].y = curvace_spi_low_level(101);
	}
	
	//CS OFF
	gpio_set_gpio_pin(slaveSelectBot);

	// Right hemisphere
	
	//CS ON
	gpio_clr_gpio_pin(slaveSelectTop);
	
	for (uint8_t i = 0; i < CURVACE_NB_OF / 2; ++i)
	{
		curvace->raw_of.right_hemisphere[i].x = curvace_spi_low_level(102);
		curvace->raw_of.right_hemisphere[i].y = curvace_spi_low_level(103);
	}
		
	//CS OFF
	gpio_set_gpio_pin(slaveSelectTop);
}

static void curvace_start(void)
{
	// Start Bottom CurvACE controller
	// CS ON
	gpio_clr_gpio_pin(slaveSelectBot);
	// Write start
	curvace_spi_low_level(0xAAAA);
	// CS OFF
	gpio_set_gpio_pin(slaveSelectBot);

	// Start Top CurvACE controller
	// CS ON
	gpio_clr_gpio_pin(slaveSelectTop);
	// Write start
	curvace_spi_low_level(0xAAAA);
	// CS OFF
	gpio_set_gpio_pin(slaveSelectTop);
}

//static void curvace_stop(void)
//{
//	// Stop and reset Bottom CurvACE controller
//	// CS ON
//	gpio_clr_gpio_pin(slaveSelectBot);
//	// Write start
//	curvace_spi_low_level(0x5555);
//	// CS OFF
//	gpio_set_gpio_pin(slaveSelectBot);
//
//	// Stop and reset Top CurvACE controller
//	// CS ON
//	gpio_clr_gpio_pin(slaveSelectTop);
//	// Write start
//	curvace_spi_low_level(0x5555);
//	// CS OFF
//	gpio_set_gpio_pin(slaveSelectTop);
//}


// static void curvace_derotate_all(curvace_t* curvace)
// {
// 	uint32_t j = 0;
// 	for (uint8_t i = 0; i < 2 * CURVACE_NB_OF; ++i)
// 	{
// 		// origin
// 		curvace->of.data[i] = 	curvace->of.data[i]
// 								- curvace->calib_matrix.data[j] 	* curvace->ahrs->angular_speed[ROLL]
// 								- curvace->calib_matrix.data[j + 1] * curvace->ahrs->angular_speed[PITCH]
// 								- curvace->calib_matrix.data[j + 2] * curvace->ahrs->angular_speed[YAW];

// 		// test
// 		// curvace->of.data[i] = 	curvace->of.data[i]
// 		// 						// - curvace->calib_matrix.data[j] 	* curvace->ahrs->angular_speed[ROLL]
// 		// 						// - curvace->calib_matrix.data[j + 1] * curvace->ahrs->angular_speed[PITCH]
// 		// 						+ 0.3f * curvace->calib_matrix.data[j + 2] * curvace->ahrs->angular_speed[YAW];
	

// 		j += 3;
// 	}
// }

// static void curvace_scale_all(curvace_t* curvace)
// {
// 	for (uint8_t i = 0; i < 2 * CURVACE_NB_OF; ++i)
// 	{
// 		curvace->of.data[i] = 	(1 - curvace->LPF) 	* curvace->scale_factor_simple * curvace->raw_of.data[i] +
// 								curvace->LPF	 	* curvace->of.data[i];
// 	}
// }


static void curvace_process_all(curvace_t* curvace)
{
	uint32_t j = 0;
	for (uint8_t i = 0; i < 2 * CURVACE_NB_OF; ++i)
	{

		switch( curvace->do_derotation )
		{
			case ON:
				curvace->of.data[i] = 	curvace->LPF	 	* curvace->of.data[i]  +
										(1 - curvace->LPF) 	* ( curvace->scale_factor_simple * curvace->raw_of.data[i] - (	curvace->calib_matrix.data[j] 		* curvace->ahrs->angular_speed[ROLL]  +
																															curvace->calib_matrix.data[j + 1] 	* curvace->ahrs->angular_speed[PITCH] +
																															curvace->calib_matrix.data[j + 2] 	* curvace->ahrs->angular_speed[YAW]		) 	);

			break;

			case OFF:
				curvace->of.data[i] = 	(1 - curvace->LPF) 	* curvace->scale_factor_simple * curvace->raw_of.data[i] +
										curvace->LPF	 	* curvace->of.data[i];
			break;
		}
	

		j += 3;
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void curvace_init(curvace_t* curvace, const ahrs_t* ahrs)
{
	// Init dependencies
	curvace->ahrs 			= ahrs;

	// Init members
	curvace->do_derotation 	= ON;
	curvace->LPF		 	= 0.0f;
	curvace->derot_factor	= 1.0f;

	// inter ommatidial angles
	float inter_ommatidia_vertical 		= 4.26;	// 4.2 degrees between each ommatidia
	float inter_ommatidia_horizontal 	= 4.2;	// 4.2 degrees between each ommatidia on middle row

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
		curvace->roi_coord.all[i].azimuth 		= maths_deg_to_rad( + inter_ommatidia_horizontal * (( pix_coord[i].x -20 ) - 0.5 ) 	);
		curvace->roi_coord.all[i].elevation 	= maths_deg_to_rad( - inter_ommatidia_vertical 	 * ( pix_coord[i].y - 7  ) 			); // - because we consider positive elevation for positive pitch in NED
	}

	// Init calib
	for (uint8_t i = 0; i < CURVACE_NB_OF; ++i)
	{
		float azimuth 	= curvace->roi_coord.all[i].azimuth;
		float elevation = curvace->roi_coord.all[i].elevation;
	
		curvace->calib_matrix.all[i].Arx = + quick_trig_cos(azimuth) 	* quick_trig_sin(elevation);
		curvace->calib_matrix.all[i].Apx = + quick_trig_sin(elevation) 	* quick_trig_sin(azimuth);
		curvace->calib_matrix.all[i].Ayx = + quick_trig_cos(elevation);
		curvace->calib_matrix.all[i].Ary = - quick_trig_sin(azimuth);
		curvace->calib_matrix.all[i].Apy = + quick_trig_cos(azimuth);
		curvace->calib_matrix.all[i].Ayy = 0.0f; 
	}

	// Init scale factor
	float range = 32768;  
									
	float frame_rate = 200; 		// Hz

	// Init simplified calib factor
	curvace->scale_factor_simple = frame_rate * maths_deg_to_rad( inter_ommatidia_horizontal ) / range;	

	for (uint8_t i = 0; i < CURVACE_NB_OF; ++i)
	{
		curvace->calib_factor.scale[i].elevation = frame_rate * maths_deg_to_rad( inter_ommatidia_vertical ) / range;
		curvace->calib_factor.scale[i].azimuth   = frame_rate * maths_deg_to_rad( inter_ommatidia_horizontal 
																				 * quick_trig_cos( curvace->roi_coord.all[i].elevation ) 
																				) / range;
	}
	
	// Init SPI peripheral
	curvace_init_spi();
	
	// Start CurvACE sensor.
	curvace_start();
}


void curvace_update(curvace_t* curvace)
{
	curvace_read_spi(curvace);
	curvace_process_all(curvace);
}