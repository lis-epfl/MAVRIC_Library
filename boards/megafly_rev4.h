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
 * \file 	megafly_rev4.h
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Autopilot board based on AVR32
 *
 ******************************************************************************/


#ifndef MEGAFLY_REV4_H_
#define MEGAFLY_REV4_H_

#include "i2c_avr32.h"

extern "C"
{
	#include "twim_default_config.h"
}


/**
 * @brief 	Configuration structure
 */
typedef struct
{
	i2c_avr32_conf_t i2c1_config;
} megafly_rev4_conf_t;


/**
 * @brief 	Default configuration for the board
 * 
 * @return 	Config structure
 */
static inline megafly_rev4_conf_t megafly_rev4_default_config();


/**
 * @brief  Boardsupport for the MegaFly board (rev4)
 * 
 */
class megafly_rev4
{
public:
	/**
	 * @brief  			Constructor
	 * @details  		Only copies configuration to contained modules, no hardware initalisation
	 * 
	 * @param 	config 	Board configuration
	 */
	megafly_rev4(megafly_rev4_conf_t config = megafly_rev4_default_config() );
	

	/**
	 * @brief  	Hardware initialisation 

	 * @return 	true 	Success
	 * @return 	false 	Failed
	 */
	bool init(void);

	/**
	 * Public Members
	 */
	i2c_avr32 	i2c1;
};


/**
 * @brief 	Default configuration for the board
 * 
 * @return 	Config structure
 */
static inline megafly_rev4_conf_t megafly_rev4_default_config()
{
	megafly_rev4_conf_t conf = {};

	// I2C1 configuration
	conf.i2c1_config            = {};
	conf.i2c1_config.i2c_device = AVR32_I2C1;
	conf.i2c1_config.twi_opt    = twim_default_config();
	conf.i2c1_config.tenbit     = false;
	conf.i2c1_config.sda_pin    = AVR32_TWIMS1_TWD_0_0_PIN;
	conf.i2c1_config.clk_pin    = AVR32_TWIMS1_TWCK_0_0_PIN;

	return conf;
}


#endif /* MEGAFLY_REV4_H_ */
