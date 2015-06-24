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

#include "serial_avr32.hpp"
#include "i2c_avr32.hpp"
#include "hmc5883l.hpp"
#include "lsm330dlc.hpp"
#include "bmp085.hpp"
#include "imu.hpp"

extern "C"
{
	#include "twim_default_config.h"
}


/**
 * @brief 	Configuration structure
 */
typedef struct
{
	serial_avr32_conf_t uart0_config;
	serial_avr32_conf_t uart3_config;
	i2c_avr32_conf_t 	i2c0_config;
	i2c_avr32_conf_t 	i2c1_config;
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
class Megafly_rev4
{
public:
	/**
	 * @brief  			Constructor
	 * @details  		Only copies configuration to contained modules, no hardware initalisation
	 * 
	 * @param 	imu 	Reference to imu structure
	 * @param 	config 	Board configuration
	 */
	Megafly_rev4( imu_t& imu, 
				  barometer_t& barometer,
				  megafly_rev4_conf_t config = megafly_rev4_default_config() );


	/**
	 * @brief  	Hardware initialisation 

	 * @return 	true 	Success
	 * @return 	false 	Failed
	 */
	bool init(void);

	/**
	 * Public Members
	 */
	Serial_avr32 	uart0;		
	Serial_avr32 	uart3;		
	I2c_avr32 		i2c0;
	I2c_avr32 		i2c1;
	Hmc5883l 		magnetometer;
	Lsm330dlc		lsm330dlc;
	Bmp085			bmp085;

private:
	imu_t& 		imu_;
};


/**
 * @brief 	Default configuration for the board
 * 
 * @return 	Config structure
 */
static inline megafly_rev4_conf_t megafly_rev4_default_config()
{
	megafly_rev4_conf_t conf = {};

	// UART0 configuration
	conf.uart0_config 						= {};
	conf.uart0_config.serial_device 		= AVR32_SERIAL_0;
	conf.uart0_config.mode 					= AVR32_SERIAL_IN_OUT;
 	conf.uart0_config.options				= {};
	conf.uart0_config.options.baudrate  	= 57600;
	conf.uart0_config.options.charlength	= 8;
	conf.uart0_config.options.paritytype 	= USART_NO_PARITY;
	conf.uart0_config.options.stopbits		= USART_1_STOPBIT;
	conf.uart0_config.options.channelmode	= USART_NORMAL_CHMODE;
	conf.uart0_config.rx_pin_map			= {AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION};
    conf.uart0_config.tx_pin_map			= {AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION};

    // UART3 configuration
	conf.uart3_config 						= {};
	conf.uart3_config.serial_device 		= AVR32_SERIAL_3;
	conf.uart3_config.mode 					= AVR32_SERIAL_IN_OUT;
 	conf.uart3_config.options				= {};
	conf.uart3_config.options.baudrate  	= 38400;
	conf.uart3_config.options.charlength	= 8;
	conf.uart3_config.options.paritytype 	= USART_NO_PARITY;
	conf.uart3_config.options.stopbits		= USART_1_STOPBIT;
	conf.uart3_config.options.channelmode	= USART_NORMAL_CHMODE;
	conf.uart3_config.rx_pin_map			= {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION};
    conf.uart3_config.tx_pin_map			= {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION};

    
	// I2C0 configuration
	conf.i2c0_config            = {};
	conf.i2c0_config.i2c_device = AVR32_I2C0;
	conf.i2c0_config.twi_opt    = twim_default_config();
	conf.i2c0_config.tenbit     = false;
	conf.i2c0_config.sda_pin    = AVR32_TWIMS0_TWD_0_0_PIN;
	conf.i2c0_config.clk_pin    = AVR32_TWIMS0_TWCK_0_0_PIN;


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
