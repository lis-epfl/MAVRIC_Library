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
 * \file 	mavrimini.hpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Autopilot board based on STM32
 *
 ******************************************************************************/


#ifndef MAVRIMINI_HPP_
#define MAVRIMINI_HPP_

#include "spektrum_satellite.hpp"
#include "battery.hpp"
#include "servo.hpp"

#include "gpio_stm32.hpp"

#include "dynamic_model_quad_diag.hpp"
#include "simulation.hpp"

#include "serial_dummy.hpp"
#include "i2c_dummy.hpp"
#include "adc_dummy.hpp"
#include "file_dummy.hpp"
#include "pwm_dummy.hpp"
#include "gpio_dummy.hpp"
#include "led_gpio.hpp"

extern "C"
{
	#include "streams.h"
}


/**
 * \brief 	Configuration structure
 */
typedef struct
{
	gpio_dummy_conf_t		dsm_receiver_gpio_config;
	gpio_dummy_conf_t		dsm_power_gpio_config;
	gpio_stm32_conf_t		green_led_gpio_config;
	gpio_stm32_conf_t		red_led_gpio_config;
	imu_conf_t				imu_config;
	servo_conf_t			servo_config[8];
} mavrimini_conf_t;


/**
 * \brief 	Default configuration for the board
 * 
 * \return 	Config structure
 */
static inline mavrimini_conf_t mavrimini_default_config();


/**
 * \brief  Boardsupport for the MegaFly board (rev4)
 * 
 */
class Mavrimini
{
public:
	/**
	 * \brief  			Constructor
	 *
	 * \param 	config 	Board configuration
	 */
	Mavrimini( mavrimini_conf_t config = mavrimini_default_config() );


	/**
	 * \brief  	Hardware initialisation 

	 * \return 	Success
	 */
	bool init(void);

	/**
	 * Public Members
	 */
	Gpio_dummy			dsm_receiver_gpio;
	Gpio_dummy			dsm_power_gpio;
	Gpio_stm32			green_led_gpio;
	Gpio_stm32			red_led_gpio;
	Led_gpio 			green_led;
	Led_gpio 			red_led;
	File_dummy 			file_flash;
	Serial_dummy 		uart0;		
	Serial_dummy 		uart1;				
	Spektrum_satellite	spektrum_satellite;
	Adc_dummy			adc_battery;
	Battery 			battery;
	Pwm_dummy			pwm_0;
	Pwm_dummy			pwm_1;
	Pwm_dummy			pwm_2;
	Pwm_dummy			pwm_3;
	Pwm_dummy			pwm_4;
	Pwm_dummy			pwm_5;
	Pwm_dummy			pwm_6;
	Pwm_dummy			pwm_7;
	Servo				servo_0;
	Servo				servo_1;
	Servo				servo_2;
	Servo				servo_3;
	Servo				servo_4;
	Servo				servo_5;
	Servo				servo_6;
	Servo				servo_7;
	Dynamic_model_quad_diag sim_model;
	Simulation 				sim;
	Imu 					imu;

private:
	byte_stream_t	dbg_stream_;  ///< Temporary member to make print_util work TODO: remove
};


/**
 * \brief 	Default configuration for the board
 * 
 * \return 	Config structure
 */
static inline mavrimini_conf_t mavrimini_default_config()
{
	mavrimini_conf_t conf = {};

	// -------------------------------------------------------------------------
	// GPIO config
	// -------------------------------------------------------------------------
	// Green led
	conf.green_led_gpio_config.port 	= GPIO_STM32_PORT_D;
	conf.green_led_gpio_config.pin 		= GPIO_STM32_PIN_12;
	conf.green_led_gpio_config.dir 		= GPIO_OUTPUT;
	conf.green_led_gpio_config.pull 	= GPIO_PULL_UPDOWN_NONE;

	// Red led
	conf.red_led_gpio_config.port 	= GPIO_STM32_PORT_D;
	conf.red_led_gpio_config.pin 	= GPIO_STM32_PIN_13;
	conf.red_led_gpio_config.dir 	= GPIO_OUTPUT;
	conf.red_led_gpio_config.pull 	= GPIO_PULL_UPDOWN_NONE;


	// -------------------------------------------------------------------------
	// Servo config
	// -------------------------------------------------------------------------
	conf.servo_config[0] = servo_default_config_esc();
	conf.servo_config[1] = servo_default_config_esc();
	conf.servo_config[2] = servo_default_config_esc();
	conf.servo_config[3] = servo_default_config_esc();
	conf.servo_config[4] = servo_default_config_esc();
	conf.servo_config[5] = servo_default_config_esc();
	conf.servo_config[6] = servo_default_config_esc();
	conf.servo_config[7] = servo_default_config_esc();

	return conf;
}


#endif /* MAVRIMINI_HPP_ */
