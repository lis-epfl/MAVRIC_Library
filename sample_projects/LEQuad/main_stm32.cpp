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
 * \file main.cpp
 * 
 * \author MAV'RIC Team
 *   
 * \brief Main file
 *
 ******************************************************************************/

// #include "mavrinux.hpp"
#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"


#include "dynamic_model_quad_diag.hpp"
#include "simulation.hpp"
#include "adc_dummy.hpp"
#include "pwm_dummy.hpp"
#include "serial_dummy.hpp"
#include "gpio_dummy.hpp"
#include "spektrum_satellite.hpp"
#include "file_dummy.hpp"
#include "led_dummy.hpp"

extern "C" 
{
	#include "print_util.h"
}






/**
 * TO REMOVE (start)
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable GPIOD clock. */
	rcc_periph_clock_enable(RCC_GPIOD);
}

static void gpio_setup(void)
{
	/* Set GPIO12-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}


static Serial_dummy* p_dbg_uart;
uint8_t serial2stream( stream_data_t data, uint8_t byte )
{
	p_dbg_uart->write(&byte);
	return 0;
}
/**
 * TO REMOVE (end)
 */




int main(int argc, char** argv)
{
	uint8_t sysid = 0;
	
	// -------------------------------------------------------------------------
	// Create board
	// -------------------------------------------------------------------------
	// mavrinux_conf_t board_config = mavrinux_default_config(); 
	
	// // Set correct sysid for UDP port and flash filename
	// board_config.serial_udp_config.local_port 	= 14000 + sysid;
	// board_config.flash_filename 				= std::string("flash") + std::to_string(sysid) + std::string(".bin");
	
	// // Create board
	// Mavrinux board(board_config);



	/**
	 * TO REMOVE (start)
	 */
	clock_setup();
	gpio_setup();
	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOD, GPIO12 | GPIO14);
	/**
	 * TO REMOVE (end)
	 */




	// -------------------------------------------------------------------------
	// Create dummy objects required by central data  
	// For objects not in board yet
	// -------------------------------------------------------------------------	
	// Dummy uart
	Serial_dummy dummy_serial;

	// Dummy satellite
	Gpio_dummy dummy_gpio;
	Spektrum_satellite dummy_satellite(dummy_serial, dummy_gpio, dummy_gpio);

	// Dummy file
	File_dummy dummy_file("dummy.txt");

	// Dummy led
	Led_dummy dummy_led;

	// Init debug stream TODO: remove
	byte_stream_t dbg_stream_;
	p_dbg_uart 					= &dummy_serial;
	dbg_stream_.get 			= NULL;
	dbg_stream_.put 			= &serial2stream;
	dbg_stream_.flush 			= NULL;
	dbg_stream_.buffer_empty 	= NULL;
	dbg_stream_.data 			= NULL;
	print_util_dbg_print_init(&dbg_stream_);



	// -------------------------------------------------------------------------
	// Create simulation
	// -------------------------------------------------------------------------
	// Simulated servos
	Pwm_dummy pwm[4];
	Servo sim_servo_0(pwm[0], servo_default_config_esc());
	Servo sim_servo_1(pwm[1], servo_default_config_esc());
	Servo sim_servo_2(pwm[2], servo_default_config_esc());
	Servo sim_servo_3(pwm[3], servo_default_config_esc());
	
	// Simulated dynamic model
	Dynamic_model_quad_diag sim_model 	= Dynamic_model_quad_diag(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
	Simulation sim 						= Simulation(sim_model);
	
	// Simulated battery
	Adc_dummy 	sim_adc_battery = Adc_dummy(11.1f);
	Battery 	sim_battery 	= Battery(sim_adc_battery);

	// Simulated IMU
	Imu 		sim_imu 		= Imu(  sim.accelerometer(),
										sim.gyroscope(),
										sim.magnetometer() );


	// -------------------------------------------------------------------------
	// Create central data
	// -------------------------------------------------------------------------
	// Create central data using simulated sensors
	Central_data cd = Central_data( sysid,
									sim_imu, 
									sim.barometer(),
									sim.gps(), 
									sim.sonar(),
									dummy_serial,
									dummy_satellite,
									dummy_led,
									dummy_file,
									sim_battery,
									sim_servo_0,
									sim_servo_1,
									sim_servo_2,
									sim_servo_3 );


	// -------------------------------------------------------------------------
	// Initialisation
	// -------------------------------------------------------------------------
	bool init_success = true;

	// Board initialisation
	// init_success &= board.init();

	// Init central data
	init_success &= cd.init();

	init_success &= mavlink_telemetry_add_onboard_parameters(&cd.mavlink_communication.onboard_parameters, &cd);

	// // Try to read from flash, if unsuccessful, write to flash
	if( onboard_parameters_read_parameters_from_storage(&cd.mavlink_communication.onboard_parameters) == false )
	{
		// onboard_parameters_write_parameters_to_storage(&cd.mavlink_communication.onboard_parameters);
		init_success = false; 
	}

	init_success &= mavlink_telemetry_init(&cd);

	cd.state.mav_state = MAV_STATE_STANDBY;	
	
	init_success &= tasks_create_tasks(&cd);	

	print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

	// -------------------------------------------------------------------------
	// Main loop
	// -------------------------------------------------------------------------
	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
		


		
		/**
		 * TO REMOVE (start)
		 */
		/* Toggle LEDs. */
		gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
		for (int i = 0; i < 6000000; i++) { /* Wait a bit. */
			__asm__("nop");
		/**
		 * TO REMOVE (end)
		 */




		}
	}

	return 0;
}
