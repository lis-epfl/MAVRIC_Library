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

#include "mavrimini.hpp"
#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

extern "C" 
{
	#include "print_util.h"
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


int main(int argc, char** argv)
{
	uint8_t sysid = 0;
	bool init_success = true;
	
	// -------------------------------------------------------------------------
	// Create board
	// -------------------------------------------------------------------------
	mavrimini_conf_t board_config = mavrimini_default_config(); 
	Mavrimini board(board_config);

	// Board initialisation
	init_success &= board.init();


	// -------------------------------------------------------------------------
	// Create central data
	// -------------------------------------------------------------------------
	// Create central data using simulated sensors
	Central_data cd = Central_data( sysid,
									board.imu, 
									board.sim.barometer(),
									board.sim.gps(), 
									board.sim.sonar(),
									// board.serial_1,
									board.serial_2,
									board.spektrum_satellite,
									board.green_led,
									board.file_flash,
									board.battery,
									board.servo_0,
									board.servo_1,
									board.servo_2,
									board.servo_3 );

	// Init central data
	init_success &= cd.init();


	// -------------------------------------------------------------------------
	// Create tasks and telemetry
	// -------------------------------------------------------------------------

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

	static uint8_t step = 0;
	while(1)
	{
		step += 1;
		board.red_led.toggle();
		
		if(step%2 == 0)
		{
			board.green_led.toggle();
		}

		// usart_send(USART2, byte);
		// board.serial_1.write(&byte);
		time_keeper_delay_ms(100);
	}


	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
	}

	return 0;
}

// void usart2_isr(void)
// {
// 	// static uint8_t data = 'A';
// 	static uint16_t data = 'A';

// 	/* Check if we were called because of RXNE. */
// 	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
// 	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

// 		/* Indicate that we got data. */
// 		gpio_toggle(GPIOD, GPIO12);

// 		/* Retrieve the data from the peripheral. */
// 		data = usart_recv(USART2);

// 		/* Enable transmit interrupt so it sends back the data. */
// 		usart_enable_tx_interrupt(USART2);
// 	}

// 	/* Check if we were called because of TXE. */
// 	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
// 	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

// 		/* Put data into the transmit register. */
// 		usart_send(USART2, data);

// 		/* Disable the TXE interrupt as we don't need it anymore. */
// 		usart_disable_tx_interrupt(USART2);
// 	}
// }





// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>

// /* Set STM32 to 168 MHz. */
// static void clock_setup(void)
// {
// 	rcc_clock_setup_hse_3v3(&hse_25mhz_3v3[CLOCK_3V3_168MHZ]);

// 	/* Enable GPIOD clock. */
// 	rcc_periph_clock_enable(RCC_GPIOC);
// 	// rcc_periph_clock_enable(RCC_GPIOD);
// }

// static void gpio_setup(void)
// {
// 	/* Set GPIO12-15 (in GPIO port D) to 'output push-pull'. */
// 	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
// 			GPIO_PUPD_NONE,  GPIO14 | GPIO15);
// 	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO14 | GPIO15);
// }

// int main(void)
// {
// 	int i;

// 	clock_setup();
// 	gpio_setup();

// 	/* Set two LEDs for wigwag effect when toggling. */
// 	gpio_set(GPIOC, GPIO14 | GPIO15);

// 	/* Blink the LEDs (PD12, PD13, PD14 and PD15) on the board. */
// 	while (1) {
// 		/* Toggle LEDs. */
// 		gpio_toggle(GPIOC, GPIO14 | GPIO15);
// 		for (i = 0; i < 6000000; i++) { /* Wait a bit. */
// 			__asm__("nop");
// 		}
// 	}

// 	return 0;
// }