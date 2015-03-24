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
 * \file piezo_speaker.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the piezzo speaker
 *
 ******************************************************************************/


#include "piezo_speaker.h"
#include "dac_dma.h"
#include "gpio.h"
// #include "delay.h"
#include "time_keeper.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief instantaneous output voltage sent to the speaker 
 * to make sounds this needs to be called repeatedly.
 *
 * \param analog_value sent to speaker
 */
static void piezo_speaker_set_value(int32_t analog_value);


/**
 * \brief Set a beeping 
 *
 * \param  binary_value different piezo_speaker_beep depending on the sign of binary_value 
 */
static void piezo_speaker_set_value_binary(int32_t binary_value);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void piezo_speaker_set_value(int32_t analog_value)
{
	dac_dma_set_value(analog_value);
	gpio_set_pin_low(PIEZO_LOW_PIN);
}


static void piezo_speaker_set_value_binary(int32_t binary_value)
{
	if (binary_value<0) 
	{
		gpio_set_pin_low(PIEZO_HIGH_PIN);
		gpio_set_pin_high(PIEZO_LOW_PIN);
	} 
	else if (binary_value == 0) 
	{
		gpio_set_pin_low(PIEZO_HIGH_PIN);
		gpio_set_pin_low(PIEZO_LOW_PIN);
	} 
	else if (binary_value>0) 
	{
		gpio_set_pin_high(PIEZO_HIGH_PIN);
		gpio_set_pin_low(PIEZO_LOW_PIN);
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void piezo_speaker_init(void) 
{
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
	dac_dma_init(0);
	piezo_speaker_set_value(0);
}


void piezo_speaker_init_binary(void) 
{
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
}


void piezo_speaker_beep(int32_t duration_ms, int32_t frequency)
{
	int32_t val = -1;
	uint32_t del_us = (uint32_t)1000000 / (uint32_t)frequency;
	if (frequency < 10) 
	{
		del_us = 100000;
	}
	uint32_t now = time_keeper_get_micros();
	uint32_t start = now;
	
	while( time_keeper_get_micros() < start + 1000 * duration_ms) 
	{
		piezo_speaker_set_value_binary(val);
		val = -val;
		now+=del_us / 2;
		time_keeper_delay_until(now);
	}
	piezo_speaker_set_value_binary(0);
}


void piezo_speaker_startup_melody(void)
{
	for (uint32_t i = 1; i < 8; i++)
	{
		piezo_speaker_beep(100, 500 * i);
		time_keeper_delay_ms(2);
	}
}


void piezo_speaker_quick_startup(void)
{
	piezo_speaker_beep(100, 440);
	time_keeper_delay_ms(50);
	piezo_speaker_beep(100, 880);
}


void piezo_speaker_startup_bb(void)
{
	for (uint32_t i = 3000; i > 500; i -= 100)
	{
		piezo_speaker_beep(5, i);
	}
	time_keeper_delay_ms(5);
	for (uint32_t i = 500; i < 3000; i += 100 )
	{
		piezo_speaker_beep(5, i);
	}
}


void piezo_speaker_critical_error_melody(void)
{
	piezo_speaker_beep(200, 700);
	piezo_speaker_beep(200, 650);
	piezo_speaker_beep(200, 500);
}


void piezo_speaker_star_wars(void)
{
	//3x beep(a, 500);
	piezo_speaker_beep(150, 440);
	time_keeper_delay_ms(100);
	piezo_speaker_beep(150, 440);
	time_keeper_delay_ms(100);
	piezo_speaker_beep(150, 440);
	time_keeper_delay_ms(100);
	
	//beep(f, 350);
	piezo_speaker_beep(105, 349);
	time_keeper_delay_ms(70);
	
	//beep(cH, 150);
	piezo_speaker_beep(45, 523);
	time_keeper_delay_ms(30);
	
	//beep(a, 500);
	piezo_speaker_beep(150, 440);
	time_keeper_delay_ms(100);
	
	//beep(f, 350);
	piezo_speaker_beep(105, 349);
	time_keeper_delay_ms(70);
	
	//beep(cH, 150);
	piezo_speaker_beep(45, 523);
	time_keeper_delay_ms(30);
	
	//beep(a, 650);
	piezo_speaker_beep(195, 440);
	time_keeper_delay_ms(130);
	
	//delay(500);
	time_keeper_delay_ms(250);
	
	//3x beep(eH, 500);
	piezo_speaker_beep(150, 659);
	time_keeper_delay_ms(100);
	piezo_speaker_beep(150, 659);
	time_keeper_delay_ms(100);
	piezo_speaker_beep(150, 659);
	time_keeper_delay_ms(100);
	
	//beep(fH, 350);
	piezo_speaker_beep(105, 698);
	time_keeper_delay_ms(70);
	
	//beep(cH, 150);
	piezo_speaker_beep(45, 523);
	time_keeper_delay_ms(30);
	
	//beep(gS, 500);
	piezo_speaker_beep(150, 415);
	time_keeper_delay_ms(100);
	
	//beep(f, 350);
	piezo_speaker_beep(105, 349);
	time_keeper_delay_ms(70);
	
	//beep(cH, 150);
	piezo_speaker_beep(45, 523);
	time_keeper_delay_ms(30);
	
	//beep(a, 650);
	piezo_speaker_beep(195, 440);
	time_keeper_delay_ms(130);
	
}


void piezo_speaker_mario_melody(void)
{
	uint32_t frequency;
	uint32_t length;

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(90);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(180);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(180);

	frequency=510;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(60);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(180);

	frequency=770;
	length=100;
	piezo_speaker_beep(length, frequency);
	time_keeper_delay_ms(540);

	frequency=380;
	length=100;
	piezo_speaker_beep(length, frequency);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=770;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(550);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(575);



	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(450);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(400);

	// frequency=320;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(500);

	// frequency=440;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(330);

	// frequency=450;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(200);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(200);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=860;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=700;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=520;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(500);


	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(450);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(400);

	// frequency=320;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(500);

	// frequency=440;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(330);

	// frequency=450;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(200);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(200);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=860;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=700;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=520;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(500);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(220);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=200;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(420);


	// frequency=585;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(450);


	// frequency=550;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(420);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(360);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(220);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=200;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(420);


	// frequency=585;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(450);


	// frequency=550;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(420);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(360);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=380;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(600);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(550);


	// frequency=870;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(325);

	// frequency=760;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(600);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=430;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=380;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(600);


	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(150);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(100);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(300);

	// frequency=770;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(550);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// time_keeper_delay_ms(575);


}