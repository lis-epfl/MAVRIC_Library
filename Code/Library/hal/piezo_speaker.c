/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file piezo_speaker.c
*
* This file is the driver for the piezzo speaker
*/


#include "piezo_speaker.h"
#include "dac_dma.h"
#include "gpio.h"
#include "delay.h"
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
	} else if (binary_value>0) 
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
		delay_ms(2);
	}
}


void piezo_speaker_mario_melody(void)
{
	uint32_t frequency;
	uint32_t length;

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(90);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(180);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(180);

	frequency=510;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(60);

	frequency=660;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(180);

	frequency=770;
	length=100;
	piezo_speaker_beep(length, frequency);
	delay_ms(540);

	frequency=380;
	length=100;
	piezo_speaker_beep(length, frequency);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=770;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(550);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(575);



	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(450);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(400);

	// frequency=320;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(500);

	// frequency=440;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(330);

	// frequency=450;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(200);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(200);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=860;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=700;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=520;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(500);


	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(450);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(400);

	// frequency=320;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(500);

	// frequency=440;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(330);

	// frequency=450;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(200);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(200);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=860;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=700;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=760;
	// length=50;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=520;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=480;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(500);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(220);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=200;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(420);


	// frequency=585;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(450);


	// frequency=550;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(420);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(360);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(220);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=200;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=102;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=760;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=720;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=680;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=620;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=650;
	// length=150;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=570;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(420);


	// frequency=585;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(450);


	// frequency=550;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(420);


	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(360);


	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=380;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(600);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(550);


	// frequency=870;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(325);

	// frequency=760;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(600);


	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=500;
	// length=60;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=580;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(350);

	// frequency=660;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=500;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=430;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=380;
	// length=80;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(600);


	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(150);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=510;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(100);

	// frequency=660;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(300);

	// frequency=770;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(550);

	// frequency=380;
	// length=100;
	// piezo_speaker_beep(length, frequency);
	// delay_ms(575);


}