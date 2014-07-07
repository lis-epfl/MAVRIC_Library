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

void piezo_speaker_init(void) 
{
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
	dac_dma_init(0);
	dac_dma_set_value(0);
}

void piezo_speaker_init_binary(void) 
{
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
}


///< instantaneous output voltage sent to the speaker - to make sounds this needs to be called repeatedly.
void piezo_speaker_set_value(int32_t analog_value)
{
	dac_dma_set_value(analog_value);
	gpio_set_pin_low(PIEZO_LOW_PIN);
}

void piezo_speaker_set_value_binary(int32_t binary_value)
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
