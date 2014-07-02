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

void init_piezo_speaker() {
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
	Init_DAC(0);
	DAC_set_value(0);
	
}

void init_piezo_speaker_binary() {
	gpio_configure_pin(PIEZO_HIGH_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_HIGH_PIN);
	gpio_configure_pin(PIEZO_LOW_PIN, GPIO_DIR_OUTPUT);
	gpio_set_pin_low(PIEZO_LOW_PIN);
	
}


// instantaneous output voltage sent to the speaker - to make sounds this needs to be called repeatedly.
void set_value(int analog_value){
	DAC_set_value(analog_value);
	gpio_set_pin_low(PIEZO_LOW_PIN);
}

void set_value_binary(int binary_value){
	if (binary_value<0) {
		gpio_set_pin_low(PIEZO_HIGH_PIN);
		gpio_set_pin_high(PIEZO_LOW_PIN);
	} else
	if (binary_value==0) {
		gpio_set_pin_low(PIEZO_HIGH_PIN);
		gpio_set_pin_low(PIEZO_LOW_PIN);
	} else
	if (binary_value>0) {
		gpio_set_pin_high(PIEZO_HIGH_PIN);
		gpio_set_pin_low(PIEZO_LOW_PIN);
	}

}

void beep(int duration_ms, int frequency) {
	int i;
	int val=-1;
	uint32_t del_us=(uint32_t)1000000/(uint32_t)frequency;
	if (frequency<10) del_us=100000;
	uint32_t now=get_micros();
	uint32_t start=now;
	while( get_micros()<start+1000*duration_ms) {
		set_value_binary(val);
		val=-val;
		now+=del_us/2;
		delay_until(now);
	}
	set_value_binary(0);
}

