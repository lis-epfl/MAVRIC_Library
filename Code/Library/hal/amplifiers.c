/*
 * amplifiers.c
 *
 * Created: 07.11.2011 11:31:17
 *  Author: David Morisod
 */ 

#include "amplifiers.h"

void init_amplifiers()
{
	gpio_configure_pin(SDN,GPIO_DIR_OUTPUT);
	//gpio_configure_pin(GAIN_0,GPIO_DIR_OUTPUT);		//doesn't work
	//gpio_configure_pin(GAIN_1,GPIO_DIR_OUTPUT);		//doesn't work
	gpio_configure_pin(GAIN_2,GPIO_DIR_OUTPUT);
	gpio_configure_pin(AMP_CS1,GPIO_DIR_OUTPUT);
	gpio_configure_pin(AMP_CS2,GPIO_DIR_OUTPUT);
	gpio_configure_pin(AMP_CS3,GPIO_DIR_OUTPUT);
	gpio_configure_pin(AMP_CS4,GPIO_DIR_OUTPUT);
	
	gpio_set_pin_high(SDN);		//enable the amplifiers
}

void reset_amplifiers()
{
	gpio_set_pin_low(AMP_CS1);
	gpio_set_pin_low(AMP_CS2);
	gpio_set_pin_low(AMP_CS3);
	gpio_set_pin_low(AMP_CS4);
	
	gpio_set_pin_low(GAIN_2);
	
	gpio_set_pin_high(AMP_CS1);
	gpio_set_pin_high(AMP_CS2);
	gpio_set_pin_high(AMP_CS3);
	gpio_set_pin_high(AMP_CS4);
}

void AMP_set_gain(int amp_number)
{
	switch(amp_number)
	{
		case 1 :
			amp_number = AMP_CS1;
			break;
		case 2 :
			amp_number = AMP_CS2;
			break;
		case 3 :
			amp_number = AMP_CS3;
			break;
		case 4 :
			amp_number = AMP_CS4;
			break;
		default:
			return 0;		
	}
	
	
	gpio_set_pin_low(amp_number);
	
	gpio_set_pin_high(GAIN_2);
	
	gpio_set_pin_high(amp_number);
	
	gpio_set_pin_low(GAIN_2);
}