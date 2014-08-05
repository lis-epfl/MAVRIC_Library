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
* \file pwm_servos.c
*
* This file is the driver for pwm servos
*/


#include "pwm_servos.h"
#include "gpio.h"
#include "print_util.h"
#include "time_keeper.h"
#include <math.h>

#include <stdint.h>

#define SERVO_PERIOD (SERVO_TIMER_FREQ/SERVO_REPEAT_FREQ)	///< Define the servo period
#define SERVO_CENTER_DUTY_TICKS 1500						///< (SERVO_CENTER_DUTY_MICROSEC*SERVO_TIMER_FREQ/1000000)


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

void write_channels(int32_t channel, int32_t val_a, int32_t val_b, uint16_t frequency);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void write_channels(int32_t channel, int32_t val_a, int32_t val_b, uint16_t frequency)
{
	int32_t period = SERVO_TIMER_FREQ / frequency;

	int32_t duty_a = val_a + SERVO_CENTER_DUTY_TICKS;
	int32_t duty_b = val_b + SERVO_CENTER_DUTY_TICKS;
	int32_t deadtime = ( period - duty_a - duty_b ) / 2;
	
	AVR32_PWM.channel[channel &0b11].cprdupd = period;
	AVR32_PWM.channel[channel &0b11].cdtyupd = duty_a + deadtime;
	AVR32_PWM.channel[channel &0b11].dtupd = deadtime << 16 | deadtime;	
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void pwm_servos_init(void)
{
	int32_t i = 0;
	
	///< To unlock registers
	AVR32_PWM.wpcr =  	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY ) 	|
						( AVR32_PWM_WPCR_WPRG0_MASK )							|
						( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );

	AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
						( AVR32_PWM_WPCR_WPRG1_MASK )							|
						( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );

	AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
						( AVR32_PWM_WPCR_WPRG2_MASK )							|
						( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );

	AVR32_PWM.wpcr = 	( AVR32_PWM_WPCR_WPKEY_KEY << AVR32_PWM_WPCR_WPKEY )	|
						( AVR32_PWM_WPCR_WPRG3_MASK )							|
						( AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD );

    ///< To setup the clock  
	AVR32_PWM.clk = ( 1 <<AVR32_PWM_DIVA_OFFSET) |  ///< /1
				    ( 1 <<AVR32_PWM_DIVB_OFFSET) |  ///< /1
				    ( 6 <<AVR32_PWM_PREA_OFFSET) |  ///< /64
				    ( 6 <<AVR32_PWM_PREB_OFFSET) |  ///< /64
				    ( 0 <<AVR32_PWM_CLKSEL_OFFSET);

	///< output override for low and high side
	AVR32_PWM.oov  = 	( 0b1111 << ( AVR32_PWM_OOVH0_OFFSET + i ) ) 	|
						( 0b1111 << ( AVR32_PWM_OOVL0_OFFSET + i ) );

	///< output selection clear: dead time generator (0)
	AVR32_PWM.osc  = 	( 0b1111 << ( AVR32_PWM_OOVH0_OFFSET + i ) ) 	|
						( 0b1111 << ( AVR32_PWM_OOVL0_OFFSET + i ) ); 
	
	///< set up channels: enable dead time insertion
	for ( i = 0; i < 4; i++) 
	{
		///< enable dead time, set channel clock to CLKA
		AVR32_PWM.channel[i].cmr = AVR32_PWM_CMR0_DTE_MASK | 11;
		AVR32_PWM.channel[i].cprd = 10000;
		AVR32_PWM.channel[i].cdty = 4000;
		AVR32_PWM.channel[i].dt= 1000 << 16 | 1000;	
	}		

	static const gpio_map_t PWM_GPIO_MAP =
	{
		{AVR32_PWM_PWML_0_0_PIN, AVR32_PWM_PWML_0_0_FUNCTION},
		{AVR32_PWM_PWMH_0_0_PIN, AVR32_PWM_PWMH_0_0_FUNCTION},
	
		#ifndef CS_ON_SERVO_7_8
			{AVR32_PWM_PWML_3_0_PIN, AVR32_PWM_PWML_3_0_FUNCTION},
			{AVR32_PWM_PWMH_3_0_PIN, AVR32_PWM_PWMH_3_0_FUNCTION},
		#endif	
	
		{AVR32_PWM_PWML_2_0_PIN, AVR32_PWM_PWML_2_0_FUNCTION},
		{AVR32_PWM_PWMH_2_0_PIN, AVR32_PWM_PWMH_2_0_FUNCTION},
		{AVR32_PWM_PWML_1_0_PIN, AVR32_PWM_PWML_1_0_FUNCTION},
		{AVR32_PWM_PWMH_1_0_PIN, AVR32_PWM_PWMH_1_0_FUNCTION}
    };			
	gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	
	AVR32_PWM.ena = 0b1111; ///< enable
}


void pwm_servos_write_to_hardware(const servos_t* servos)
{
	uint16_t pulse_us[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
	uint16_t freq_channel[4] = { 50, 50, 50, 50};

	// Set pulse length per servo
	for (uint8_t i = 0; i < servos->servos_count; ++i)
	{
		pulse_us[i] = 500 * servos->servo[i].value + 1500;
	}

	// Set update frequency per channel with conservative method:
	// if two servos on the same channel ask for two different frequencies,
	// then the lowest frequecy is used
	for (int8_t i = 0; i < 4; ++i)
	{
		freq_channel[i] = min( servos->servo[2 * i].repeat_freq, servos->servo[2 * i + 1].repeat_freq );
	}

	write_channels( 	0, pulse_us[0], pulse_us[1], freq_channel[0]);
	write_channels(	1, pulse_us[2], pulse_us[3], freq_channel[1]);
	write_channels(	2, pulse_us[4], pulse_us[5], freq_channel[2]);
	#ifndef CS_ON_SERVO_7_8
	write_channels(	3, pulse_us[6], pulse_us[7], freq_channel[3]);
	#endif
}