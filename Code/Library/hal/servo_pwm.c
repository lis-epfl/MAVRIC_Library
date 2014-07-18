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
* \file servo_pwm.c
*
* This file is the driver for the remote control
*/


#include "servo_pwm.h"
#include "gpio.h"
#include "print_util.h"

#include <stdint.h>

#define SERVO_PERIOD (SERVO_TIMER_FREQ/SERVO_REPEAT_FREQ)	///< Define the servo period
#define SERVO_CENTER_DUTY_TICKS 1500						///< (SERVO_CENTER_DUTY_MICROSEC*SERVO_TIMER_FREQ/1000000)

///< Function prototype definitions
void set_servo(int32_t channel, int32_t val_a, int32_t val_b);

void servo_pwm_hardware_init(void)
{
	int32_t i = 0;
	
	  ///< To unlock registers
	  AVR32_PWM.wpcr = (AVR32_PWM_WPCR_WPKEY_KEY   << AVR32_PWM_WPCR_WPKEY) |
	          AVR32_PWM_WPCR_WPRG0_MASK                            |
	          (AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD);      
      AVR32_PWM.wpcr = (AVR32_PWM_WPCR_WPKEY_KEY   << AVR32_PWM_WPCR_WPKEY) |
	          AVR32_PWM_WPCR_WPRG1_MASK                            |
	          (AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD);      
      AVR32_PWM.wpcr = (AVR32_PWM_WPCR_WPKEY_KEY   << AVR32_PWM_WPCR_WPKEY) |
	          AVR32_PWM_WPCR_WPRG2_MASK                            |
	          (AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD);      
      AVR32_PWM.wpcr = (AVR32_PWM_WPCR_WPKEY_KEY   << AVR32_PWM_WPCR_WPKEY) |
	          AVR32_PWM_WPCR_WPRG3_MASK                            |
	          (AVR32_PWM_WPCR_WPCMD_SWDIS << AVR32_PWM_WPCR_WPCMD);      
    ///< To setup the clock  
	AVR32_PWM.clk =
    ( 1 <<AVR32_PWM_DIVA_OFFSET) |  ///< /1
    ( 1 <<AVR32_PWM_DIVB_OFFSET) |  ///< /1
    ( 6 <<AVR32_PWM_PREA_OFFSET) |  ///< /64
    ( 6 <<AVR32_PWM_PREB_OFFSET) |  ///< /64
    ( 0 <<AVR32_PWM_CLKSEL_OFFSET);

	///< output override for low and high side
	AVR32_PWM.oov  = ( 0b1111 <<(AVR32_PWM_OOVH0_OFFSET+i))|( 0b1111 <<(AVR32_PWM_OOVL0_OFFSET+i)); 
	///< output selection clear: dead time generator (0)
	AVR32_PWM.osc  = ( 0b1111 <<(AVR32_PWM_OOVH0_OFFSET+i))|( 0b1111 <<(AVR32_PWM_OOVL0_OFFSET+i)); 
	
	///< set up channels: enable dead time insertion
	for (i=0; i<4; i++) 
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
	
	///< Init servo values
	// set_servos_to_failsafe(servo_outputs);
}

void servo_pwm_init(servo_output_t servos[])
{
	// Init servos
	for (int32_t i = 0; i < NUMBER_OF_SERVO_OUTPUTS; ++i)
	{
		servos[i].value = -600;
		servos[i].min = -600;
		servos[i].max = 600;
		servos[i].failsafe_position = -600;
	}
	
	servo_pwm_failsafe(servos);
	servo_pwm_set(servos);
	
	print_util_dbg_print("Servo PWM initialized.\n");
}

void set_servo(int32_t channel, int32_t val_a, int32_t val_b)
{
	int32_t duty_a = val_a + SERVO_CENTER_DUTY_TICKS;
	int32_t duty_b = val_b + SERVO_CENTER_DUTY_TICKS;
	int32_t deadtime=(SERVO_PERIOD - duty_a - duty_b) / 2;
	
	AVR32_PWM.channel[channel &0b11].cprdupd = SERVO_PERIOD;
	AVR32_PWM.channel[channel &0b11].cdtyupd = duty_a + deadtime;
	AVR32_PWM.channel[channel &0b11].dtupd= deadtime << 16 | deadtime;	
}

void servo_pwm_set(const servo_output_t *servo_outputs) 
{
	set_servo(0, servo_outputs[0].value, servo_outputs[1].value);
	set_servo(1, servo_outputs[2].value, servo_outputs[3].value);
	set_servo(2, servo_outputs[4].value, servo_outputs[5].value);
	#ifndef CS_ON_SERVO_7_8
		set_servo(3, servo_outputs[6].value, servo_outputs[7].value);
	#endif
}

void servo_pwm_failsafe(servo_output_t *servo_outputs)
{
	for(int32_t i = 0; i < NUMBER_OF_SERVO_OUTPUTS; i++)
	{
		servo_outputs[i].value = servo_outputs[i].failsafe_position;
	}
}

task_return_t servo_pwm_send_servo_output(servo_output_t* servos)
{
	mavlink_msg_servo_output_raw_send(	MAVLINK_COMM_0,
										time_keeper_get_micros(),
										0,
										(uint16_t)(servos[0].value + 1500),
										(uint16_t)(servos[1].value + 1500),
										(uint16_t)(servos[2].value + 1500),
										(uint16_t)(servos[3].value + 1500),
										(uint16_t)(servos[4].value + 1500),
										(uint16_t)(servos[5].value + 1500),
										(uint16_t)(servos[6].value + 1500),
										(uint16_t)(servos[7].value + 1500)	);
	return TASK_RUN_SUCCESS;
}