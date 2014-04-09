/*
 * servo_pwm.c
 *
 * Created: 20/04/2012 17:47:52
 *  Author: sfx
 */ 

#include "servo_pwm.h"
#include "gpio.h"

#include "compiler.h"

#define SERVO_PERIOD (SERVO_TIMER_FREQ/SERVO_REPEAT_FREQ)
#define SERVO_CENTER_DUTY_TICKS 1500//(SERVO_CENTER_DUTY_MICROSEC*SERVO_TIMER_FREQ/1000000)

void init_Servos(void){
      int i;
	  // unlock registers
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
    // clock setup  
	AVR32_PWM.clk =
    ( 1 <<AVR32_PWM_DIVA_OFFSET) |  // /1
    ( 1 <<AVR32_PWM_DIVB_OFFSET) |  // /1
    ( 6 <<AVR32_PWM_PREA_OFFSET) |  // /64
    ( 6 <<AVR32_PWM_PREB_OFFSET) |  // /64
    ( 0 <<AVR32_PWM_CLKSEL_OFFSET);

	// output override for low and high side
	AVR32_PWM.oov  = ( 0b1111 <<(AVR32_PWM_OOVH0_OFFSET+i))|( 0b1111 <<(AVR32_PWM_OOVL0_OFFSET+i)); 
	// output selection clear: dead time generator (0)
	AVR32_PWM.osc  = ( 0b1111 <<(AVR32_PWM_OOVH0_OFFSET+i))|( 0b1111 <<(AVR32_PWM_OOVL0_OFFSET+i)); 
	
	// set up channels: enable dead time insertion
	for (i=0; i<4; i++) {
		// enable dead time, set channel clock to CLKA
		AVR32_PWM.channel[i].cmr=AVR32_PWM_CMR0_DTE_MASK | 11;
		AVR32_PWM.channel[i].cprd=10000;
		AVR32_PWM.channel[i].cdty=4000;
		AVR32_PWM.channel[i].dt= 1000<<16 | 1000;
		
	}		
	static const gpio_map_t PWM_GPIO_MAP =
  {
	#ifndef GPS_ON_SERVO_1_2
		{AVR32_PWM_PWML_0_0_PIN, AVR32_PWM_PWML_0_0_FUNCTION},
		{AVR32_PWM_PWMH_0_0_PIN, AVR32_PWM_PWMH_0_0_FUNCTION},
	#endif
	{AVR32_PWM_PWML_1_0_PIN, AVR32_PWM_PWML_1_0_FUNCTION},
    {AVR32_PWM_PWML_2_0_PIN, AVR32_PWM_PWML_2_0_FUNCTION},
    {AVR32_PWM_PWML_3_0_PIN, AVR32_PWM_PWML_3_0_FUNCTION},
    {AVR32_PWM_PWMH_1_0_PIN, AVR32_PWM_PWMH_1_0_FUNCTION},
    {AVR32_PWM_PWMH_2_0_PIN, AVR32_PWM_PWMH_2_0_FUNCTION},
    {AVR32_PWM_PWMH_3_0_PIN, AVR32_PWM_PWMH_3_0_FUNCTION}
    };			
	gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP) / sizeof(PWM_GPIO_MAP[0]));
	// enable
	AVR32_PWM.ena=0b1111;
	
	// Init servo values
	// set_servos_to_failsafe(servo_outputs);
}

void set_servo(int channel, int val_a, int val_b){
	int duty_a=val_a+SERVO_CENTER_DUTY_TICKS;
	int duty_b=val_b+SERVO_CENTER_DUTY_TICKS;
	int deadtime=(SERVO_PERIOD-duty_a-duty_b)/2;
	AVR32_PWM.channel[channel &0b11].cprdupd=SERVO_PERIOD;
	AVR32_PWM.channel[channel &0b11].cdtyupd=duty_a+deadtime;
	AVR32_PWM.channel[channel &0b11].dtupd= deadtime<<16 | deadtime;	
}

void set_servos_to_failsafe(servo_output *servo_outputs)
{
	int i;
	#ifndef GPS_ON_SERVO_1_2
		set_servo(0, servo_outputs[0].failsafe_position, servo_outputs[1].failsafe_position);
	#endif
		set_servo(1, servo_outputs[0].failsafe_position, servo_outputs[1].failsafe_position);
		set_servo(2, servo_outputs[2].failsafe_position, servo_outputs[3].failsafe_position);
		set_servo(3, servo_outputs[4].failsafe_position, servo_outputs[5].failsafe_position);

	for(i=0; i<NUMBER_OF_SERVO_OUTPUTS; i++)
	{
		servo_outputs[i].value = servo_outputs[i].failsafe_position;
	}
}

void set_servos(const servo_output *servo_outputs) {
	#ifndef GPS_ON_SERVO_1_2
		set_servo(0, servo_outputs[0].value, servo_outputs[1].value);
	#endif
		set_servo(1, servo_outputs[0].value, servo_outputs[1].value);
		set_servo(2, servo_outputs[2].value, servo_outputs[3].value);
		set_servo(3, servo_outputs[4].value, servo_outputs[5].value);
}

