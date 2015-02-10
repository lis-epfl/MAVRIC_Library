/*
 * servo_pwm.h
 *
 * Created: 20/04/2012 17:47:30
 *  Author: sfx
 */ 


#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#include <stdint.h>

#define SERVO_TIMER_FREQ 1000000
#define SERVO_CENTER_DUTY_MICROSEC 1500
#define SERVO_REPEAT_FREQ 200

#ifdef GPS_ENABLE_OFF
	#define NUMBER_OF_SERVO_OUTPUTS 6
#else
	#define NUMBER_OF_SERVO_OUTPUTS 8
#endif

typedef struct {
	int32_t value;
	int32_t min, max, failsafe_position;
} servo_output_t;


void servo_pwm_init_old(void);


void servo_pwm_set(servo_output_t *servo_outputs);


#endif /* SERVO_PWM_H_ */
