/*
 * servo_pwm.h
 *
 * Created: 20/04/2012 17:47:30
 *  Author: sfx
 */ 


#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#include "conf_platform.h"

#define SERVO_TIMER_FREQ 1000000
#define SERVO_CENTER_DUTY_MICROSEC 1500
#define SERVO_REPEAT_FREQ 200

// GLE: Modification for CS on servos
#ifdef GPS_ON_SERVO_1_2
	#ifdef CS_ON_SERVO_7_8
		#define NUMBER_OF_SERVO_OUTPUTS 4
	#else
		#define NUMBER_OF_SERVO_OUTPUTS 6
	#endif
#else
	#ifdef CS_ON_SERVO_7_8
		#define NUMBER_OF_SERVO_OUTPUTS 6
	#else
		#define NUMBER_OF_SERVO_OUTPUTS 8
	#endif
#endif
// GLE: End

typedef struct {
	int value;
	int min, max, failsafe_position;
} servo_output;

void init_Servos(void);
void set_servos(const servo_output *servo_outputs);
void servos_failsafe(servo_output *servo_outputs);

#endif /* SERVO_PWM_H_ */
