/*
 * servo_pwm.h
 *
 * Created: 20/04/2012 17:47:30
 *  Author: sfx
 */ 


#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#define SERVO_TIMER_FREQ 1000000
#define SERVO_CENTER_DUTY_MICROSEC 1500
#define SERVO_REPEAT_FREQ 200

#define NUMBER_OF_SERVO_OUTPUTS 8

typedef struct {
	int value;
	int min, max, failsafe_position;
} servo_output;


void init_Servos(void);

servo_output *get_servos();

void set_servos(servo_output *servo_outputs);


#endif /* SERVO_PWM_H_ */
