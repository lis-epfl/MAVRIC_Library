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

void init_Servos(void);

void set_servo(int channel, int val_a, int val_b);



#endif /* SERVO_PWM_H_ */
