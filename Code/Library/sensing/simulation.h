/*
 * simulation.h
 *
 * Created: 06/08/2013 17:02:44
 *  Author: sfx
 */ 


#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "compiler.h"
#include "qfilter.h"

#include "conf_platform.h"
#include "imu.h"
#include "servo_pwm.h"


typedef struct {
	float torques_bf[3], rates_bf[3], lin_forces_bf[3];
	float rotorspeeds[ROTORCOUNT];                              // estimated rotor speeds
	float rotor_lpf, rotor_rpm_gain, rotor_rpm_offset;          // low pass filter to simulate rotor inertia and lag, gain/offset to convert servo commands to rpm
	float rpm_to_lift, rpm_to_yaw_torque;						// constants to estimate torque from estimated rotor speed
	float total_mass;											// vehicle mass in kg
	float roll_pitch_momentum,  yaw_momentum;                   // angular momentum constants (assumed to be independent)
	float rotor_arm_length;							 			// distance between CoG and motor (in meter)
	double last_update;										// last update in system ticks
	float dt;													// time base of current update
} simulation_model_t;


void init_simulation(simulation_model_t *sim);

// computes artificial gyro and accelerometer values based on motor commands
void simu_update(Imu_Data_t *imu1);


#endif /* SIMULATION_H_ */