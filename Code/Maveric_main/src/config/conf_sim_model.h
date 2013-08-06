/*
 * conf_sim_model.h
 *
 * Created: 06/08/2013 20:26:35
 *  Author: sfx
 */ 


#ifndef CONF_SIM_MODEL_H_
#define CONF_SIM_MODEL_H_
#include "simulation.h"

static simulation_model_t vehicle_model_parameters= {
	.rotor_lpf			=  0.1, 		// low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
	.rotor_rpm_gain		=  3000.0, 
	.rotor_rpm_offset	=	 -1.0,		// offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	
	.rpm_to_lift		= 0.45/6000.0,	// constants to estimate lift force from estimated rotor speed (kg/rpm)
	.rpm_to_yaw_torque	= 0.1/6000,		// constants to estimate yaw torque from estimated rotor speed (Nm/rpm)
	
	.total_mass			=  0.3, 		// vehicle mass in kg
	.roll_pitch_momentum=  0.1, 		// angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		=  0.2,
	
	.rotor_arm_length	= 0.17  			// distance between CoG and motor (in meter)
	
};




#endif /* CONF_SIM_MODEL_H_ */