/*
 * conf_sim_model.h
 *
 * Created: 06/08/2013 20:26:35
 *  Author: sfx
 */ 


#ifndef CONF_SIM_MODEL_H_
#define CONF_SIM_MODEL_H_
#include "simulation.h"

#define AIR_DENSITY 1.2f

static simulation_model_t vehicle_model_parameters= {
	.rotor_lpf			=  0.1f, 		// low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0f=no inertia, 0.0f=infinite inertia
	.rotor_rpm_gain		=  4000.0f, 
	.rotor_rpm_offset	=  -1.0f,		// offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	.vehicle_drag       =  0.01f,         // vehicle drag coefficient * vehicle area
	.rotor_cd			=  0.03f,	    // coefficient of drag of rotor blade
	.rotor_cl			=  1.0f,			// coefficient of lift of rotor blade
	.rotor_diameter     =  0.14f,        // mean "effective" rotor diameter
	.rotor_foil_area	=  0.18f * 0.015f,  // area of the propeller blades in m^2
	.rotor_pitch        =  0.15f,        // rotor pitch in m/revolution (7x6" roughly 0.15fm)
	.total_mass			=  0.35f, 		// vehicle mass in kg
	.roll_pitch_momentum=  0.1f * 0.17f / 1.4142f, 	// angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		=  0.1f * 0.17f ,  // approximate motor arm mass * rotor arm length
	.rotor_momentum     =  0.005f*0.03f,  // rotor inertia  (5g off center mass * rotor radius)
	.rotor_arm_length	=  0.17f,  	    // distance between CoG and motor (in meter)
	.wind_x=0.0f,
	.wind_y=0.0f
};




#endif /* CONF_SIM_MODEL_H_ */