/*
 * conf_sim_model.h
 *
 * Created: 06/08/2013 20:26:35
 *  Author: sfx
 */ 


#ifndef CONF_SIM_MODEL_H_
#define CONF_SIM_MODEL_H_
#include "simulation.h"


// default home location (EFPL Esplanade)
#define HOME_LONGITUDE 6.566044801857777
#define HOME_LATITUDE 46.51852236174565
#define HOME_ALTITUDE 398.0

#define AIR_DENSITY 1.2

static simulation_model_t vehicle_model_parameters= {
	.rotor_lpf			=  0.95, 		// low pass filter constant to express rotor inertia/lag. 0.0=no inertia, 1.0=infinite inertia
	.rotor_rpm_gain		=  3500.0, 
	.rotor_rpm_offset	=  -1.0,		// offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	.vehicle_drag       =  0.1,         // vehicle drag coefficient * vehicle area
	.rotor_cd			=  0.03,	    // coefficient of drag of rotor blade
	.rotor_cl			=  0.9,			// coefficient of lift of rotor blade
	.rotor_diameter     =  0.14,        // mean "effective" rotor diameter
	.rotor_foil_area	=  0.18 * 0.015,  // area of the propeller blades in m^2
	.rotor_pitch        =  0.15,        // rotor pitch in m/revolution (7x6" roughly 0.15m)
	.total_mass			=  0.4, 		// vehicle mass in kg
	.roll_pitch_momentum=  0.05 * 0.17 / 1.4142, 	// angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		=  0.1 * 0.17 ,  // approximate motor arm mass * rotor arm length
	.rotor_momentum     =  0.003*0.03,  // rotor inertia  (5g off center mass * rotor radius)
	.rotor_arm_length	=  0.17  	    // distance between CoG and motor (in meter)
	
};




#endif /* CONF_SIM_MODEL_H_ */