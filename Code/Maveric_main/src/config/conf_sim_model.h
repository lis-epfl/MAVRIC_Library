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
#define HOME_ALTITUDE 400.0

#define AIR_DENSITY 1.2

static simulation_model_t vehicle_model_parameters= {
	.rotor_lpf			=  0.1, 		// low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
	.rotor_rpm_gain		=  5000.0, 
	.rotor_rpm_offset	=	 -1.0,		// offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	
	.rotor_cd			=  0.05,			// coefficient of lift of rotor blade
	.rotor_cl			=  1.0,			// coefficient of drag of rotor blade
	.rotor_diameter     =  0.08,         // mean "effective" rotor diameter
	.rotor_foil_area	=  0.002,       // area of the propeller blades in m^2
	.rotor_pitch        =  0.1,         // rotor pitch in m/revolution (7x4" roughly 0.1m)
	.total_mass			=  0.35, 		// vehicle mass in kg
	.roll_pitch_momentum=  0.05, 		// angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		=  0.1,
	
	.rotor_arm_length	= 0.17  			// distance between CoG and motor (in meter)
	
};




#endif /* CONF_SIM_MODEL_H_ */