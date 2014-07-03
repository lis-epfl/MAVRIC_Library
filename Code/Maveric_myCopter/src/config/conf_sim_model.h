/* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file conf_sim_model.h
 *
 *  Initialization of all simulation variables
 */


#ifndef CONF_SIM_MODEL_H_
#define CONF_SIM_MODEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "simulation.h"

#define AIR_DENSITY 1.2								///< The air density

static simulation_model_t vehicle_model_parameters= {
	.rotor_lpf			 =  0.1, 					///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
	.rotor_rpm_gain		 =  4000.0,					///< The gain linking the rotor command to rpm
	.rotor_rpm_offset	 =  -1.0,					///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	.vehicle_drag        =  0.01,					///< Vehicle drag coefficient * vehicle area
	.rotor_cd			 =  0.03,					///< Coefficient of drag of rotor blade
	.rotor_cl			 =  1.0,					///< Coefficient of lift of rotor blade
	.rotor_diameter      =  0.14,					///< Mean "effective" rotor diameter
	.rotor_foil_area	 =  0.18 * 0.015,			///< Area of the propeller blades in m^2
	.rotor_pitch         =  0.15,					///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
	.total_mass			 =  0.35,					///< Vehicle mass in kg
	.roll_pitch_momentum =  0.1 * 0.17 / 1.4142,	///< Angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		 =  0.1 * 0.17 ,			///< Approximate motor arm mass * rotor arm length
	.rotor_momentum      =  0.005*0.03,				///< Rotor inertia  (5g off center mass * rotor radius)
	.rotor_arm_length	 =  0.17,					///< Distance between CoG and motor (in meter)
	.wind_x				 =  0.0,					///< Wind in x axis, global frame
	.wind_y				 =  0.0						///< Wind in y axis, global frame
};

#ifdef __cplusplus
}
#endif

#endif /* CONF_SIM_MODEL_H_ */