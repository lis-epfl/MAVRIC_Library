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
	.rotor_lpf			 =  0.1f, 					///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
	.rotor_rpm_gain		 =  4000.0f,					///< The gain linking the rotor command to rpm
	.rotor_rpm_offset	 =  -1.0f,					///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	.vehicle_drag        =  0.01f,					///< Vehicle drag coefficient * vehicle area
	.rotor_cd			 =  0.03f,					///< Coefficient of drag of rotor blade
	.rotor_cl			 =  1.0f,					///< Coefficient of lift of rotor blade
	.rotor_diameter      =  0.14f,					///< Mean "effective" rotor diameter
	.rotor_foil_area	 =  0.18f * 0.015f,			///< Area of the propeller blades in m^2
	.rotor_pitch         =  0.15f,					///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
	.total_mass			 =  0.35f,					///< Vehicle mass in kg
	.roll_pitch_momentum =  0.1f * 0.17f / 1.4142f,	///< Angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		 =  0.1f * 0.17f ,			///< Approximate motor arm mass * rotor arm length
	.rotor_momentum      =  0.005f * 0.03f,				///< Rotor inertia  (5g off center mass * rotor radius)
	.rotor_arm_length	 =  0.17f,					///< Distance between CoG and motor (in meter)
	.wind_x				 =  0.0f,					///< Wind in x axis, global frame
	.wind_y				 =  0.0f						///< Wind in y axis, global frame
};

#ifdef __cplusplus
}
#endif

#endif /* CONF_SIM_MODEL_H_ */