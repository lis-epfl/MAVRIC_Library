/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file simulation_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief Default configuration for the simulated vehicle
 *
 ******************************************************************************/


#ifndef SIMULATION_DEFAULT_CONFIG_H_
#define SIMULATION_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "simulation.h"


simulation_config_t simulation_default_config=
{
	.rotor_lpf			 =  0.1f, 					///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0=no inertia, 0.0=infinite inertia
	.rotor_rpm_gain		 =  4000.0f,				///< The gain linking the rotor command to rpm
	.rotor_rpm_offset	 =  -1.0f,					///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
	.rotor_cd			 =  0.03f,					///< Coefficient of drag of rotor blade
	.rotor_cl			 =  1.0f,					///< Coefficient of lift of rotor blade
	.rotor_diameter      =  0.14f,					///< Mean "effective" rotor diameter
	.rotor_foil_area	 =  0.18f * 0.015f,			///< Area of the propeller blades in m^2
	.rotor_pitch         =  0.15f,					///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
	.total_mass			 =  0.35f,					///< Vehicle mass in kg
	.vehicle_drag        =  0.01f,					///< Vehicle drag coefficient * vehicle area
	.roll_pitch_momentum =  0.1f * 0.17f / 1.4142f,	///< Angular momentum constants (assumed to be independent) (in kg/m^2)
	.yaw_momentum		 =  0.1f * 0.17f ,			///< Approximate motor arm mass * rotor arm length
	.rotor_momentum      =  0.005f * 0.03f,			///< Rotor inertia  (5g off center mass * rotor radius)
	.rotor_arm_length	 =  0.17f,					///< Distance between CoG and motor (in meter)
	.wind_x				 =  0.0f,					///< Wind in x axis, global frame
	.wind_y				 =  0.0f,					///< Wind in y axis, global frame
	//default home location (EFPL Esplanade)
	.home_coordinates[X]   = 46.51852236174565f,	///< Latitude of the simulation home waypoint
	.home_coordinates[Y]  = 6.566044801857777f,		///< Longitude of the simulation home waypoint
	.home_coordinates[Z]   = 400.0f,				///< Altitude of the simulation home waypoint
	.sim_gravity		 =  9.8f					///< Simulation gravity
};

#ifdef __cplusplus
}
#endif

#endif // SIMULATION_DEFAULT_CONFIG_H_
