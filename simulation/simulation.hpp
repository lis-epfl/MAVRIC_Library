/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file simulation.hpp 
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Simulation
 *
 ******************************************************************************/


#ifndef SIMULATION_HPP_
#define SIMULATION_HPP_


#include "simulation/dynamic_model.hpp"
#include "simulation/accelerometer_sim.hpp"
#include "simulation/gyroscope_sim.hpp"
#include "simulation/magnetometer_sim.hpp"
#include "simulation/barometer_sim.hpp"
#include "simulation/gps_sim.hpp"
#include "simulation/sonar_sim.hpp"


/**
 * \brief Configuration for simulation
 */
typedef struct
{
} simulation_conf_t;


/**
 * \brief 	Default configuration
 * 
 * \return 	Config structure
 */
static inline simulation_conf_t simulation_default_config();


/**
 * \brief 	Simulated dynamics of a quadcopter in diag configuration
 */
class Simulation
{
public:
	/**
	 * @brief 	Constructor
	 * 
	 * \param 	dynamic_model	Reference to dynamic model
	 * \param 	config			Configuration 	 
	 */
	Simulation( Dynamic_model& dynamic_model, simulation_conf_t config = simulation_default_config() );


	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	bool update(void);

	
	/**
	 * \brief 	Get last update time in microseconds
	 * 
	 * \return 	Update time
	 */
	const float& last_update_us(void) const;


	/**
	 * \brief 	Get simulated accelerometer
	 * 
	 * \return 	Reference to accelerometer
	 */	
	Accelerometer& accelerometer(void);

	
	/**
	 * \brief 	Get simulated gyroscope
	 * 
	 * \return 	Reference to gyroscope
	 */	
	Gyroscope& gyroscope(void);


	/**
	 * \brief 	Get simulated magnetometer
	 * 
	 * \return 	Reference to magnetometer
	 */	
	Magnetometer& magnetometer(void);


	/**
	 * \brief 	Get simulated barometer
	 * 
	 * \return 	Reference to barometer
	 */	
	Barometer& barometer(void);


	/**
	 * \brief 	Get simulated sonar
	 * 
	 * \return 	Reference to sonar
	 */	
	Sonar& sonar(void);


	/**
	 * \brief 	Get simulated gps
	 * 
	 * \return 	Reference to gps
	 */	
	Gps& gps(void);


private:
	Dynamic_model& 		dynamic_model_;			///< Reference to dynamic model

	Accelerometer_sim 	accelerometer_;			///< Simulated accelerometer
	Gyroscope_sim  		gyroscope_;				///< Simulated gyroscope
	Magnetometer_sim 	magnetometer_;			///< Simulated magnetometer
	Barometer_sim 		barometer_;				///< Simulated barometer
	Sonar_sim 			sonar_;					///< Simulated sonar
	Gps_sim 			gps_;					///< Simulated gps
	float 				last_update_us_; 		///< Last update time in microseconds
};



static inline simulation_conf_t simulation_default_config()
{
	simulation_conf_t conf = {};

	return conf;
}


#endif /* SIMULATION_HPP_ */