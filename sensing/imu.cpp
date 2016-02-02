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
 * \file imu.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *   
 * \brief Inertial measurement unit (IMU)
 *
 ******************************************************************************/


#include "sensing/imu.hpp"

extern "C"
{
	#include "hal/common/time_keeper.hpp"
	#include "util/print_util.h"
	#include "util/constants.h"
}



Imu::Imu(Accelerometer& accelerometer,
		Gyroscope& gyroscope,
		Magnetometer& magnetometer,
		imu_conf_t config):
	accelerometer_(accelerometer),
	gyroscope_(gyroscope),
	magnetometer_(magnetometer),
	config_(config),
	oriented_acc_( 	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	oriented_gyro_(	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	oriented_mag_( 	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	scaled_acc_(	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	scaled_gyro_(	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	scaled_mag_(	std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	do_accelerometer_bias_calibration_(false),
	do_gyroscope_bias_calibration_(false),
	do_magnetometer_bias_calibration_(false),
	is_ready_(false),
	dt_s_(0.004f),
	last_update_us_(time_keeper_get_us()),
	timestamp_gyro_stable(time_keeper_get_s())
{}


bool Imu::update(void)
{
	bool success = false;

	// Update timing
	uint32_t t 		= time_keeper_get_us();
	dt_s_ 			= (float)( t - last_update_us_ ) / 1000000.0f;
	last_update_us_ = t;

	// Read new values from sensors
	success &= accelerometer_.update();
	success &= gyroscope_.update();
	success &= magnetometer_.update();

	// Retrieve data
	std::array<float, 3> raw_acc  = accelerometer_.acc();
	std::array<float, 3> raw_gyro = gyroscope_.gyro();
	std::array<float, 3> raw_mag  = magnetometer_.mag();

	// Rotate sensor values
	for( uint8_t i=0; i<3; i++ )
	{
		oriented_acc_[i]  = raw_acc[  config_.accelerometer.axis[i]	] * config_.accelerometer.sign[i];
		oriented_gyro_[i] = raw_gyro[ config_.gyroscope.axis[i]		] * config_.gyroscope.sign[i];
		oriented_mag_[i]  = raw_mag[  config_.magnetometer.axis[i]	] * config_.magnetometer.sign[i];
	}

	// Scale sensor values
	float new_scaled_acc[3];
	float new_scaled_gyro[3];
	float new_scaled_mag[3];
	for( int8_t i = 0; i < 3; i++ )
	{
		// Scale, then remove bias
		new_scaled_acc[i]  = oriented_acc_[i]  / config_.accelerometer.scale_factor[i] - config_.accelerometer.bias[i];
		new_scaled_gyro[i] = oriented_gyro_[i] / config_.gyroscope.scale_factor[i]     - config_.gyroscope.bias[i];
		new_scaled_mag[i]  = oriented_mag_[i]  / config_.magnetometer.scale_factor[i]  - config_.magnetometer.bias[i];

		// Low pass filter
		scaled_acc_[i]  = config_.lpf_acc  	* new_scaled_acc[i]  + (1.0f - config_.lpf_acc ) * scaled_acc_[i];
		scaled_gyro_[i] = config_.lpf_gyro  * new_scaled_gyro[i] + (1.0f - config_.lpf_gyro) * scaled_gyro_[i];
		scaled_mag_[i] 	= config_.lpf_mag   * new_scaled_mag[i]  + (1.0f - config_.lpf_mag ) * scaled_mag_[i];
	}

	// Do automatic gyroscope calibration at startup
	do_startup_calibration();

	// Perform calibration of sensors
	do_calibration();

	return success;
}


const float& Imu::last_update_us(void) const
{
	return last_update_us_;
}


const float& Imu::dt_s(void) const
{
	return dt_s_;
}


const std::array<float, 3>& Imu::acc(void) const
{
	return scaled_acc_;
}


const std::array<float, 3>& Imu::gyro(void) const
{
	return scaled_gyro_;
}


const std::array<float, 3>& Imu::mag(void) const
{
	return scaled_mag_;
}


imu_conf_t* Imu::get_config(void)
{
	return &config_;
}


bool Imu::start_accelerometer_bias_calibration(void)
{
	// Success if not already doing magnetometer calibration
	bool success = !do_magnetometer_bias_calibration_;

	if( success )
	{
		do_accelerometer_bias_calibration_ 		= true;
		config_.accelerometer.mean_values[X] 	= scaled_acc_[X];
		config_.accelerometer.mean_values[Y] 	= scaled_acc_[Y];
		config_.accelerometer.mean_values[Z] 	= scaled_acc_[Z];
	}

	return success;
}


bool Imu::start_gyroscope_bias_calibration(void)
{
	// Success if not already doing magnetometer calibration
	bool success = !do_magnetometer_bias_calibration_;

	if( success )
	{
		do_gyroscope_bias_calibration_ 		= true;
		config_.gyroscope.mean_values[X] 	= scaled_gyro_[X];
		config_.gyroscope.mean_values[Y] 	= scaled_gyro_[Y];
		config_.gyroscope.mean_values[Z] 	= scaled_gyro_[Z];
	}

	return success;
}


bool Imu::start_magnetometer_bias_calibration(void)
{
	// Success if not already doing accelerometer or gyroscope calibration
	bool success  = true;
	success &= !do_accelerometer_bias_calibration_;
	success &= !do_gyroscope_bias_calibration_;

	if( success )
	{
		do_magnetometer_bias_calibration_ 	= true;
		config_.magnetometer.max_values[X] 	= -10000.0f;
		config_.magnetometer.max_values[Y] 	= -10000.0f;
		config_.magnetometer.max_values[Z] 	= -10000.0f;
		config_.magnetometer.min_values[X] 	=  10000.0f;
		config_.magnetometer.min_values[Y] 	=  10000.0f;
		config_.magnetometer.min_values[Z] 	=  10000.0f;
	}

	return success;
}


bool Imu::stop_accelerometer_bias_calibration(void)
{
	// Success if calibration is ongoing
	bool success = do_accelerometer_bias_calibration_;
	
	// Stop calibrating
	do_accelerometer_bias_calibration_ = false;

	// Update biases
	if( success )
	{
		config_.accelerometer.bias[X] += config_.accelerometer.mean_values[X];
		config_.accelerometer.bias[Y] += config_.accelerometer.mean_values[Y];
		config_.accelerometer.bias[Z] += config_.accelerometer.mean_values[Z] - (-1.0f);
	}

	return success;
}


bool Imu::stop_gyroscope_bias_calibration(void)
{
	// Success if calibration is ongoing
	bool success = do_gyroscope_bias_calibration_;

	// Stop calibrating
	do_gyroscope_bias_calibration_ = false;

	// Update biases
	if( success )
	{
		config_.gyroscope.bias[X] += config_.gyroscope.mean_values[X];
		config_.gyroscope.bias[Y] += config_.gyroscope.mean_values[Y];
		config_.gyroscope.bias[Z] += config_.gyroscope.mean_values[Z];
	}
	
	return success;
}


bool Imu::stop_magnetometer_bias_calibration(void)
{
	// Success if calibration is ongoing
	bool success = do_magnetometer_bias_calibration_;

	// Stop calibrating
	do_magnetometer_bias_calibration_ = false;

	// Update biases
	config_.magnetometer.bias[X] += 0.5f * (config_.magnetometer.max_values[X] + config_.magnetometer.min_values[X]);
	config_.magnetometer.bias[Y] += 0.5f * (config_.magnetometer.max_values[Y] + config_.magnetometer.min_values[Y]);
	config_.magnetometer.bias[Z] += 0.5f * (config_.magnetometer.max_values[Z] + config_.magnetometer.min_values[Z]);

	return success;
}


const bool Imu::is_ready(void) const
{
	return is_ready_;
}


void Imu::do_startup_calibration(void)
{
	if( is_ready_ == false )
	{
		// Make sure calibration is ongoing
		if( do_gyroscope_bias_calibration_ == false )
		{
			start_gyroscope_bias_calibration();
			timestamp_gyro_stable = time_keeper_get_s();
		}

		// Check if the gyroscope values are stable
		bool gyro_is_stable = ( maths_f_abs(config_.gyroscope.mean_values[X] - scaled_gyro_[X]) < config_.startup_calib_gyro_threshold )
							&&( maths_f_abs(config_.gyroscope.mean_values[Y] - scaled_gyro_[Y]) < config_.startup_calib_gyro_threshold )
							&&( maths_f_abs(config_.gyroscope.mean_values[Z] - scaled_gyro_[Z]) < config_.startup_calib_gyro_threshold );

		
		if( gyro_is_stable == false )
		{
			is_ready_ = false;
			// Reset timestamp
			timestamp_gyro_stable = time_keeper_get_s();
		}

		// If gyros have been stable for long enough
		if( (gyro_is_stable == true) && ((time_keeper_get_s() - timestamp_gyro_stable) > config_.startup_calib_duration_s) )
		{
			// Sartup calibration is done
			is_ready_ = true;
			stop_gyroscope_bias_calibration();
		}
	}
}


void Imu::do_calibration(void)
{
	// Do accelero bias calibration
	if( do_accelerometer_bias_calibration_ )
	{
		for( uint8_t i=0; i<3; i++ )
		{
			config_.accelerometer.mean_values[i] = (1.0f - config_.lpf_mean) * config_.accelerometer.mean_values[i] + config_.lpf_mean * scaled_acc_[i];
		}
	}

	// Do gyroscope bias calibration
	if( do_gyroscope_bias_calibration_ )
	{
		for( uint8_t i=0; i<3; i++ )
		{
			config_.gyroscope.mean_values[i] = (1.0f - config_.lpf_mean) * config_.gyroscope.mean_values[i] + config_.lpf_mean * scaled_gyro_[i];
		}
	}

	// Do magnetometer bias calibration
	if( do_magnetometer_bias_calibration_ )
	{
		for( uint8_t i=0; i<3; i++ )
		{
			config_.magnetometer.max_values[i]  = maths_f_max(config_.magnetometer.max_values[i], scaled_mag_[i]);
			config_.magnetometer.min_values[i]  = maths_f_min(config_.magnetometer.min_values[i], scaled_mag_[i]);
		}
	}
}