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


#include "imu.hpp"

extern "C"
{
	#include "time_keeper.h"
	#include "print_util.h"
	#include "constants.h"
}



Imu::Imu(Accelerometer& accelerometer,
		Gyroscope& gyroscope,
		Magnetometer& magnetometer,
		// state_t& state,
		imu_conf_t config):
	accelerometer_(accelerometer),
	gyroscope_(gyroscope),
	magnetometer_(magnetometer),
	// state_(state),
	config_(config),
	oriented_acc_( 	{0.0f, 0.0f, 0.0f}),
	oriented_gyro_(	{0.0f, 0.0f, 0.0f}),
	oriented_mag_( 	{0.0f, 0.0f, 0.0f}),
	scaled_acc_(	{0.0f, 0.0f, 0.0f}),
	scaled_gyro_(	{0.0f, 0.0f, 0.0f}),
	scaled_mag_(	{0.0f, 0.0f, 0.0f}),
	do_accelerometer_bias_calibration_(false),
	do_accelerometer_scale_calibration_(false),
	do_gyroscope_bias_calibration_(false),
	do_gyroscope_scale_calibration_(false),
	do_magnetometer_bias_calibration_(false),
	do_magnetometer_scale_calibration_(false),
	dt_s_(0.004f),
	last_update_us_(time_keeper_get_micros())
{}


bool Imu::update(void)
{
	bool success = false;

	// Update timing
	uint32_t t 		= time_keeper_get_micros();
	dt_s_ 			= time_keeper_ticks_to_seconds( t - last_update_us_ );
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



	// -------------------------------------------------------------------------
	//
	// TODO implement calibration
	//
	// -------------------------------------------------------------------------

	// // Do accelero bias calibration
	// if( do_accelerometer_bias_calibration_ )
	// {
	// 	for (i=0; i<3; i++)
	// 	{
	// 		config_.accelerometer.max_values[i]  = maths_f_max(config_.accelerometer.max_values[i], oriented_acc_[i]);
	// 		config_.accelerometer.min_values[i]  = maths_f_min(config_.accelerometer.min_values[i], oriented_acc_[i]);
	// 		config_.accelerometer.mean_values[i] = 0.5 * ( config_.accelerometer.mean_values[i] + oriented_acc_[i] );
	// 	}
	// }

	// // Do accelero scale calibration
	// if( do_accelerometer_scale_calibration_ )
	// {
	// 	;
	// }

	// // Do gyroscope bias calibration
	// if( do_gyroscope_bias_calibration_ )
	// {
	// 	for (i=0; i<3; i++)
	// 	{
	// 		config_.gyroscope.max_values[i]  = maths_f_max(config_.gyroscope.max_values[i], oriented_gyro_[i]);
	// 		config_.gyroscope.min_values[i]  = maths_f_min(config_.gyroscope.min_values[i], oriented_gyro_[i]);
	// 		config_.gyroscope.mean_values[i] = 0.5 * ( config_.gyroscope.mean_values[i] + oriented_acc_[i] );
	// 	}
	// }

	// // Do gyroscope scale calibration
	// if( do_gyroscope_scale_calibration_ )
	// {
	// 	;
	// }

	// // Do magnetometer bias calibration
	// if( do_magnetometer_bias_calibration_ )
	// {
	// 	for (i=0; i<3; i++)
	// 	{
	// 		config_.magnetometer.max_values[i]  = maths_f_max(config_.magnetometer.max_values[i], oriented_mag_[i]);
	// 		config_.magnetometer.min_values[i]  = maths_f_min(config_.magnetometer.min_values[i], oriented_mag_[i]);
	// 		config_.magnetometer.mean_values[i] = 0.5 * ( config_.magnetometer.mean_values[i] + oriented_acc_[i] );
	// 	}
	// }

	// // Do magnetometer scale calibration
	// if( do_magnetometer_scale_calibration_ )
	// {
	// 	;
	// }

	// Scale sensor values
	for( int8_t i = 0; i < 3; i++ )
	{
		// 1: Unbias, 
		// 2: scale 
	// 	scaled_acc_[i]  = (1.0f - config_.lpf_acc ) * scaled_acc_[i]  + config_.lpf_acc  * ( ( oriented_acc_[i]  - config_.accelerometer.bias[i] ) * config_.accelerometer.scale_factor[i]  );
	// 	scaled_gyro_[i] = (1.0f - config_.lpf_gyro) * scaled_gyro_[i] + config_.lpf_gyro * ( ( oriented_gyro_[i] - config_.gyroscope.bias[i]     ) * config_.gyroscope.scale_factor[i]     		);
	// 	scaled_mag_[i] 	= (1.0f - config_.lpf_mag ) * scaled_mag_[i]  + config_.lpf_mag  * ( ( oriented_mag_[i]  - config_.magnetometer.bias[i]  ) * config_.magnetometer.scale_factor[i]  	);

		// 1: scale
		// 2: unbias
		scaled_acc_[i]  = config_.lpf_acc  	* (oriented_acc_[i]   * config_.accelerometer.scale_factor[i] - config_.accelerometer.bias[i])
				+ (1.0f - config_.lpf_acc ) * scaled_acc_[i];
		scaled_gyro_[i] = config_.lpf_gyro  * ( oriented_gyro_[i] * config_.gyroscope.scale_factor[i]    - config_.gyroscope.bias[i]     ) 
				+ (1.0f - config_.lpf_gyro) * scaled_gyro_[i];
		scaled_mag_[i] 	= config_.lpf_mag   * ( oriented_mag_[i]  * config_.magnetometer.scale_factor[i] - config_.magnetometer.bias[i]  ) 
				+ (1.0f - config_.lpf_mag ) * scaled_mag_[i];
	}

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


// -------------------------------------------------------------------------
//
// TODO implement calibration
//
// -------------------------------------------------------------------------

// bool Imu::start_accelerometer_bias_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_accelerometer_bias_calibration_;

// 	// And nothing incompatible is ongoing
// 	// success &= 

// 	if( success )
// 	{
// 		do_accelerometer_bias_calibration_ = true;
// 	}

// 	return success;
// }


// bool Imu::start_accelerometer_scale_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_accelerometer_scale_calibration_;

// 	do_accelerometer_scale_calibration_ = true;

// 	return success;
// }


// bool Imu::stop_accelerometer_bias_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_accelerometer_bias_calibration_;

// 	do_accelerometer_bias_calibration_ = false;

// 	return success;
// }


// bool Imu::stop_accelerometer_scale_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_accelerometer_scale_calibration_;

// 	do_accelerometer_scale_calibration_ = false;

// 	return success;
// }


// bool Imu::start_gyroscope_bias_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_gyroscope_bias_calibration_;

// 	do_gyroscope_bias_calibration_ = true;

// 	return success;
// }


// bool Imu::start_gyroscope_scale_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_gyroscope_scale_calibration_;

// 	do_gyroscope_scale_calibration_ = true;

// 	return success;
// }



// bool Imu::stop_gyroscope_bias_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_gyroscope_bias_calibration_;

// 	do_gyroscope_bias_calibration_ = false;

// 	return success;
// }


// bool Imu::stop_gyroscope_scale_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_gyroscope_scale_calibration_;

// 	do_gyroscope_scale_calibration_ = false;

// 	return success;
// }


// bool Imu::start_magnetometer_bias_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_magnetometer_bias_calibration_;

// 	do_magnetometer_bias_calibration_ = true;

// 	return success;
// }


// bool Imu::start_magnetometer_scale_calibration(void)
// {
// 	// Success if not already doing calibration
// 	bool success = !do_magnetometer_scale_calibration_;

// 	do_magnetometer_scale_calibration_ = true;

// 	return success;
// }


// bool Imu::stop_magnetometer_bias_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_magnetometer_bias_calibration_;

// 	do_magnetometer_bias_calibration_ = false;

// 	for (i = 0; i < 3; i++)
// 	{
// 		config_.magnetometer.bias[i] 		= (config_.magnetometer.max_values[i] + config_.magnetometer.min_values[i]) / 2.0f;
// 		config_.magnetometer.max_values[i]  = -10000.0;
// 		config_.magnetometer.min_values[i]  =  10000.0;
// 	}

// 	return success;
// }

// bool Imu::stop_magnetometer_scale_calibration(void)
// {
// 	// Success if already doing calibration
// 	bool success = do_magnetometer_scale_calibration_;

// 	do_magnetometer_scale_calibration_ = false;

// 	// for (i = 0; i < 3; i++)
// 	// {
// 	// 	config_.magnetometer.bias[i] 		= (config_.magnetometer.max_values[i] + config_.magnetometer.min_values[i]) / 2.0f;
// 	// 	config_.magnetometer.max_values[i]  = -10000.0;
// 	// 	config_.magnetometer.min_values[i]  =  10000.0;
// 	// }

// 	return success;
// }