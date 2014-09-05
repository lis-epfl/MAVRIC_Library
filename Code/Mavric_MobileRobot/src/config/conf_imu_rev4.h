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
 * \file conf_imu_rev4.h
 * 
 * \author MAV'RIC Team
 *   
 ******************************************************************************/


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_

#ifdef __cplusplus
	extern "C" {
#endif

#define GYRO_AXIS_X 0				///< Define the index for the gyroscope raw values array
#define GYRO_AXIS_Y 1				///< Define the index for the gyroscope raw values array
#define GYRO_AXIS_Z 2				///< Define the index for the gyroscope raw values array

#define ACC_AXIS_X 0					///< Define the index for the accelerometer raw values array
#define ACC_AXIS_Y 1					///< Define the index for the accelerometer raw values array
#define ACC_AXIS_Z 2					///< Define the index for the accelerometer raw values array

#define MAG_AXIS_X 2					///< Define the index for the magnetometer raw values array
#define MAG_AXIS_Y 0					///< Define the index for the magnetometer raw values array
#define MAG_AXIS_Z 1					///< Define the index for the magnetometer raw values array

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111f
#define RAW_GYRO_X_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values
#define RAW_GYRO_Y_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values
#define RAW_GYRO_Z_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values

#define GYRO_BIAIS_X 0.0f								///< Gyroscope x biais
#define GYRO_BIAIS_Y 0.0f								///< Gyroscope y biais
#define GYRO_BIAIS_Z 0.0f								///< Gyroscope z biais

#define GYRO_ORIENTATION_X  1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)
#define GYRO_ORIENTATION_Y -1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)
#define GYRO_ORIENTATION_Z -1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)

#define RAW_ACC_X_SCALE  3924.0f		///< Define the scale factor for the accelerometer raw values
#define RAW_ACC_Y_SCALE  3844.8f		///< Define the scale factor for the accelerometer raw values
#define RAW_ACC_Z_SCALE  4119.6f		///< Define the scale factor for the accelerometer raw values

#define ACC_BIAIS_X   0.0f				///< Define the bias for the accelerometer raw values
#define ACC_BIAIS_Y   64.0f				///< Define the bias for the accelerometer raw values
#define ACC_BIAIS_Z   90.0				///< Define the bias for the accelerometer raw values

#define ACC_ORIENTATION_X  1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)
#define ACC_ORIENTATION_Y -1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)
#define ACC_ORIENTATION_Z -1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)

#define RAW_MAG_X_SCALE 614.51f			///< Define the scale factor for the magnetometer raw values
#define RAW_MAG_Y_SCALE 584.28f			///< Define the scale factor for the magnetometer raw values
#define RAW_MAG_Z_SCALE 531.95f			///< Define the scale factor for the magnetometer raw values

#define MAG_BIAIS_X   42.38f			///< Define the bias for the magnetometer raw values
#define MAG_BIAIS_Y -107.51f			///< Define the bias for the magnetometer raw values
#define MAG_BIAIS_Z    2.47f			///< Define the bias for the magnetometer raw values

#define MAG_ORIENTATION_X -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)
#define MAG_ORIENTATION_Y -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)
#define MAG_ORIENTATION_Z -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)

#ifdef __cplusplus
	}
#endif

#endif /* CONF_IMU_REV4_H_ */