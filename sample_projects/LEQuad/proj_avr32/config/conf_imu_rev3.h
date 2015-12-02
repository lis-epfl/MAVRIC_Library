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
 * \file conf_imu_rev3.h
 * 
 * \author MAV'RIC Team
 *   
 ******************************************************************************/


#ifndef CONF_IMU_REV3_H_
#define CONF_IMU_REV3_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define GYRO_AXIS_X 1
#define GYRO_AXIS_Y 0
#define GYRO_AXIS_Z 2

#define RAW_GYRO_X_SCALE   12600.0f
#define RAW_GYRO_Y_SCALE  -12600.0f
#define RAW_GYRO_Z_SCALE   12600.0f

#define ACC_AXIS_X 0
#define ACC_AXIS_Y 1
#define ACC_AXIS_Z 2

//#define RAW_ACC_X_SCALE  261.5f
//#define RAW_ACC_Y_SCALE  262.5f
//#define RAW_ACC_Z_SCALE  255.0f
//#define RAW_ACC_X_SCALE  259.67f
//#define RAW_ACC_Y_SCALE  261.324f
//#define RAW_ACC_Z_SCALE  256.724f
// Felix outside
//#define RAW_ACC_X_SCALE  264.9173f
#define RAW_ACC_X_SCALE  258.9173f
#define RAW_ACC_Y_SCALE  258.9853f
#define RAW_ACC_Z_SCALE  258.0829f

#define MAG_AXIS_X 2
#define MAG_AXIS_Y 0
#define MAG_AXIS_Z 1

// Inside values
//#define RAW_MAG_X_SCALE 579.41f
//#define RAW_MAG_Y_SCALE 540.3f
//#define RAW_MAG_Z_SCALE 525.59f

// Outside values
//#define RAW_MAG_X_SCALE 534.90f
//#define RAW_MAG_Y_SCALE 514.85f
//#define RAW_MAG_Z_SCALE 478.57f

// Felix Outside values
#define RAW_MAG_X_SCALE 530.2771f
#define RAW_MAG_Y_SCALE 525.2934f
#define RAW_MAG_Z_SCALE 498.4476f

#define ACC_BIAIS_X 18.0f
#define ACC_BIAIS_Y 9.0f
#define ACC_BIAIS_Z -16.0f
 
//#define ACC_BIAIS_X 4.685f
//#define ACC_BIAIS_Y 4.376f
//#define ACC_BIAIS_Z -16.26f

// Felix Outside values
// #define ACC_BIAIS_X  21.5871f
// #define ACC_BIAIS_Y  10.0884f
// #define ACC_BIAIS_Z  -14.9891f


// Inside values
//#define MAG_BIAIS_X 34.20f
//#define MAG_BIAIS_Y -47.07f
//#define MAG_BIAIS_Z -76.93f

// Outside values
//#define MAG_BIAIS_X 47.62f
//#define MAG_BIAIS_Y -47.29f
//#define MAG_BIAIS_Z -74.38f

// Felix Outside values
#define MAG_BIAIS_X  131.7582f
#define MAG_BIAIS_Y -26.1298f
#define MAG_BIAIS_Z  61.1646f


#ifdef __cplusplus
}
#endif



#endif /* CONF_IMU_REV3_H_ */