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
 * \file imu_config.h
 * 
 * \author MAV'RIC Team
 * 
 * \brief  This file configures the imu of the maveric autopilot
 *   
 ******************************************************************************/


#ifndef IMU_CONFIG_H_
#define IMU_CONFIG_H_

extern "C" 
{
	#include "conf_platform.h"	// To get the MAVLINK_SYS_ID
}

///< Define which configuration of the imu to use, depending on the autopilot ID
#if MAVLINK_SYS_ID == 1
#include "MAVcalib/MAV001_imu_config.hpp"
#elif MAVLINK_SYS_ID == 2
#include "MAVcalib/MAV002_imu_config.hpp"
#elif MAVLINK_SYS_ID == 3
#include "MAVcalib/MAV003_imu_config.hpp"
#elif MAVLINK_SYS_ID == 4
#include "MAVcalib/MAV004_imu_config.hpp"
#elif MAVLINK_SYS_ID == 5
#include "MAVcalib/MAV005_imu_config.hpp"
#elif MAVLINK_SYS_ID == 6
#include "MAVcalib/MAV006_imu_config.hpp"
#elif MAVLINK_SYS_ID == 7
#include "MAVcalib/MAV007_imu_config.hpp"
#elif MAVLINK_SYS_ID == 8
#include "MAVcalib/MAV008_imu_config.hpp"
#elif MAVLINK_SYS_ID == 9
#include "MAVcalib/MAV009_imu_config.hpp"
#elif MAVLINK_SYS_ID == 10
#include "MAVcalib/MAV010_imu_config.hpp"
#elif MAVLINK_SYS_ID == 42
#include "MAVcalib/MAV042_imu_config.hpp"
#elif MAVLINK_SYS_ID == 51
#include "MAVcalib/MAV051_imu_config.hpp"
#elif MAVLINK_SYS_ID == 101
#include "MAVcalib/MAV101_imu_config.hpp"
#elif MAVLINK_SYS_ID == 102
#include "MAVcalib/MAV102_imu_config.hpp"
#elif MAVLINK_SYS_ID == 128
#include "MAVcalib/MAV128_imu_config.hpp"
#elif MAVLINK_SYS_ID == 150
#include "MAVcalib/MAV150_imu_config.h"
#elif MAVLINK_SYS_ID == 201
#include "MAVcalib/MAV201_imu_config.hpp"
#elif MAVLINK_SYS_ID == 202
#include "MAVcalib/MAV202_imu_config.hpp"
#elif MAVLINK_SYS_ID == 203
#include "MAVcalib/MAV203_imu_config.hpp"
#elif MAVLINK_SYS_ID == 204
#include "MAVcalib/MAV204_imu_config.hpp"
#elif MAVLINK_SYS_ID == 205
#include "MAVcalib/MAV205_imu_config.h"
#elif MAVLINK_SYS_ID == 206
#include "MAVcalib/MAV206_imu_config.h"
#else
#warning "Unknown IMU calibration for the board with ID MAVLINK_SYS_ID, falling back to imu_default_config"
#include "imu.hpp"
#include "megafly_rev4.hpp"

static inline imu_conf_t imu_config(void)
{
	return megafly_rev4_default_config().imu_config;
}
#endif

#endif /* IMU_CONFIG_H_ */