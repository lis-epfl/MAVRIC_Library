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
 * \file mav_mode.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Place where the mav modes and flags are defined
 *
 ******************************************************************************/


#ifndef MAV_MODE_H
#define MAV_MODE_H

#include "communication/mavlink_stream.hpp"

extern "C"
{
	#include <stdint.h>
	#include <stdbool.h>
}

/**
 * \brief Enum defining the arming/disarming events
 */
typedef enum
{
	ARM_ACTION_DISARMING 	= -1,	///< The next action is the disarming of the motors
	ARM_ACTION_NONE 		= 0,	///< The next action is nothing
	ARM_ACTION_ARMING 		= 1,	///< The next action is arming the motors
} arm_action_t;

/*
 * \brief enum for defining ARM/DISARM status
 */
typedef enum
{
	ARMED_OFF = 0,				///< Motors are disarmed
	ARMED_ON  = 1,				///< Motors are armed
} mode_flag_armed_t;

/*
 * \brief enum for defining simulation mode
 */
typedef enum
{
	HIL_OFF = 0,				///< Real mode
	HIL_ON  = 1,				///< Simulation mode
} mode_flag_hil_t;

typedef enum MAV_MODE_FLAG mav_flag_mask_t;

typedef enum MAV_STATE mav_state_t;


/*
 * \brief enum for defining modes
 */
typedef enum
{
	MAV_MODE_PRE = 0,						///< 0b00*00000
	MAV_MODE_SAFE = 64,						///< 0b01*00000
	MAV_MODE_ATTITUDE_CONTROL = 192,		///< 0b11*00000
	MAV_MODE_VELOCITY_CONTROL = 208,		///< 0b11*10000
	MAV_MODE_POSITION_HOLD = 216,			///< 0b11*11000
	MAV_MODE_GPS_NAVIGATION = 156			///< 0b10*11100
} mav_mode_predefined_t;


/*
 * \brief enum for defining custom mode
 */
typedef enum
{
	CUSTOM_BASE_MODE = 0,
	CUST_CRITICAL_CLIMB_TO_SAFE_ALT = 1,		///< First critical behavior
	CUST_CRITICAL_FLY_TO_HOME_WP = 2,			///< Second critical behavior, comes after CLIMB_TO_SAFE_ALT
	CUST_CRITICAL_LAND = 4,						///< Third critical behavior, comes after FLY_TO_HOME_WP

	CUST_DESCENT_TO_SMALL_ALTITUDE = 8,			///< First auto landing behavior
	CUST_DESCENT_TO_GND = 16,					///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
	
	CUST_COLLISION_AVOIDANCE = 32,				///< Collision avoidance 

	CUST_BATTERY_LOW = 64,						///< Battery low flag
    CUST_FENCE_1 = 128,							///< Fence 1 violation flag
    CUST_FENCE_2 = 256,							///< Fence 2 violation flag
    CUST_HEARTBEAT_LOST = 512,					///< Heartbeat loss flag
    CUST_REMOTE_LOST = 1024,					///< Remote lost flag
    CUST_GPS_BAD = 2048							///< GPS loss flag
} mav_mode_custom_t;

#define mav_mode_t uint8_t

/**
 * \brief  Funtion to allow logic operations on enum in C++
 */
inline mav_mode_custom_t operator |=(mav_mode_custom_t a, mav_mode_custom_t b)
{	
	return static_cast<mav_mode_custom_t>(static_cast<int>(a) | static_cast<int>(b));
}

inline mav_mode_custom_t operator&=(mav_mode_custom_t a, mav_mode_custom_t b)
{	
	return static_cast<mav_mode_custom_t>(static_cast<int>(a) & static_cast<int>(b));
}

inline mav_mode_custom_t operator~(mav_mode_custom_t a)
{	
	return static_cast<mav_mode_custom_t>(~static_cast<int>(a));
}


/*
 * \brief Returns whether motors are armed or not
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if motor are armed, false otherwise
 */
static inline bool mav_modes_is_armed(const mav_mode_t mav_mode)
{
	if( (mav_mode&MAV_MODE_FLAG_SAFETY_ARMED)==MAV_MODE_FLAG_SAFETY_ARMED )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * \brief Returns whether the MAV is in simulation mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in simulation mode, false otherwise
 */
static inline bool mav_modes_is_hil(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_HIL_ENABLED) == MAV_MODE_FLAG_HIL_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/*
 * \brief Returns whether MAV is in manual piloting mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in manual piloting mode, false otherwise
 */
static inline bool mav_modes_is_manual(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) == MAV_MODE_FLAG_MANUAL_INPUT_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/*
 * \brief Returns whether MAV is in stabilise piloting mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in stabilise piloting mode, false otherwise
 */
static inline bool mav_modes_is_stabilise(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_STABILIZE_ENABLED) == MAV_MODE_FLAG_STABILIZE_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/*
 * \brief Returns whether MAV is in hover piloting mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in hover piloting mode, false otherwise
 */
static inline bool mav_modes_is_guided(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_GUIDED_ENABLED) == MAV_MODE_FLAG_GUIDED_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * \brief Returns whether MAV is in waypoint navigation piloting mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in waypoint navigation piloting mode, false otherwise
 */
static inline bool mav_modes_is_auto(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_AUTO_ENABLED) == MAV_MODE_FLAG_AUTO_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/*
 * \brief Returns whether MAV is in test mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in test mode, false otherwise
 */
static inline bool mav_modes_is_test(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_TEST_ENABLED) == MAV_MODE_FLAG_TEST_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}


/*
 * \brief Returns whether MAV is in custom mode
 * 
 * \param mav_mode	correspond to the mode in which the MAV is
 *
 * \return true if MAV is in custom mode, false otherwise
 */
static inline bool mav_modes_is_custom(const mav_mode_t mav_mode)
{
	if ( (mav_mode&MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * \brief compare two MAV modes
 * 
 * \param mode1	correspond to one mode in which the MAV may be
 * \param mode2	correspond to an other mode in which the MAV may be
 *
 * \return true if modes are equal, false otherwise
 */
static inline bool mav_modes_are_equal(const mav_mode_t mode1, const mav_mode_t mode2)
{
	if ( mode1 == mode2 )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * \brief compare two MAV modes, whatever simulation flag is set or not
 * 
 * \param mode1	correspond to one mode in which the MAV may be
 * \param mode2	correspond to an other mode in which the MAV may be
 *
 * \return true if modes are equal, false otherwise
 */
static inline bool mav_modes_are_equal_wo_hil(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_ &= ~MAV_MODE_FLAG_HIL_ENABLED;
	mode2_ &= ~MAV_MODE_FLAG_HIL_ENABLED;
	
	return mav_modes_are_equal(mode1_, mode2_);
}


/*
 * \brief compare two MAV modes, whatever armed flag is set or not
 * 
 * \param mode1	correspond to one mode in which the MAV may be
 * \param mode2	correspond to an other mode in which the MAV may be
 *
 * \return true if modes are equal, false otherwise
 */
static inline bool mav_modes_are_equal_wo_armed(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_ &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	mode2_ &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	
	return mav_modes_are_equal(mode1_, mode2_);
}


/*
 * \brief compare two MAV modes, whatever simulation and armed flag is set or not
 * 
 * \param mode1	correspond to one mode in which the MAV may be
 * \param mode2	correspond to an other mode in which the MAV may be
 *
 * \return true if modes are equal, false otherwise
 */
static inline bool mav_modes_are_equal_wo_hil_and_armed(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_ &= ~MAV_MODE_FLAG_HIL_ENABLED;
	mode2_ &= ~MAV_MODE_FLAG_HIL_ENABLED;
	mode1_ &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	mode2_ &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	
	return mav_modes_are_equal(mode1_, mode2_);
}

static inline bool mav_modes_are_equal_autonomous_modes(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_ &= 0b00011100;
	mode2_ &= 0b00011100;
	
	return mav_modes_are_equal(mode1_, mode2_);
}

#endif //MAV_MODE_H