/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file mav_mode.h
 *
 *  Place where the mav modes and flags aredefined
 */


#ifndef MAV_MODE_H
#define MAV_MODE_H

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
	ARMED_OFF = 0,
	ARMED_ON  = 1,
} mode_flag_armed_t;


typedef enum
{
	MANUAL_OFF = 0,
	MANUAL_ON  = 1,
} mode_flag_manual_t;


typedef enum
{
	HIL_OFF = 0,			
	HIL_ON  = 1,
} mode_flag_hil_t;


typedef enum
{
	STABILISE_OFF = 0,
	STABILISE_ON  = 1,
} mode_flag_stabilise_t;


typedef enum
{
	GUIDED_OFF = 0,
	GUIDED_ON  = 1,
} mode_flag_guided_t;


typedef enum
{
	AUTO_OFF = 0,
	AUTO_ON  = 1,
} mode_flag_auto_t;


typedef enum
{
	TEST_OFF = 0,
	TEST_ON  = 1,
} mode_flag_test_t;


typedef enum
{
	CUSTOM_OFF = 0,
	CUSTOM_ON  = 1,
} mode_flag_custom_t;


typedef struct 
{
    mode_flag_custom_t 	  CUSTOM       : 1;
    mode_flag_test_t 	  TEST         : 1;
    mode_flag_auto_t 	  AUTO         : 1;
    mode_flag_guided_t 	  GUIDED       : 1;
    mode_flag_stabilise_t STABILISE    : 1;
    mode_flag_hil_t 	  HIL          : 1;
    mode_flag_manual_t 	  MANUAL       : 1;
    mode_flag_armed_t 	  ARMED        : 1;
} mav_mode_bitfield_t; 


typedef union
{
	uint8_t byte;
	mav_mode_bitfield_t flags;
} mav_mode_t;


typedef enum
{
	MODE_FLAG_CUSTOM    = 0,
	MODE_FLAG_TEST      = 1,
	MODE_FLAG_AUTO      = 2,
	MODE_FLAG_GUIDED    = 3,
	MODE_FLAG_STABILISE = 4,
	MODE_FLAG_HIL       = 5,
	MODE_FLAG_MANUAL    = 6,
	MODE_FLAG_ARMED     = 7,
} mode_flags_t;


typedef enum MAV_MODE_FLAG mav_flag_mask_t;


typedef enum
{
	MAV_MODE_PRE = 0,
	MAV_MODE_SAFE = 64,
	MAV_MODE_ATTITUDE_CONTROL = 192,
	MAV_MODE_VELOCITY_CONTROL = 208,
	MAV_MODE_POSITION_HOLD = 216,
	MAV_MODE_GPS_NAVIGATION = 148
} mav_mode_predefined_t;


static inline bool mav_modes_is_armed(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.ARMED == ARMED_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_hil(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.HIL == HIL_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_manual(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.MANUAL == MANUAL_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_stabilise(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.STABILISE == STABILISE_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_guided(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.GUIDED == GUIDED_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_auto(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.AUTO == AUTO_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_test(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.TEST == TEST_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_is_custom(const mav_mode_t mav_mode)
{
	if ( mav_mode.flags.CUSTOM == CUSTOM_ON )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_are_equal(const mav_mode_t mode1, const mav_mode_t mode2)
{
	if ( mode1.byte == mode2.byte )
	{
		return true;
	}
	else
	{
		return false;
	}
}


static inline bool mav_modes_are_equal_wo_hil(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_.flags.HIL = HIL_OFF;
	mode2_.flags.HIL = HIL_OFF;
	
	return mav_modes_are_equal(mode1_, mode2_);
}


static inline bool mav_modes_are_equal_wo_armed(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_.flags.ARMED = ARMED_OFF;
	mode2_.flags.ARMED = ARMED_OFF;
	
	return mav_modes_are_equal(mode1_, mode2_);
}


static inline bool mav_modes_are_equal_wo_hil_and_armed(const mav_mode_t mode1, const mav_mode_t mode2)
{
	mav_mode_t mode1_ = mode1;
	mav_mode_t mode2_ = mode2;
	
	mode1_.flags.HIL = HIL_OFF;
	mode2_.flags.HIL = HIL_OFF;
	mode1_.flags.ARMED = ARMED_OFF;
	mode2_.flags.ARMED = ARMED_OFF;
	
	return mav_modes_are_equal(mode1_, mode2_);
}


static inline mav_mode_t mav_modes_get_from_flags(const mode_flag_armed_t armed, const mode_flag_hil_t hil, const mode_flag_manual_t manual, const mode_flag_stabilise_t stabilise, const mode_flag_guided_t guided, const mode_flag_auto_t autoo, const mode_flag_test_t test, const mode_flag_custom_t custom)
{
	mav_mode_t mode;

	mode.flags.ARMED     = armed;
	mode.flags.HIL       = hil;
	mode.flags.MANUAL    = manual;
	mode.flags.STABILISE = stabilise;
	mode.flags.GUIDED    = guided;
	mode.flags.AUTO      = autoo;
	mode.flags.TEST      = test;
	mode.flags.CUSTOM    = custom;

	return mode;
}

#endif