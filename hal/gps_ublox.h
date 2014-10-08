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
 * \file gps_ublox.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file decodes the messages from the UBLOX GPS
 * 
 ******************************************************************************/


#ifndef GPS_UBLOX_H__
#define GPS_UBLOX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "maths.h"
#include "streams.h"
#include "buffer.h"

/*
 *  try to put a UBlox into binary mode. This is in two parts. First we
 *  send a PUBX asking the UBlox to receive NMEA and UBX, and send UBX,
 *  with a baudrate of 38400. Then we send a UBX message setting rate 1
 *  for the NAV_SOL message. The setup of NAV_SOL is to cope with
 *  configurations where all UBX binary message types are disabled.
 * 0001: UBX
 * 0002: NMEA
 * 0003: NMEA + UBX
 */
//#define UBLOX_SET_BINARY "$PUBX,41,1,0003,0001,38400,0*26\n\265\142\006\001\003\000\001\006\001\022\117"

// changed by ndousse
#define UBLOX_SET_BINARY "$PUBX,41,1,0003,0001,38400,0*25\r\n"//\265\142\006\001\003\000\001\006\001\022\117"

/*
Structure of an UBlox binary message

PREAMBLE1		PREAMBLE2		CLASS		MSG_ID		LENGTH1			LENGTH2			MESSAGE			CHECKSUMA		CHECKSUMB
1 byte			1 byte			1 byte		1 byte		Lowest byte		Highest byte	LENGTH size		1 byte			1 byte

The information is received in the Little endian format (least significant byte first).
*/

#define UBX_PREAMBLE1 0xb5
#define UBX_PREAMBLE2 0x62

/*
Name Class Description
NAV 0x01 Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
RXM 0x02 Receiver Manager Messages: Satellite Status, RTC Status
INF 0x04 Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
ACK 0x05 Ack/Nack Messages: as replies to CFG Input Messages
CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
MON 0x0A Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
AID 0x0B AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
TIM 0x0D Timing Messages: Timepulse Output, Timemark Results
*/
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_AID 0x0B
#define UBX_CLASS_TIM 0x0D
#define UBX_CLASS_ESF 0x10

// Type of Messages that can be received in each class
#define MSG_ACK_NACK 0x00
#define MSG_ACK_ACK 0x01

#define MSG_INF_ERROR 0x00
#define MSG_INF_WARNING 0x01
#define MSG_INF_NOTICE 0x02
#define MSG_INF_TEST 0x03
#define MSG_INF_DEBUG 0x04

#define MSG_NAV_POSLLH 0x02
#define MSG_NAV_STATUS 0x03
#define MSG_NAV_AOPSTATUS 0x60
#define MSG_NAV_CLOCK 0x22
#define MSG_NAV_SOL 0x06
#define MSG_NAV_VELNED 0x12
#define MSG_NAV_VELCEF 0x11
#define MSG_NAV_TIMEGPS 0x20
#define MSG_NAV_TIMEUTC 0x21
#define MSG_NAV_SVINFO 0x30

#define MSG_CFG_PRT 0x00
#define MSG_CFG_RATE 0x08
#define MSG_CFG_SET_RATE 0x01
#define MSG_CFG_NAV_SETTINGS 0x24
#define MSG_CFG_NMEA 0x17

#define MSG_MON_RXR 0x21

#define MSG_TIM_TP 0x01
#define MSG_TIM_VRFY 0x06

#define UBX_PLATFORM_PORTABLE 0x00
#define UBX_PLATFORM_STATIONARY 0x02
#define UBX_PLATFORM_PEDESTRIAN 0x03
#define UBX_PLATFORM_AUTO   0x04
#define UBX_PLATFORM_SEA    0x05
#define UBX_PLATFORM_1GAIR  0x06
#define UBX_PLATFORM_2GAIR  0x07
#define UBX_PLATFORM_4GAIR  0x08

#define GPS_FIX_TYPE_NOFIX 0x00
#define GPS_FIX_TYPE_DEADRECK 0x01
#define GPS_FIX_TYPE_2DFIX 0x02
#define GPS_FIX_TYPE_3DFIX 0x03
#define GPS_FIX_TYPE_GPSDEADRECK 0x04
#define GPS_FIX_TYPE_TIMEONLY 0x05

// For binary message
#define UBX_CFG_MSG 0xF1
#define UBX_CFG_MSG_ID 0x41

// Sizes
#define UBX_SIZE_NAV_POSLLH 28
#define UBX_SIZE_NAV_STATUS 16
#define UBX_SIZE_NAV_SOL 52
#define UBX_SIZE_NAV_VELNED 36
#define UBX_SIZE_NAV_SVINFO 30 //8 + 12*num_channel
#define UBX_SIZE_NAV_SETTINGS 36

#define UBX_SIZE_CFG_RATE 6
#define UBX_SIZE_CFG_GETSET_RATE 3

#define UBX_SIZE_MON_RXR 1

#define UBX_SIZE_TIM_TP 16
#define UBX_SIZE_TIM_VRFY 20

#define NAV_STATUS_FIX_NVALID 0
#define NAV_STATUS_FIX_VALID 1

//epoch
#define TIME_OF_DAY 0 //<
#define TIME_OF_WEEK 1 //< Ublox
#define TIME_OF_YEAR 2 //< MTK, NMEA
#define UNIX_EPOCH 3

#define DEG2RAD PI/180

// The UART bytes are sent in a little endian format from the GPS, if the processor is big endian, define BIG_ENDIAN
// Otherwise comment the following line
#define BIG_ENDIAN

#ifdef BIG_ENDIAN
	/**
	 * \brief The U-Blox header structure definition
	 */
	typedef struct
	{
		uint16_t length;					///< The length of the message
		uint8_t msg_id_header;				///< The msg id header
		uint8_t msg_class;					///< The class of the message
		uint8_t preamble2;					///< The 2nd preamble of the message
		uint8_t preamble1;					///< The 1st preamble of the message
	}ubx_header_t;

	/**
	 * \brief The U-Blox CFG_NAV structure definition
	 */
	typedef struct
	{
		uint16_t timeref;					///< The time reference
		uint16_t nav_rate;					///< The rate
		uint16_t measure_rate_ms;			///< The measure rate of the cfg_nav message in ms
	}ubx_cfg_nav_rate_t;
	
	// We still have to send to message in the correct order to the GPS
	/**
	 * \brief The U-Blox CFG_NAV rate send structure definition
	 */
	typedef struct
	{
		uint16_t measure_rate_ms;			///< The measure_rate
		uint16_t nav_rate;					///< The rate
		uint16_t timeref;					///< The time reference, 0:UTC time, 1:GPS time
	}ubx_cfg_nav_rate_send_t;

	/**
	 * \brief The U-Blox CFG_MSG rate structure definition
	 */
	typedef struct
	{
		uint8_t rate;						///< The rate
		uint8_t msg_id_rate;				///< The msg id
		uint8_t msg_class;					///< The msg_class
	}ubx_cfg_msg_rate_t;

	// We still have to send to message in the correct order to the GPS
	/**
	 * \brief The U-Blox CFG_MSG rate send structure definition
	 */
	typedef struct
	{
		uint8_t msg_class;					///< The msg class
		uint8_t msg_id_rate;				///< The msg id
		uint8_t rate;						///< The rate of the message id
	}ubx_cfg_msg_rate_send_t;

	/**
	 * \brief The U-Blox CFG_NAV settings structure definition
	 */
	typedef struct
	{
		uint32_t res4;						///< Reserved slot
		uint32_t res3;						///< Reserved slot
		uint32_t res2;						///< Reserved slot
		uint8_t dgps_timeout;				///< DGPS timeout in sec
		uint8_t static_hold_thresh;			///< Static hold threshold cm/s
		uint16_t t_acc;						///< Time accuracy mask in m
		uint16_t p_acc;						///< Position accuracy mask in m
		uint16_t t_dop;						///< Time DOP mask to use
		uint16_t p_dop;						///< Position DOP mask to use
		uint8_t dr_limit;					///< Maximum time to perform dead reckoning in case of GPS signal loos, in sec
		int8_t min_elev;						///< Minimum elevation for a GNSS satellite to be used in NAV in deg
		uint32_t fixed_alt_var;				///< Fixed altitude variance in 2D mode in m^2
		int32_t fixed_alt;					///< Fixed altitude for 2D fix mode in m
		uint8_t fix_mode;					///< Fixing mode, 1:2D, 2:3D, 3:auto 2D/3D
		uint8_t dyn_model;					///< UBX_PLATFORM_... type
		uint16_t mask;						///< Bitmask, see U-Blox 6 documentation
	}ubx_cfg_nav_settings_t;

	/**
	 * \brief The U-Blox NAV-POSLLH message structure definition
	 */
	typedef struct
	{
		uint32_t vertical_accuracy;			///< Vertical accuracy in mm
		uint32_t horizontal_accuracy;		///< Horizontal accuracy in mm
		int32_t altitude_msl;				///< Height above mean sea level in mm
		int32_t altitude_ellipsoid;			///< Height above ellipsoid in mm
		int32_t latitude;					///< Latitude in deg 1e-7
		int32_t longitude;					///< Longitude in deg 1e-7
		uint32_t itow;						///< GPS msToW
	}ubx_nav_pos_llh_t;

	/**
	 * \brief The U-Blox NAV-STATUS message structure definition
	 */
	typedef struct
	{
		uint32_t uptime;					///< Milliseconds since startup
		uint32_t time_to_first_fix;			///< Time to first fix in milliseconds
		uint8_t flags2;						///< Information about navigatio output
		uint8_t fix_status;					///< Fix status information
		uint8_t flags;						///< Nav status flag
		uint8_t fix_type;					///< Fix type
		uint32_t itow;						///< GPS msToW
	}ubx_nav_status_t;

	/**
	 * \brief The U-Blox NAV-SOL message structure definition
	 */
	typedef struct
	{
		uint32_t res2;						///< Reserved slot
		uint8_t satellites;					///< Number of of SVs used in Nav solution
		uint8_t res;						///< Reserved slot
		uint16_t position_DOP;				///< Position DOP, scaling 0.01f
		uint32_t speed_accuracy;			///< Speed accuracy estimate in cm/s
		int32_t ecef_z_velocity;			///< Earth centered, earth frame, z velocity coordinate in cm/s
		int32_t ecef_y_velocity;			///< Earth centered, earth frame, y velocity coordinate in cm/s
		int32_t ecef_x_velocity;			///< Earth centered, earth frame, x velocity coordinate in cm/s
		uint32_t position_accuracy_3d;		///< 3D position accuracy estimate in cm
		int32_t ecef_z;						///< Earth centered, earth frame, z coordinate in cm
		int32_t ecef_y;						///< Earth centered, earth frame, y coordinate in cm
		int32_t ecef_x;						///< Earth centered, earth frame, x coordinate in cm
		uint8_t fix_status;					///< The fix status
		uint8_t fix_type;					///< The fix type
		int16_t week;						///< GPS week (GPS time)
		int32_t time_nsec;					///< Fractional nanoseconds remainder of rounded ms above
		uint32_t itow;						///< GPS msToW
	}ubx_nav_solution_t;

	/**
	 * \brief The U-Blox NAV-VELNED message structure definition
	 */
	typedef struct
	{
		uint32_t heading_accuracy;			///< Course/heading estimate accuracy in deg 1e-5
		uint32_t speed_accuracy;			///< Speed accuracy estimate cm/s
		int32_t heading_2d;					///< Heading of motion in deg 1e-5
		uint32_t ground_speed_2d;			///< Ground speed in cm/s
		uint32_t speed_3d;					///< 3D speed in cm/s
		int32_t ned_down;					///< NED Down velocity in cm/s
		int32_t ned_east;					///< NED East velocity in cm/s
		int32_t ned_north;					///< NED North velocity in cm/s
		uint32_t itow;						///< GPS msToW
	}ubx_nav_vel_ned_t;

	/**
	 * \brief The U-Blox NAV-SVINFO message structure definition
	 */
	typedef struct
	{
		/**
		* \brief The structure definition defining a specific message from a GPS satellite
		*/
		struct
		{
			int32_t pr_res;					///< Pseudo range in residual in centimeters
			int16_t azim;					///< Azimuth in integer degrees
			int8_t elev;					///< Elevation in integer degrees
			uint8_t cno;					///< Carrier to Noise ratio in dbHz
			uint8_t quality;				///< Bitmask, see U-Blox 6 documentation
			uint8_t flags;					///< Bitmask, see U-Blox 6 documentation
			uint8_t svid;					///< Satellite ID
			uint8_t chn;					///< GPS msToW
		} channel_data[16];
		
		uint16_t reserved;					///< Reserved slot
		uint8_t global_flags;				///< Bitmask, 0:antaris, 1:u-blox 5, 2:u-blox 6
		uint8_t num_ch;						///< Number of channels
		uint32_t itow;						///< GPS msToW
	}ubx_nav_sv_info_t;

	/**
	 * \brief The U-Blox MON-RXR message structure definition
	 */
 	typedef struct
	 {
 		uint8_t awake_flag;					///< Receiver status flag
 	}ubx_mon_rxr_struct_t;

	/**
	 * \brief The U-Blox TIM-TP message structure definition
	 */
	typedef struct
	{
		uint8_t res;						///< Unused
		uint8_t flags;						///< Bitmask, 0,1:gps timebase, UTC not available, 2,3:UTC timebase, UTC available
		uint16_t week;						///< Timepulse week number according to timebase
		int32_t q_err;						///< Quantization error of timepulse
		uint32_t tow_sub_ms;					///< Sumbmillisecond part of ToWms scaling: 2^-32
		uint32_t tow_ms;						///< Timepulse time of week according to time base in ms
	}ubx_tim_tp_t;

	/**
	 * \brief The U-Blox TIM-VRFY message structure definition
	 */
	typedef struct
	{
		uint8_t res;						///< Reserved slot
		uint8_t flags;						///< Aiding time source, 0:no time aiding done, 2:source was RTC, 3:source was AID-INI
		uint16_t wno;						///< Week number
		int32_t delta_ns;					///< Sub-millisecond part of delta time
		int32_t delta_ms;					///< Inter ms of delta time
		int32_t frac;						///< Sub-millisecond part of ToW in ns
		int32_t itow;						///< Integer ms ToW received by source
	}ubx_tim_vrfy_t;

#else	

	/**
	 * \brief The U-Blox header structure definition
	 */
	typedef struct
	{
		uint8_t preamble1;					///< The 1st preamble of the message
		uint8_t preamble2;					///< The 2nd preamble of the message
		uint8_t msg_class;					///< The class of the message
		uint8_t msg_id_header;				///< The msg id header
		uint16_t length;					///< The length of the message
	}ubx_header_t;

	/**
	 * \brief The U-Blox CFG-NAV structure definition
	 */
	typedef struct
	{
		uint16_t measure_rate_ms;			///< The measure rate of the cfg_nav message in ms
		uint16_t nav_rate;					///< The rate
		uint16_t timeref;					///< The time reference
	}ubx_cfg_nav_rate_t;

	/**
	 * \brief The U-Blox CFG-NAV rate structure definition
	 */
	typedef struct
	{
		uint8_t msg_class;					///< The msg_class
		uint8_t msg_id_rate;				///< The msg id
		uint8_t rate;						///< The rate
	}ubx_cfg_msg_rate_t;

	/**
	 * \brief The U-Blox CFG-NAV setttings structure definition
	 */
	typedef struct
	{
		uint16_t mask;						///< Bitmask, see U-Blox 6 documentation
		uint8_t dyn_model;					///< UBX_PLATFORM_... type
		uint8_t fix_mode;					///< Fixing mode, 1:2D, 2:3D, 3:auto 2D/3D
		int32_t fixed_alt;					///< Fixed altitude for 2D fix mode in m
		uint32_t fixed_alt_var;				///< Fixed altitude variance in 2D mode in m^2
		int8_t min_elev;						///< Minimum elevation for a GNSS satellite to be used in NAV in deg
		uint8_t dr_limit;					///< Maximum time to perform dead reckoning in case of GPS signal loos, in sec
		uint16_t p_dop;						///< Position DOP mask to use
		uint16_t t_dop;						///< Time DOP mask to use
		uint16_t p_acc;						///< Position accuracy mask in m
		uint16_t t_acc;						///< Time accuracy mask in m
		uint8_t static_hold_thresh;			///< Static hold threshold cm/s
		uint8_t dgps_timeout;				///< DGPS timeout in sec
		uint32_t res2;						///< Reserved slot
		uint32_t res3;						///< Reserved slot
		uint32_t res4;						///< Reserved slot
	}ubx_cfg_nav_settings_t;

	/**
	 * \brief The U-Blox NAV-POSLLH message structure definition
	 */
	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		int32_t longitude;					///< Longitude in deg 1e-7
		int32_t latitude;					///< Latitude in deg 1e-7
		int32_t altitude_ellipsoid;			///< Height above ellipsoid in mm
		int32_t altitude_msl;				///< Height above mean sea level in mm
		uint32_t horizontal_accuracy;		///< Horizontal accuracy in mm
		uint32_t vertical_accuracy;			///< Vertical accuracy in mm
	}ubx_nav_pos_llh_t;

	/**
	 * \brief The U-Blox NAV-STATUS message structure definition
	 */
	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		uint8_t fix_type;					///< Fix type
		uint8_t flags;						///< Nav status flag
		uint8_t fix_status;					///< Fix status information
		uint8_t flags2;						///< Information about navigatio output
		uint32_t time_to_first_fix;			///< Time to first fix in milliseconds
		uint32_t uptime;					///< Milliseconds since startup
	}ubx_nav_status_t;

	/**
	 * \brief The U-Blox NAV-SOL message structure definition
	 */
	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		int32_t time_nsec;					///< Fractional nanoseconds remainder of rounded ms above
		int16_t week;						///< GPS week (GPS time)
		uint8_t fix_type;					///< The fix type
		uint8_t fix_status;					///< The fix status
		int32_t ecef_x;						///< Earth centered, earth frame, x coordinate in cm
		int32_t ecef_y;						///< Earth centered, earth frame, y coordinate in cm
		int32_t ecef_z;						///< Earth centered, earth frame, z coordinate in cm
		uint32_t position_accuracy_3d;		///< 3D position accuracy estimate in cm
		int32_t ecef_x_velocity;			///< Earth centered, earth frame, x velocity coordinate in cm/s
		int32_t ecef_y_velocity;			///< Earth centered, earth frame, y velocity coordinate in cm/s
		int32_t ecef_z_velocity;			///< Earth centered, earth frame, z velocity coordinate in cm/s
		uint32_t speed_accuracy;			///< Speed accuracy estimate in cm/s
		uint16_t position_DOP;				///< Position DOP, scaling 0.01f
		uint8_t res;						///< Reserved slot
		uint8_t satellites;					///< Number of of SVs used in Nav solution
		uint32_t res2;						///< Reserved slot
	}ubx_nav_solution_t;

	/**
	 * \brief The U-Blox NAV-VELNED message structure definition
	 */
	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		int32_t ned_north;					///< NED North velocity in cm/s
		int32_t ned_east;					///< NED East velocity in cm/s
		int32_t ned_down;					///< NED Down velocity in cm/s
		uint32_t speed_3d;					///< 3D speed in cm/s
		uint32_t ground_speed_2d;			///< Ground speed in cm/s
		int32_t heading_2d;					///< Heading of motion in deg 1e-5
		uint32_t speed_accuracy;			///< Speed accuracy estimate cm/s
		uint32_t heading_accuracy;			///< Course/heading estimate accuracy in deg 1e-5
	}ubx_nav_vel_ned_t;

	/**
	 * \brief The U-Blox NAV-SVINFO message structure definition
	 */
	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		uint8_t num_ch;						///< Number of channels
		uint8_t global_flags;				///< Bitmask, 0:antaris, 1:u-blox 5, 2:u-blox 6
		uint16_t reserved;					///< Reserved slot
	
		/**
		 * \brief The structure definition defining a specific message from a GPS satellite
		*/
		struct
		{
			uint8_t chn;					///< GPS msToW
			uint8_t svid;					///< Satellite ID
			uint8_t flags;					///< Bitmask, see U-Blox 6 documentation
			uint8_t quality;				///< Bitmask, see U-Blox 6 documentation
			uint8_t cno;					///< Carrier to Noise ratio in dbHz
			int8_t elev;					///< Elevation in integer degrees
			int16_t azim;					///< Azimuth in integer degrees
			int32_t pr_res;					///< Pseudo range in residual in centimeters
		} channel_data[16];
	}ubx_nav_sv_info_t;

	/**
	 * \brief The U-Blox MON-RXR message structure definition
	 */
	typedef struct
	{
		uint8_t awake_flag;					///< Receiver status flag
	}ubx_mon_rxr_struct_t;

	/**
	 * \brief The U-Blox TIM-TP message structure definition
	 */
	typedef struct  
	{
		uint32_t tow_ms;						///< Timepulse time of week according to time base in ms
		uint32_t tow_sub_ms;					///< Sumbmillisecond part of ToWms scaling: 2^-32
		int32_t q_err;						///< Quantization error of timepulse
		uint16_t week;						///< Timepulse week number according to timebase
		uint8_t flags;						///< Bitmask, 0,1:gps timebase, UTC not available, 2,3:UTC timebase, UTC available
		uint8_t res;						///< Unused
	}ubx_tim_tp_t;

	/**
	 * \brief The U-Blox TIM-VRFY message structure definition
	 */
	typedef struct
	{
		int32_t itow;						///< Integer ms ToW received by source
		int32_t frac;						///< Sub-millisecond part of ToW in ns
		int32_t delta_ms;					///< Inter ms of delta time
		int32_t delta_ns;					///< Sub-millisecond part of delta time
		uint16_t wno;						///< Week number
		uint8_t flags;						///< Aiding time source, 0:no time aiding done, 2:source was RTC, 3:source was AID-INI
		uint8_t res;						///< Reserved slot
	}ubx_tim_vrfy_t;

#endif

ubx_cfg_nav_settings_t nav_settings;		///< CFG-NAV settings structure

uint8_t cksum_a;							///< Checksum a
uint8_t cksum_b;							///< Checksum b

// State machine state
uint8_t         step;						///< Variable defining the state machine in the U-Blox decoding function
uint8_t         msg_id;						///< The U-Blox message ID
uint16_t        payload_length;				///< The length of the message
uint16_t        payload_counter;			///< The incremental counter to receive bytes of data

uint8_t         ubx_class;///< The U-Blox message class

// do we have new position and speed information?
bool new_position;							///< Boolean value to check if we received new position message
bool new_speed;								///< Boolean value to check if we received new velocity message

uint8_t disable_counter;					///< Counter used to deactivate unwanted messages

bool next_fix;								///< Boolean variable to get whether we have a correct GPS fix or not
bool have_raw_velocity;						///< Boolean variable that could be used to get a speed approximate with heading and 2D velocity

#define NO_GPS 0							///< No GPS
#define NO_FIX 1							///< No GPS fix
#define GPS_OK 2							///< GPS ok

typedef enum {
	GPS_ENGINE_NONE        = -1,			///< None
	GPS_ENGINE_PORTABLE    = 0,				///< Portable
	GPS_ENGINE_STATIONARY  = 2,				///< Stationary
	GPS_ENGINE_PEDESTRIAN  = 3,				///< Pedestrian
	GPS_ENGINE_AUTOMOTIVE  = 4,				///< Automotive
	GPS_ENGINE_SEA         = 5,				///< Sea
	GPS_ENGINE_AIRBORNE_1G = 6,				///< Airborne with <1g acceleration
	GPS_ENGINE_AIRBORNE_2G = 7,				///< Airborne with <2g acceleration
	GPS_ENGINE_AIRBORNE_4G = 8				///< Airborne with <4g acceleration
}gps_engine_setting_t;

gps_engine_setting_t engine_nav_setting;		///< Enum GPS engine setting


#define UBX_TIMEOUT_CYCLES 2				///< Number of times ubx_CheckTimeout() must be called without response from GPS before it is considered as timed out
#define UBX_POSITION_PRECISION 20			///< The minimum precision to consider a position as correct (in m)
#define UBX_ALTITUDE_PRECISION 20			///< The minimum precision to consider an altitude as correct (in m)
#define UBX_SPEED_PRECISION 5				///< The minimum precision to consider a speed as correct (in m/s)

#define UBX_HEADING_PRECISION 5000000		///< The minimum precision to consider a heading as correct (in deg*10^5)


/**
 * \brief Type definition for GPS data
 */
typedef struct
{
	double latitude;						///< Latitude in degrees
	double longitude;						///< Longitude in degrees
	float altitude;							///< Altitude in m
	float alt_elips;						///< Altitude above ellipsoid in m
	float speed;							///< 3D speed in m/s
	float ground_speed;						///< 2D ground speed in m/s
	float north_speed;						///< The speed to the north in m/s
	float east_speed;						///< The speed to the east in m/s
	float vertical_speed;					///< The vertical speed in m/s
	float course;							///< Heading in degree * 100
	
	float horizontal_accuracy;				///< Horizontal accuracy in m
	float vertical_accuracy;					///< Vertical accuracy in m
	
	float speed_accuracy;					///< Speed accuracy in m
	float heading_accuracy;					///< Heading accuracy in m
	
	uint8_t num_sats;						///< Number of visible satellites
	uint16_t hdop;							///< Height DOP
	
	uint32_t time_last_msg;					///< Time reference in ms of microcontroller
	uint32_t time_gps;						///< Time reference in ms of gps
	
	uint8_t  status;						///< GPS status
	
	uint8_t  horizontal_status;				///< Horizontal status
	
 	uint8_t  altitude_status;				///< Altitude status
 	uint8_t  speed_status;					///< Speed status
 	uint8_t  course_status;					///< Course status
 	uint8_t  accuracy_status;				///< Accuracy status
	
	buffer_t gps_buffer;					///< The GPS buffer
	byte_stream_t gps_stream_in;			///< The incoming GPS byte stream
	byte_stream_t gps_stream_out;			///< The outgoing GPS byte stream
} gps_t;


uint32_t idle_timer;							///< Last time that the GPS driver got a good packet from the GPS
uint32_t idle_timeout;						///< Time in milliseconds after which we will assume the GPS is no longer sending us updates and attempt a re-init. 1200ms allows a small amount of slack over the worst-case 1Hz update rate.
uint32_t last_fix_time;						///< Last fix time


/**
 * \brief	Initialize the gps U-Blox module
 *
 * \param	gps			The pointer to the GPS structure
 * \param	UID					The uart ID
 */
void gps_ublox_init(gps_t *gps, int32_t UID);


/**
 * \brief	To configure the GPS in binary mode and the Navigation messages we want
 *
 * The GPS and UART channel should already be configured in the good baudrate 38400U
 *
 * \param	gps			The pointer to the GPS structure
 */
void gps_ublox_configure_gps(gps_t *gps);


/**
 * \brief	The function that needs to be called to get the GPS information
 *
 * \param	gps			The pointer to the GPS structure
 */
void gps_ublox_update(gps_t *gps);

#ifdef __cplusplus
}
#endif

#endif //GPS_UBLOX_H__