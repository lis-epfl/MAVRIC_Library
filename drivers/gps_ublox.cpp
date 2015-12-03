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
 * \file gps_ublox.cpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Driver for UBLOX GPS
 * 
 *  \detail 	Only one instance can be used
 * 				TODO: remove global variables in gps_ublox.cpp
 * 
 ******************************************************************************/


#include "gps_ublox.hpp"

extern "C"
{
	#include "print_util.h"
	#include "time_keeper.h"
	#include "endianness.h"
	#include <string.h>
	#include "maths.h"
}


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

#define MSG_NAV_POSECEF 0x01
#define MSG_NAV_POSLLH 0x02
#define MSG_NAV_STATUS 0x03
#define MSG_NAV_DOP 0x04
#define MSG_NAV_SOL 0x06
#define MSG_NAV_VELCEF 0x11
#define MSG_NAV_VELNED 0x12
#define MSG_NAV_TIMEGPS 0x20
#define MSG_NAV_TIMEUTC 0x21
#define MSG_NAV_CLOCK 0x22
#define MSG_NAV_SVINFO 0x30
#define MSG_NAV_DGPS 0x31
#define MSG_NAV_SBAS 0x32
#define MSG_NAV_EKFSTATUS 0x40
#define MSG_NAV_AOPSTATUS 0x60

#define MSG_CFG_SET_RATE 0x01
#define MSG_CFG_NAV_SETTINGS 0x24
#define MSG_CFG_NAV_EXPERT_SETTINGS 0x23
#define MSG_CFG_NMEA 0x17
#define MSG_CFG_PM 0x32
#define MSG_CFG_PM2 0x3B
#define MSG_CFG_PRT 0x00
#define MSG_CFG_RATE 0x08
#define MSG_CFG_RINV 0x34
#define MSG_CFG_RXM 0x11
#define MSG_CFG_SBAS 0x16
#define MSG_CFG_TP 0x07
#define MSG_CFG_TP5 0x31
#define MSG_CFG_USB 0x1B
#define MSG_CFG_ITFM 0x39
#define MSG_CFG_INF 0x02
#define MSG_CFG_FXN 0x0E
#define MSG_CFG_DAT 0x06
#define MSG_CFG_ANT 0x13
#define MSG_CFG_CFG 0x09

#define MSG_MON_HW2 0x0B
#define MSG_MON_HW 0x09
#define MSG_MON_IO 0x02
#define MSG_MON_MSGPP 0x06
#define MSG_MON_RXBUF 0x07
#define MSG_MON_RXR 0x21
#define MSG_MON_TXBUF 0x08
#define MSG_MON_VER 0x04

#define MSG_AID_ALM 0x30
#define MSG_AID_EPH 0x31
#define MSG_AID_ALPSRV 0x32
#define MSG_AID_AOP 0x33
#define MSG_AID_REQ 0x00

#define MSG_TIM_TM2 0x03
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
#define UBX_SIZE_NAV_SVINFO 200 //8 + 12*num_channel = 200
#define UBX_SIZE_NAV_TIMEUTC 20
#define UBX_SIZE_NAV_DGPS 272 // 16 + 16*num_channel = 272

#define UBX_SIZE_CFG_RATE 6
#define UBX_SIZE_CFG_GETSET_RATE 3
#define UBX_SIZE_CFG_NAV_SETTINGS 36
#define UBX_SIZE_CFG_NAV_EXPERT_SETTINGS 40
#define UBX_SIZE_CFG_PM 24
#define UBX_SIZE_CFG_PM2 44
#define UBX_SIZE_CFG_PRT 20
#define UBX_SIZE_CFG_RINV 24
#define UBX_SIZE_CFG_RXM 2
#define UBX_SIZE_CFG_SBAS 8
#define UBX_SIZE_CFG_TP 20
#define UBX_SIZE_CFG_TP5 32
#define UBX_SIZE_CFG_USB 108
#define UBX_SIZE_CFG_ITFM 8
#define UBX_SIZE_CFG_INF 10
#define UBX_SIZE_CFG_FXN 36
#define UBX_SIZE_CFG_DAT 2
#define UBX_SIZE_CFG_ANT 4
#define UBX_SIZE_CFG_CFG 13

#define UBX_SIZE_MON_RXR 1
#define UBX_SIZE_MON_VER 70

#define UBX_SIZE_TIM_TP 16
#define UBX_SIZE_TIM_VRFY 20

#define UBX_SIZE_ACK 2

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
// #define BIG_ENDIAN


	/**
	 * \brief The U-Blox CFG_NAV rate send structure definition
	 */
typedef struct
{
	uint16_t measure_rate_ms;			///< The measure_rate
	uint16_t nav_rate;					///< The rate
	uint16_t timeref;					///< The time reference, 0:UTC time, 1:GPS time
} ubx_cfg_nav_rate_send_t;




/**
 * \brief The U-Blox CFG_MSG rate send structure definition
 */
typedef struct
{
	uint8_t msg_class;					///< The msg class
	uint8_t msg_id_rate;				///< The msg id
	uint8_t rate;						///< The rate of the message id
} ubx_cfg_msg_rate_send_t;



/** 
 *\brief The U-Blox ACK-ACK and ACK-NACK message structure definition
 */
typedef struct  
{
	uint8_t msg_id;						///< Message ID of the acknowledged messages
	uint8_t class_id;					///< Class ID of the acknowledged messages
} ubx_ack_t;


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
} ubx_header_t;

/**
 * \brief The U-Blox CFG-NAV structure definition
 */
typedef struct
{
	uint16_t measure_rate_ms;			///< The measure rate of the cfg_nav message in ms
	uint16_t nav_rate;					///< The rate
	uint16_t timeref;					///< The time reference
} ubx_cfg_nav_rate_t;

/**
 * \brief The U-Blox CFG-NAV rate structure definition
 */
typedef struct
{
	uint8_t msg_class;					///< The msg_class
	uint8_t msg_id_rate;				///< The msg id
	uint8_t rate;						///< The rate
} ubx_cfg_msg_rate_t;

/**
 * \brief The U-Blox CFG-NAV5 settings structure definition
 */
typedef struct
{
	uint16_t mask;						///< Bitmask, see U-Blox 6 documentation
	uint8_t dyn_model;					///< UBX_PLATFORM_... type
	uint8_t fix_mode;					///< Fixing mode, 1:2D, 2:3D, 3:auto 2D/3D
	int32_t fixed_alt;					///< Fixed altitude for 2D fix mode in m
	uint32_t fixed_alt_var;				///< Fixed altitude variance in 2D mode in m^2
	int8_t min_elev;					///< Minimum elevation for a GNSS satellite to be used in NAV in deg
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
} ubx_cfg_nav_settings_t;

/**
 * \brief The U-Blox CFG-NAVX5 settings structure definition
 */
typedef struct  
{
	uint16_t version;					///< Message version
	uint16_t mask1;						///< First parameter bitmask
	uint16_t mak2;						///< Second parameter bitmask
	uint8_t res1;						///< Reserved slot
	uint8_t res2;						///< Reserved slot
	uint8_t min_sv_s;					///< Minimum number of satellites for navigation
	uint8_t max_sv_s;					///< Maximum number of satellites for navigation
	uint8_t min_cn_o;					///< Minimum satellite signal level for navigation
	uint8_t res3;						///< Reserved slot
	uint8_t ini_fix_3d;					///< Initial fix must be 3D flag (0=false/1=true)
	uint8_t res4;						///< Reserved slot
	uint8_t res5;						///< Reserved slot
	uint8_t res6;						///< Reserved slot
	uint16_t wkn_roll_over;				///< GPS week rollover number
	uint32_t res7;						///< Reserved slot
	uint8_t res8;						///< Reserved slot
	uint8_t res9;						///< Reserved slot
	uint8_t use_ppp;					///< use Precise Point Positioning flag
	uint8_t use_aop;					///< AssistNow Autonomous
	uint8_t res11;						///< Reserved slot
	uint8_t res12;						///< Reserved slot
	uint8_t aop_opb_max_err;			///< Maximum acceptable AssistNow Autonomus orbit error
	uint8_t res13;						///< Reserved slot
	uint8_t res14;						///< Reserved slot
} ubx_cfg_nav_expert_settings_t;

/**
 * \brief The U-Blox CFG-PM structure definition
 */
typedef struct  
{
	uint8_t version;					///< Message version
	uint8_t res1;						///< Reserved
	uint8_t res2;						///< Reserved
	uint8_t res3;						///< Reserved
	uint32_t flags;						///< PSM configuation flags
	uint32_t update_period;				///< Positin update period
	uint32_t search_period;				///< Acquisition retry period
	uint32_t grid_offset;	   			///< Grid offset relative to GPS start of week
	uint16_t on_time;	   				///< On time after first succeful fix
	uint16_t min_acq_time; 				///< Minimal search time
} ubx_cfg_pm_t;

/**
 * \brief The U-Blox CFG-PM2 structure definition
 */
typedef struct  
{
	uint8_t version;					///< Message version
	uint8_t res1;						///< Reserved
	uint8_t res2;						///< Reserved
	uint8_t res3;						///< Reserved
	uint32_t flags;						///< PSM configuation flags
	uint32_t update_period;				///< Positin update period
	uint32_t search_period;				///< Acquisition retry period
	uint32_t grid_offset;	   			///< Grid offset relative to GPS start of week
	uint16_t on_time;	   				///< On time after first succeful fix
	uint16_t min_acq_time; 				///< Minimal search time
	uint16_t res4; 						///< Reserved
	uint16_t res5; 						///< Reserved
	uint32_t res6; 						///< Reserved
	uint32_t res7; 						///< Reserved
	int8_t res8;   						///< Reserved
	int8_t res9;   						///< Reserved
	uint16_t res10;						///< Reserved
	uint32_t res11;						///< Reserved
} ubx_cfg_pm2_t;

/**
 * \brief The U-Blox CFG-PRT structure definition
 */
typedef struct
{
	uint8_t port_id;					///< Port ID (=1 or 2 for UART ports)
	uint8_t res0;						///< Reserved
	uint16_t tx_ready;					///< Reserved up to firmware 7.0
	uint32_t mode;						///< Bit mask describing UART mode
	uint32_t baud_rate;					///< Baudrate in bits/second
	uint16_t in_proto_mask;				///< A mask describing which input protocols are active
	uint16_t out_proto_mask;			///< A mask describing which ouput protocols are active
	uint16_t flags;						///< Reserved, set to 0
	uint16_t res3;						///< Reserved, set to 0
} ubx_cfg_prt_t;

/**
 * \brief The U-Blox CFG-PRT structure definition
 */
typedef struct
{
	uint16_t measure_rate;				///< Measurement rate, GPS measurements are taken every measure_rate milliseconds
	uint16_t nav_rate;					///< Navigation rate, in number of measurements cycles. Cannot be changed on u-blox 5 and 6, always equal 1.
	uint16_t time_ref;					///< Alignment to reference time, 0=UTC, 1=GPS time.
} ubx_cfg_rate_t;

/**
 * \brief The U-Blox CFG-RINV structure definition
 */
typedef struct
{
	uint8_t flags;						///< 0=dumb, 1=binary
	uint8_t data;						///< Data to store/stored in Remote Inventory
	uint8_t data2;	  					///< Data to store/stored in Remote Inventory
	uint8_t data3;	  					///< Data to store/stored in Remote Inventory
	uint8_t data4;	  					///< Data to store/stored in Remote Inventory
	uint8_t data5;	  					///< Data to store/stored in Remote Inventory
	uint8_t data6;	  					///< Data to store/stored in Remote Inventory
	uint8_t data7;	  					///< Data to store/stored in Remote Inventory
	uint8_t data8;	  					///< Data to store/stored in Remote Inventory
	uint8_t data9;	  					///< Data to store/stored in Remote Inventory
	uint8_t data10;	  					///< Data to store/stored in Remote Inventory
	uint8_t data11;	  					///< Data to store/stored in Remote Inventory
	uint8_t data12;	  					///< Data to store/stored in Remote Inventory
	uint8_t data13;	  					///< Data to store/stored in Remote Inventory
	uint8_t data14;	  					///< Data to store/stored in Remote Inventory
	uint8_t data15;	  					///< Data to store/stored in Remote Inventory
	uint8_t data16;	  					///< Data to store/stored in Remote Inventory
	uint8_t data17;	  					///< Data to store/stored in Remote Inventory
	uint8_t data18;	  					///< Data to store/stored in Remote Inventory
	uint8_t data19;	  					///< Data to store/stored in Remote Inventory
	uint8_t data20;	  					///< Data to store/stored in Remote Inventory
	uint8_t data21;	  					///< Data to store/stored in Remote Inventory
	uint8_t data22;	  					///< Data to store/stored in Remote Inventory
	uint8_t data23;	  					///< Data to store/stored in Remote Inventory
} ubx_cfg_rinv_t;

/**
 * \brief The U-Blox CFG-RXM structure definition
 */
typedef struct
{
	uint8_t res;						///< Reserved set to 8
	uint8_t lp_mode;					///< Low power mode
} ubx_cfg_rxm_t;

/**
 * \brief The U-Blox CFG-SBAS structure definition
 */
typedef struct
{
	uint8_t mode;						///< SBAS mode
	uint8_t usage;						///< SBAS usage
	uint8_t max_sbas;					///< Maximum number of SBAS prioritized tracking channels
	uint8_t scan_mode2;					///< Continuation of scanmode bitmask
	uint32_t scan_mode1;				///< Which SBAS PRN numbers to search for, all bits to 0=auto_scan
} ubx_cfg_sbas_t;

/**
 * \brief The U-Blox CFG-TP structure definition
 */
typedef struct
{
	uint32_t interval;					///< Time interval for time pulse
	uint32_t length;					///< Length of time pulse
	int8_t status;						///< Time pulse config setting, +1:positive, 0:off, -1:negative
	uint8_t time_ref;					///< Alignment to reference time, 0=UTC, 1:GPS, 2:Local time
	uint8_t flags;						///< Bitmask
	uint8_t res;						///< Reserved
	int16_t antenna_cable_delay;		///< Antenna cable delay
	int16_t rf_group_delay;				///< Receiver RF group delay
	int32_t user_delay;					///< User time function delay
} ubx_cfg_tp_t;

/**
 * \brief The U-Blox CFG-TP5 structure definition
 */
typedef struct
{
	uint8_t tp_idx;						///< Timepulse selection
	uint8_t res0;						///< Reserved
	uint16_t res1;						///< Reserved
	int16_t ant_cable_delay;			///< Antenna cable delay
	int16_t rf_group_delay;				///< RF group delay
	uint32_t freq_period;				///< Frequency or period time, depending on setting of bit 'isFreq'
	uint32_t freq_perid_lock;			///< Frequency or period time when locked to GPS time, only used if 'lockedOtherSet' is set
	uint32_t pulse_len_ratio;			///< Pulse length or duty cycle, depending on 'isLength'
	uint32_t pulse_len_ratio_lock;		///< Pulse length or duty cycle when locked to GPS time, only used if 'lockedOtherSet' is set
	int32_t user_config_delay;			///< User configurable delay
	uint32_t flags;						///< Configuratin flags
} ubx_cfg_tp5_t;

/**
 * \brief The U-Blox CFG-USB structure definition
 */
typedef struct
{
	uint16_t vendor_id;					///< Vendor ID. This field shall only be set to registered
	uint16_t product_id;				///< Product ID
	uint16_t res1;						///< Reserved, set to 0
	uint16_t res2;						///< Reserved for special use, set to 1
	uint16_t power_consumption;			///< Power consumed by the device in mA
	uint16_t flags;						///< Various configuration flag
	char vendor_string[32];				///< String containing the vendor name, including 0-termination
	char product_string[32];			///< String containing the product name, including 0-termination
	char serial_number[32];				///< String containing the serial number, including 0-termination
} ubx_cfg_usb_t;

/**
 * \brief The U-Blox CFG-ITFM structure definition
 */
typedef struct
{
	uint32_t config;					///< Interference config word
	uint32_t config2;					///< Extra settings for jamming/interference monitor
} ubx_cfg_itfm_t;

/**
 * \brief The U-Blox CFG-INF structure definition
 */
typedef struct
{
	uint8_t protocol_id;				///< Protocol identifier, 0:UBX, 1:NMEA, 2-255:reserved
	uint8_t res0;						///< Reserved
	uint16_t res1;						///< Reserved
	uint8_t inf_msg_mask1;				///< A bit mask saying which information messages are enabled on each I/O target
	uint8_t inf_msg_mask2;				///< A bit mask saying which information messages are enabled on each I/O target
	uint8_t inf_msg_mask3;				///< A bit mask saying which information messages are enabled on each I/O target
	uint8_t inf_msg_mask4;				///< A bit mask saying which information messages are enabled on each I/O target
	uint8_t inf_msg_mask5;				///< A bit mask saying which information messages are enabled on each I/O target
	uint8_t inf_msg_mask6;				///< A bit mask saying which information messages are enabled on each I/O target
} ubx_cfg_inf_t;

/**
 * \brief The U-Blox CFG-FXN structure definition
 */
typedef struct
{
	uint32_t flags;						///< FXN configuration flags
	uint32_t t_reacq;					///< Time the receiver tries to re-acquire satellites, before going to off state
	uint32_t t_acq;						///< Time the receiver tries to acquire satellites, before going to off state
	uint32_t t_reacq_off;				///< Time the receiver stays in off-state, if re-acquisition failed
	uint32_t t_acq_off;					///< Time the receiver stays in off-state, if acquisitin failed
	uint32_t t_on;						///< On time
	uint32_t t_off;						///< Sleep time after normal ontime
	uint32_t res;						///< Reserved
	uint32_t base_tow;					///< Base time of week to which t_on/t_sleep are aligned if ABSOLUTE_SIGN is set
} ubx_cfg_fxn_t;

/**
 * \brief The U-Blox CFG-DAT structure definition
 */
typedef struct
{
	uint16_t datum_num;					///< Geodetic Datum number, 0:WGS84, 1:WGS72, 2:ETH90, 3 ADI-M, ...
} ubx_cfg_dat_t;

/**
 * \brief The U-Blox CFG-ANT structure definition
 */
typedef struct
{
	uint16_t flags;						///< Antenna flag mask
	uint16_t pins;						///< Antenna pin configuration
} ubx_cfg_ant_t;

/**
 * \brief The U-Blox MON-VER struture definition
 */
typedef struct
{
	char sw_version[30];				///< Zero-terminated software version string
	char hw_version[10];				///< Zero-terminated hardware version string
	char rom_version[30];				///< Zero-terminated ROM version string
} ubx_mon_ver_t;

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
} ubx_nav_pos_llh_t;

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
} ubx_nav_status_t;

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
} ubx_nav_solution_t;

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
} ubx_nav_vel_ned_t;

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
} ubx_nav_sv_info_t;

/**
 * \brief The U-Blox NAV-DGPS message structure definition
 */
typedef struct
{
	uint32_t itow;						///< GPS msTow
	int32_t age;						///< Age of the newest correction data
	int16_t base_id;					///< DGPS base station ID
	int16_t base_health;				///< DGPS base station health status
	uint8_t num_channel;				///< Number of channels for which correction data is following
	uint8_t status;						///< DGPS correction type status, 00:None, 01:PR+PRR correction
	uint16_t res1;						///< Reservec

	/**
	 * \brief The structure definition of a particular GPS
	 */
	struct
	{
		uint8_t sv_id;					///< Satellite ID
		uint8_t flags;					///< Bitmask/channel number
		uint16_t age_c;					///< Age of the latest correction data
		float prc;						///< Pseudo range correction
		float prrc;						///< Pseudo range rate correction
	}chan_data[16];
} ubx_nav_dgps_t;

/**
 * \brief The U-Blox MON-RXR message structure definition
 */
typedef struct
{
	uint8_t awake_flag;					///< Receiver status flag
} ubx_mon_rxr_struct_t;

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
} ubx_tim_tp_t;

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
} ubx_tim_vrfy_t;

/** 
 *\brief The U-Blox NAV-TIMEUTC message structure definition
 */
typedef struct
{
	uint32_t itow;						///< GPS msToW
	uint32_t t_acc;						///< Time accuracy estimate
	int32_t nano;						///< Nanoseconds of second, range -1e9..1e9 (UTC)
	uint16_t year;						///< Year range 1999..2099
	uint8_t month;						///< Month 1..12 (UTC)
	uint8_t day;						///< Day of month 
	uint8_t hour;						///< Hour of the day
	uint8_t minute;						///< Minute of the hour
	uint8_t seconds;					///< Second of minute
	uint8_t valid;						///< Validity of the time
} ubx_nav_timeutc_t;

/** 
 *\brief The U-Blox ACK-ACK and ACK-NACK message structure definition
 */
typedef struct  
{
	uint8_t class_id;
	uint8_t msg_id;
} ubx_ack_ack_t;

/** 
 *\brief The U-Blox CFG-CFG message structure definition
 */
typedef struct
{
	uint32_t clear_mask;				///< Mask with configuration sub-sections to clear, i.e. loading default config to permanent non-volatile memory
	uint32_t save_mask;					///< Mask with configuration sub-sections to save, i.e. saving current config to permanent non-volatile memory
	uint32_t load_mask;					///< Mask with configuration sub-sections to load, i.e. loading permanent config to current config
	uint8_t device_mask;				///< Mask which selects the devices for this command
} ubx_cfg_cfg_t;


#define NO_GPS 0							///< No GPS
#define NO_FIX 1							///< No GPS fix
#define GPS_OK 2							///< GPS ok

#define UTC_TIME_UNVALID 0
#define UTC_TIME_VALID 1

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

#define UBX_TIMEOUT_CYCLES 2				///< Number of times ubx_CheckTimeout() must be called without response from GPS before it is considered as timed out
#define UBX_POSITION_PRECISION 20			///< The minimum precision to consider a position as correct (in m)
#define UBX_ALTITUDE_PRECISION 20			///< The minimum precision to consider an altitude as correct (in m)
#define UBX_SPEED_PRECISION 5				///< The minimum precision to consider a speed as correct (in m/s)

#define UBX_HEADING_PRECISION 5000000		///< The minimum precision to consider a heading as correct (in deg*10^5)

typedef enum {
	CHECK_IF_PREAMBLE_1,
	CHECK_IF_PREAMBLE_2,
	GET_MSG_CLASS,
	GET_MSG_ID,
	GET_MSG_PAYLOAD_1,
	GET_MSG_PAYLOAD_2,
	GET_MSG_BYTES,
	CHECK_CHECKSUM_A,
	CHECK_CHECKSUM_B
}gps_decode_msg_state_machine_t;

typedef struct  
{
	uint16_t year;							///< Year
	uint8_t month;							///< Month
	uint8_t day;							///< Day
	uint8_t hour;							///< Hour
	uint8_t minute;							///< Minute
	uint8_t second;							///< Second
	uint8_t validity;						///< Time validity
} date_time_t;


/**
 * \brief Type definition for GPS data
 */
typedef struct
{
	double latitude;							///< Latitude in degrees
	double longitude;							///< Longitude in degrees
	float altitude;								///< Altitude in m
	float alt_elips;							///< Altitude above ellipsoid in m
	float speed;								///< 3D speed in m/s
	float ground_speed;							///< 2D ground speed in m/s
	float north_speed;							///< The speed to the north in m/s
	float east_speed;							///< The speed to the east in m/s
	float vertical_speed;						///< The vertical speed in m/s
	float course;								///< Heading in degree * 100
	
	float horizontal_accuracy;					///< Horizontal accuracy in m
	float vertical_accuracy;					///< Vertical accuracy in m
	
	float speed_accuracy;						///< Speed accuracy in m
	float heading_accuracy;						///< Heading accuracy in m
	
	uint8_t num_sats;							///< Number of visible satellites
	uint16_t hdop;								///< Height DOP
	
	uint32_t time_last_msg;						///< Time reference in ms of microcontroller
	uint32_t time_gps;							///< Time reference in ms of gps
	
	uint8_t  status;							///< GPS status
	
	uint8_t  horizontal_status;					///< Horizontal status
	
 	uint8_t  altitude_status;					///< Altitude status
 	uint8_t  speed_status;						///< Speed status
 	uint8_t  course_status;						///< Course status
 	uint8_t  accuracy_status;					///< Accuracy status
	
 	bool healthy;								///< Healthiness of the GPS

	date_time_t date;							///< The date type
	uint8_t time_zone;							///< The current time zone
	
	uint8_t disable_counter;					///< Counter used to deactivate unwanted messages
	uint32_t idle_timer;						///< Last time that the GPS driver got a good packet from the GPS
	uint32_t idle_timeout;						///< Time in milliseconds after which we will assume the GPS is no longer sending us updates and attempt a re-init. 1200ms allows a small amount of slack over the worst-case 1Hz update rate.

	bool new_position;							///< Boolean value to check if we received new position message
	bool new_speed;								///< Boolean value to check if we received new velocity message

	bool next_fix;								///< Boolean variable to get whether we have a correct GPS fix or not
	bool have_raw_velocity;						///< Boolean variable that could be used to get a speed approximate with heading and 2D velocity

	uint8_t num_skipped_msg;					///< Number of skipped messages
	uint8_t loop_pos_llh;						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_vel_ned; 						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_status; 						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_solution; 						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_tim_tp; 						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_tim_vrfy;						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_nav_timeutc;					///< Counter used to print one message every num_skipped_msg
	uint8_t loop_mon_rxr;						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_sv_info;						///< Counter used to print one message every num_skipped_msg
	uint8_t loop_nav_dgps;						///< Counter used to print one message every num_skipped_msg
	
	bool print_nav_on_debug;					///< Flag to print messages on debug console
	bool debug;									///< Indicates if debug messages should be printed
	
	uint32_t time_last_posllh_msg;				///< Time at which the last POSLLH message was received
	uint32_t time_last_velned_msg;				///< Time at which the last VELNED message was received
	
	gps_decode_msg_state_machine_t step;		///< Variable defining the state machine in the U-Blox decoding function
	uint8_t  ubx_class;							///< The U-Blox message class
	uint8_t  msg_id;							///< The U-Blox message ID
	uint16_t payload_counter;					///< The incremental counter to receive bytes of data
	uint16_t payload_length;					///< The length of the message
	uint8_t cksum_a;							///< Checksum a
	uint8_t cksum_b;							///< Checksum b

	gps_engine_setting_t engine_nav_setting;	///< Enum GPS engine setting
	ubx_cfg_nav_settings_t nav_settings;		///< CFG-NAV settings structure

	bool configure_gps;								///< A flag to start the configuration of the GPS
	uint16_t config_loop_count;					///< The counter for the configuration of the GPS
	uint16_t config_nav_msg_count;				///< The counter for the configuration of the GPS when there is multiple message of the same kind
	bool acknowledged_received;					///< A flag to know if the GPS received the configuration message

	Serial* serial;								///< Pointer to serial device
} gps_t;

uint8_t  **ubx_current_message = 0;					///<  The pointer to the pointer to the structure of the current message to fill
uint8_t  ** ubx_last_message = 0;					///<  The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
uint16_t * ubx_valid_message = 0;					///<  The pointer to the number to increment when a message of the type has been received

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
ubx_nav_pos_llh_t ubx_pos_llh_message[2];			///<  The Posllh message buffer
ubx_nav_status_t ubx_status_message[2];				///<  The Status message buffer
ubx_nav_solution_t ubx_solution_message[2];			///<  The Solution message buffer
ubx_nav_vel_ned_t ubx_vel_ned_message[2];			///<  The Velned message buffer
ubx_nav_sv_info_t ubx_sv_info_message[2];			///<  The SVInfo message buffer
ubx_cfg_nav_settings_t ubx_nav_settings_message[2]; ///<  The Nav Settings message buffer
ubx_cfg_nav_rate_t ubx_cfg_rate_message[2];			///<  The CFG Rate message buffer
ubx_cfg_msg_rate_t ubx_cfg_set_get_rate_message[2];	///<  The CFG Set/get Rate message buffer
ubx_mon_rxr_struct_t ubx_mon_rxr_message[2];		///<  The MON RXR message buffer
ubx_tim_tp_t ubx_tim_tp_message[2];					///<  The TIM TP message buffer
ubx_tim_vrfy_t ubx_tim_vrfy_message[2];				///<  The TIM VRFY message buffer
ubx_nav_timeutc_t ubx_nav_timeutc_message[2];		///<  The NAV TIMEUTC message buffer
ubx_ack_t ubx_ack_message[2];						///<  The ACK ACK message buffer
ubx_nav_dgps_t ubx_nav_dgps_message[2];				///<  The NAV DGPS message buffer

// NAV-POSLLH
ubx_nav_pos_llh_t * ubx_current_pos_llh_message = &ubx_pos_llh_message[0];						///<  The pointer to the Posllh message that is being filled (not usable)
ubx_nav_pos_llh_t * ubx_last_pos_llh_message = &ubx_pos_llh_message[1];							///<  The pointer to the last Posllh message that was completed
uint16_t ubx_number_of_valid_pos_llh_message = 0;												///<  Number of valid Posllh message received

// NAV-STATUS
ubx_nav_status_t *ubx_current_status_message = &ubx_status_message[0];							///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_status_t *ubx_last_status_message = &ubx_status_message[1];								///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_status_message = 0;												///<  Number of valid Status message received

// NAV-Sol
ubx_nav_solution_t *ubx_current_solution_message = &ubx_solution_message[0];					///<  The pointer to the Solution message that is being filled (not usable)
ubx_nav_solution_t *ubx_last_solution_message = &ubx_solution_message[1];						///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_solution_message = 0;												///<  Number of valid Status message received

// NAV-VELNED
ubx_nav_vel_ned_t *ubx_current_vel_ned_message = &ubx_vel_ned_message[0];						///<  The pointer to the Velned message that is being filled (not usable)
ubx_nav_vel_ned_t *ubx_last_vel_ned_message = &ubx_vel_ned_message[1];							///<  The pointer to the last Velned message that was completed
uint16_t ubx_number_of_valid_vel_ned_message = 0;												///<  Number of valid Velned message received

// NAV-SVINFO
ubx_nav_sv_info_t *ubx_current_sv_info_message = &ubx_sv_info_message[0];						///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_sv_info_t *ubx_last_sv_info_message = &ubx_sv_info_message[1];							///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_sv_info_message = 0;												///<  Number of valid Status message received

// NAV-Settings
ubx_cfg_nav_settings_t *ubx_current_nav_settings_message = &ubx_nav_settings_message[0];		///<  The pointer to the Nav Settings message that is being filled (not usable)
ubx_cfg_nav_settings_t *ubx_last_nav_settings_message = &ubx_nav_settings_message[1];			///<  The pointer to the last Nav Settings message that was completed
uint16_t ubx_number_of_valid_nav_settings_message = 0;											///<  Number of valid Nav Settings message received

// CFG message rate
ubx_cfg_nav_rate_t *ubx_current_cfg_rate_message = &ubx_cfg_rate_message[0];					///<  The pointer to the CFG Rate message that is being filled (not usable)
ubx_cfg_nav_rate_t *ubx_last_cfg_rate_message = &ubx_cfg_rate_message[1];						///<  The pointer to the last CFG Rate message that was completed
uint16_t ubx_number_of_valid_cfg_rate_message = 0;												///<  Number of valid CFG Rate message received

// CFG Set/Get message rate
ubx_cfg_msg_rate_t *ubx_current_cfg_set_get_rate_message = &ubx_cfg_set_get_rate_message[0];	///<  The pointer to the CFG Set/get Rate message that is being filled (not usable)
ubx_cfg_msg_rate_t *ubx_last_cfg_set_get_rate_message = &ubx_cfg_set_get_rate_message[1];		///<  The pointer to the last CFG Set/get Rate message that was completed
uint16_t ubx_number_of_valid_cfg_set_get_rate_message = 0;										///<  Number of valid CFG Set/get Rate message received

// MON RXR message
ubx_mon_rxr_struct_t *ubx_current_mon_rxr_message = &ubx_mon_rxr_message[0];					///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_mon_rxr_struct_t *ubx_last_mon_rxr_message = &ubx_mon_rxr_message[1];						///<  The pointer to the last MON RXR message that was completed
uint16_t ubx_number_of_valid_mon_rxr_message = 0;												///<  Number of valid MON RXR message received

// TIM TP message
ubx_tim_tp_t *ubx_current_tim_tp_message = &ubx_tim_tp_message[0];								///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_tim_tp_t *ubx_last_tim_tp_message = &ubx_tim_tp_message[1];									///<  The pointer to the last TIM TP message that was completed
uint16_t ubx_number_of_valid_tim_tp_message = 0;												///<  Number of valid TIM TP message received

// TIM VRFY message
ubx_tim_vrfy_t *ubx_current_tim_vrfy_message = &ubx_tim_vrfy_message[0];						///<  The pointer to the TIM VRFY message that is being filled (not usable)
ubx_tim_vrfy_t *ubx_last_tim_vrfy_message = &ubx_tim_vrfy_message[1];							///<  The pointer to the last TIM VRFY message that was completed
uint16_t ubx_number_of_valid_tim_vrfy_message = 0;												///<  Number of valid TIM VRFY message received

// NAV-TIMEUTC
ubx_nav_timeutc_t *ubx_current_nav_timeutc_message = &ubx_nav_timeutc_message[0];				///<  The pointer to the NAV TIMEUTC message that is being filled (not usable)
ubx_nav_timeutc_t *ubx_last_nav_timeutc_message = &ubx_nav_timeutc_message[1];					///<  The pointer to the last NAV TIMEUTC message that was completed
uint16_t ubx_number_of_valid_nav_timeutc_message = 0;											///<  Number of valid NAV TIMEUTC message received

// ACK-ACK
ubx_ack_t *ubx_current_ack_message = &ubx_ack_message[0];										///< The pointer to the ACK ACK message that is being filled (not usable)
ubx_ack_t *ubx_last_ack_message = &ubx_ack_message[1];											///< The pointer to the last ACK ACK message that was completed
uint16_t ubx_number_of_valid_ack_message = 0;													///< Number of valid ACK message received

// NAV-TIMEUTC
ubx_nav_dgps_t *ubx_current_nav_dgps_message = &ubx_nav_dgps_message[0];						///<  The pointer to the NAV DGPS message that is being filled (not usable)
ubx_nav_dgps_t *ubx_last_nav_dgps_message = &ubx_nav_dgps_message[1];							///<  The pointer to the last NAV DGPS message that was completed
uint16_t ubx_number_of_valid_nav_dgps_message = 0;												///<  Number of valid NAV DGPS message received


// The date is defined as global parameter such that we can retrieve its value 
//  without the need of pointers, e.g. when working with fat_fs, in diskio function
//  get_fattime, the date can be set without pointer to the gps structure
date_time_t date;

// Old gps struct declared as global 
gps_t gps;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Initialize the gps U-Blox module
 *
 * \param	gps				The pointer to the GPS structure
 * \param	serial			Pointer to serial interface
 */
static void gps_ublox_init(gps_t *gps, Serial* serial);


/**
 * \brief	To configure the GPS in binary mode and the Navigation messages we want
 *
 * The GPS and UART channel should already be configured in the good baudrate 38400U
 *
 * \param	gps			The pointer to the GPS structure
 */
static void gps_ublox_configure_gps(gps_t *gps);


/**
 * \brief	The function that needs to be called to get the GPS information
 *
 * \param	gps			The pointer to the GPS structure
 */
static void gps_ublox_update(gps_t *gps);

/**
 * \brief	Tranforming UTC to local time
 *
 * \param	today_date	The pointer to the date structure
 * \param	time_zone	The current time zone
 */
static void gps_ublox_utc_to_local(date_time_t *today_date, uint8_t time_zone);

/**
 * \brief	Gets the current date and time
 *
 * \return	The current date and time
 */
// static date_time_t gps_ublox_get_date(void);


/**
 * \brief	Reset the gps U-Blox module
 *
 * \param	gps						The pointer to the GPS structure
 * \param	engine_nav_setting		The GPS Nav settings 
 */
static void gps_ublox_reset(gps_t *gps, gps_engine_setting_t engine_nav_setting);


/**
 * \brief	Process bytes available from the gps
 *
 * The gps is assumed to contain only messages we recognise.  If it
 * contains other messages, and those messages contain the preamble
 * bytes, it is possible for this code to fail to synchronise to the
 * gps immediately.  Without buffering the entire message and
 * re-processing it from the top, this is unavoidable. The parser
 * attempts to avoid this when possible.
 *
 * \param gps					The pointer to the GPS structure
 *
 * \return	true if new velocity and new position message
 */
static bool gps_ublox_message_decode(gps_t *gps);


/**
 * \brief	Process the new received message, class by class
 *
 * \param gps					The pointer to the GPS structure
 * \param ubx_class				The U-blox class message
 * \param msg_id 				The ID of the U-blox message
 *
 * \return	true if new velocity and new position message
 */
static bool gps_ublox_process_data(gps_t *gps, uint8_t ubx_class, uint8_t msg_id);


/**
 * \brief	Checksum update
 *
 * \param	data				The pointer to the data to update the checksum
 * \param	len					The length of the data to update the checksum
 * \param	ck_a				The checksum a: sum of all the data
 * \param	ck_b				The checksum b: sum of checksum a
 */
static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);


/**
 * \brief	To get the lower bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint16 uint
 */
static uint8_t endian_lower_bytes_uint16(uint16_t bytes);


/**
 * \brief	To get the higher bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint16 uint
 */
static uint8_t endian_higher_bytes_uint16(uint16_t bytes);


/**
 * \brief	To get the lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint32_t uint
 */
static uint8_t endian_lower_bytes_uint32(uint32_t bytes);


/**
 * \brief	To get the mid lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the mid lower 8 bytes of the uint32_t uint
 */
static uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes);


/**
 * \brief	To get the mid higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the mid higher 8 bytes of the uint32_t uint
 */
static uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes);


/**
 * \brief	To get the higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes				The uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint32_t uint
 */
static uint8_t endian_higher_bytes_uint32(uint32_t bytes);


/**
 * \brief	To send a uint8 value on the gps
 *
 * \param	gps				The pointer to the gps
 * \param	byte				The uint8 value to be transformed
 * \param	ck_a				The checksum A
 * \param	ck_b				The checksum B
 */
static void ubx_send_uint8(gps_t *gps, uint8_t byte, uint8_t *ck_a, uint8_t *ck_b);


/**
 * \brief	To send a uint16 value on the gps in the Little endian format
 *
 * \param	gps				The gps to the gps
 * \param	byte				The uint16 value to be transformed
 * \param	ck_a				The checksum A
 * \param	ck_b				The checksum B
 */
static void ubx_send_uint16(gps_t *gps, uint16_t byte, uint8_t *ck_a, uint8_t *ck_b);

/**
 * \brief	To send a uint32 value on the gps in the Little endian format
 *
 * \param	gps				The pointer to the gps
 * \param	byte				The uint32 byte to be transformed
 * \param	ck_a				The checksum A
 * \param	ck_b				The checksum B
 */
static void ubx_send_uint32(gps_t *gps, uint32_t byte, uint8_t *ck_a, uint8_t *ck_b);

/**
 * \brief	To send the UBX header of all messages
 *
 * \param	gps				The pointer to the gps structure
 * \param	msg_class			The U-Blox class of the message
 * \param	msg_id				The U-Blox message ID
 * \param	size				The size of the U-Blox following message
 * \param	ck_a				The checksum A: sum of all the data
 * \param	ck_b				The checksum B: sum of all the checksum A
 */
static void ubx_send_header(gps_t *gps, uint8_t msg_class, uint8_t msg_id, uint16_t size, uint8_t *ck_a, uint8_t *ck_b);


/**
 * \brief	To send the checksum of every message
 *
 * \param	gps				The pointer to the gps structure
 * \param	ck_sum_a			The checksum a
 * \param	ck_sum_b			The checksum b
 */
static void ubx_send_cksum(gps_t *gps, uint8_t ck_sum_a, uint8_t ck_sum_b);

/**
 * \brief	To send the Mon_VER settings message, if the ubx_mon_ver_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x0A	UBX_CLASS_MON
 * Msg_id:	0x04	MSG_MON_VER
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_mon_ver			The VER structure sent
 */
static void ubx_send_message_mon_ver(gps_t *gps, ubx_mon_ver_t *gps_mon_ver);


/**
 * \brief	To send the CFG-ANT settings message, if the ubx_cfg_ant_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x13	MSG_CFG_ANT
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_ant			The ANT structure sent
 */
static void ubx_send_message_cfg_ant(gps_t *gps, ubx_cfg_ant_t *gps_cfg_ant);


/**
 * \brief	To send the CFG-DAT settings message, if the ubx_cfg_dat_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x06	MSG_CFG_DAT
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_dat			The DAT structure sent
 */
static void ubx_send_message_cfg_dat(gps_t *gps, ubx_cfg_dat_t *gps_cfg_dat);


/**
 * \brief	To send the CFG-FXN settings message, if the ubx_cfg_fxn_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x0E	MSG_CFG_FXN
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_fxn			The FXN structure sent
 */
static void ubx_send_message_cfg_fxn(gps_t *gps, ubx_cfg_fxn_t *gps_cfg_fxn);

/**
 * \brief	To send the CFG-INF settings message, if the gps_cfg_inf_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x02	MSG_CFG_INF
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_inf			The INF structure sent
 */
static void ubx_send_message_cfg_inf(gps_t *gps, ubx_cfg_inf_t *gps_cfg_inf);

/**
 * \brief	To send the CFG-ITFM settings message, if the gps_cfg_itfm pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x39	MSG_CFG_ITFM
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_itfm		The ITFM structure sent
 */
static void ubx_send_message_cfg_itfm(gps_t *gps, ubx_cfg_itfm_t *gps_cfg_itfm);


/**
 * \brief	To send the NAV settings message, if the ubx_cfg_pm_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x24	MSG_CFG_NAV_SETTINGS
 *
 * \param	gps				The pointer to the gps structure
 * \param	msg_class			The U-Blox class of the message
 * \param	msg_id				The U-Blox message ID
 * \param	engine_settings		The engine_settings sent
 * \param	size				The size of the U-Blox following message
 */
static void ubx_send_message_nav_settings(gps_t *gps, uint8_t msg_class, uint8_t msg_id, ubx_cfg_nav_settings_t *engine_settings, uint16_t size);


/**
 * \brief	To send the CFG-NAVX settings message, if the ubx_cfg_pm_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x24	MSG_CFG_NAV_EXPERT_SETTINGS
 *
 * \param	gps				The pointer to the gps structure
 * \param	engine_settings		The engine_settings expert sent
 */
static void ubx_send_message_nav_expert_settings(gps_t *gps, ubx_cfg_nav_expert_settings_t *expert_engine_settings);


/**
 * \brief	To send the CFG Power management configuration, if the ubx_cfg_pm_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x32	MSG_CFG_PM
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_pm			The power management configuration sent
 */
static void ubx_send_message_cfg_pm(gps_t *gps, ubx_cfg_pm_t *gps_cfg_pm);


/**
 * \brief	To send the CFG Power management 2 configuration, if the ubx_cfg_pm2_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x3B	MSG_CFG_PM2
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_pm2			The power management configuration sent
 */
static void ubx_send_message_cfg_pm2(gps_t *gps, ubx_cfg_pm2_t *gps_cfg_pm2);


/**
 * \brief	To send the CFG Port configuration, if the ubx_cfg_prt_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x00	MSG_CFG_PRT
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_prt			The port configuration sent
 */
static void ubx_send_message_cfg_prt(gps_t *gps, ubx_cfg_prt_t *gps_cfg_prt);


/**
 * \brief	To send the CFG Rate configuration, if the ubx_cfg_rate_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x08	MSG_CFG_RATE
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_rate		The rate configuration sent
 */
static void ubx_send_message_cfg_rate(gps_t *gps, ubx_cfg_rate_t *gps_cfg_rate);


/**
 * \brief	To send the CFG-RINV remote inventory configuration, if the ubx_cfg_rinv_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x34	MSG_CFG_RINV
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_rinv		The RINV configuration sent
 */
static void ubx_send_message_cfg_rinv(gps_t *gps, ubx_cfg_rinv_t *gps_cfg_rinv);


/**
 * \brief	To send the CFG-RXM configuration, if the ubx_cfg_rxm_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x11	MSG_CFG_RXM
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_rxm			The RXM configuration sent
 */
static void ubx_send_message_cfg_rxm(gps_t *gps, ubx_cfg_rxm_t *gps_cfg_rxm);


/**
 * \brief	To send the CFG-SBAS receiver subsystem configuration, if the ubx_cfg_sbas_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x16	MSG_CFG_SBAS
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_sbas		The SBAS configuration sent
 */
static void ubx_send_message_cfg_sbas(gps_t *gps, ubx_cfg_sbas_t *gps_cfg_sbas);


/**
 * \brief	To send the CFG-TP time pulse configuration, if the ubx_cfg_tp_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x07	MSG_CFG_TP
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_tp			The TP configuration sent
 */
static void ubx_send_message_cfg_tp(gps_t *gps, ubx_cfg_tp_t *gps_cfg_tp);


/**
 * \brief	To send the CFG-TP5 time pulse configuration, if the ubx_cfg_tp5_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x31	MSG_CFG_TP5
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_tp5			The TP5 configuration sent
 */
static void ubx_send_message_cfg_tp5(gps_t *gps, ubx_cfg_tp5_t *gps_cfg_tp5);


/**
 * \brief	To send the CFG-USB configuration, if the ubx_cfg_usb_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x1B	MSG_CFG_USB
 *
 * \param	gps				The pointer to the gps structure
 * \param	gps_cfg_usb			The USB configuration sent
 */
static void ubx_send_message_cfg_usb(gps_t *gps, ubx_cfg_usb_t *gps_cfg_usb);


/**
 * \brief	To send the CFG-CFG configuration, if the ubx_cfg_usb_t pointer is null,
 *			asks for the current settings
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x09	MSG_CFG_CFG
 *
 * \param	stream				The pointer to the stream structure
 * \param	gps_cfg_cfg			The USB configuration sent
 */
static void ubx_send_message_cfg_cfg(gps_t* gps, ubx_cfg_cfg_t *gps_cfg_cfg);


/**
 * \brief	To send the NAV messages that we want to receive
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x01	MSG_CFG_SET_RATE
 *
 * \param	gps				The pointer to the gps structure
 * \param	msg_class			The U-Blox class of the message
 * \param	msg_id				The U-Blox message ID
 * \param	rate				The rate of the desired message
 */
static void ubx_configure_message_rate(gps_t *gps, uint8_t msg_class, uint8_t msg_id, uint8_t rate);


/**
 * \brief	This function returns a pointer to the last NAV-POSLLH message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid posllh message, or 0.
 */
static ubx_nav_pos_llh_t * ubx_get_pos_llh(void);


/**
 * \brief	This function returns a pointer to the last NAV-STATUS message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid status message, or 0.
 */
static ubx_nav_status_t * ubx_get_status(void);


/**
 * \brief	This function returns a pointer to the last NAV-SOL message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid NAV-SOL message, or 0.
 */
static ubx_nav_solution_t * ubx_get_solution(void);


/**
* \brief	This function returns a pointer to the last NAV-VELNED message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid velned message, or 0.
*/
static ubx_nav_vel_ned_t * ubx_get_vel_ned(void);


/**
* \brief	This function returns a pointer to the last NAV-SVINFO message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_nav_sv_info_t * ubx_get_sv_info(void);


/**
* \brief	This function returns a pointer to the last NAV-Settings message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_nav_settings_t * ubx_get_nav_settings(void);


/**
* \brief	This function returns a pointer to the last CFG set/get rate message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_msg_rate_t * ubx_get_msg_rate(void);


/**
* \brief	This function returns a pointer to the last MON RXR message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_mon_rxr_struct_t * ubx_get_mon_rxr(void);


/**
* \brief	This function returns a pointer to the last TIM TP message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_tp_t * ubx_get_tim_tp(void);


/**
* \brief	This function returns a pointer to the last TIM VRFY message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_vrfy_t * ubx_get_tim_vrfy(void);

/**
* \brief	This function returns a pointer to the last NAV TIMEUTC message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_nav_timeutc_t * ubx_get_nav_timeutc(void);

/**
* \brief	This function returns a pointer to the last ACK message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_ack_t * ubx_get_ack(void);

/**
* \brief	This function returns a pointer to the last NAV DGPS message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_nav_dgps_t * ubx_get_nav_dgps(void);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void gps_ublox_init(gps_t *gps, Serial* serial)
{
	gps->serial = serial;

	gps->disable_counter = 1;

	gps->time_zone = 1;

	// Set to true to print all data
	gps->print_nav_on_debug = false;
	
	//disable debug message prints
	gps->debug = true;

	gps->idle_timeout = 1200;

	gps->new_position = false;
	gps->new_speed = false;

	gps->next_fix = false;
	gps->have_raw_velocity = false;

	gps->num_skipped_msg = 10;

	gps->loop_pos_llh = 0;
	gps->loop_vel_ned = 0;
	gps->loop_status = 0;
	gps->loop_solution = 0;
	gps->loop_tim_tp = 0;
	gps->loop_tim_vrfy = 0;
	gps->loop_nav_timeutc = 0;
	gps->loop_mon_rxr = 0;
	gps->loop_sv_info = 0;
	
	gps->step = CHECK_IF_PREAMBLE_1;
	gps->ubx_class = 0;
	gps->msg_id = 0;
	gps->payload_counter = 0;
	gps->payload_length = 0;
	gps->cksum_a = 0;
	gps->cksum_b = 0;

	gps->time_last_posllh_msg = time_keeper_get_millis();
	gps->time_last_velned_msg = time_keeper_get_millis();

	gps->engine_nav_setting = GPS_ENGINE_AIRBORNE_4G;

	gps->status = NO_GPS;
	gps->healthy = false;
	
	gps->configure_gps = false;
	gps->config_nav_msg_count = 0;
	gps->acknowledged_received = true;
}


static void gps_ublox_configure_gps(gps_t *gps)
{
	ubx_mon_ver_t gps_mon_ver;
	ubx_cfg_ant_t gps_cfg_ant;
	ubx_cfg_dat_t gps_cfg_dat;
	ubx_cfg_fxn_t gps_cfg_fxn;
	ubx_cfg_inf_t gps_cfg_inf;
	ubx_cfg_itfm_t gps_cfg_itfm;
	ubx_cfg_nav_settings_t gps_engine_settings;
	ubx_cfg_nav_expert_settings_t gps_nav_expert_settings;
	ubx_cfg_pm_t gps_cfg_pm;
	ubx_cfg_pm2_t gps_cfg_pm2;
	ubx_cfg_prt_t gps_cfg_prt;
	ubx_cfg_rate_t gps_cfg_rate;
	ubx_cfg_rinv_t gps_cfg_rinv;
	ubx_cfg_rxm_t gps_cfg_rxm;
	ubx_cfg_sbas_t gps_cfg_sbas;
	ubx_cfg_tp_t gps_cfg_tp;
	ubx_cfg_tp5_t gps_cfg_tp5;
	ubx_cfg_usb_t gps_cfg_usb;
	ubx_cfg_cfg_t gps_cfg_cfg;
	
	if (!gps->acknowledged_received)
	{
		return;
	}
	
	gps->acknowledged_received = false;
	gps->config_loop_count++;
	
	uint8_t i;
	switch(gps->config_loop_count)
	{
		case 1:
			// Configure the MON-VER messages
			print_util_dbg_print("Setting MON-VER messages...\r\n");
			for (i=0; i<30; i++)
			{
				gps_mon_ver.sw_version[i] = 0;
			}
			for (i=0; i<10; i++)
			{
				gps_mon_ver.hw_version[i] = 0;
			}
			for (i=0; i<30; i++)
			{
				gps_mon_ver.rom_version[i] = 0;
			}
			strcpy(gps_mon_ver.sw_version,"7.03 (45970)");
			strcpy(gps_mon_ver.hw_version,"00040007");
			strcpy(gps_mon_ver.rom_version,"7.03 (45969)");
			ubx_send_message_mon_ver(gps, &gps_mon_ver);
			
			// No acknowledgment message for MON class
			gps->acknowledged_received = true;
			break;
			
		case 2:
			// Configure the ANT messages
			print_util_dbg_print("Setting CFG-ANT messages...\r\n");
			gps_cfg_ant.flags = 0x001B;
			gps_cfg_ant.pins = 0xA98B;
			ubx_send_message_cfg_ant(gps, &gps_cfg_ant);
			break;
			
		case 3:
			// Configure the DAT messages
			print_util_dbg_print("Setting CFG-DAT messages...\r\n");
			gps_cfg_dat.datum_num = 0x0001;
			ubx_send_message_cfg_dat(gps, &gps_cfg_dat);
			break;
			
		case 4:
			// Configure the FXN messages
			print_util_dbg_print("Setting CFG-FXN messages...\r\n");
			gps_cfg_fxn.flags = 0x0000000C;
			gps_cfg_fxn.t_reacq = 0x00000000;
			gps_cfg_fxn.t_acq = 0x00000000;
			gps_cfg_fxn.t_reacq_off = 0x00002710;
			gps_cfg_fxn.t_acq_off = 0x000002710;
			gps_cfg_fxn.t_on = 0x000007D0;
			gps_cfg_fxn.t_off = 0xFFFFFC18;
			gps_cfg_fxn.res = 0x00000000;
			gps_cfg_fxn.base_tow = 0x00000000;
			ubx_send_message_cfg_fxn(gps, &gps_cfg_fxn);
			break;
			
		case 5:
			// Configure the INF messages
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Setting CFG-INF messages...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					gps_cfg_inf.protocol_id = 0x00;
					gps_cfg_inf.res0 = 0x00;
					gps_cfg_inf.res1 = 0x0000;
					gps_cfg_inf.inf_msg_mask1 = 0x00;
					gps_cfg_inf.inf_msg_mask2 = 0x00;
					gps_cfg_inf.inf_msg_mask3 = 0x00;
					gps_cfg_inf.inf_msg_mask4 = 0x00;
					gps_cfg_inf.inf_msg_mask5 = 0x00;
					gps_cfg_inf.inf_msg_mask6 = 0x00;
					ubx_send_message_cfg_inf(gps, &gps_cfg_inf);
					gps->config_loop_count--;
					break;
					
				case 2:
					gps_cfg_inf.protocol_id = 0x01;
					gps_cfg_inf.res0 = 0x00;
					gps_cfg_inf.res1 = 0x0000;
					gps_cfg_inf.inf_msg_mask1 = 0x87;
					gps_cfg_inf.inf_msg_mask2 = 0x87;
					gps_cfg_inf.inf_msg_mask3 = 0x87;
					gps_cfg_inf.inf_msg_mask4 = 0x87;
					gps_cfg_inf.inf_msg_mask5 = 0x87;
					gps_cfg_inf.inf_msg_mask6 = 0x87;
					ubx_send_message_cfg_inf(gps, &gps_cfg_inf);
					gps->config_loop_count--;
					break;
					
				case 3:
					gps_cfg_inf.protocol_id = 0x03;
					gps_cfg_inf.res0 = 0x00;
					gps_cfg_inf.res1 = 0x0000;
					gps_cfg_inf.inf_msg_mask1 = 0x00;
					gps_cfg_inf.inf_msg_mask2 = 0x00;
					gps_cfg_inf.inf_msg_mask3 = 0x00;
					gps_cfg_inf.inf_msg_mask4 = 0x00;
					gps_cfg_inf.inf_msg_mask5 = 0x00;
					gps_cfg_inf.inf_msg_mask6 = 0x00;
					ubx_send_message_cfg_inf(gps, &gps_cfg_inf);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
			
		case 6:
			// Configure the ITFM messages
			print_util_dbg_print("Setting CFG-ITFM messages...\r\n");
			gps_cfg_itfm.config = 0x2D62ACF3;
			gps_cfg_itfm.config2 = 0x0000031E;
			ubx_send_message_cfg_itfm(gps, &gps_cfg_itfm);
			break;
			
		case 7:
			// Configure the NAV messages
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Setting NAV messages...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_AOPSTATUS, 0);
					gps->config_loop_count--;
					break;
					
				case 2:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_CLOCK, 0);
					gps->config_loop_count--;
					break;
					
				case 3:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_DGPS, 0);
					gps->config_loop_count--;
					break;
					
				case 4:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_DOP, 0);
					gps->config_loop_count--;
					break;
					
				case 5:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_POSECEF, 0);
					gps->config_loop_count--;
					break;
					
				case 6:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_POSLLH, 1);
					gps->config_loop_count--;
					break;
					
				case 7:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_SBAS, 0);
					gps->config_loop_count--;
					break;
					
				case 8:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_SOL, 5);
					gps->config_loop_count--;
					break;
					
				case 9:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_STATUS, 5);
					gps->config_loop_count--;
					break;
					
				case 10:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_SVINFO, 0);
					gps->config_loop_count--;
					break;
					
				case 11:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_TIMEGPS, 0);
					gps->config_loop_count--;
					break;
					
				case 12:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_TIMEUTC,10);
					gps->config_loop_count--;
					break;
					
				case 13:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_VELCEF, 0);
					gps->config_loop_count--;
					break;
					
				case 14:
					ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_VELNED, 1);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
			
		case 8:
			// Configure MON messages
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Setting MON messages...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_HW2, 0);
					gps->config_loop_count--;
					break;
					
				case 2:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_HW, 0);
					gps->config_loop_count--;
					break;
					
				case 3:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_IO, 0);
					gps->config_loop_count--;
					break;
					
				case 4:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_MSGPP, 0);
					gps->config_loop_count--;
					break;
					
				case 5:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_RXBUF, 0);
					gps->config_loop_count--;
					break;
					
				case 6:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_RXR, 5);
					gps->config_loop_count--;
					break;
				case 7:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, MSG_MON_TXBUF, 0);
					gps->config_loop_count--;
					break;
					
				case 8:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, 0x05, 0);
					gps->config_loop_count--;
					break;
					
				case 9 :
					ubx_configure_message_rate(gps, UBX_CLASS_MON, 0x0A, 0);
					gps->config_loop_count--;
					break;
				case 10:
					ubx_configure_message_rate(gps, UBX_CLASS_MON, 0x20, 0);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
			
		case 9:
			// Configure AID messages
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Setting AID messages...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					ubx_configure_message_rate(gps, UBX_CLASS_AID, MSG_AID_ALM,1);
					gps->config_loop_count--;
					break;
					
				case 2:
					ubx_configure_message_rate(gps, UBX_CLASS_AID, MSG_AID_EPH,1);
					gps->config_loop_count--;
					break;
					
				case 3:
					ubx_configure_message_rate(gps, UBX_CLASS_AID, MSG_AID_ALPSRV,0);
					gps->config_loop_count--;
					break;
					
				case 4:
					ubx_configure_message_rate(gps, UBX_CLASS_AID, MSG_AID_AOP,0);
					gps->config_loop_count--;
					break;
					
				case 5:
					ubx_configure_message_rate(gps, UBX_CLASS_AID, MSG_AID_REQ,0);
					gps->config_nav_msg_count = 0;
			}
			break;
			
		case 10:
			// Configure TIM messages
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Setting TIM messages...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					ubx_configure_message_rate(gps, UBX_CLASS_TIM, MSG_TIM_TM2, 0);
					gps->config_loop_count--;
					break;
					
				case 2:
					ubx_configure_message_rate(gps, UBX_CLASS_TIM, MSG_TIM_TP, 10);
					gps->config_loop_count--;
					break;
					
				case 3:
					ubx_configure_message_rate(gps, UBX_CLASS_TIM, MSG_TIM_VRFY, 10);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
			
		case 11:
			// Setting navigation settings
			print_util_dbg_print("Settings CFG engine settings...\r\n");
			gps_engine_settings.mask = 0xFFFF;
			gps_engine_settings.dyn_model = 0x08;
			gps_engine_settings.fix_mode = 0x03;
			gps_engine_settings.fixed_alt = 0x0000;
			gps_engine_settings.fixed_alt_var = 0x00002710;
			gps_engine_settings.min_elev = 0x05;
			gps_engine_settings.dr_limit = 0x00;
			gps_engine_settings.p_dop = 0x00FA;
			gps_engine_settings.t_dop = 0x00FA;
			gps_engine_settings.p_acc = 0x0064;
			gps_engine_settings.t_acc = 0x012C;
			gps_engine_settings.static_hold_thresh = 0x00;
			gps_engine_settings.dgps_timeout = 0x3C;
			gps_engine_settings.res2 = 0x0000;
			gps_engine_settings.res3 = 0x0000;
			gps_engine_settings.res4 = 0x0000;
			ubx_send_message_nav_settings(gps, UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, &gps_engine_settings, UBX_SIZE_CFG_NAV_SETTINGS);
			break;
			
		case 12:
			// Setting navigation expert settings
			print_util_dbg_print("Settings CFG expert engine settings...\r\n");
			gps_nav_expert_settings.version = 0x0000;
			gps_nav_expert_settings.mask1 = 0xFFFF;
			gps_nav_expert_settings.mak2 = 0x00000003;
			gps_nav_expert_settings.res1 = 0x03;
			gps_nav_expert_settings.res2 = 0x02;
			gps_nav_expert_settings.min_sv_s = 0x03;
			gps_nav_expert_settings.max_sv_s = 0x10;
			gps_nav_expert_settings.min_cn_o = 0x07;
			gps_nav_expert_settings.res3 = 0x00;
			gps_nav_expert_settings.ini_fix_3d = 0x00;
			gps_nav_expert_settings.res4 = 0x01;
			gps_nav_expert_settings.res5 = 0x00;
			gps_nav_expert_settings.res6 = 0x00;
			gps_nav_expert_settings.wkn_roll_over = 0x0643;
			gps_nav_expert_settings.res7 = 0x00000000;
			gps_nav_expert_settings.res8 = 0x01;
			gps_nav_expert_settings.res9 = 0x01;
			gps_nav_expert_settings.use_ppp = 0x00;
			gps_nav_expert_settings.use_aop = 0x00;
			gps_nav_expert_settings.res11 = 0x00;
			gps_nav_expert_settings.res12 = 0x64;
			gps_nav_expert_settings.aop_opb_max_err = 0x0078;
			gps_nav_expert_settings.res13 = 0x00000000;
			gps_nav_expert_settings.res14 = 0x00000000;
			ubx_send_message_nav_expert_settings(gps,&gps_nav_expert_settings);
			break;
			
		case 13:
			// Setting power management settings
			print_util_dbg_print("Settings CFG-PM power management settings...\r\n");
			gps_cfg_pm.version = 0x00;
			gps_cfg_pm.res1 = 0x06;
			gps_cfg_pm.res2 = 0x00;
			gps_cfg_pm.res3 = 0x00;
			gps_cfg_pm.flags = 0x00009004;
			gps_cfg_pm.update_period = 0x000003E8;
			gps_cfg_pm.search_period = 0x00002710;
			gps_cfg_pm.grid_offset = 0x00000000;
			gps_cfg_pm.on_time = 0x0002;
			gps_cfg_pm.min_acq_time = 0x0000;
			ubx_send_message_cfg_pm(gps, &gps_cfg_pm);
			break;
			
		case 14:
			// Setting power management 2 settings
			print_util_dbg_print("Settings CFG-PM2 power management settings...\r\n");
			gps_cfg_pm2.version = 0x01;
			gps_cfg_pm2.res1 = 0x06;
			gps_cfg_pm2.res2 = 0x00;
			gps_cfg_pm2.res3 = 0x00;
			gps_cfg_pm2.flags = 0x00029000;
			gps_cfg_pm2.update_period = 0x000003E8;
			gps_cfg_pm2.search_period = 0x00002710;
			gps_cfg_pm2.grid_offset = 0x00000000;
			gps_cfg_pm2.on_time = 0x0002;
			gps_cfg_pm2.min_acq_time = 0x0000;
			gps_cfg_pm2.res4 = 0x012C;
			gps_cfg_pm2.res5 = 0x0000;
			gps_cfg_pm2.res6 = 0x0003C14F;
			gps_cfg_pm2.res7 = 0x00000286;
			gps_cfg_pm2.res8 = 0xFE;
			gps_cfg_pm2.res9 = 0x00;
			gps_cfg_pm2.res10 = 0x0000;
			gps_cfg_pm2.res11 = 0x00014064;
			ubx_send_message_cfg_pm2(gps, &gps_cfg_pm2);
			break;
	
		case 15:
			// Setting port settings
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Settings CFG-PRT port configuration...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					gps_cfg_prt.port_id = 0x00;
					gps_cfg_prt.res0 = 0x00;
					gps_cfg_prt.tx_ready = 0x0000;
					gps_cfg_prt.mode = 0x00000000;
					gps_cfg_prt.baud_rate = 0x00000000;
					gps_cfg_prt.in_proto_mask = 0x0000;
					gps_cfg_prt.out_proto_mask = 0x0000;
					gps_cfg_prt.flags = 0x0000;
					gps_cfg_prt.res3 = 0x0000;
					ubx_send_message_cfg_prt(gps, &gps_cfg_prt);
					gps->config_loop_count--;
					break;
					
				case 2:
					gps_cfg_prt.port_id = 0x01; // uart
					gps_cfg_prt.res0 = 0x00;
					gps_cfg_prt.tx_ready = 0x0000;
					gps_cfg_prt.mode = 0x000008C0;
					gps_cfg_prt.baud_rate = 0x00009600; // 38400
					gps_cfg_prt.in_proto_mask = 0x0007;
					gps_cfg_prt.out_proto_mask = 0x0001;
					gps_cfg_prt.flags = 0x0000;
					gps_cfg_prt.res3 = 0x0000;
					ubx_send_message_cfg_prt(gps, &gps_cfg_prt);
					gps->config_loop_count--;
					break;
					
				case 3:
					gps_cfg_prt.port_id = 0x02; // second uart
					gps_cfg_prt.res0 = 0x00;
					gps_cfg_prt.tx_ready = 0x0000;
					gps_cfg_prt.mode = 0x000008C0;
					gps_cfg_prt.baud_rate = 0x00002580; // 9600
					gps_cfg_prt.in_proto_mask = 0x0000;
					gps_cfg_prt.out_proto_mask = 0x0000;
					gps_cfg_prt.flags = 0x0000;
					gps_cfg_prt.res3 = 0x0000;
					ubx_send_message_cfg_prt(gps, &gps_cfg_prt);
					gps->config_loop_count--;
					break;
					
				case 4:
					gps_cfg_prt.port_id = 0x03; // USB
					gps_cfg_prt.res0 = 0x00;
					gps_cfg_prt.tx_ready = 0x0000;
					gps_cfg_prt.mode = 0x00000000;
					gps_cfg_prt.baud_rate = 0x00000000; // 0
					gps_cfg_prt.in_proto_mask = 0x0007;
					gps_cfg_prt.out_proto_mask = 0x0007;
					gps_cfg_prt.flags = 0x0000;
					gps_cfg_prt.res3 = 0x0000;
					ubx_send_message_cfg_prt(gps, &gps_cfg_prt);
					gps->config_loop_count--;
					break;
					
				case 5:
					gps_cfg_prt.port_id = 0x04; // SPI
					gps_cfg_prt.res0 = 0x00;
					gps_cfg_prt.tx_ready = 0x0000;
					gps_cfg_prt.mode = 0x00003200;
					gps_cfg_prt.baud_rate = 0x00000000; // 0
					gps_cfg_prt.in_proto_mask = 0x0007;
					gps_cfg_prt.out_proto_mask = 0x0007;
					gps_cfg_prt.flags = 0x0000;
					gps_cfg_prt.res3 = 0x0000;
					ubx_send_message_cfg_prt(gps, &gps_cfg_prt);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
			
		case 16:
			// Setting rate settings
			print_util_dbg_print("Settings CFG-RATE rate configuration...\r\n");
			gps_cfg_rate.measure_rate = 0x00C8;
			gps_cfg_rate.nav_rate = 0x0001;
			gps_cfg_rate.time_ref = 0x0001;
			ubx_send_message_cfg_rate(gps,&gps_cfg_rate);
			break;
			
		case 17:
			// Setting RINV settings
			print_util_dbg_print("Settings CFG-RINV configuration...\r\n");
			gps_cfg_rinv.flags = 0x00;
			gps_cfg_rinv.data = 0x4E;
			gps_cfg_rinv.data2 = 0x6F;
			gps_cfg_rinv.data3 = 0x74;
			gps_cfg_rinv.data4 = 0x69;
			gps_cfg_rinv.data5 = 0x63;
			gps_cfg_rinv.data6 = 0x65;
			gps_cfg_rinv.data7 = 0x3A;
			gps_cfg_rinv.data8 = 0x20;
			gps_cfg_rinv.data9 = 0x6E;
			gps_cfg_rinv.data10 = 0x6F;
			gps_cfg_rinv.data11 = 0x20;
			gps_cfg_rinv.data12 = 0x64;
			gps_cfg_rinv.data13 = 0x61;
			gps_cfg_rinv.data14 = 0x74;
			gps_cfg_rinv.data15 = 0x61;
			gps_cfg_rinv.data16 = 0x20;
			gps_cfg_rinv.data17 = 0x73;
			gps_cfg_rinv.data18 = 0x61;
			gps_cfg_rinv.data19 = 0x76;
			gps_cfg_rinv.data20 = 0x65;
			gps_cfg_rinv.data21 = 0x64;
			gps_cfg_rinv.data22 = 0x21;
			gps_cfg_rinv.data23 = 0x00;
			ubx_send_message_cfg_rinv(gps, &gps_cfg_rinv);
			break;
			
		case 18:
			// Setting RXM settings
			print_util_dbg_print("Settings CFG-RXM configuration...\r\n");
			gps_cfg_rxm.res = 0x08;
			gps_cfg_rxm.lp_mode = 0x00;
			ubx_send_message_cfg_rxm(gps, &gps_cfg_rxm);
			break;
			
		case 19:
			// Setting SBAS settings
			print_util_dbg_print("Settings CFG-SBAS configuration...\r\n");
			gps_cfg_sbas.mode = 0x01;
			gps_cfg_sbas.usage = 0x03;
			gps_cfg_sbas.max_sbas = 0x03;
			gps_cfg_sbas.scan_mode2 = 0x00;
			gps_cfg_sbas.scan_mode1 = 0x00066251;
			ubx_send_message_cfg_sbas(gps, &gps_cfg_sbas);
			break;
			
		case 20:
			// Setting TP time pulse settings
			print_util_dbg_print("Settings CFG-TP time pulse configuration...\r\n");
			gps_cfg_tp.interval = 0x000F4240;
			gps_cfg_tp.length = 0x000186A0;
			gps_cfg_tp.status = 0x01;
			gps_cfg_tp.time_ref = 0x01;
			gps_cfg_tp.flags = 0x00;
			gps_cfg_tp.res = 0x00;
			gps_cfg_tp.antenna_cable_delay = 0x0032;
			gps_cfg_tp.rf_group_delay = 0x0000;
			gps_cfg_tp.user_delay = 0x00000000;
			ubx_send_message_cfg_tp(gps, &gps_cfg_tp);
			break;
			
		case 21:
			// Setting TP time pulse 5 settings
			if (gps->config_nav_msg_count == 0)
			{
				print_util_dbg_print("Settings CFG-TP5 time pulse TP5 configuration...\r\n");
			}
			
			gps->config_nav_msg_count++;
			switch(gps->config_nav_msg_count)
			{
				case 1:
					gps_cfg_tp5.tp_idx = 0x00;
					gps_cfg_tp5.res0 = 0x29;
					gps_cfg_tp5.res1 = 0x0003;
					gps_cfg_tp5.ant_cable_delay = 0x0032;
					gps_cfg_tp5.rf_group_delay = 0x0000;
					gps_cfg_tp5.freq_period = 0x000F4240;
					gps_cfg_tp5.freq_perid_lock = 0x000F4240;
					gps_cfg_tp5.pulse_len_ratio = 0x00000000;
					gps_cfg_tp5.pulse_len_ratio_lock = 0x000186A0;
					gps_cfg_tp5.user_config_delay = 0x00000000;
					gps_cfg_tp5.flags = 0x000000F7;
					ubx_send_message_cfg_tp5(gps, &gps_cfg_tp5);
					gps->config_loop_count--;
					break;
					
				case 2:
					gps_cfg_tp5.tp_idx = 0x01;
					gps_cfg_tp5.res0 = 0x29;
					gps_cfg_tp5.res1 = 0x0003;
					gps_cfg_tp5.ant_cable_delay = 0x0032;
					gps_cfg_tp5.rf_group_delay = 0x0000;
					gps_cfg_tp5.freq_period = 0x00000004;
					gps_cfg_tp5.freq_perid_lock = 0x00000001;
					gps_cfg_tp5.pulse_len_ratio = 0x0001E848;
					gps_cfg_tp5.pulse_len_ratio_lock = 0x000186A0;
					gps_cfg_tp5.user_config_delay = 0x00000000;
					gps_cfg_tp5.flags = 0x000000FE;
					ubx_send_message_cfg_tp5(gps, &gps_cfg_tp5);
					gps->config_nav_msg_count = 0;
					break;
			}
			break;
		case 22:
			// Setting USB settings
			print_util_dbg_print("Settings CFG-USB configuration...\r\n");
			gps_cfg_usb.vendor_id = 0x1546;
			gps_cfg_usb.product_id = 0x01A6;
			gps_cfg_usb.res1 = 0x0000;
			gps_cfg_usb.res2 = 0x0000;
			gps_cfg_usb.power_consumption = 0x0064;
			gps_cfg_usb.flags = 0x0000;
			for (i=0; i<32; i++)
			{
				gps_cfg_usb.vendor_string[i] = 0;
			}
			for (i=0; i<32; i++)
			{
				gps_cfg_usb.product_string[i] = 0;
			}
			for (i=0; i<32; i++)
			{
				gps_cfg_usb.serial_number[i] = 0;
			}
			strcpy(gps_cfg_usb.vendor_string, "u-blox AG - www.u-blox.com");
			strcpy(gps_cfg_usb.product_string, "u-blox 6  -  GPS Receiver");
			strcpy(gps_cfg_usb.serial_number, "");
			ubx_send_message_cfg_usb(gps, &gps_cfg_usb);
			break;
		case 23:
			// Saving configuration to flash
			print_util_dbg_print("Saving configuration to permanent memory...\r\n");
			gps_cfg_cfg.clear_mask = 0x00000000;
			gps_cfg_cfg.save_mask = 0x0000061F;
			gps_cfg_cfg.load_mask = 0x0000001F;
			gps_cfg_cfg.device_mask = 0x17;
			ubx_send_message_cfg_cfg(gps, &gps_cfg_cfg);
			break;
			
		default:
			print_util_dbg_print("GPS configuration completed!\r\n");
			gps->configure_gps = false;
			break;
	}
}


static void gps_ublox_update(gps_t *gps)
{
	bool result;
	uint32_t tnow;

	result = gps_ublox_message_decode(gps);
	
	tnow = time_keeper_get_millis();
	
	if (tnow > 5000 && gps->configure_gps)
	{
		gps_ublox_configure_gps(gps);
	}
	
	if (! result)
	{
		if ((tnow - gps->idle_timer) > gps->idle_timeout)
		{
			gps->status = NO_GPS;
			
			gps->healthy = false;

			gps_ublox_reset(gps, GPS_ENGINE_AIRBORNE_4G);
			gps->idle_timer = tnow;
		}
	}
	else
	{
		// reset the idle timer
		gps->idle_timer = tnow;
		
		gps->time_last_msg = tnow;
		
		gps->healthy = true;

		if(gps->status == GPS_OK)
		{
			// Check for horizontal accuracy
			if (gps->horizontal_accuracy < UBX_POSITION_PRECISION)
			{
				gps->horizontal_status = 1;
			}
			else
			{  
				gps->horizontal_status = 0;
			}
			// Check for vertical accuracy
			if (gps->vertical_accuracy < UBX_ALTITUDE_PRECISION)
			{
				gps->altitude_status = 1;
			}
			else
			{
				gps->altitude_status = 0;
			}
			// Check for speed accuracy
			if (gps->speed_accuracy < UBX_SPEED_PRECISION)
			{
				gps->speed_status = 1;
			}
			else
			{
				gps->speed_status = 0;
			}
			// Check for heading accuracy
			if (gps->heading_accuracy < UBX_HEADING_PRECISION)
			{
				gps->course_status = 1;
			}
			else
			{
				gps->course_status = 0;
			}
			
			gps->accuracy_status = gps->horizontal_status & gps->altitude_status & gps->speed_status & gps->course_status;
			
			// speed approximation with the 
// 			if (!gps->have_raw_velocity)
// 			{
// 				float gps_heading = to_rad(gps->course);
// 				float cos_heading,sin_heading;
// 				
// 				cos_heading = cosf(gps_heading);
// 				sin_heading = sinf(gps_heading);
// 				
// 				gps->northspeed = gps->ground_speed * cos_heading;
// 				gps->eastspeed = gps->ground_speed * sin_heading;
// 				
// 				// no good way to get descent rate
// 				gps->vertical_speed = 0;
// 			}
		}
		else
		{
			gps->horizontal_status = 0;
			gps->altitude_status = 0;
			gps->speed_status = 0;
			gps->course_status = 0;
			
			gps->accuracy_status = 0;
		}
	}
}

static void gps_ublox_utc_to_local(date_time_t *today_date, uint8_t time_zone)
{
	uint16_t begin_dst[] = { // begin day saving time, format: MonthDay
		329, 327, 326, 325, 331, 329, 328, 327
	};
	
	uint16_t end_dst[] = { // end day saving time, format: MonthDay
		1025, 1030, 1029, 1028, 1027, 1025, 1031, 1030
	};
	
	uint8_t days_per_month[] = { //number of days a month
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
	
	// Day saving time active
	if ( ((today_date->month*100 + today_date->day) >= begin_dst[today_date->year - 2015]) && ((today_date->month*100 + today_date->day)< end_dst[today_date->year - 2015]))
	{
		today_date->hour += time_zone + 1;
	}
	else
	{
		today_date->hour += time_zone;
	}

	//Check if leap year
	if ( today_date->year%4 == 0 )
	{
		if ( today_date->year%100 == 0 )
		{
			if ( today_date->year%400 == 0 )
			{
				days_per_month[1] = 29;
			}
		}
		else
		{
			days_per_month[1] = 29; 
		}
	}

	// Check hour range min
	if (today_date->hour < 0)
	{
		today_date->hour += 24;
		today_date->day -= 1;
		
		if (today_date->day < 1)
		{
			today_date->month -= 1;
			if (today_date->month < 1)
			{
				today_date->month = 12;
				today_date->year -= 1;
			}
			today_date->day = days_per_month[today_date->month];
		}
	}

	// Check hour range max
	if (today_date->hour >= 24)
	{
		today_date->hour -= 24;
		today_date->day += 1;
		
		if (today_date->day > days_per_month[today_date->month-1])
		{
			today_date->day = 1;
			today_date->month += 1;
			if (today_date->month > 12)
			{
				today_date->month = 1;
				today_date->year += 1;
			}
		}
		
	}
}

// static date_time_t gps_ublox_get_date()
// {
// 	return date;
// }


static void gps_ublox_reset(gps_t *gps, gps_engine_setting_t engine_nav_setting)
{
	//gps_ublox_configure_gps(gps);
	
	gps->next_fix = false;
	gps->have_raw_velocity = false;
	
	gps->new_position = false;
	gps->new_speed = false;
}


static bool gps_ublox_message_decode(gps_t *gps)
{
	uint8_t data;
	bool msg_ok = false;

	uint8_t  * temporary_message_for_swaping;
	
	while( gps->serial->readable() > 0 )
	{
		gps->serial->read(&data);
		reset:
		
		switch (gps->step)
		{
			// Message preamble detection
			//
			// If we fail to match any of the expected bytes, we reset
			// the state machine and re-consider the failed byte as
			// the first byte of the preamble.  This improves our
			// chances of recovering from a mismatch and makes it less
			// likely that we will be fooled by the preamble appearing
			// as data in some other message.
			//
			case CHECK_IF_PREAMBLE_2:
				if (data == UBX_PREAMBLE2)
				{
					gps->step = GET_MSG_CLASS;
				}
				else
				{
					gps->step = CHECK_IF_PREAMBLE_1;
				}
				break;
				
			case CHECK_IF_PREAMBLE_1:
				if (data == UBX_PREAMBLE1)
				{
					gps->step = CHECK_IF_PREAMBLE_2;
				}
				else
				{
					gps->step = CHECK_IF_PREAMBLE_1;
				}
				break;
			// Message header processing
			//
			// We sniff the class and message ID to decide whether we
			// are going to gather the message bytes or just discard
			// them.
			//
			// We always collect the length so that we can avoid being
			// fooled by preamble bytes in messages.
			//
			case GET_MSG_CLASS:
				gps->step = GET_MSG_ID;
				gps->ubx_class = data;
				gps->cksum_a = data;
				gps->cksum_b = gps->cksum_a; // reset the checksum accumulators
				break;
			
			case GET_MSG_ID:
				gps->step = GET_MSG_PAYLOAD_1;
				gps->cksum_a += data;
				gps->cksum_b += gps->cksum_a; // checksum byte
				gps->msg_id = data;
				break;
			
			case GET_MSG_PAYLOAD_1:
				gps->step = GET_MSG_PAYLOAD_2;
				gps->cksum_a += data;
				gps->cksum_b += gps->cksum_a; // checksum byte
				gps->payload_length = data;
				break;
			
			case GET_MSG_PAYLOAD_2:
				gps->step = GET_MSG_BYTES;
				gps->payload_length |= data<<8;
				gps->cksum_a += data;
				gps->cksum_b += gps->cksum_a; // checksum byte
			
				if (gps->payload_length > 512)
				{
					// we assume very large payloads are line noise
					print_util_dbg_print("large payload: ");
					print_util_dbg_print_num(gps->payload_length,10);
					print_util_dbg_print(", for class:0x");
					print_util_dbg_print_num(gps->ubx_class,16);
					print_util_dbg_print(", and msg_id: 0x");
					print_util_dbg_print_num(gps->msg_id,16);
					print_util_dbg_print("\r\n");
					gps->payload_length = 0;
					gps->step = CHECK_IF_PREAMBLE_1;
					goto reset;
				}
				gps->payload_counter = 0; // prepare to receive payload

				if(gps->ubx_class == UBX_CLASS_NAV)
				{
					switch(gps->msg_id)
					{
						case MSG_NAV_POSLLH:
							if(gps->payload_length == UBX_SIZE_NAV_POSLLH)
							{
								ubx_current_message = (uint8_t **)&ubx_current_pos_llh_message;
								ubx_last_message = (uint8_t **)&ubx_last_pos_llh_message;
								ubx_valid_message = &ubx_number_of_valid_pos_llh_message;
							}
							else
							{
								print_util_dbg_print("Wrong Posllh message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_POSLLH,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_NAV_STATUS:
							if(gps->payload_length == UBX_SIZE_NAV_STATUS)
							{
								ubx_current_message = (uint8_t **)&ubx_current_status_message;
								ubx_last_message = (uint8_t **)&ubx_last_status_message;
								ubx_valid_message = &ubx_number_of_valid_status_message;
							}
							else
							{
								print_util_dbg_print("Wrong Nav Status message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_STATUS,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_NAV_SOL:
							if(gps->payload_length == UBX_SIZE_NAV_SOL)
							{
								ubx_current_message = (uint8_t **)&ubx_current_solution_message;
								ubx_last_message = (uint8_t **)&ubx_last_solution_message;
								ubx_valid_message = &ubx_number_of_valid_solution_message;;
							}
							else
							{
								print_util_dbg_print("Wrong Solution message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_SOL,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_NAV_VELNED:
							if(gps->payload_length == UBX_SIZE_NAV_VELNED)
							{
								ubx_current_message = (uint8_t **)&ubx_current_vel_ned_message;
								ubx_last_message = (uint8_t **)&ubx_last_vel_ned_message;
								ubx_valid_message = &ubx_number_of_valid_vel_ned_message;
							}
							else
							{
								print_util_dbg_print("Wrong Velned message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_VELNED,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_NAV_SVINFO:
							if(gps->payload_length == UBX_SIZE_NAV_SVINFO)
							{
								ubx_current_message = (uint8_t **)&ubx_current_sv_info_message;
								ubx_last_message = (uint8_t **)&ubx_last_sv_info_message;
								ubx_valid_message = &ubx_number_of_valid_sv_info_message;
							}
							else
							{
								print_util_dbg_print("Wrong SV Info message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_SVINFO,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_NAV_TIMEUTC:
							if (gps->payload_length == UBX_SIZE_NAV_TIMEUTC)
							{
								ubx_current_message = (uint8_t **)&ubx_current_nav_timeutc_message;
								ubx_last_message = (uint8_t **)&ubx_last_nav_timeutc_message;
								ubx_valid_message = & ubx_number_of_valid_nav_timeutc_message;
							}
							else
							{
								print_util_dbg_print("Wrong NAV TIMEUTC message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_TIMEUTC,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;

						case MSG_NAV_DGPS:
							if ( (gps->payload_length == UBX_SIZE_NAV_DGPS) || (gps->payload_length == 16) )
							{
								ubx_current_message = (uint8_t **)&ubx_current_nav_dgps_message;
								ubx_last_message = (uint8_t **)&ubx_last_nav_dgps_message;
								ubx_valid_message = & ubx_number_of_valid_nav_dgps_message;
							}
							else
							{
								print_util_dbg_print("Wrong NAV DGPS message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_NAV_DGPS,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						default:
							gps->step = CHECK_IF_PREAMBLE_1;
							if (gps->debug)
							{
								print_util_dbg_print("Unexpected NAV message, Class: 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(", msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" of size ");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print("\r\n");
							}
							goto reset;
					}
				}
				else if(gps->ubx_class == UBX_CLASS_CFG)
				{
					
					switch(gps->msg_id)
					{
						case MSG_CFG_NAV_SETTINGS:
							if(gps->payload_length == UBX_SIZE_CFG_NAV_SETTINGS)
							{
								ubx_current_message = (uint8_t **)&ubx_current_nav_settings_message;
								ubx_last_message = (uint8_t **)&ubx_last_nav_settings_message;
								ubx_valid_message = &ubx_number_of_valid_nav_settings_message;
							}
							else
							{
								print_util_dbg_print("Wrong Nav Settings message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_CFG_NAV_SETTINGS,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_CFG_RATE:
							if(gps->payload_length == UBX_SIZE_CFG_RATE)
							{
								ubx_current_message = (uint8_t **)&ubx_current_cfg_rate_message;
								ubx_last_message = (uint8_t **)&ubx_last_cfg_rate_message;
								ubx_valid_message = &ubx_number_of_valid_cfg_rate_message;
							}
							else
							{
								print_util_dbg_print("Wrong CFG Rate message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_CFG_RATE,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_CFG_SET_RATE:
							if (gps->payload_length == UBX_SIZE_CFG_GETSET_RATE)
							{
								ubx_current_message = (uint8_t **)&ubx_current_cfg_set_get_rate_message;
								ubx_last_message = (uint8_t **)&ubx_last_cfg_set_get_rate_message;
								ubx_valid_message = &ubx_number_of_valid_cfg_set_get_rate_message;
							}
							else
							{
								print_util_dbg_print("Wrong CFG Set/get message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_CFG_GETSET_RATE,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						default:
							gps->step = CHECK_IF_PREAMBLE_1;
							if (gps->debug)
							{
								print_util_dbg_print("Unexpected CFG message, Class: 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(", msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" of size ");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print("\r\n");
							}
							goto reset;
					}
				} else if (gps->ubx_class == UBX_CLASS_MON)
				{
					switch (gps->msg_id)
					{
						case MSG_MON_RXR:
							if(gps->payload_length == UBX_SIZE_MON_RXR)
							{
								ubx_current_message = (uint8_t **)&ubx_current_mon_rxr_message;
								ubx_last_message = (uint8_t **)&ubx_last_mon_rxr_message;
								ubx_valid_message = &ubx_number_of_valid_mon_rxr_message;
							}
							else
							{
								print_util_dbg_print("Wrong MON RXR message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_MON_RXR,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						default:
							gps->step = CHECK_IF_PREAMBLE_1;
							if (gps->debug)
							{
								print_util_dbg_print("Unexpected TIM message, Class: 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(", msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" of size ");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be : 0x");
								print_util_dbg_print_num(MSG_MON_RXR,16);
								print_util_dbg_print("\r\n");
							}
							goto reset;
					}
					
				}
				else if(gps->ubx_class == UBX_CLASS_TIM)
				{
					switch(gps->msg_id)
					{
						case MSG_TIM_TP:
							if (gps->payload_length == UBX_SIZE_TIM_TP)
							{
								ubx_current_message = (uint8_t **)&ubx_current_tim_tp_message;
								ubx_last_message = (uint8_t **)&ubx_last_tim_tp_message;
								ubx_valid_message = &ubx_number_of_valid_tim_tp_message;
							}
							else
							{
								print_util_dbg_print("Wrong TIM TP message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_TIM_TP,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						case MSG_TIM_VRFY:
							if (gps->payload_length == UBX_SIZE_TIM_VRFY)
							{
								ubx_current_message = (uint8_t **)&ubx_current_tim_vrfy_message;
								ubx_last_message = (uint8_t **)&ubx_last_tim_vrfy_message;
								ubx_valid_message = &ubx_number_of_valid_tim_vrfy_message;
							}
							else
							{
								print_util_dbg_print("Wrong TIM VRFY message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_TIM_VRFY,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						
						default:
							gps->step = CHECK_IF_PREAMBLE_1;
							if (gps->debug)
							{
								print_util_dbg_print("Unexpected TIM message, Class: 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(", msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" of size ");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be : 0x");
								print_util_dbg_print_num(MSG_TIM_TP,16);
								print_util_dbg_print("\r\n");
							}
							goto reset;
					}
				}
				else if (gps->ubx_class == UBX_CLASS_ACK)
				{
					switch(gps->msg_id)
					{
						case MSG_ACK_ACK:
						case MSG_ACK_NACK:
							if (gps->payload_length == UBX_SIZE_ACK)
							{
								ubx_current_message = (uint8_t **)&ubx_current_ack_message;
								ubx_last_message = (uint8_t **)&ubx_last_ack_message;
								ubx_valid_message = &ubx_number_of_valid_ack_message;
							}
							else
							{
								print_util_dbg_print("Wrong ACK message 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(" Msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" Received size:");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be:");
								print_util_dbg_print_num(UBX_SIZE_ACK,10);
								print_util_dbg_print("\r\n");
								gps->step = CHECK_IF_PREAMBLE_1;
								goto reset;
							}
							break;
						default:
							gps->step = CHECK_IF_PREAMBLE_1;
							if (gps->debug)
							{
								print_util_dbg_print("Unexpected ACK message, Class: 0x");
								print_util_dbg_print_num(gps->ubx_class,16);
								print_util_dbg_print(", msg id: 0x");
								print_util_dbg_print_num(gps->msg_id,16);
								print_util_dbg_print(" of size ");
								print_util_dbg_print_num(gps->payload_length,10);
								print_util_dbg_print(" should be : 0x");
								print_util_dbg_print_num(MSG_ACK_ACK,16);
								print_util_dbg_print(", or : 0x");
								print_util_dbg_print_num(MSG_ACK_NACK,16);
								print_util_dbg_print("\r\n");
							}
							goto reset;
					}
				}
				else
				{
					gps->step = CHECK_IF_PREAMBLE_1;
					if (gps->debug)
					{
						print_util_dbg_print("Unexpected message, Class: 0x");
						print_util_dbg_print_num(gps->ubx_class,16);
						print_util_dbg_print(", msg id: 0x");
						print_util_dbg_print_num(gps->msg_id,16);
						print_util_dbg_print(" of size ");
						print_util_dbg_print_num(gps->payload_length,10);
						print_util_dbg_print("\r\n");
					}
					goto reset;
				}
				break;
			
			case GET_MSG_BYTES:
				gps->cksum_a += data;
				gps->cksum_b += gps->cksum_a; // checksum byte
				
				#ifdef BIG_ENDIAN
				(*ubx_current_message)[gps->payload_length - 1 - gps->payload_counter] = data;
				#else
				(*ubx_current_message)[gps->payload_counter] = data;
				#endif
				
				gps->payload_counter++;
				
				if (gps->payload_counter == gps->payload_length)
				{
					gps->step = CHECK_CHECKSUM_A;
				}
				break;
			
			case CHECK_CHECKSUM_A:
				gps->step = CHECK_CHECKSUM_B;
				if (gps->cksum_a != data)
				{
					print_util_dbg_print("bad cksum_a ");
					print_util_dbg_print_num(data,16);
					print_util_dbg_print(" should be ");
					print_util_dbg_print_num(gps->cksum_a,16);
					print_util_dbg_print(" class : 0x");
					print_util_dbg_print_num(gps->ubx_class,16);
					print_util_dbg_print(" msg_id : 0x");
					print_util_dbg_print_num(gps->msg_id,16);
					print_util_dbg_print("\r\n");
					gps->step = CHECK_IF_PREAMBLE_1;
					goto reset;
				}
				break;
			
			case CHECK_CHECKSUM_B:
				gps->step = CHECK_IF_PREAMBLE_1;
				if (gps->cksum_b != data)
				{	
					print_util_dbg_print("bad cksum_b ");
					print_util_dbg_print_num(data,16);
					print_util_dbg_print(" should be ");
					print_util_dbg_print_num(gps->cksum_b,16);
					print_util_dbg_print(" class : 0x");
					print_util_dbg_print_num(gps->ubx_class,16);
					print_util_dbg_print(" msg_id : 0x");
					print_util_dbg_print_num(gps->msg_id,16);
					print_util_dbg_print("\r\n");
					break;
				}
				++(*ubx_valid_message);
				//print_util_dbg_print("Valid message");
				
				// swap message buffers, old message is discarded and becomes incoming buffer, new message become valid message (=old)
				temporary_message_for_swaping = *ubx_current_message;
				*ubx_current_message = *ubx_last_message;
				*ubx_last_message = temporary_message_for_swaping;
				
				if (gps_ublox_process_data(gps, gps->ubx_class, gps->msg_id))
				{
					msg_ok = true;
				}
		}
	}

	return msg_ok;
}


static bool gps_ublox_process_data(gps_t *gps, uint8_t ubx_class, uint8_t msg_id)
{
	ubx_nav_pos_llh_t *gps_pos_llh; 
	ubx_nav_status_t *gps_status;
	ubx_nav_solution_t *gps_solution;
	ubx_nav_vel_ned_t *gps_vel_ned;
	ubx_nav_sv_info_t *gps_sv_info;
	ubx_nav_timeutc_t *gps_nav_timeutc;
	ubx_nav_dgps_t *gps_nav_dgps;

	if (ubx_class == UBX_CLASS_ACK)
	{
		ubx_ack_t *gps_ack = ubx_get_ack();
		if (gps_ack)
		{
			print_util_dbg_print("Answer for class: 0x");
			print_util_dbg_print_num(gps_ack->class_id,16);
			print_util_dbg_print(", msg id: 0x");
			print_util_dbg_print_num(gps_ack->msg_id,16);
			print_util_dbg_print("=>");
			
			gps->acknowledged_received = true;
			
			if (msg_id)
			{
				print_util_dbg_print("Ok");
			}
			else
			{
				//print_util_dbg_print("Acknowledge answer for class: 0x");
				//print_util_dbg_print_num(gps_ack->class_id,16);
				//print_util_dbg_print(", and msg id: 0x");
				//print_util_dbg_print_num(gps_ack->msg_id,16);
				//print_util_dbg_print("=>");
				print_util_dbg_print("Nok");
			}
			print_util_dbg_print("\r\n");
		}
		return false;
	}
 	if (ubx_class == UBX_CLASS_MON)
 	{
 		ubx_mon_rxr_struct_t *gps_rxr = ubx_get_mon_rxr();
 		if (gps_rxr)
 		{
 			++gps->loop_mon_rxr;
 			gps->loop_mon_rxr %= gps->num_skipped_msg;
 			if ((gps->print_nav_on_debug)&&(gps->loop_mon_rxr == 0))
 			{
 				print_util_dbg_print("MSG_MON GPS awake\r\n");
 			}
 		}
		 return false;
 	}
	if (ubx_class == UBX_CLASS_TIM)
	{
		ubx_tim_tp_t *gps_tim_tp = ubx_get_tim_tp();
		if (gps_tim_tp)
		{
			++gps->loop_tim_tp;
			gps->loop_tim_tp %= gps->num_skipped_msg;
			if((gps->print_nav_on_debug)&&(gps->loop_tim_tp == 0))
			{
				print_util_dbg_print("MSG_TIM_TP GPS awake\r\n");
			}
		}
		ubx_tim_vrfy_t *gps_tim_vrfy = ubx_get_tim_vrfy();
		if (gps_tim_vrfy)
		{
			++gps->loop_tim_vrfy;
			gps->loop_tim_vrfy %= gps->num_skipped_msg;
			if((gps->print_nav_on_debug)&&(gps->loop_tim_vrfy == 0))
			{
				print_util_dbg_print("MSG_TIM_VRFY");
				print_util_dbg_print(" itow :");
				print_util_dbg_print_num(gps_tim_vrfy->itow,10);
				print_util_dbg_print(" frac :");
				print_util_dbg_print_num(gps_tim_vrfy->frac,10);
				print_util_dbg_print(" delta_ms :");
				print_util_dbg_print_num(gps_tim_vrfy->delta_ms,10);
				print_util_dbg_print(" delta_ns :");
				print_util_dbg_print_num(gps_tim_vrfy->delta_ns,10);
			}
		}
		return false;
	}
	 
	if (ubx_class == UBX_CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS)
	{
		ubx_cfg_nav_settings_t *gps_nav_settings = ubx_get_nav_settings();
		
		/*
		Dynamic Platform model:
		- 0 Portable
		- 2 Stationary
		- 3 Pedestrian
		- 4 Automotive
		- 5 Sea
		- 6 Airborne with <1g Acceleration
		- 7 Airborne with <2g Acceleration
		- 8 Airborne with <4g Acceleration
		*/
		if(gps_nav_settings)
		{
			if (gps->print_nav_on_debug)
			{
				print_util_dbg_print("Got engine settings ");
				print_util_dbg_print_num(gps_nav_settings->dyn_model,10);
				print_util_dbg_print(", fix_mode:");
				print_util_dbg_print_num(gps_nav_settings->fix_mode,10);
				print_util_dbg_print(", fixed_alt:");
				print_util_dbg_print_num(gps_nav_settings->fixed_alt,10);
				print_util_dbg_print(", fixed_alt_var:");
				print_util_dbg_print_num(gps_nav_settings->fixed_alt_var,10);
				print_util_dbg_print(", min_elev:");
				print_util_dbg_print_num(gps_nav_settings->min_elev,10);
				print_util_dbg_print(", dr_limit:");
				print_util_dbg_print_num(gps_nav_settings->dr_limit,10);
				print_util_dbg_print(", p_dop:");
				print_util_dbg_print_num(gps_nav_settings->p_dop,10);
				print_util_dbg_print(", t_dop:");
				print_util_dbg_print_num(gps_nav_settings->t_dop,10);
				print_util_dbg_print(", p_acc:");
				print_util_dbg_print_num(gps_nav_settings->p_acc,10);
				print_util_dbg_print(", t_acc:");
				print_util_dbg_print_num(gps_nav_settings->t_acc,10);
				print_util_dbg_print(", static_hold_thresh:");
				print_util_dbg_print_num(gps_nav_settings->static_hold_thresh,10);
				print_util_dbg_print(", dgps_timeout:");
				print_util_dbg_print_num(gps_nav_settings->dgps_timeout,10);
				print_util_dbg_print("\r\n");
			}
		}
		else
		{
			if (gps->engine_nav_setting != GPS_ENGINE_NONE)
			{
				if(gps_nav_settings->dyn_model != gps->engine_nav_setting)
				{
					// Here we change only the dynamic model, all the other parameters are the ones from the GPS
					gps->nav_settings.dyn_model = gps->engine_nav_setting;
					if (gps->print_nav_on_debug)
					{
						print_util_dbg_print("Send Nav settings");
					}
					ubx_send_message_nav_settings(gps, UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, &gps->nav_settings, sizeof(gps->nav_settings));
				}				
			}
			else
			{
				if (gps->print_nav_on_debug)
				{
					print_util_dbg_print("No engine settings received ");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print("\r\n");
				}
			}	
		}
		return false;
	}
	
	if (ubx_class == UBX_CLASS_CFG && msg_id == MSG_CFG_SET_RATE)
	{
		ubx_cfg_msg_rate_t *gps_msg_rate;
		gps_msg_rate = ubx_get_msg_rate();
		
		if (gps_msg_rate)
		{
			if (gps->print_nav_on_debug)
			{
				print_util_dbg_print("Message CFG Rate 0x");
				print_util_dbg_print_num(gps_msg_rate->msg_class,16);
				print_util_dbg_print_num(gps_msg_rate->msg_id_rate,16);
				print_util_dbg_print_num(gps_msg_rate->rate,10);
				print_util_dbg_print("\r\n");
			}
		}		
		return false;
	}
	
	if (ubx_class != UBX_CLASS_NAV)
	{
		if (gps->debug)
		{
			print_util_dbg_print("Unexpected message 0x");
			print_util_dbg_print_num(ubx_class,16);
			print_util_dbg_print("02x 0x");
			print_util_dbg_print_num(msg_id,10);
			print_util_dbg_print("02x\r\n");
		}
		if (++gps->disable_counter == 256)
		{
			// disable future sends of this message id, but
			// only do this every 256 messages, as some
			// message types can't be disabled and we don't
			// want to get into an ack war
			
			gps->disable_counter = 1;
			
			if (gps->debug)
			{
				print_util_dbg_print("Disabling message 0x");
				print_util_dbg_print_num(ubx_class,16);
				print_util_dbg_print("02x 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print("02x\r\n");
			}
			ubx_configure_message_rate(gps,ubx_class, msg_id, 0);
		}
		return false;
	}
	
	// Class NAV:
	switch (msg_id)
	{
		case MSG_NAV_POSLLH:
			gps_pos_llh = ubx_get_pos_llh();
			if (gps_pos_llh)
			{
				++gps->loop_pos_llh;
				gps->loop_pos_llh %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_pos_llh == 0))
				{
					print_util_dbg_print("MSG_NAV_POSLLH");
					print_util_dbg_print(" itow :");
					print_util_dbg_print_num(gps_pos_llh->itow,10);
					print_util_dbg_print(" longitude :");
					print_util_dbg_print_num(gps_pos_llh->longitude,10);
					print_util_dbg_print(" latitude :");
					print_util_dbg_print_num(gps_pos_llh->latitude,10);
					print_util_dbg_print(" alt_ellips :");
					print_util_dbg_print_num(gps_pos_llh->altitude_ellipsoid,10);
					print_util_dbg_print(" alt_msl :");
					print_util_dbg_print_num(gps_pos_llh->altitude_msl,10);
					print_util_dbg_print(" horz_acc :");
					print_util_dbg_print_num(gps_pos_llh->horizontal_accuracy,10);
					print_util_dbg_print(" vert_acc :");
					print_util_dbg_print_num(gps_pos_llh->vertical_accuracy,10);
					print_util_dbg_print("\r\n");
				}
				
				gps->time_gps = gps_pos_llh->itow;
				gps->longitude = gps_pos_llh->longitude / 10000000.0f;
				gps->latitude = gps_pos_llh->latitude / 10000000.0f;
				gps->alt_elips = ((float)gps_pos_llh->altitude_ellipsoid) / 1000.0f;
				gps->altitude = ((float)gps_pos_llh->altitude_msl) / 1000.0f;
				gps->horizontal_accuracy = ((float)gps_pos_llh->horizontal_accuracy) / 1000.0f;
				gps->vertical_accuracy = ((float)gps_pos_llh->vertical_accuracy) / 1000.0f;
				
				gps->time_last_posllh_msg = time_keeper_get_millis();
				
				gps->new_position = true;
			}
			break;
			
		case MSG_NAV_STATUS:
			gps_status = ubx_get_status();
			
			if (gps_status)
			{
				++gps->loop_status;
				gps->loop_status %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_status == 0))
				{
					print_util_dbg_print("MSG_STATUS fix_type = 0x");
					print_util_dbg_print_num(gps_status->fix_type,16);
					print_util_dbg_print(", uptime =");
					print_util_dbg_print_num(gps_status->uptime,10);
					print_util_dbg_print("\r\n");
				}
				gps->next_fix = (gps_status->fix_type == GPS_FIX_TYPE_3DFIX);
				if (!gps->next_fix)
				{
					gps->status = NO_FIX;
				}
				else
				{
					gps->status = GPS_OK;
				}
			}
			break;
			
		case MSG_NAV_SOL:
			gps_solution = ubx_get_solution();
			
			if (gps_solution)
			{
				++gps->loop_solution;
				gps->loop_solution %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_solution == 0))
				{
					print_util_dbg_print("MSG_SOL ");
					print_util_dbg_print("itow :");
					print_util_dbg_print_num(gps_solution->itow,10);
					print_util_dbg_print(" week :");
					print_util_dbg_print_num(gps_solution->week,10);
					print_util_dbg_print(" fix_type = 0x0");
					print_util_dbg_print_num(gps_solution->fix_type,16);
					print_util_dbg_print(" pos_acc_3d :");
					print_util_dbg_print_num(gps_solution->position_accuracy_3d,10);
					print_util_dbg_print(" ecefx :");
					print_util_dbg_print_num(gps_solution->ecef_x,10);
					print_util_dbg_print(" ecefy :");
					print_util_dbg_print_num(gps_solution->ecef_y,10);
					print_util_dbg_print(" ecefz :");
					print_util_dbg_print_num(gps_solution->ecef_z,10);
					print_util_dbg_print(" pos_DOP :");
					print_util_dbg_print_num(gps_solution->position_DOP,10);
					print_util_dbg_print(" num sat :");
					print_util_dbg_print_num(gps_solution->satellites,10);
					print_util_dbg_print("\r\n");
				}
				gps->next_fix = (gps_solution->fix_type == GPS_FIX_TYPE_3DFIX);
				if (!gps->next_fix)
				{
					gps->status = NO_FIX;
				}
				else
				{
					gps->status = GPS_OK;
				}
			
				gps->num_sats = gps_solution->satellites;
				gps->hdop = gps_solution->position_DOP;
			}
			break;
			
		case MSG_NAV_VELNED:
			gps_vel_ned = ubx_get_vel_ned();
			
			if (gps_vel_ned)
			{
				++gps->loop_vel_ned;
				gps->loop_vel_ned %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_vel_ned == 0))
				{
				
					print_util_dbg_print("MSG_NAV_VELNED ");
			
					print_util_dbg_print("itow :");
					print_util_dbg_print_num(gps_vel_ned->itow,10);
					print_util_dbg_print(" ned_north :");
					print_util_dbg_print_num(gps_vel_ned->ned_north,10);
					print_util_dbg_print(" ned_east :");
					print_util_dbg_print_num(gps_vel_ned->ned_east,10);
					print_util_dbg_print(" ned_down :");
					print_util_dbg_print_num(gps_vel_ned->ned_down,10);
					print_util_dbg_print(" speed_3d :");
					print_util_dbg_print_num(gps_vel_ned->speed_3d,10);
					print_util_dbg_print(" heading_2d :");
					print_util_dbg_print_num(gps_vel_ned->heading_2d,10);
					print_util_dbg_print(" speed_accuracy :");
					print_util_dbg_print_num(gps_vel_ned->speed_accuracy,10);
					print_util_dbg_print(" heading_accuracy :");
					print_util_dbg_print_num(gps_vel_ned->heading_accuracy,10);
					print_util_dbg_print("\r\n");
				}
				gps->time_gps         = gps_vel_ned->itow;
				gps->speed           = ((float)gps_vel_ned->speed_3d) / 100.; // m/s
				gps->ground_speed     = ((float)gps_vel_ned->ground_speed_2d) / 100.; // m/s
				gps->course          = ((float)gps_vel_ned->heading_2d) / 100000.; // Heading 2D deg * 100000 rescaled to deg * 100
				gps->have_raw_velocity    = true;
				gps->north_speed      = ((float)gps_vel_ned->ned_north) / 100.0f;
				gps->east_speed       = ((float)gps_vel_ned->ned_east) / 100.;
				gps->vertical_speed   = ((float)gps_vel_ned->ned_down) / 100.;
				gps->speed_accuracy   = ((float)gps_vel_ned->speed_accuracy) / 100.;
				gps->heading_accuracy = gps_vel_ned->heading_accuracy;
				
				gps->time_last_velned_msg = time_keeper_get_millis();
				
				gps->new_speed            = true;
			}
			break;
			
		case MSG_NAV_SVINFO:
			gps_sv_info = ubx_get_sv_info();
			
			if (gps_sv_info)
			{
				++gps->loop_sv_info;
				gps->loop_sv_info %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_sv_info == 0))
				{
					print_util_dbg_print("MSG_NAV_SVINFO, num_channel:");
					print_util_dbg_print_num(gps_sv_info->num_ch,10);
					for (int i=0; i<gps_sv_info->num_ch; i++)
					{
						print_util_dbg_print(", svid:");
						print_util_dbg_print_num(gps_sv_info->channel_data[i].svid,10);
					}
					print_util_dbg_print("\r\n");
				}
			}
			break;
			
		case MSG_NAV_TIMEUTC:
			gps_nav_timeutc = ubx_get_nav_timeutc();
			
			if (gps_nav_timeutc)
			{
				++gps->loop_nav_timeutc;
				gps->loop_nav_timeutc %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_nav_timeutc == 0))
				{
					print_util_dbg_print("MSG_NAV_TIMEUTC:");
					print_util_dbg_print(" itow :");
					print_util_dbg_print_num(gps_nav_timeutc->itow,10);
					print_util_dbg_print(" t_acc:");
					print_util_dbg_print_num(gps_nav_timeutc->t_acc,10);
					print_util_dbg_print(" nano:");
					print_util_dbg_print_num(gps_nav_timeutc->nano,10);
					print_util_dbg_print(" year:");
					print_util_dbg_print_num(gps_nav_timeutc->year,10);
					print_util_dbg_print(" month:");
					print_util_dbg_print_num(gps_nav_timeutc->month,10);
					print_util_dbg_print(" day:");
					print_util_dbg_print_num(gps_nav_timeutc->day,10);
					print_util_dbg_print(" hour:");
					print_util_dbg_print_num(gps_nav_timeutc->hour,10);
					print_util_dbg_print(" minute:");
					print_util_dbg_print_num(gps_nav_timeutc->minute,10);
					print_util_dbg_print(" seconds:");
					print_util_dbg_print_num(gps_nav_timeutc->seconds,10);
					print_util_dbg_print(" valid:");
					print_util_dbg_print_num(gps_nav_timeutc->valid,10);
					print_util_dbg_print("\r\n");
				}
				
				gps->date.year = gps_nav_timeutc->year;
				gps->date.month = gps_nav_timeutc->month;
				gps->date.day = gps_nav_timeutc->day;
				gps->date.hour = gps_nav_timeutc->hour;
				gps->date.minute = gps_nav_timeutc->minute;
				gps->date.second = gps_nav_timeutc->seconds;
				if (gps_nav_timeutc->valid&0b10)
				{
					gps->date.validity = UTC_TIME_VALID;
				}
				else
				{
					gps->date.validity = UTC_TIME_UNVALID;
				}
				
				if (gps->date.validity == UTC_TIME_VALID)
				{
					gps_ublox_utc_to_local(&gps->date,gps->time_zone);
					date = gps->date;
				}
				else
				{
					date.year = 2015;
					date.month = 5;
					date.day = 8;
					date.hour = 10;
					date.minute = 10;
					date.second = 0;
					date.validity = 0;
				}
				
			}
			break;
		
		case MSG_NAV_DGPS:
			gps_nav_dgps = ubx_get_nav_dgps();
			if (gps_nav_dgps)
			{
				++gps->loop_nav_dgps;
				gps->loop_nav_dgps %= gps->num_skipped_msg;
				if (gps->print_nav_on_debug && (gps->loop_nav_dgps == 0))
				{
					print_util_dbg_print("MSG_NAV_DGPS:");
					print_util_dbg_print("itow: ");
					print_util_dbg_print_num(gps_nav_dgps->itow,10);
					print_util_dbg_print(", age:");
					print_util_dbg_print_num(gps_nav_dgps->age,10);
					print_util_dbg_print(", base_id:");
					print_util_dbg_print_num(gps_nav_dgps->base_id,10);
					print_util_dbg_print(", base_health :");
					print_util_dbg_print_num(gps_nav_dgps->base_health,10);
					print_util_dbg_print(", num_channel :");
					print_util_dbg_print_num(gps_nav_dgps->num_channel,10);
					print_util_dbg_print(", status :");
					print_util_dbg_print_num(gps_nav_dgps->status,10);
					print_util_dbg_print("\r\n");
					for (int i = 0; i < maths_f_min(gps_nav_dgps->num_channel, 16); ++i)
					{
						print_util_dbg_print("Channel:");
						print_util_dbg_print_num(i,10);
						print_util_dbg_print("svid:");
						print_util_dbg_print_num(gps_nav_dgps->chan_data[i].sv_id,10);
						print_util_dbg_print("flags:");
						print_util_dbg_print_num(gps_nav_dgps->chan_data[i].flags,10);
						print_util_dbg_print("age_c:");
						print_util_dbg_print_num(gps_nav_dgps->chan_data[i].age_c,10);
						print_util_dbg_print("prc:");
						print_util_dbg_print_num(gps_nav_dgps->chan_data[i].prc,10);
						print_util_dbg_print("prrc:");
						print_util_dbg_print_num(gps_nav_dgps->chan_data[i].prrc,10);
						print_util_dbg_print("\r\n");
					}
				}
			}
			break;

		default:
			if (gps->debug)
			{
				print_util_dbg_print("Unexpected NAV message 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print("\r\n");
			}

			if (++(gps->disable_counter) == 256)
			{
				gps->disable_counter = 1;

				if (gps->debug)
				{
					print_util_dbg_print("Disabling NAV message 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print("\r\n");
				}
				
				ubx_configure_message_rate(gps, UBX_CLASS_NAV, msg_id, 0);
			}
			return false;
	}

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (gps->new_position && gps->new_speed)
	{
		gps->new_speed = false;
		gps->new_position = false;
		return true;
	}
	
	return false;
}


static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	while (len--)
	{
		*ck_a += *data;
		*ck_b += *ck_a;
		data++;
	}
}


static uint8_t endian_lower_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0x00FF);
}


static uint8_t endian_higher_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0xFF00)>>8;
}


static uint8_t endian_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x000000FF);
}


static uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x0000FF00)>>8;
}


static uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x00FF0000)>>16;
}


static uint8_t endian_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0xFF000000)>>24;
}

static void ubx_send_uint8(gps_t *gps, uint8_t byte, uint8_t *ck_a, uint8_t *ck_b)
{
	uint8_t data = byte;
	
	update_checksum(&data, 1, ck_a, ck_b);
	gps->serial->write(&data);
	
}

static void ubx_send_uint16(gps_t *gps, uint16_t byte, uint8_t *ck_a, uint8_t *ck_b)
{
	#ifdef GPS_LITTLE_ENDIAN
		uint8_t data = endian_lower_bytes_uint16(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_higher_bytes_uint16(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
	#else
		uint8_t data = endian_higher_bytes_uint16(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_lower_bytes_uint16(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
	#endif
}

static void ubx_send_uint32(gps_t *gps, uint32_t byte, uint8_t *ck_a, uint8_t *ck_b)
{
	#ifdef GPS_LITTLE_ENDIAN
		uint8_t data = endian_lower_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_mid_lower_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_mid_higher_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_higher_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
	#else
		uint8_t data = endian_higher_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_mid_higher_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_mid_lower_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
		
		data = endian_lower_bytes_uint32(byte);
		ubx_send_uint8(gps,data,ck_a,ck_b);
	#endif
}

static void ubx_send_header(gps_t *gps, uint8_t msg_class, uint8_t msg_id, uint16_t size, uint8_t *ck_a, uint8_t *ck_b)
{
	ubx_header_t header;
	header.preamble1		= UBX_PREAMBLE1;
	header.preamble2		= UBX_PREAMBLE2;
	header.msg_class		= msg_class;
	header.msg_id_header    	= msg_id;
	header.length			= size;
	
	gps->serial->write(&header.preamble1);
	gps->serial->write(&header.preamble2);
	
	ubx_send_uint8(gps,header.msg_class,ck_a,ck_b);
	ubx_send_uint8(gps,header.msg_id_header,ck_a,ck_b);
	ubx_send_uint16(gps,header.length,ck_a,ck_b);
}


static void ubx_send_cksum(gps_t *gps, uint8_t ck_sum_a, uint8_t ck_sum_b)
{
	gps->serial->write(&ck_sum_a);
	gps->serial->write(&ck_sum_b);
}

static void ubx_send_message_mon_ver(gps_t *gps, ubx_mon_ver_t *gps_mon_ver)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_MON;
	uint8_t msg_id = MSG_MON_VER;
	uint16_t size;
	
	if (gps_mon_ver != NULL)
	{
		size = UBX_SIZE_MON_VER;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	uint8_t i;
	if (gps_mon_ver != NULL)
	{
		for (i=0; i<30; i++)
		{
			ubx_send_uint8(gps, gps_mon_ver->sw_version[i], &ck_a, &ck_b);
		}
		for (i=0; i<10; i++)
		{
			ubx_send_uint8(gps, gps_mon_ver->hw_version[i], &ck_a, &ck_b);
		}
		for (i=0; i<30; i++)
		{
			ubx_send_uint8(gps, gps_mon_ver->rom_version[i], &ck_a, &ck_b);
		}
	}
	ubx_send_cksum(gps, ck_a,ck_b);
}




static void ubx_send_message_cfg_ant(gps_t *gps, ubx_cfg_ant_t *gps_cfg_ant)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_ANT;
	uint16_t size;
	
	if (gps_cfg_ant != NULL)
	{
		size = UBX_SIZE_CFG_ANT;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_ant != NULL)
	{
		ubx_send_uint16(gps, gps_cfg_ant->flags, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_ant->pins, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps, ck_a,ck_b);
}

static void ubx_send_message_cfg_dat(gps_t *gps, ubx_cfg_dat_t *gps_cfg_dat)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_DAT;
	uint16_t size;
	
	if (gps_cfg_dat != NULL)
	{
		size = UBX_SIZE_CFG_DAT;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_dat != NULL)
	{
		ubx_send_uint16(gps, gps_cfg_dat->datum_num, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a,ck_b);
}


static void ubx_send_message_cfg_fxn(gps_t *gps, ubx_cfg_fxn_t *gps_cfg_fxn)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_FXN;
	uint16_t size;
	
	if (gps_cfg_fxn != NULL)
	{
		size = UBX_SIZE_CFG_FXN;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_fxn != NULL)
	{
		ubx_send_uint32(gps, gps_cfg_fxn->flags, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_reacq, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_acq, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_reacq_off, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_acq_off, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_on, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->t_off, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->res, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_fxn->base_tow, &ck_a, &ck_b);
	}

	ubx_send_cksum(gps, ck_a,ck_b);
}

static void ubx_send_message_cfg_inf(gps_t *gps, ubx_cfg_inf_t *gps_cfg_inf)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_INF;
	uint16_t size;
	
	if (gps_cfg_inf != NULL)
	{
		size = UBX_SIZE_CFG_INF;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_inf != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_inf->protocol_id, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->res0, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_inf->res1, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask1, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask2, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask3, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask4, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask5, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_inf->inf_msg_mask6, &ck_a, &ck_b);
	}

	ubx_send_cksum(gps, ck_a,ck_b);
}

static void ubx_send_message_cfg_itfm(gps_t *gps, ubx_cfg_itfm_t *gps_cfg_itfm)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_ITFM;
	uint16_t size;
	
	if (gps_cfg_itfm != NULL)
	{
		size = UBX_SIZE_CFG_ITFM;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_itfm != NULL)
	{
		ubx_send_uint32(gps,gps_cfg_itfm->config,&ck_a, &ck_b);
		ubx_send_uint32(gps,gps_cfg_itfm->config2,&ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a,ck_b);
}

static void ubx_send_message_nav_settings(gps_t *gps, uint8_t msg_class, uint8_t msg_id, ubx_cfg_nav_settings_t *engine_settings, uint16_t size)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);

	if (engine_settings != NULL)
	{
		ubx_send_uint16(gps,engine_settings->mask,&ck_a,&ck_b);
		
		ubx_send_uint8(gps,engine_settings->dyn_model,&ck_a,&ck_b);

		ubx_send_uint8(gps,engine_settings->fix_mode,&ck_a,&ck_b);

		ubx_send_uint32(gps,engine_settings->fixed_alt,&ck_a,&ck_b);

		ubx_send_uint32(gps,engine_settings->fixed_alt_var,&ck_a,&ck_b);

		ubx_send_uint8(gps,engine_settings->min_elev,&ck_a,&ck_b);

		ubx_send_uint8(gps,engine_settings->dr_limit,&ck_a,&ck_b);

		ubx_send_uint16(gps,engine_settings->p_dop,&ck_a,&ck_b);

		ubx_send_uint16(gps,engine_settings->t_dop,&ck_a,&ck_b);

		ubx_send_uint16(gps,engine_settings->p_acc,&ck_a,&ck_b);

		ubx_send_uint16(gps,engine_settings->t_acc,&ck_a,&ck_b);

		ubx_send_uint8(gps,engine_settings->static_hold_thresh,&ck_a,&ck_b);

		ubx_send_uint8(gps,engine_settings->dgps_timeout,&ck_a,&ck_b);
		
		ubx_send_uint32(gps,engine_settings->res2,&ck_a,&ck_b);
		
		ubx_send_uint32(gps,engine_settings->res3,&ck_a,&ck_b);

		ubx_send_uint32(gps,engine_settings->res4,&ck_a,&ck_b);
	}
	
	ubx_send_cksum(gps, ck_a,ck_b);
}

static void ubx_send_message_nav_expert_settings(gps_t *gps, ubx_cfg_nav_expert_settings_t *expert_engine_settings)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_NAV_EXPERT_SETTINGS;
	uint16_t size;
	
	if (expert_engine_settings != NULL)
	{
		size = UBX_SIZE_CFG_NAV_EXPERT_SETTINGS;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (expert_engine_settings != NULL)
	{
		ubx_send_uint16(gps, expert_engine_settings->version, &ck_a, &ck_b);
		ubx_send_uint16(gps, expert_engine_settings->mask1, &ck_a, &ck_b);
		ubx_send_uint32(gps, expert_engine_settings->mak2, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res1, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res2, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->min_sv_s, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->max_sv_s, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->min_cn_o, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res3, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->ini_fix_3d, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res4, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res5, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res6, &ck_a, &ck_b);
		ubx_send_uint16(gps, expert_engine_settings->wkn_roll_over, &ck_a, &ck_b);
		ubx_send_uint32(gps, expert_engine_settings->res7, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res8, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res9, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->use_ppp, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->use_aop, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res11, &ck_a, &ck_b);
		ubx_send_uint8(gps, expert_engine_settings->res12, &ck_a, &ck_b);
		ubx_send_uint16(gps, expert_engine_settings->aop_opb_max_err, &ck_a, &ck_b);
		ubx_send_uint32(gps, expert_engine_settings->res13, &ck_a, &ck_b);
		ubx_send_uint32(gps, expert_engine_settings->res14, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_pm(gps_t *gps, ubx_cfg_pm_t *gps_cfg_pm)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_PM;
	uint16_t size;
	
	if (gps_cfg_pm != NULL)
	{
		size = UBX_SIZE_CFG_PM;
	}
	else
	{
		size = 0;
	}
	
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_pm != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_pm->version, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm->res1, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm->res2, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm->res3, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm->flags, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm->update_period, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm->search_period, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm->grid_offset, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm->on_time, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm->min_acq_time, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a, ck_b);
}

static void ubx_send_message_cfg_pm2(gps_t *gps, ubx_cfg_pm2_t *gps_cfg_pm2)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_PM2;
	uint16_t size;

	if (gps_cfg_pm2 != NULL)
	{
		size = UBX_SIZE_CFG_PM2;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_pm2 != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_pm2->version, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm2->res1, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm2->res2, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm2->res3, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->flags, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->update_period, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->search_period, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->grid_offset, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm2->on_time, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm2->min_acq_time, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm2->res4, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm2->res5, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->res6, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->res7, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm2->res8, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_pm2->res9, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_pm2->res10, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_pm2->res11, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a, ck_b);
}

static void ubx_send_message_cfg_prt(gps_t *gps, ubx_cfg_prt_t *gps_cfg_prt)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_PRT;
	uint16_t size;

	if (gps_cfg_prt != NULL)
	{
		size = UBX_SIZE_CFG_PRT;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_prt != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_prt->port_id, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_prt->res0, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_prt->tx_ready, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_prt->mode, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_prt->baud_rate, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_prt->in_proto_mask, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_prt->out_proto_mask, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_prt->flags, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_prt->res3, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a, ck_b);
}

static void ubx_send_message_cfg_rate(gps_t *gps, ubx_cfg_rate_t *gps_cfg_rate)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_RATE;
	uint16_t size;

	if (gps_cfg_rate != NULL)
	{
		size = UBX_SIZE_CFG_RATE;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_rate != NULL)
	{
		ubx_send_uint16(gps, gps_cfg_rate->measure_rate, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_rate->nav_rate, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_rate->time_ref, &ck_a, &ck_b);
	}
	
	ubx_send_cksum(gps, ck_a, ck_b);
}

static void ubx_send_message_cfg_rinv(gps_t *gps, ubx_cfg_rinv_t *gps_cfg_rinv)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_RINV;
	uint16_t size;

	if (gps_cfg_rinv != NULL)
	{
		size = UBX_SIZE_CFG_RINV;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_rinv != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_rinv->flags, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data2, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data3, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data4, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data5, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data6, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data7, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data8, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data9, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data10, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data11, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data12, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data13, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data14, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data15, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data16, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data17, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data18, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data19, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data20, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data21, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data22, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rinv->data23, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_rxm(gps_t *gps, ubx_cfg_rxm_t *gps_cfg_rxm)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_RXM;
	uint16_t size;

	if (gps_cfg_rxm != NULL)
	{
		size = UBX_SIZE_CFG_RXM;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_rxm != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_rxm->res, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_rxm->lp_mode, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_sbas(gps_t *gps, ubx_cfg_sbas_t *gps_cfg_sbas)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_SBAS;
	uint16_t size;

	if (gps_cfg_sbas != NULL)
	{
		size = UBX_SIZE_CFG_SBAS;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_sbas != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_sbas->mode, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_sbas->usage, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_sbas->max_sbas, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_sbas->scan_mode2, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_sbas->scan_mode1, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_tp(gps_t *gps, ubx_cfg_tp_t *gps_cfg_tp)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_TP;
	uint16_t size;

	if (gps_cfg_tp != NULL)
	{
		size = UBX_SIZE_CFG_TP;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_tp != NULL)
	{
		ubx_send_uint32(gps, gps_cfg_tp->interval, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp->length, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_tp->status, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_tp->time_ref, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_tp->flags, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_tp->res, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_tp->antenna_cable_delay, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_tp->rf_group_delay, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp->user_delay, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_tp5(gps_t *gps, ubx_cfg_tp5_t *gps_cfg_tp5)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_TP5;
	uint16_t size;

	if (gps_cfg_tp5 != NULL)
	{
		size = UBX_SIZE_CFG_TP5;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_tp5 != NULL)
	{
		ubx_send_uint8(gps, gps_cfg_tp5->tp_idx, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_tp5->res0, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_tp5->res1, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_tp5->ant_cable_delay, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_tp5->rf_group_delay, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->freq_period, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->freq_perid_lock, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->pulse_len_ratio, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->pulse_len_ratio_lock, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->user_config_delay, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_tp5->flags, &ck_a, &ck_b);
	}
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_usb(gps_t *gps, ubx_cfg_usb_t *gps_cfg_usb)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_USB;
	uint16_t size;

	if (gps_cfg_usb != NULL)
	{
		size = UBX_SIZE_CFG_USB;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_usb != NULL)
	{
	
		ubx_send_uint16(gps, gps_cfg_usb->vendor_id, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_usb->product_id, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_usb->res1, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_usb->res2, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_usb->power_consumption, &ck_a, &ck_b);
		ubx_send_uint16(gps, gps_cfg_usb->flags, &ck_a, &ck_b);
		
		int i;
		for (i=0; i<32; i++)
		{
			ubx_send_uint8(gps, gps_cfg_usb->vendor_string[i], &ck_a, &ck_b);
		}
		
		for (i=0; i<32; i++)
		{
			ubx_send_uint8(gps, gps_cfg_usb->product_string[i], &ck_a, &ck_b);
		}
		
		for (i=0; i<32; i++)
		{
			ubx_send_uint8(gps, gps_cfg_usb->serial_number[i], &ck_a, &ck_b);
		}
	}
	
	ubx_send_cksum(gps,ck_a,ck_b);
}

static void ubx_send_message_cfg_cfg(gps_t* gps, ubx_cfg_cfg_t *gps_cfg_cfg)
{
	uint8_t ck_a = 0, ck_b = 0;

	uint8_t msg_class = UBX_CLASS_CFG;
	uint8_t msg_id = MSG_CFG_CFG;
	uint16_t size;

	if (gps_cfg_cfg != NULL)
	{
		size = UBX_SIZE_CFG_CFG;
	}
	else
	{
		size = 0;
	}
	ubx_send_header(gps, msg_class, msg_id, size, &ck_a, &ck_b);
	
	if (gps_cfg_cfg != NULL)
	{
		ubx_send_uint32(gps, gps_cfg_cfg->clear_mask, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_cfg->save_mask, &ck_a, &ck_b);
		ubx_send_uint32(gps, gps_cfg_cfg->load_mask, &ck_a, &ck_b);
		ubx_send_uint8(gps, gps_cfg_cfg->device_mask, &ck_a, &ck_b);
	}

	ubx_send_cksum(gps, ck_a, ck_b);
}

static void ubx_configure_message_rate(gps_t* gps, uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	ubx_cfg_msg_rate_send_t msg;
	msg.msg_class = msg_class;
	msg.msg_id_rate = msg_id;
	msg.rate = rate;
	
	uint8_t header_class = UBX_CLASS_CFG;
	uint8_t header_msg_id = MSG_CFG_SET_RATE;
	uint16_t size = sizeof(msg);
	
	ubx_send_header(gps, header_class, header_msg_id, size, &ck_a, &ck_b);
	
	ubx_send_uint8(gps, msg.msg_class, &ck_a, &ck_b);
	ubx_send_uint8(gps, msg.msg_id_rate, &ck_a, &ck_b);
	ubx_send_uint8(gps, msg.rate, &ck_a, &ck_b);
	
	ubx_send_cksum(gps, ck_a,ck_b);
}

static ubx_nav_pos_llh_t * ubx_get_pos_llh()
{
	if (ubx_number_of_valid_pos_llh_message)
	{
		return ubx_last_pos_llh_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_status_t * ubx_get_status()
{
	if (ubx_number_of_valid_status_message)
	{
		return ubx_last_status_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_solution_t * ubx_get_solution()
{
	if (ubx_number_of_valid_solution_message)
	{
		return ubx_last_solution_message;
	}
	else
	{
		return 0;
	}
}

static ubx_nav_vel_ned_t * ubx_get_vel_ned()
 {
	if (ubx_number_of_valid_vel_ned_message)
	{
		return ubx_last_vel_ned_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_sv_info_t * ubx_get_sv_info()
{
	if (ubx_number_of_valid_sv_info_message)
	{
		return ubx_last_sv_info_message;
	}
	else
	{
		return 0;
	}
}


static ubx_cfg_nav_settings_t * ubx_get_nav_settings()
{
	if (ubx_number_of_valid_nav_settings_message)
	{
		return ubx_last_nav_settings_message;
	}
	else
	{
		return 0;
	}
}


static ubx_cfg_msg_rate_t * ubx_get_msg_rate()
{
	if (ubx_number_of_valid_cfg_set_get_rate_message)
	{
		return ubx_last_cfg_set_get_rate_message;
	}
	else
	{
		return 0;
	}
}


static ubx_mon_rxr_struct_t * ubx_get_mon_rxr()
{
	if (ubx_number_of_valid_mon_rxr_message)
	{
		return ubx_last_mon_rxr_message;
	}
	else
	{
		return 0;
	}
}


static ubx_tim_tp_t * ubx_get_tim_tp()
{
	if(ubx_number_of_valid_tim_tp_message)
	{
		return ubx_last_tim_tp_message;
	}
	else
	{
		return 0;
	}
}


static ubx_tim_vrfy_t * ubx_get_tim_vrfy()
{
	if(ubx_number_of_valid_tim_vrfy_message)
	{
		return ubx_last_tim_vrfy_message;
	}
	else
	{
		return 0;
	}
}

static ubx_nav_timeutc_t * ubx_get_nav_timeutc()
{
	if(ubx_number_of_valid_nav_timeutc_message)
	{
		return ubx_last_nav_timeutc_message;
	}
	else
	{
		return 0;
	}
}

static ubx_ack_t * ubx_get_ack()
{
	if (ubx_number_of_valid_ack_message)
	{
		return ubx_last_ack_message;
	}
	else
	{
		return 0;
	}
}

static ubx_nav_dgps_t * ubx_get_nav_dgps()
{
	if (ubx_number_of_valid_nav_dgps_message)
	{
		return ubx_last_nav_dgps_message;
	}
	else
	{
		return 0;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Gps_ublox::Gps_ublox(Serial& serial):
	serial_( serial ),
	last_update_us_( time_keeper_get_micros() ),
	last_position_update_us_( time_keeper_get_micros() ),
	last_velocity_update_us_( time_keeper_get_micros() ),
	global_position_( {0.0, 0.0, 0.0f, 0.0f} ),
	horizontal_position_accuracy_( 0.0f ),
	vertical_position_accuracy_( 0.0f ),
	global_velocity_( std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	velocity_accuracy_( 0.0f ),
	heading_( 0.0f ),
	heading_accuracy_( 0.0f ),
	num_sats_( 0 ),
	fix_( false ),
	healthy_( false	)
{
	gps_ublox_init(&gps, &serial_);
}


bool Gps_ublox::update(void)
{
	// Update old structure
	// TODO: remove global structure
	gps_ublox_update(&gps);

	// Copy relevant fields in class members
	last_update_us_ 		 = time_keeper_get_micros();
	last_position_update_us_ = 1000.0f * gps.time_last_posllh_msg;
	last_velocity_update_us_ = 1000.0f * gps.time_last_velned_msg;
	global_position_.longitude 	= gps.longitude;
	global_position_.latitude 	= gps.latitude;
	global_position_.altitude 	= gps.altitude;
	global_position_.heading 	= gps.course / 100.0f;
	horizontal_position_accuracy_ 	= gps.horizontal_accuracy;
	vertical_position_accuracy_ 	= gps.vertical_accuracy;
	global_velocity_[0] = gps.north_speed;
	global_velocity_[1] = gps.east_speed;
	global_velocity_[2] = gps.vertical_speed;
	velocity_accuracy_ 	= gps.speed_accuracy;
	heading_ 			= gps.course/ 100.0f;
	heading_accuracy_ 	= gps.heading_accuracy;
	num_sats_ 	= gps.num_sats;
	healthy_ 	= gps.healthy;
	
	if( gps.status > 0 )
	{
		fix_ = true;
	}
	else
	{
		fix_ = false;
	}
	
	return true;
}


const float& Gps_ublox::last_update_us(void) const
{
	return last_update_us_;
}


const float& Gps_ublox::last_position_update_us(void) const
{
	return last_position_update_us_;
}


const float& Gps_ublox::last_velocity_update_us(void) const
{
	return last_velocity_update_us_;
}


const global_position_t& Gps_ublox::global_position(void) const
{
	return global_position_;
}


const float& Gps_ublox::horizontal_position_accuracy(void) const
{
	return horizontal_position_accuracy_;
}


const float& Gps_ublox::vertical_position_accuracy(void) const
{
	return vertical_position_accuracy_;
}


const std::array<float,3>& Gps_ublox::global_velocity(void) const
{
	return global_velocity_;
}


const float& Gps_ublox::velocity_accuracy(void) const
{
	return velocity_accuracy_;
}


const float& Gps_ublox::heading(void) const
{
	return heading_;
}


const float& Gps_ublox::heading_accuracy(void) const
{
	return heading_accuracy_;
}


const uint8_t& Gps_ublox::num_sats(void) const
{
	return num_sats_;
}


const bool& Gps_ublox::fix(void) const
{
	return fix_;
}


const bool& Gps_ublox::healthy(void) const
{
	return healthy_;
}


void Gps_ublox::configure(void)
{
	gps.configure_gps = true;
}