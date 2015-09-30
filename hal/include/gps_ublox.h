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
#include "uart_int.h"

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
#define MSG_CFG_ITFM 0x39 // ok
#define MSG_CFG_INF 0x02 // ok
#define MSG_CFG_FXN 0x0E // ok
#define MSG_CFG_DAT 0x06 //nok
#define MSG_CFG_ANT 0x13 //ok

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
#define BIG_ENDIAN

// If your GPS is using little endian format, use this line, comment it otherwise
#define GPS_LITTLE_ENDIAN

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
	 * \brief The U-Blox CFG-NAVX5 settings structure definition
	 */
	typedef struct  
	{
		uint8_t res14;						///< Reserved slot
		uint8_t res13;						///< Reserved slot
		uint8_t aop_opb_max_err;			///< Maximum acceptable AssistNow Autonomus orbit error
		uint8_t res12;						///< Reserved slot
		uint8_t res11;						///< Reserved slot
		uint8_t use_aop;					///< AssistNow Autonomous
		uint8_t use_ppp;					///< use Precise Point Positioning flag
		uint8_t res9;						///< Reserved slot
		uint8_t res8;						///< Reserved slot
		uint32_t res7;						///< Reserved slot
		uint16_t wkn_roll_over;				///< GPS week rollover number
		uint8_t res6;						///< Reserved slot
		uint8_t res5;						///< Reserved slot
		uint8_t res4;						///< Reserved slot
		uint8_t ini_fix_3d;					///< Initial fix must be 3D flag (0=false/1=true)
		uint8_t res3;						///< Reserved slot
		uint8_t min_cn_o;					///< Minimum satellite signal level for navigation
		uint8_t max_sv_s;					///< Maximum number of satellites for navigation
		uint8_t min_sv_s;					///< Minimum number of satellites for navigation
		uint8_t res2;						///< Reserved slot
		uint8_t res1;						///< Reserved slot
		uint16_t mak2;						///< Second parameter bitmask
		uint16_t mask1;						///< First parameter bitmask
		uint16_t version;					///< Message version
	}ubx_cfg_nav_expert_settings_t;

	/**
	 * \brief The U-Blox CFG-PM structure definition
	 */
	typedef struct  
	{
		uint16_t min_acq_time; 				///< Minimal search time
		uint16_t on_time;	   				///< On time after first succeful fix
		uint32_t grid_offset;	   			///< Grid offset relative to GPS start of week
		uint32_t search_period;				///< Acquisition retry period
		uint32_t update_period;				///< Positin update period
		uint32_t flags;						///< PSM configuation flags
		uint8_t res3;						///< Reserved
		uint8_t res2;						///< Reserved
		uint8_t res1;						///< Reserved
		uint8_t version;					///< Message version
	}ubx_cfg_pm_t;

	/**
	 * \brief The U-Blox CFG-PM2 structure definition
	 */
	typedef struct  
	{
		uint32_t res11;						///< Reserved
		uint16_t res10;						///< Reserved
		int8_t res9;   						///< Reserved
		int8_t res8;   						///< Reserved
		uint32_t res7; 						///< Reserved
		uint32_t res6; 						///< Reserved
		uint16_t res5; 						///< Reserved
		uint16_t res4; 						///< Reserved
		uint16_t min_acq_time; 				///< Minimal search time
		uint16_t on_time;	   				///< On time after first succeful fix
		uint32_t grid_offset;	   			///< Grid offset relative to GPS start of week
		uint32_t search_period;				///< Acquisition retry period
		uint32_t update_period;				///< Positin update period
		uint32_t flags;						///< PSM configuation flags
		uint8_t res3;						///< Reserved
		uint8_t res2;						///< Reserved
		uint8_t res1;						///< Reserved
		uint8_t version;					///< Message version
	}ubx_cfg_pm2_t;

	/**
	 * \brief The U-Blox CFG-PRT structure definition
	 */
	typedef struct
	{
		uint16_t res3;						///< Reserved, set to 0
		uint16_t flags;						///< Reserved, set to 0
		uint16_t out_proto_mask;			///< A mask describing which ouput protocols are active
		uint16_t in_proto_mask;				///< A mask describing which input protocols are active
		uint32_t baud_rate;					///< Baudrate in bits/second
		uint32_t mode;						///< Bit mask describing UART mode
		uint16_t tx_ready;					///< Reserved up to firmware 7.0
		uint8_t res0;						///< Reserved
		uint8_t port_id;					///< Port ID (=1 or 2 for UART ports)
	}ubx_cfg_prt_t;

	/**
	 * \brief The U-Blox CFG-PRT structure definition
	 */
	typedef struct
	{
		uint16_t time_ref;					///< Alignment to reference time, 0=UTC, 1=GPS time.
		uint16_t nav_rate;					///< Navigation rate, in number of measurements cycles. Cannot be changed on u-blox 5 and 6, always equal 1.
		uint16_t measure_rate;				///< Measurement rate, GPS measurements are taken every measure_rate milliseconds
	}ubx_cfg_rate_t;

	/**
	 * \brief The U-Blox CFG-RINV structure definition
	 */
	typedef struct
	{
		uint8_t data23;	  					///< Data to store/stored in Remote Inventory
		uint8_t data22;	  					///< Data to store/stored in Remote Inventory
		uint8_t data21;	  					///< Data to store/stored in Remote Inventory
		uint8_t data20;	  					///< Data to store/stored in Remote Inventory
		uint8_t data19;	  					///< Data to store/stored in Remote Inventory
		uint8_t data18;	  					///< Data to store/stored in Remote Inventory
		uint8_t data17;	  					///< Data to store/stored in Remote Inventory
		uint8_t data16;	  					///< Data to store/stored in Remote Inventory
		uint8_t data15;	  					///< Data to store/stored in Remote Inventory
		uint8_t data14;	  					///< Data to store/stored in Remote Inventory
		uint8_t data13;	  					///< Data to store/stored in Remote Inventory
		uint8_t data12;	  					///< Data to store/stored in Remote Inventory
		uint8_t data11;	  					///< Data to store/stored in Remote Inventory
		uint8_t data10;	  					///< Data to store/stored in Remote Inventory
		uint8_t data9;	  					///< Data to store/stored in Remote Inventory
		uint8_t data8;	  					///< Data to store/stored in Remote Inventory
		uint8_t data7;	  					///< Data to store/stored in Remote Inventory
		uint8_t data6;	  					///< Data to store/stored in Remote Inventory
		uint8_t data5;	  					///< Data to store/stored in Remote Inventory
		uint8_t data4;	  					///< Data to store/stored in Remote Inventory
		uint8_t data3;	  					///< Data to store/stored in Remote Inventory
		uint8_t data2;	  					///< Data to store/stored in Remote Inventory
		uint8_t data;						///< Data to store/stored in Remote Inventory
		uint8_t flags;						///< 0=dumb, 1=binary
	}ubx_cfg_rinv_t;

	/**
	 * \brief The U-Blox CFG-RXM structure definition
	 */
	typedef struct
	{
		uint8_t lp_mode;					///< Low power mode
		uint8_t res;						///< Reserved set to 8
	}ubx_cfg_rxm_t;

	/**
	 * \brief The U-Blox CFG-SBAS structure definition
	 */
	typedef struct
	{
		uint32_t scan_mode1;				///< Which SBAS PRN numbers to search for, all bits to 0=auto_scan
		uint8_t scan_mode2;					///< Continuation of scanmode bitmask
		uint8_t max_sbas;					///< Maximum number of SBAS prioritized tracking channels
		uint8_t usage;						///< SBAS usage
		uint8_t mode;						///< SBAS mode
	}ubx_cfg_sbas_t;

	/**
	 * \brief The U-Blox CFG-TP structure definition
	 */
	typedef struct
	{
		int32_t user_delay;					///< User time function delay
		int16_t rf_group_delay;				///< Receiver RF group delay
		int16_t antenna_cable_delay;		///< Antenna cable delay
		uint8_t res;						///< Reserved
		uint8_t flags;						///< Bitmask
		uint8_t time_ref;					///< Alignment to reference time, 0=UTC, 1:GPS, 2:Local time
		int8_t status;						///< Time pulse config setting, +1:positive, 0:off, -1:negative
		uint32_t length;					///< Length of time pulse
		uint32_t interval;					///< Time interval for time pulse
	}ubx_cfg_tp_t;

	/**
	 * \brief The U-Blox CFG-TP5 structure definition
	 */
	typedef struct
	{
		uint32_t flags;						///< Configuratin flags
		int32_t user_config_delay;			///< User configurable delay
		uint32_t pulse_len_ratio_lock;		///< Pulse length or duty cycle when locked to GPS time, only used if 'lockedOtherSet' is set
		uint32_t pulse_len_ratio;			///< Pulse length or duty cycle, depending on 'isLength'
		uint32_t freq_perid_lock;			///< Frequency or period time when locked to GPS time, only used if 'lockedOtherSet' is set
		uint32_t freq_period;				///< Frequency or period time, depending on setting of bit 'isFreq'
		int16_t rf_group_delay;				///< RF group delay
		int16_t ant_cable_delay;			///< Antenna cable delay
		uint16_t res1;						///< Reserved
		uint8_t res0;						///< Reserved
		uint8_t tp_idx;						///< Timepulse selection
	}ubx_cfg_tp5_t;

	/**
	 * \brief The U-Blox CFG-USB structure definition
	 */
	typedef struct
	{
		char serial_number[32];				///< String containing the serial number, including 0-termination
		char product_string[32];			///< String containing the product name, including 0-termination
		char vendor_string[32];				///< String containing the vendor name, including 0-termination
		uint16_t flags;						///< Various configuration flag
		uint16_t power_consumption;			///< Power consumed by the device in mA
		uint16_t res2;						///< Reserved for special use, set to 1
		uint16_t res1;						///< Reserved, set to 0
		uint16_t product_id;				///< Product ID
		uint16_t vendor_id;					///< Vendor ID. This field shall only be set to registered
	}ubx_cfg_usb_t;

	/**
	 * \brief The U-Blox CFG-ITFM structure definition
	 */
	typedef struct
	{
		uint32_t config2;					///< Extra settings for jamming/interference monitor
		uint32_t config;					///< Interference config word
	}ubx_cfg_itfm_t;

	/**
	 * \brief The U-Blox CFG-INF structure definition
	 */
	typedef struct
	{
		uint8_t inf_msg_mask6;				///< A bit mask saying which information messages are enabled on each I/O target
		uint8_t inf_msg_mask5;				///< A bit mask saying which information messages are enabled on each I/O target
		uint8_t inf_msg_mask4;				///< A bit mask saying which information messages are enabled on each I/O target
		uint8_t inf_msg_mask3;				///< A bit mask saying which information messages are enabled on each I/O target
		uint8_t inf_msg_mask2;				///< A bit mask saying which information messages are enabled on each I/O target
		uint8_t inf_msg_mask1;				///< A bit mask saying which information messages are enabled on each I/O target
		uint16_t res1;						///< Reserved
		uint8_t res0;						///< Reserved
		uint8_t protocol_id;				///< Protocol identifier, 0:UBX, 1:NMEA, 2-255:reserved
	}ubx_cfg_inf_t;

	/**
	 * \brief The U-Blox CFG-FXN structure definition
	 */
	typedef struct
	{
		uint32_t base_tow;					///< Base time of week to which t_on/t_sleep are aligned if ABSOLUTE_SIGN is set
		uint32_t res;						///< Reserved
		uint32_t t_off;						///< Sleep time after normal ontime
		uint32_t t_on;						///< On time
		uint32_t t_acq_off;					///< Time the receiver stays in off-state, if acquisitin failed
		uint32_t t_reacq_off;				///< Time the receiver stays in off-state, if re-acquisition failed
		uint32_t t_acq;						///< Time the receiver tries to acquire satellites, before going to off state
		uint32_t t_reacq;					///< Time the receiver tries to re-acquire satellites, before going to off state
		uint32_t flags;						///< FXN configuration flags
	}ubx_cfg_fxn_t;

	/**
	 * \brief The U-Blox CFG-DAT structure definition
	 */
	typedef struct
	{
		uint16_t datum_num;					///< Geodetic Datum number, 0:WGS84, 1:WGS72, 2:ETH90, 3 ADI-M, ...
	}ubx_cfg_dat_t;

	/**
	 * \brief The U-Blox CFG-ANT structure definition
	 */
	typedef struct
	{
		uint16_t pins;						///< Antenna pin configuration
		uint16_t flags;						///< Antenna flag mask
	}ubx_cfg_ant_t;

	/**
	 * \brief The U-Blox MON-VER struture definition
	 */
	typedef struct
	{
		char rom_version[30];				///< Zero-terminated ROM version string
		char hw_version[10];				///< Zero-terminated hardware version string
		char sw_version[30];				///< Zero-terminated software version string
	}ubx_mon_ver_t;

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
	 * \brief The U-Blox NAV-DGPS message structure definition
	 */
	typedef struct
	{
		/**
		 * \brief The structure definition of a particular GPS
		 */
		struct
		{
			float prrc;						///< Pseudo range rate correction
			float prc;						///< Pseudo range correction
			uint16_t age_c;					///< Age of the latest correction data
			uint8_t flags;					///< Bitmask/channel number
			uint8_t sv_id;					///< Satellite ID
		}chan_data[16];

		uint16_t res1;						///< Reservec
		uint8_t status;						///< DGPS correction type status, 00:None, 01:PR+PRR correction
		uint8_t num_channel;				///< Number of channels for which correction data is following
		int16_t base_health;				///< DGPS base station health status
		int16_t base_id;					///< DGPS base station ID
		int32_t age;						///< Age of the newest correction data
		uint32_t itow;						///< GPS msTow
	}ubx_nav_dgps_t;

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

	/** 
	 *\brief The U-Blox NAV-TIMEUTC message structure definition
	 */
	typedef struct
	{
		uint8_t valid;						///< Validity of the time
		uint8_t seconds;					///< Second of minute
		uint8_t minute;						///< Minute of the hour
		uint8_t hour;						///< Hour of the day
		uint8_t day;						///< Day of month 
		uint8_t month;						///< Month 1..12 (UTC)
		uint16_t year;						///< Year range 1999..2099
		int32_t nano;						///< Nanoseconds of second, range -1e9..1e9 (UTC)
		uint32_t t_acc;						///< Time accuracy estimate
		uint32_t itow;						///< GPS msToW		
	}ubx_nav_timeutc_t;

	/** 
	 *\brief The U-Blox ACK-ACK and ACK-NACK message structure definition
	 */
	typedef struct  
	{
		uint8_t msg_id;						///< Message ID of the acknowledged messages
		uint8_t class_id;					///< Class ID of the acknowledged messages
	}ubx_ack_t;

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
	}ubx_cfg_nav_settings_t;
	
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
	}ubx_cfg_nav_expert_settings_t;

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
	}ubx_cfg_pm_t;

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
	}ubx_cfg_pm2_t;

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
	}ubx_cfg_prt_t;

	/**
	 * \brief The U-Blox CFG-PRT structure definition
	 */
	typedef struct
	{
		uint16_t measure_rate;				///< Measurement rate, GPS measurements are taken every measure_rate milliseconds
		uint16_t nav_rate;					///< Navigation rate, in number of measurements cycles. Cannot be changed on u-blox 5 and 6, always equal 1.
		uint16_t time_ref;					///< Alignment to reference time, 0=UTC, 1=GPS time.
	}ubx_cfg_rate_t;

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
	}ubx_cfg_rinv_t;

	/**
	 * \brief The U-Blox CFG-RXM structure definition
	 */
	typedef struct
	{
		uint8_t res;						///< Reserved set to 8
		uint8_t lp_mode;					///< Low power mode
	}ubx_cfg_rxm_t;

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
	}ubx_cfg_sbas_t;

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
	}ubx_cfg_tp_t;

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
	}ubx_cfg_tp5_t;

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
	}ubx_cfg_usb_t;

	/**
	 * \brief The U-Blox CFG-ITFM structure definition
	 */
	typedef struct
	{
		uint32_t config;					///< Interference config word
		uint32_t config2;					///< Extra settings for jamming/interference monitor
	}ubx_cfg_itfm_t;
	
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
	}ubx_cfg_inf_t;

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
	}ubx_cfg_fxn_t;

	/**
	 * \brief The U-Blox CFG-DAT structure definition
	 */
	typedef struct
	{
		uint16_t datum_num;					///< Geodetic Datum number, 0:WGS84, 1:WGS72, 2:ETH90, 3 ADI-M, ...
	}ubx_cfg_dat_t;

	/**
	 * \brief The U-Blox CFG-ANT structure definition
	 */
	typedef struct
	{
		uint16_t flags;						///< Antenna flag mask
		uint16_t pins;						///< Antenna pin configuration
	}ubx_cfg_ant_t;
	
	/**
	 * \brief The U-Blox MON-VER struture definition
	 */
	typedef struct
	{
		char sw_version[30];				///< Zero-terminated software version string
		char hw_version[10];				///< Zero-terminated hardware version string
		char rom_version[30];				///< Zero-terminated ROM version string
	}ubx_mon_ver_t;

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
	}ubx_nav_dgps_t;

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
	}ubx_nav_timeutc_t;

	/** 
	 *\brief The U-Blox ACK-ACK and ACK-NACK message structure definition
	 */
	typedef struct  
	{
		uint8_t class_id;
		uint8_t msg_id;
	}ubx_ack_ack_t;

#endif

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
}date_time_t;

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

	buffer_t gps_buffer;						///< The GPS buffer
	byte_stream_t gps_stream_in;				///< The incoming GPS byte stream
	byte_stream_t gps_stream_out;				///< The outgoing GPS byte stream
} gps_t;


/**
 * \brief	Initialize the gps U-Blox module
 *
 * \param	gps				The pointer to the GPS structure
 * \param	UID				The uart ID
 * \param	usart_conf_gps	The configuration of the GPS' uart
 */
void gps_ublox_init(gps_t *gps, int32_t UID, usart_config_t usart_conf_gps);


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

/**
 * \brief	Tranforming UTC to local time
 *
 * \param	today_date	The pointer to the date structure
 * \param	time_zone	The current time zone
 */
void gps_ublox_utc_to_local(date_time_t *today_date, uint8_t time_zone);

/**
 * \brief	Gets the current date and time
 *
 * \return	The current date and time
 */
date_time_t gps_ublox_get_date(void);

#ifdef __cplusplus
}
#endif

#endif //GPS_UBLOX_H__