/**
* This file decods the messages from the UBLOX GPS
*
* The MAV'RIC Framework
* Copyright © 2011-2014
*
* Laboratory of Intelligent Systems, EPFL
*
* This file is part of the MAV'RIC Framework.
*/


#ifndef GPS_UBLOX_H__
#define GPS_UBLOX_H__

#include "stdint.h"
#include "stdbool.h"

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
#define UBX_SIZE_NAV_SVINFO 30 //8 + 12*numChannel
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

#ifndef PI
#define PI 3.141592
#endif

#define DEG2RAD PI/180

// The UART bytes are sent in a little endian format from the GPS, if the processor is big endian, define BIG_ENDIAN
// Otherwise comment the following line
#define BIG_ENDIAN

#ifdef BIG_ENDIAN
	typedef struct {
		uint16_t length;					///> the length of the message
		uint8_t msg_id_header;				///> the msg id header
		uint8_t msg_class;					///> the class of the message
		uint8_t preamble2;					///> the 2nd preamble of the message
		uint8_t preamble1;					///> the 1st preamble of the message
	}ubx_header;

	typedef struct {
		uint16_t timeref;					///> the time reference
		uint16_t nav_rate;					///> the rate
		uint16_t measure_rate_ms;			///> the measure rate of the cfg_nav message in ms
	}ubx_cfg_nav_rate;
	
	// We still have to send to message in the correct order to the GPS
	typedef struct {
		uint16_t measure_rate_ms;			///> the measure_rate
		uint16_t nav_rate;					///> the rate
		uint16_t timeref;					///< the time reference, 0:UTC time, 1:GPS time
	}ubx_cfg_nav_rate_send;

	typedef struct {
		uint8_t rate;						///> the rate
		uint8_t msg_id_rate;				///> the msg id
		uint8_t msg_class;					///> the msg_class
	}ubx_cfg_msg_rate;

	// We still have to send to message in the correct order to the GPS
	typedef struct {
		uint8_t msg_class;					///> the msg class
		uint8_t msg_id_rate;				///> the msg id
		uint8_t rate;						///> the rate of the message id
	}ubx_cfg_msg_rate_send;

	typedef struct {
		uint32_t res4;						///< reserved slot
		uint32_t res3;						///< reserved slot
		uint32_t res2;						///< reserved slot
		uint8_t dgpsTimeOut;				///< DGPS timeout in sec
		uint8_t staticHoldThresh;			///< Static hold threshold cm/s
		uint16_t tAcc;						///< Time accuracy mask in m
		uint16_t pAcc;						///< Position accuracy mask in m
		uint16_t tDop;						///< Time DOP mask to use
		uint16_t pDop;						///< Position DOP mask to use
		uint8_t drLimit;					///< Maximum time to perform dead reckoning in case of GPS signal loos, in sec
		int8_t minElev;						///< Minimum elevation for a GNSS satellite to be used in NAV in deg
		uint32_t fixedAltVar;				///< Fixed altitude variance in 2D mode in m^2
		int32_t fixedAlt;					///< Fixed altitude for 2D fix mode in m
		uint8_t fixMode;					///< Fixing mode, 1:2D, 2:3D, 3:auto 2D/3D
		uint8_t dynModel;					///< UBX_PLATFORM_... type
		uint16_t mask;						///< Bitmask, see U-Blox 6 documentation
	}ubx_cfg_nav_settings;

	typedef struct {
		uint32_t vertical_accuracy;			///< vertical accuracy in mm
		uint32_t horizontal_accuracy;		///< horizontal accuracy in mm
		int32_t altitude_msl;				///< height above mean sea level in mm
		int32_t altitude_ellipsoid;			///< height above ellipsoid in mm
		int32_t latitude;					///< latitude in deg 1e-7
		int32_t longitude;					///< longitude in deg 1e-7
		uint32_t itow;						///< GPS msToW
	}ubx_nav_posllh;

	typedef struct {
		uint32_t uptime;					///< milliseconds since startup
		uint32_t time_to_first_fix;			///< time to first fix in milliseconds
		uint8_t flags2;						///< information about navigatio output
		uint8_t fix_status;					///< Fix status information
		uint8_t flags;						///< Nav status flag
		uint8_t fix_type;					///< fix type
		uint32_t itow;						///< GPS msToW
	}ubx_nav_status;

	typedef struct {
		uint32_t res2;						///< reserved slot
		uint8_t satellites;					///< number of of SVs used in Nav solution
		uint8_t res;						///< reserved slot
		uint16_t position_DOP;				///< Position DOP, scaling 0.01
		uint32_t speed_accuracy;			///< Speed accuracy estimate in cm/s
		int32_t ecef_z_velocity;			///< Earth centered, earth frame, z velocity coordinate in cm/s
		int32_t ecef_y_velocity;			///< Earth centered, earth frame, y velocity coordinate in cm/s
		int32_t ecef_x_velocity;			///< Earth centered, earth frame, x velocity coordinate in cm/s
		uint32_t position_accuracy_3d;		///< 3D position accuracy estimate in cm
		int32_t ecef_z;						///< Earth centered, earth frame, z coordinate in cm
		int32_t ecef_y;						///< Earth centered, earth frame, y coordinate in cm
		int32_t ecef_x;						///< Earth centered, earth frame, x coordinate in cm
		uint8_t fix_status;					///< the fix status
		uint8_t fix_type;					///< the fix type
		int16_t week;						///< GPS week (GPS time)
		int32_t time_nsec;					///< Fractional nanoseconds remainder of rounded ms above
		uint32_t itow;						///< GPS msToW
	}ubx_nav_solution;

	typedef struct {
		uint32_t heading_accuracy;			///< course/heading estimate accuracy in deg 1e-5
		uint32_t speed_accuracy;			///< speed accuracy estimate cm/s
		int32_t heading_2d;					///< heading of motion in deg 1e-5
		uint32_t groundSpeed_2d;			///< ground speed in cm/s
		uint32_t speed_3d;					///< 3D speed in cm/s
		int32_t ned_down;					///< NED Down velocity in cm/s
		int32_t ned_east;					///< NED East velocity in cm/s
		int32_t ned_north;					///< NED North velocity in cm/s
		uint32_t itow;						///< GPS msToW
	}ubx_nav_velned;

	typedef struct
	{
		struct
		{
			int32_t prRes;					///< Pseudo range in residual in centimeters
			int16_t azim;					///< Azimuth in integer degrees
			int8_t elev;					///< Elevation in integer degrees
			uint8_t cno;					///< Carrier to Noise ratio in dbHz
			uint8_t quality;				///< Bitmask, see U-Blox 6 documentation
			uint8_t flags;					///< Bitmask, see U-Blox 6 documentation
			uint8_t svid;					///< Satellite ID
			uint8_t chn;					///< GPS msToW
		} channelData[16];
		
		uint16_t reserved;					///< reserved slot
		uint8_t globalFlags;				///< Bitmask, 0:antaris, 1:u-blox 5, 2:u-blox 6
		uint8_t numCh;						///< Number of channels
		uint32_t itow;						///< GPS msToW
	}ubx_nav_SVInfo;
	
 	typedef struct{
 		uint8_t awake_flag;					///< Receiver status flag
 	}ubx_mon_rxr_struct;
	 
	typedef struct{
		uint8_t res;						///< unused
		uint8_t flags;						///< Bitmask, 0,1:gps timebase, UTC not available, 2,3:UTC timebase, UTC available
		uint16_t week;						///< Timepulse week number according to timebase
		int32_t qErr;						///< Quantization error of timepulse
		uint32_t towSubMS;					///< Sumbmillisecond part of ToWms scaling: 2^-32
		uint32_t towMS;						///< Timepulse time of week according to time base in ms
	}ubx_tim_tp;

	typedef struct
	{
		uint8_t res;						///< reserved slot
		uint8_t flags;						///< aiding time source, 0:no time aiding done, 2:source was RTC, 3:source was AID-INI
		uint16_t wno;						///< week number
		int32_t deltaNs;					///< sub-millisecond part of delta time
		int32_t deltaMs;					///< inter ms of delta time
		int32_t frac;						///< sub-millisecond part of ToW in ns
		int32_t itow;						///< integer ms ToW received by source
	}ubx_tim_vrfy;

#else	

	typedef struct {
		uint8_t preamble1;					///> the 1st preamble of the message
		uint8_t preamble2;					///> the 2nd preamble of the message
		uint8_t msg_class;					///> the class of the message
		uint8_t msg_id_header;				///> the msg id header
		uint16_t length;					///> the length of the message
	}ubx_header;

	typedef struct {
		uint16_t measure_rate_ms;			///> the measure rate of the cfg_nav message in ms
		uint16_t nav_rate;					///> the rate
		uint16_t timeref;					///> the time reference
	}ubx_cfg_nav_rate;

	typedef struct {
		uint8_t msg_class;					///> the msg_class
		uint8_t msg_id_rate;				///> the msg id
		uint8_t rate;						///> the rate
	}ubx_cfg_msg_rate;

	typedef struct {
		uint16_t mask;						///< Bitmask, see U-Blox 6 documentation
		uint8_t dynModel;					///< UBX_PLATFORM_... type
		uint8_t fixMode;					///< Fixing mode, 1:2D, 2:3D, 3:auto 2D/3D
		int32_t fixedAlt;					///< Fixed altitude for 2D fix mode in m
		uint32_t fixedAltVar;				///< Fixed altitude variance in 2D mode in m^2
		int8_t minElev;						///< Minimum elevation for a GNSS satellite to be used in NAV in deg
		uint8_t drLimit;					///< Maximum time to perform dead reckoning in case of GPS signal loos, in sec
		uint16_t pDop;						///< Position DOP mask to use
		uint16_t tDop;						///< Time DOP mask to use
		uint16_t pAcc;						///< Position accuracy mask in m
		uint16_t tAcc;						///< Time accuracy mask in m
		uint8_t staticHoldThresh;			///< Static hold threshold cm/s
		uint8_t dgpsTimeOut;				///< DGPS timeout in sec
		uint32_t res2;						///< reserved slot
		uint32_t res3;						///< reserved slot
		uint32_t res4;						///< reserved slot
	}ubx_cfg_nav_settings;

	typedef struct {
		uint32_t itow;						///< GPS msToW
		int32_t longitude;					///< longitude in deg 1e-7
		int32_t latitude;					///< latitude in deg 1e-7
		int32_t altitude_ellipsoid;			///< height above ellipsoid in mm
		int32_t altitude_msl;				///< height above mean sea level in mm
		uint32_t horizontal_accuracy;		///< horizontal accuracy in mm
		uint32_t vertical_accuracy;			///< vertical accuracy in mm
	}ubx_nav_posllh;

	typedef struct {
		uint32_t itow;						///< GPS msToW
		uint8_t fix_type;					///< fix type
		uint8_t flags;						///< Nav status flag
		uint8_t fix_status;					///< Fix status information
		uint8_t flags2;						///< information about navigatio output
		uint32_t time_to_first_fix;			///< time to first fix in milliseconds
		uint32_t uptime;					///< milliseconds since startup
	}ubx_nav_status;

	typedef struct {
		uint32_t itow;						///< GPS msToW
		int32_t time_nsec;					///< Fractional nanoseconds remainder of rounded ms above
		int16_t week;						///< GPS week (GPS time)
		uint8_t fix_type;					///< the fix type
		uint8_t fix_status;					///< the fix status
		int32_t ecef_x;						///< Earth centered, earth frame, x coordinate in cm
		int32_t ecef_y;						///< Earth centered, earth frame, y coordinate in cm
		int32_t ecef_z;						///< Earth centered, earth frame, z coordinate in cm
		uint32_t position_accuracy_3d;		///< 3D position accuracy estimate in cm
		int32_t ecef_x_velocity;			///< Earth centered, earth frame, x velocity coordinate in cm/s
		int32_t ecef_y_velocity;			///< Earth centered, earth frame, y velocity coordinate in cm/s
		int32_t ecef_z_velocity;			///< Earth centered, earth frame, z velocity coordinate in cm/s
		uint32_t speed_accuracy;			///< Speed accuracy estimate in cm/s
		uint16_t position_DOP;				///< Position DOP, scaling 0.01
		uint8_t res;						///< reserved slot
		uint8_t satellites;					///< number of of SVs used in Nav solution
		uint32_t res2;						///< reserved slot
	}ubx_nav_solution;

	typedef struct {
		uint32_t itow;						///< GPS msToW
		int32_t ned_north;					///< NED North velocity in cm/s
		int32_t ned_east;					///< NED East velocity in cm/s
		int32_t ned_down;					///< NED Down velocity in cm/s
		uint32_t speed_3d;					///< 3D speed in cm/s
		uint32_t groundSpeed_2d;			///< ground speed in cm/s
		int32_t heading_2d;					///< heading of motion in deg 1e-5
		uint32_t speed_accuracy;			///< speed accuracy estimate cm/s
		uint32_t heading_accuracy;			///< course/heading estimate accuracy in deg 1e-5
	}ubx_nav_velned;

	typedef struct
	{
		uint32_t itow;						///< GPS msToW
		uint8_t numCh;						///< Number of channels
		uint8_t globalFlags;				///< Bitmask, 0:antaris, 1:u-blox 5, 2:u-blox 6
		uint16_t reserved;					///< reserved slot
	
		struct
		{
			uint8_t chn;					///< GPS msToW
			uint8_t svid;					///< Satellite ID
			uint8_t flags;					///< Bitmask, see U-Blox 6 documentation
			uint8_t quality;				///< Bitmask, see U-Blox 6 documentation
			uint8_t cno;					///< Carrier to Noise ratio in dbHz
			int8_t elev;					///< Elevation in integer degrees
			int16_t azim;					///< Azimuth in integer degrees
			int32_t prRes;					///< Pseudo range in residual in centimeters
		} channelData[16];
	}ubx_nav_SVInfo;

	typedef struct{
		uint8_t awake_flag;					///< Receiver status flag
	}ubx_mon_rxr_struct;

	typedef struct  
	{
		uint32_t towMS;						///< Timepulse time of week according to time base in ms
		uint32_t towSubMS;					///< Sumbmillisecond part of ToWms scaling: 2^-32
		int32_t qErr;						///< Quantization error of timepulse
		uint16_t week;						///< Timepulse week number according to timebase
		uint8_t flags;						///< Bitmask, 0,1:gps timebase, UTC not available, 2,3:UTC timebase, UTC available
		uint8_t res;						///< unused
	}ubx_tim_tp;

	typedef struct
	{
		int32_t itow;						///< integer ms ToW received by source
		int32_t frac;						///< sub-millisecond part of ToW in ns
		int32_t deltaMs;					///< inter ms of delta time
		int32_t deltaNs;					///< sub-millisecond part of delta time
		uint16_t wno;						///< week number
		uint8_t flags;						///< aiding time source, 0:no time aiding done, 2:source was RTC, 3:source was AID-INI
		uint8_t res;						///< reserved slot
	}ubx_tim_vrfy;

#endif

ubx_cfg_nav_settings nav_settings;

uint8_t cksum_a;
uint8_t cksum_b;

// State machine state
uint8_t         step;
uint8_t         msg_id;
uint16_t        payload_length;
uint16_t        payload_counter;

// 8 bit count of fix messages processed, used for periodic
// processing
uint8_t			fix_count;

uint8_t         ubxclass;

// do we have new position and speed information?
bool new_position;
bool new_speed;

uint8_t         disable_counter;

bool fix;
bool next_fix;
bool have_raw_velocity;
// bool valid_read;
// bool new_data;

//uint32_t last_ground_speed_cm;

#define NO_GPS 0						///< No GPS
#define NO_FIX 1						///< No GPS fix
#define GPS_OK 2						///< GPS ok

enum GPS_Engine_Setting{
	GPS_ENGINE_NONE        = -1,		///< None
	GPS_ENGINE_PORTABLE    = 0,			///< Portable
	GPS_ENGINE_STATIONARY  = 2,			///< Stationary
	GPS_ENGINE_PEDESTRIAN  = 3,			///< Pedestrian
	GPS_ENGINE_AUTOMOTIVE  = 4,			///< Automotive
	GPS_ENGINE_SEA         = 5,			///< Sea
	GPS_ENGINE_AIRBORNE_1G = 6,			///< Airborne with <1g acceleration
	GPS_ENGINE_AIRBORNE_2G = 7,			///< Airborne with <2g acceleration
	GPS_ENGINE_AIRBORNE_4G = 8			///< Airborne with <4g acceleration
};

enum GPS_Engine_Setting engine_nav_setting;


#define UBX_TIMEOUT_CYCLES 2			///< Number of times ubx_CheckTimeout() must be called without response from GPS before it is considered as timed out
#define UBX_POSITION_PRECISION 20		///< The minimum precision to consider a position as correct (in m)
#define UBX_ALTITUDE_PRECISION 20		///< The minimum precision to consider an altitude as correct (in m)
#define UBX_SPEED_PRECISION 5			///< The minimum precision to consider a speed as correct (in m/s)

#define UBX_HEADING_PRECISION 5000000	///< The minimum precision to consider a heading as correct (in deg*10^5)


typedef struct
{
	double latitude;						///< Latitude in degrees
	double longitude;						///< Longitude in degrees
	float altitude;							///< Altitude in m
	float alt_elips;						///< Altitude above ellipsoid in m
	float speed;							///< 3D speed in m/s
	float groundSpeed;						///< 2D ground speed in m/s
	float northSpeed;						///< The speed to the north in m/s
	float eastSpeed;						///< The speed to the east in m/s
	float verticalSpeed;					///< The vertical speed in m/s
	float course;							///< Heading in degree * 100
	
	float horizontalAccuracy;				///< Horizontal accuracy in m
	float verticalAccuracy;					///< Vertical accuracy in m
	
	float speedAccuracy;					///< Speed accuracy in m
	float headingAccuracy;					///< Heading accuracy in m
	
	uint8_t num_sats;						///< Number of visible satellites
	uint16_t hdop;							///< Height DOP
	
	uint32_t timeLastMsg;					///< Time reference in ms of microcontroller
	uint32_t timegps;						///< Time reference in ms of gps
	
	unsigned char status;					///< GPS status
	
	unsigned char horizontalStatus;			///< Horizontal status
	
 	unsigned char altitudeStatus;			///< Altitude status
 	unsigned char speedStatus;				///< Speed status
 	unsigned char courseStatus;				///< Course status
	unsigned char accuracyStatus;			///< Accuracy status
} gps_Data_type;						///< Type definition for GPS data


uint32_t idleTimer;							///< Last time that the GPS driver got a good packet from the GPS


uint32_t idleTimeout;						///< Time in milliseconds after which we will assume the GPS is no longer sending us updates and attempt a re-init. 1200ms allows a small amount of slack over the worst-case 1Hz update rate.

uint32_t last_fix_time;						///< Last fix time


/**
 * \brief	Initialize the gps U-Blox module
 *
 * \param	_engine_nav_setting		the GPS Nav settings 
 *
 * \return	void
 */
void init_gps_ubx(enum GPS_Engine_Setting _engine_nav_setting);

/**
 * \brief	Process bytes available from the stream
 *
 * The stream is assumed to contain only messages we recognise.  If it
 * contains other messages, and those messages contain the preamble
 * bytes, it is possible for this code to fail to synchronise to the
 * stream immediately.  Without buffering the entire message and
 * re-processing it from the top, this is unavoidable. The parser
 * attempts to avoid this when possible.
 *
 * \param	void
 *
 * \return	true if new velocity and new position message
 */
bool ubx_read(void);

/**
 * \brief	Process the new received message, class by class
 *
 * \param	void
 *
 * \return	true if new velocity and new position message
 */
bool ubx_process_data(void);

/**
 * \brief	Checksum update
 *
 * \param	data	pointer to the data to update the checksum
 * \param	len		length of the data to update the checksum
 * \param	ck_a	checksum a: sum of all the data
 * \param	ck_b	checksum b: sum of checksum a
 *
 * \return	true if new velocity and new position message
 */
void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);

/**
 * \brief	To send the lower bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint16 bytes
 */
uint8_t endian_lower_bytes_uint16(uint16_t bytes);

/**
 * \brief	To send the higher bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint16 bytes
 */
uint8_t endian_higher_bytes_uint16(uint16_t bytes);

/**
 * \brief	To send the UBX header of all messages
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	_msg_id		the U-Blox message ID
 * \param	size		the size of the U-Blox following message
 *
 * \return	void
 */
void ubx_send_header(uint8_t msg_class, uint8_t _msg_id, uint8_t size);

/**
 * \brief	To send the checksum of every message
 *
 * \param	ck_sum_a	the checksum a
 * \param	ck_sum_b	the checksum b
 *
 * \return	void
 */
void ubx_send_cksum(uint8_t ck_sum_a, uint8_t ck_sum_b);

/**
 * \brief	To send a CFG NAV RATE message
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x08	MSG_CFG_RATE
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	_msg_id		the U-Blox message ID
 * \param	msg			the CFG_NAV_RATE message
 * \param	size		the size of the U-Blox following message
 *
 * \return	void
 */
void ubx_send_message_CFG_nav_rate(uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_rate_send msg, uint8_t size);

/**
 * \brief	To send the NAV settings message
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x24	MSG_CFG_NAV_SETTINGS
 *
 * \param	msg_class			the U-Blox class of the message
 * \param	_msg_id				the U-Blox message ID
 * \param	engine_settings		the engine_settings sent
 * \param	size				the size of the U-Blox following message
 *
 * \return	void
 */
void ubx_send_message_nav_settings(uint8_t msg_class, uint8_t _msg_id, enum GPS_Engine_Setting *engine_settings, uint8_t size);

/**
 * \brief	To send the NAV messages that we want to receive
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x01	MSG_CFG_SET_RATE
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	msg_id		the U-Blox message ID
 * \param	rate		the rate of the CFG message
 *
 * \return	void
 */
void ubx_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

/**
 * \brief	To configure the GPS in binary mode and the Navigation messages we want
 *
 * The GPS and UART channel should already be configured in the good baudrate 38400U
 *
 * \param	void
 *
 * \return	void
 */
void configure_gps(void);


/**
 * \brief	The function that needs to be called to get the GPS information
 *
 * \param	void
 *
 * \return	void
 */
void gps_update(void);

/**
 * \brief	The function that tells if a message is arrived at time tnow
 *
 * \param	prevGpsMsgTime		the time of the previous GPS message
 *
 * \return	if the latest GPS message is arrived at tnow
 */
bool newValidGpsMsg(uint32_t *prevGpsMsgTime);

/**
 * \brief	This function returns a pointer to the last NAV-POSLLH message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid posllh message, or 0.
 */
ubx_nav_posllh * ubx_GetPosllh(void);

/**
 * \brief	This function returns a pointer to the last NAV-STATUS message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid status message, or 0.
 */
ubx_nav_status * ubx_GetStatus(void);
/**
 * \brief	This function returns a pointer to the last NAV-SOL message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid NAV-SOL message, or 0.
 */
ubx_nav_solution * ubx_GetSolution(void);

/**
* \brief	This function returns a pointer to the last NAV-VELNED message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid velned message, or 0.
*/
ubx_nav_velned * ubx_GetVelned(void);

/**
* \brief	This function returns a pointer to the last NAV-SVINFO message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_nav_SVInfo * ubx_GetSVInfo(void);

/**
* \brief	This function returns a pointer to the last NAV-Settings message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_cfg_nav_settings * ubx_GetNavSettings(void);

/**
* \brief	This function returns a pointer to the last CFG set/get rate message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_cfg_msg_rate * ubx_GetMsgRate(void);

/**
* \brief	This function returns a pointer to the last MON RXR message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_mon_rxr_struct * ubx_GetMonRXR(void);

/**
* \brief	This function returns a pointer to the last TIM TP message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_tim_tp * ubx_GetTimTP(void);

/**
* \brief	This function returns a pointer to the last TIM VRFY message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_tim_vrfy * ubx_GetTimVRFY(void);

/**
* \brief	This function transforms a float angle in degree in a float angle in radians
*
* \return	The value in radians
*/
float ToRad(float numdeg);

#endif //GPS_UBLOX_H__