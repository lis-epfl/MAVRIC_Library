/*
* gps_ublox.h
* 
* created on March 28 2013
*
* Author: N.Dousse
*
*/


#ifndef GPS_UBLOX_H__ND
#define GPS_UBLOX_H__ND

//#include "gps_maveric.h"
#include "stdint.h"
#include "stdbool.h"

#define UBLOX_SET_BINARY "$PUBX,41,1,0003,0001,38400,0*26\n\265\142\006\001\003\000\001\006\001\022\117"

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

// Sizes
#define UBX_SIZE_NAV_POSLLH 28
#define UBX_SIZE_NAV_STATUS 16
#define UBX_SIZE_NAV_SOL 52
#define UBX_SIZE_NAV_VELNED 36
#define UBX_SIZE_NAV_SVINFO 30 //8 + 12*numChannel

#define NAV_STATUS_FIX_NVALID 0
#define NAV_STATUS_FIX_VALID 1

//epoch
#define TIME_OF_DAY 0 //<
#define TIME_OF_WEEK 1 //< Ublox
#define TIME_OF_YEAR 2 //< MTK, NMEA
#define UNIX_EPOCH 3

#define PI 3.141592
#define DEG2RAD PI/180

typedef struct {
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id_header;
	uint16_t length;
}ubx_header;

typedef struct {
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}ubx_cfg_nav_rate;

typedef struct {
	uint8_t msg_class;
	uint8_t msg_id_rate;
	uint8_t rate;
}ubx_cfg_msg_rate;

typedef struct {
	uint16_t mask;
	uint8_t dynModel;
	uint8_t fixMode;
	int32_t fixedAlt;
	uint32_t fixedAltVar;
	int8_t minElev;
	uint8_t drLimit;
	uint16_t pDop;
	uint16_t tDop;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t staticHoldThresh;
	uint8_t res1;
	uint32_t res2;
	uint32_t res3;
	uint32_t res4;
}ubx_cfg_nav_settings;

typedef struct {
	uint32_t itow;                                  // GPS msToW
	int32_t longitude;
	int32_t latitude;
	int32_t altitude_ellipsoid;
	int32_t altitude_msl;
	uint32_t horizontal_accuracy;
	uint32_t vertical_accuracy;
}ubx_nav_posllh;

typedef struct {
	uint32_t itow;                                  // GPS msToW
	uint8_t fix_type;
	uint8_t fix_status;
	uint8_t differential_status;
	uint8_t res;
	uint32_t time_to_first_fix;						// milliseconds
	uint32_t uptime;                                // milliseconds
}ubx_nav_status;

typedef struct {
	uint32_t itow;									// milliseconds
	int32_t time_nsec;								// nanoseconds
	int16_t week;
	uint8_t fix_type;
	uint8_t fix_status;
	int32_t ecef_x;									// cm
	int32_t ecef_y;									// cm
	int32_t ecef_z;									// cm
	uint32_t position_accuracy_3d;					// cm
	int32_t ecef_x_velocity;						// cm/s
	int32_t ecef_y_velocity;						// cm/s
	int32_t ecef_z_velocity;						// cm/s
	uint32_t speed_accuracy;						// cm/s
	uint16_t position_DOP;							// scaling 0.01
	uint8_t res;
	uint8_t satellites;
	uint32_t res2;
}ubx_nav_solution;

typedef struct {
	uint32_t itow;                                  // milliseconds GPS msToW
	int32_t ned_north;								// cm/s
	int32_t ned_east;								// cm/s
	int32_t ned_down;								// cm/s
	uint32_t speed_3d;								// cm/s
	uint32_t groundSpeed_2d;								// cm/s
	int32_t heading_2d;								// deg
	uint32_t speed_accuracy;						// cm/s
	uint32_t heading_accuracy;						// deg
}ubx_nav_velned;

typedef struct
{
	uint32_t itow;
	uint8_t numCh;
	uint8_t globalFlags;
	uint16_t reserved;
	
	struct
	{
		uint8_t chn;
		uint8_t svid;
		uint8_t flags;
		uint8_t quality;
		uint8_t cno;
		int8_t elev;
		int16_t azim;
		int32_t prRes;
	} channelData[16];
}
ubx_nav_SVInfo;

// Receive buffer
//ubx_nav_posllh posllh;
//ubx_nav_status status;
//ubx_nav_solution solution;
//ubx_nav_velned velned;
ubx_cfg_nav_settings nav_settings;

//! The pointer to the pointer to the structure of the current message to fill
unsigned char **ubx_currentMessage = 0;
//! The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
unsigned char ** ubx_lastMessage = 0;
//! The pointer to the number to increment when a message of the type has been received
unsigned short * ubx_validMessage = 0;

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
//! The Posllh message buffer
ubx_nav_posllh ubx_posllhMessage[2];
//! The Status message buffer
ubx_nav_status ubx_statusMessage[2];
//! The Solution message buffer
ubx_nav_solution ubx_solutionMessage[2];
//! The Velned message buffer
ubx_nav_velned ubx_velnedMessage[2];
//! The SVInfo message buffer
ubx_nav_SVInfo ubx_svInfoMessage[2];

// NAV-POSLLH
//! The pointer to the Posllh message that is being filled (not usable)
ubx_nav_posllh * ubx_currentPosllhMessage = &ubx_posllhMessage[0];
//! The pointer to the last Posllh message that was completed
ubx_nav_posllh * ubx_lastPosllhMessage = &ubx_posllhMessage[1];
//! Number of valid Posllh message received
unsigned short ubx_numberOfValidPosllhMessage = 0;

// NAV-STATUS
//! The pointer to the Status message that is being filled (not usable)
ubx_nav_status *ubx_currentStatusMessage = &ubx_statusMessage[0];
//! The pointer to the last Status message that was completed
ubx_nav_status *ubx_lastStatusMessage = &ubx_statusMessage[1];
//! Number of valid Status message received
unsigned short ubx_numberOfValidStatusMessage = 0;

// NAV-Sol
//! The pointer to the Solution message that is being filled (not usable)
ubx_nav_solution *ubx_currentSolutionMessage = &ubx_solutionMessage[0];
//! The pointer to the last Status message that was completed
ubx_nav_solution *ubx_lastSolutionMessage = &ubx_solutionMessage[1];
//! Number of valid Status message received
unsigned short ubx_numberOfValidSolutionMessage = 0;

// NAV-VELNED
//! The pointer to the Velned message that is being filled (not usable)
ubx_nav_velned *ubx_currentVelnedMessage = &ubx_velnedMessage[0];
//! The pointer to the last Velned message that was completed
ubx_nav_velned *ubx_lastVelnedMessage = &ubx_velnedMessage[1];
//! Number of valid Velned message received
unsigned short ubx_numberOfValidVelnedMessage = 0;

// NAV-SVINFO
//! The pointer to the Status message that is being filled (not usable)
ubx_nav_SVInfo *ubx_currentSVInfoMessage = &ubx_svInfoMessage[0];
//! The pointer to the last Status message that was completed
ubx_nav_SVInfo *ubx_lastSVInfoMessage = &ubx_svInfoMessage[1];
//! Number of valid Status message received
unsigned short ubx_numberOfValidSVInfoMessage = 0;






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
bool valid_read;
bool new_data;

uint32_t last_ground_speed_cm;

#define NO_GPS 0
#define NO_FIX 1
#define GPS_OK 2

enum GPS_Engine_Setting{
	GPS_ENGINE_NONE        = -1,
	GPS_ENGINE_PORTABLE    = 0,
	GPS_ENGINE_STATIONARY  = 2,
	GPS_ENGINE_PEDESTRIAN  = 3,
	GPS_ENGINE_AUTOMOTIVE  = 4,
	GPS_ENGINE_SEA         = 5,
	GPS_ENGINE_AIRBORNE_1G = 6,
	GPS_ENGINE_AIRBORNE_2G = 7,
	GPS_ENGINE_AIRBORNE_4G = 8
};

enum GPS_Engine_Setting nav_setting;

//! Number of times ubx_CheckTimeout() must be called without response from GPS before it is considered as timed out
#define UBX_TIMEOUT_CYCLES 2
//! The minimum precision to consider a position as correct (in mm)
#define UBX_POSITION_PRECISION 20000
//! The minimum precision to consider an altitude as correct (in mm)
#define UBX_ALTITUDE_PRECISION 20000
//! The minimum precision to consider a speed as correct (in cm/s)
#define UBX_SPEED_PRECISION 500
//! The minimum precision to consider a heading as correct (in deg*10^5)
#define UBX_HEADING_PRECISION 3000000

// Type definition for GPS data
typedef struct
{
	long latitude; //!< latitude in degree E+7
	long longitude; //!< longitude in degree E+7
	float altitude; //!< altitude in m
	float alt_elips; //!< altitude above ellipsoid in m
	float speed; //!< 3D speed in m/s
	float groundSpeed; //!< 2D ground speed in m/s
	float northSpeed; //!< the speed to the north in m/s
	float eastSpeed; //!< the speed to the east in m/s
	float verticalSpeed; //!< the vertical speed in m/s
	float course; //!< heading in degree
	
	float speedAccuracy;
	float headingAccuracy;
	
	uint8_t num_sats;
	uint16_t hdop;
	
	unsigned long timegps; //!< time reference in ms
	unsigned long itow; //!< time reference in ms ???????
	
	unsigned char status;
	
	
	
	unsigned char latitudeStatus;
	unsigned char longitudeStatus;
	unsigned char altitudeStatus;
	unsigned char speedStatus;
	unsigned char groundSpeedStatus;
	unsigned char northSpeedStatus;
	unsigned char eastSpeedStatus;
	unsigned char verticalSpeedStatus;
	unsigned char courseStatus;
} gps_Data_type;

/// Last time that the GPS driver got a good packet from the GPS
uint32_t idleTimer;

/// Time in milliseconds after which we will assume the GPS is no longer
/// sending us updates and attempt a re-init.
///
/// 1200ms allows a small amount of slack over the worst-case 1Hz update
/// rate.
uint32_t idleTimeout;

uint32_t last_fix_time;

float velocity_north;
float velocity_east;
float velocity_down;

//float get_lag() { return 0.5; };
	
void init_gps_ubx(enum GPS_Engine_Setting _nav_setting);

bool ubx_read(void);
bool ubx_process_data(void);

void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);
void ubx_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
void ubx_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
void configure_gps(void);

bool ubx_detect(uint8_t data);

void gps_update(void);

ubx_nav_posllh * ubx_GetPosllh();
ubx_nav_status * ubx_GetStatus();
ubx_nav_solution * ubx_GetSolution();
ubx_nav_velned * ubx_GetVelned();
ubx_nav_SVInfo * ubx_GetSVInfo();

float ToRad(float numdeg);

#endif //GPS_UBLOX_H__