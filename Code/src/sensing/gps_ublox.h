
#ifndef GPS_UBLOX_H__
#define GPS_UBLOX_H__

#include "gps_maveric.h"

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
#define UBX_SIZE_NAV_VELNED 36
#define UBX_SIZE_NAV_STATUS 16
#define UBS_SIZE_NAV_SOL 52

#define NAV_STATUS_FIX_VALID 1

#define TIME_OF_DAY 0 //<
#define TIME_OF_WEEK 1 //< Ublox
#define TIME_OF_YEAR 2 //< MTK, NMEA
#define UNIX_EPOCH 3

typedef struct {
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
}ubx_header;

typedef struct {
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}ubx_cfg_nav_rate;

typedef struct {
	uint8_t msg_class;
	uint8_t msg_id;
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
	uint32_t time;                                  // GPS msToW
	int32_t longitude;
	int32_t latitude;
	int32_t altitude_ellipsoid;
	int32_t altitude_msl;
	uint32_t horizontal_accuracy;
	uint32_t vertical_accuracy;
}ubx_nav_posllh;

typedef struct {
	uint32_t time;                                  // GPS msToW
	uint8_t fix_type;
	uint8_t fix_status;
	uint8_t differential_status;
	uint8_t res;
	uint32_t time_to_first_fix;
	uint32_t uptime;                                // milliseconds
}ubx_nav_status;

typedef struct {
	uint32_t time;
	int32_t time_nsec;
	int16_t week;
	uint8_t fix_type;
	uint8_t fix_status;
	int32_t ecef_x;
	int32_t ecef_y;
	int32_t ecef_z;
	uint32_t position_accuracy_3d;
	int32_t ecef_x_velocity;
	int32_t ecef_y_velocity;
	int32_t ecef_z_velocity;
	uint32_t speed_accuracy;
	uint16_t position_DOP;
	uint8_t res;
	uint8_t satellites;
	uint32_t res2;
}ubx_nav_solution;

typedef struct {
	uint32_t time;                                  // GPS msToW
	int32_t ned_north;
	int32_t ned_east;
	int32_t ned_down;
	uint32_t speed_3d;
	uint32_t speed_2d;
	int32_t heading_2d;
	uint32_t speed_accuracy;
	uint32_t heading_accuracy;
}ubx_nav_velned;

// Receive buffer
union {
	ubx_nav_posllh posllh;
	ubx_nav_status status;
	ubx_nav_solution solution;
	ubx_nav_velned velned;
	ubx_cfg_nav_settings nav_settings;
	uint8_t bytes[];
} buffergps;

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
bool            new_position;
bool            new_speed;

uint8_t         disable_counter;

bool next_fix;

float get_lag() { return 0.5; };

void init_gps_ubx(GPS_Engine_Setting _nav_setting);

bool ubx_read(void);
bool ubx_process_data(void);

void update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
void ubx_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
void ubx_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
void configure_gps(void);

#endif //GPS_UBLOX_H__