#ifndef GPS_H_MAV__
#define GPS_H_MAV__

#define GPS_TYPE_NONE	0
#define GPS_TYPE_UBX	1
#define GPS_TYPE_MTIG	2
#define GPS_TYPE_IG500N	3
#define GPS_TYPE_NMEA	4
#define GPS_TYPE_EXTERNAL 255

#ifndef GPS_TYPE
#define GPS_TYPE GPS_TYPE_UBX
#endif

// Conversion of GPS longitude and latitude degrees to radians
#define GPS_LONLAT_TO_RAD	(1/10000000.*MATH_PI/180.)


#if GPS_TYPE == GPS_TYPE_UBX
#include "gps_ublox.h"
#elif GPS_TYPE == GPS_TYPE_MTIG
#include "mtig.h"
#elif GPS_TYPE == GPS_TYPE_IG500N
#include "ig500n.h"
#elif GPS_TYPE == GPS_TYPE_NMEA
#include "nmea.h"
#endif

// void gps_ReceivedDataByte(unsigned char byte)
// void gps_GetData(gps_Data* data)
#if GPS_TYPE == GPS_TYPE_UBX
#define gps_GetGPSData			ubx_GetGPSData
#define gps_ReceiveDataByte		ubx_ReceiveDataByte
#elif GPS_TYPE == GPS_TYPE_MTIG
#define gps_GetGPSData			mtig_GetGPSData
#define gps_ReceiveDataByte		mtig_ReceiveDataByte
#elif GPS_TYPE == GPS_TYPE_IG500N
#define gps_GetGPSData			ig5_GetGPSData
#define gps_ReceiveDataByte		ig5_ReceiveDataByte
#elif GPS_TYPE == GPS_TYPE_NMEA
#define gps_GetGPSData			nmea_Get
GPSData
#define gps_ReceiveDataByte		nmea_ReceiveDataByte
#elif GPS_TYPE == GPS_TYPE_EXTERNAL
#define gps_GetGPSData			gps_ExternalGetGPSData
#endif


#if GPS_TYPE == GPS_TYPE_EXTERNAL
//! Defines a pointer to a user-specific GPS read function
void gps_SetExternalGPSDataFct(void (*func)(gps_Data*));

//! Call the user-specific GPS read function
void gps_ExternalGetGPSData(gps_Data* data);
#endif


#endif