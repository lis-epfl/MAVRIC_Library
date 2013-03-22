/*
  Copyright (C) 2008. LIS Laboratory, EPFL, Lausanne

  This file is part of Aeropic.

  Aeropic is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  Aeropic is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Aeropic.  If not, see <http://www.gnu.org/licenses/>.
*/ 
/*!
*	\file gps_ubx.h
*	\brief Header file of the GPS UBX binary protocol decoding module
*/

#ifndef __GPS_UBX_H__
#define __GPS_UBX_H__

#include "gps_aeropic.h"


//--------------------
// Public definitions
//--------------------

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


//-------------------------
// Public type definitions
//-------------------------

//! The structure for UBX protocol, Posllh messages
typedef struct
{
	unsigned long itow;
	long lon;
	long lat;
	long height;
	long hmsl;
	unsigned long hAcc;
	unsigned long vAcc;
}
ubx_MessagePosllh;

//! The structure for UBX protocol, Velned messages
typedef struct
{
	unsigned long itow;
	long velN;
	long velE;
	long velD;
	unsigned long speed;
	unsigned long gSpeed;
	long heading;
	unsigned long sAcc;
	unsigned long cAcc;
}
ubx_MessageVelned;

//! The structure for UBX protocol, Status messages
typedef struct
{
	unsigned long itow;
	unsigned char gpsFix;
	unsigned char flags;
	unsigned char diffS;
	unsigned char res;
	unsigned long ttff;
	unsigned long msss;
}
ubx_MessageStatus;


//! The structure for UBX protocol, SVInfo message
typedef struct
{
	unsigned long itow;
	unsigned char numCh;
	unsigned char globalFlags;
	unsigned short reserved;
	
	struct
	{
		unsigned char chn;
		unsigned char svid;
		unsigned char flags;
		unsigned char quality;
		unsigned char cno;
		signed char elev;
		signed short azim;
		signed long prRes;
	} channelData[16];	
}
ubx_MessageSVInfo;

enum state_gps{
	CORRECT=0,
	TIMEOUT,
	INACCURATE,
	OUT_OF_BOUND
};

//----------------------------
// Public function prototypes
//----------------------------

//! This function must be called frequently to detect GPS timeout
unsigned char ubx_CheckTimeout();

//!	This function returns if there is a timeout on the GPS
unsigned char ubx_GetTimeout();


// Implementation of the GPS interface

//! This function must be called each time when a byte is received from communication
void ubx_ReceiveDataByte(unsigned char data);

//! This function fills the data structure with the latest data
void ubx_GetGPSData(gps_Data * data);



// Low-level access to received UBX messages

//! This function returns a pointer to the last NAV-POSLLH message that was received
ubx_MessagePosllh * ubx_GetPosllh();

//! This function returns a pointer to the last NAV-VELNED message that was received
ubx_MessageVelned * ubx_GetVelned();

//! This function returns a pointer to the last NAV-STATUS message that was received
ubx_MessageStatus * ubx_GetStatus();

//! This function returns a pointer to the last NAV-SVINFO message that was received
ubx_MessageSVInfo * ubx_GetSVInfo();


#ifdef AEROPIC_ISHTAR
//! registers variables to Ishtar.
void ubx_RegisterToIshtar();
#endif

#endif

