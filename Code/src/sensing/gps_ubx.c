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
*	\file gps_ubx.c
*	\brief Source file of the GPS UBX binary protocol decoding module
*
*
*	This file contains all the internal variables and functions of the module.
*	This module is designed to read messages of the UBX binary protocol. The messages
*	that are supported so far are : nav-posllh, nav-velned and nav-status.
*
*/

//----------
// Includes
//----------
#include "gps_ubx.h"

//---------------------
// Private definitions
//---------------------


//! The first header byte of every message sent in UBX protocol
#define UBX_HEADER1 0xb5
//! The second header byte of every message sent in UBX protocol
#define UBX_HEADER2 0x62

// Classes
//! The class value for NAV-class messages
#define UBX_CLASS_NAV 0x01

// Ids
//! The id value for NAV-POSLLH messages
#define UBX_ID_NAV_POSLLH 0x02
//! The id value for NAV-VELNED messages
#define UBX_ID_NAV_VELNED 0x12
//! The id value for NAV-STATUS messages
#define UBX_ID_NAV_STATUS 0x03
//! The id value for NAV-SVINFO
#define UBX_ID_NAV_SVINFO 0x30


// Sizes
//! The size of the NAV-POSLLH messages
#define UBX_SIZE_NAV_POSLLH 28
//! The size of the NAV-VELNED messages
#define UBX_SIZE_NAV_VELNED 36
//! The size of the NAV-STATUS messages
#define UBX_SIZE_NAV_STATUS 16


//! Max number of channel to be used
#define UBX_MAX_CHANNEL 16

//---------------------------
// Private types definitions
//---------------------------

//! Enumeration of states for the receiving state machine
typedef enum
{
	UBX_STATE_HEADER1 = 0,
	UBX_STATE_HEADER2,
	UBX_STATE_CLASS,
	UBX_STATE_ID,
	UBX_STATE_LENGTH,
	UBX_STATE_CONTENT,
	UBX_STATE_CHECKSUM_A,
	UBX_STATE_CHECKSUM_B
}
ubx_State;

#define TRUE 1
#define FALSE 0

//-------------------
// Private variables
//-------------------

//! The current state of the finite-state-machine for the decoding of incoming messages
ubx_State ubx_state = UBX_STATE_HEADER1;
//! The class of the current message
unsigned char ubx_currentMessageClass = 0;
//! The id of the current message
unsigned char ubx_currentMessageId = 0;
//! The size of the current message
unsigned short ubx_currentMessageSize = 0;
//! The current byte of the size being received
unsigned char ubx_currentMessageSizeByte = 0;
//! The checksum A calculated of the current message
unsigned char ubx_currentMessageCkA = 0;
//! The checksum B calculated of the current message
unsigned char ubx_currentMessageCkB = 0;
//! The checksum A read for the current message
unsigned char ubx_currentMessageCkARead = 0;
//! The checksum B read for the current message
unsigned char ubx_currentMessageCkBRead = 0;

//! If a timeout occured on the GPS (GPS not connected or not working)
unsigned char ubx_timeout = FALSE;

//! The current size of the current message (message full if currentMessageCurrentByte == currentMessageSize)
unsigned short ubx_currentMessageCurrentByte = 0;
//! The pointer to the pointer to the structure of the current message to fill
unsigned char ** ubx_currentMessage = 0;
//! The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
unsigned char ** ubx_lastMessage = 0;
//! The pointer to the number to increment when a message of the type has been received
unsigned short * ubx_validMessage = 0;

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
//! The Posllh message buffer
ubx_MessagePosllh ubx_posllhMessage[2];
//! The Velned message buffer
ubx_MessageVelned ubx_velnedMessage[2];
//! The Status message buffer
ubx_MessageStatus ubx_statusMessage[2];
//! The SVInfo message buffer
ubx_MessageSVInfo ubx_svInfoMessage[2];

// NAV-POSLLH
//! The pointer to the Posllh message that is being filled (not usable)
ubx_MessagePosllh * ubx_currentPosllhMessage = &ubx_posllhMessage[0];
//! The pointer to the last Posllh message that was completed
ubx_MessagePosllh * ubx_lastPosllhMessage = &ubx_posllhMessage[1];
//! Number of valid Posllh message received
unsigned short ubx_numberOfValidPosllhMessage = 0;
// NAV-VELNED
//! The pointer to the Velned message that is being filled (not usable)
ubx_MessageVelned * ubx_currentVelnedMessage = &ubx_velnedMessage[0];
//! The pointer to the last Velned message that was completed
ubx_MessageVelned * ubx_lastVelnedMessage = &ubx_velnedMessage[1];
//! Number of valid Velned message received
unsigned short ubx_numberOfValidVelnedMessage = 0;
// NAV-STATUS
//! The pointer to the Status message that is being filled (not usable)
ubx_MessageStatus * ubx_currentStatusMessage = &ubx_statusMessage[0];
//! The pointer to the last Status message that was completed
ubx_MessageStatus * ubx_lastStatusMessage = &ubx_statusMessage[1];
//! Number of valid Status message received
unsigned short ubx_numberOfValidStatusMessage = 0;
// NAV-SVINFO
//! The pointer to the Status message that is being filled (not usable)
ubx_MessageSVInfo * ubx_currentSVInfoMessage = &ubx_svInfoMessage[0];
//! The pointer to the last Status message that was completed
ubx_MessageSVInfo * ubx_lastSVInfoMessage = &ubx_svInfoMessage[1];
//! Number of valid Status message received
unsigned short ubx_numberOfValidSVInfoMessage = 0;


//! Satellite strength buffer
unsigned char ubx_satelliteStrength[UBX_MAX_CHANNEL];
//! Number of satellite used for navigation
unsigned char ubx_satelliteCount;

// Statistics
//! The number of bytes dropped in header1 state
unsigned short ubx_errorHeader1 = 0;
//! The number of bytes dropped in header2 state
unsigned short ubx_errorHeader2 = 0;
//! The number of messages dropped due to checksum error
unsigned short ubx_errorChecksum = 0;
//! The number of messages dropped due to unknown message class (or not taken into account)
unsigned short ubx_errorUnknownClass = 0;
//! The number of messages dropped due to unknown message id or wrong size read
unsigned short ubx_errorUnknownIdSize = 0;
//! The ID of the last message that was dropped due to unknown id or wrong size
unsigned char ubx_errorWrongId = 0;
//! The size of the last message that was dropped due to unknown id or wrong size
unsigned short ubx_errorWrongSize = 0;
//! The number of major errors that occured (should always be 0)
unsigned short ubx_errorMajor = 0;


//---------------------------------
// Public functions implementation
//---------------------------------

/*!
*	This function receives bytes from the communication
*	It runs a state machine which fills decodes the messages and put their content into structures
*
*	@param data The data byte that arrived through communication
*/
void ubx_ReceiveDataByte(unsigned char data)
{
	// the 
	unsigned char * temporaryMessageForSwaping;
	
	// read header 1
	if (ubx_state == UBX_STATE_HEADER1)
	{
		if (data == UBX_HEADER1)
		{
			ubx_state = UBX_STATE_HEADER2;	
		}
		else
		{
			++ubx_errorHeader1;
		}	
	}
	// read header 2
	else if (ubx_state == UBX_STATE_HEADER2)
	{
		if (data == UBX_HEADER2)
		{
			// set up next state
			ubx_currentMessageCkA = 0;
			ubx_currentMessageCkB = 0;
			ubx_state = UBX_STATE_CLASS;	
		}
		else if (data == UBX_HEADER1)
		{
			// stay in this state	
		}
		else
		{
			++ubx_errorHeader2;
			// go back to first state
			ubx_state = UBX_STATE_HEADER1;	
		}	
	}
	// read the class of the message
	else if (ubx_state == UBX_STATE_CLASS)
	{
		// read class
		ubx_currentMessageClass = data;
		// checksum
		ubx_currentMessageCkA += data;
		ubx_currentMessageCkB += ubx_currentMessageCkA;
		// go to next state: id
		ubx_state = UBX_STATE_ID;
	}
	// read the id of the message
	else if (ubx_state == UBX_STATE_ID)
	{
		// read id
		ubx_currentMessageId = data;
		// checksum
		ubx_currentMessageCkA += data;
		ubx_currentMessageCkB += ubx_currentMessageCkA;
		// set up next state: length
		ubx_currentMessageSizeByte = 0;
		ubx_currentMessageSize = 0;
		ubx_state = UBX_STATE_LENGTH;
	}
	// read the size of the message
	else if (ubx_state == UBX_STATE_LENGTH)
	{
		((unsigned char*)(&ubx_currentMessageSize))[ubx_currentMessageSizeByte++] = data;
		// checksum
		ubx_currentMessageCkA += data;
		ubx_currentMessageCkB += ubx_currentMessageCkA;
		
		// size read completely
		if (ubx_currentMessageSizeByte == 2)
		{
			
			// select structure to fill
			// NAV class messages
			if (ubx_currentMessageClass == UBX_CLASS_NAV)
			{
				// NAV-POSLLH
				if (ubx_currentMessageId == UBX_ID_NAV_POSLLH && ubx_currentMessageSize == UBX_SIZE_NAV_POSLLH)
				{
					ubx_currentMessage = (unsigned char**)&ubx_currentPosllhMessage;
					ubx_lastMessage = (unsigned char**)&ubx_lastPosllhMessage;
					ubx_validMessage = &ubx_numberOfValidPosllhMessage;
					
					ubx_state = UBX_STATE_CONTENT;
				}
				// NAV-VELNED
				else if	(ubx_currentMessageId == UBX_ID_NAV_VELNED && ubx_currentMessageSize == UBX_SIZE_NAV_VELNED)
				{
					ubx_currentMessage = (unsigned char**)&ubx_currentVelnedMessage;
					ubx_lastMessage = (unsigned char**)&ubx_lastVelnedMessage;
					ubx_validMessage = &ubx_numberOfValidVelnedMessage;
					
					ubx_state = UBX_STATE_CONTENT;
				}
				// NAV-STATUS
				else if (ubx_currentMessageId == UBX_ID_NAV_STATUS && ubx_currentMessageSize == UBX_SIZE_NAV_STATUS)
				{
					ubx_currentMessage = (unsigned char**)&ubx_currentStatusMessage;
					ubx_lastMessage = (unsigned char**)&ubx_lastStatusMessage;
					ubx_validMessage = &ubx_numberOfValidStatusMessage;
					
					ubx_state = UBX_STATE_CONTENT;
				}
				// NAV-SVINFO
				else if (ubx_currentMessageId == UBX_ID_NAV_SVINFO)
				{
					ubx_currentMessage = (unsigned char**)&ubx_currentSVInfoMessage;
					ubx_lastMessage = (unsigned char**)&ubx_lastSVInfoMessage;
					ubx_validMessage = &ubx_numberOfValidSVInfoMessage;
					
					ubx_state = UBX_STATE_CONTENT;
				}
				// not taken into account, so wait for next message
				else
				{
					++ubx_errorUnknownIdSize;
					ubx_errorWrongId = ubx_currentMessageId;
					ubx_errorWrongSize = ubx_currentMessageSize;
					
					ubx_state = UBX_STATE_HEADER1;
				}		
			}	
			// not taken into account, so wait for next message
			else
			{
				++ubx_errorUnknownClass;
				ubx_state = UBX_STATE_HEADER1;
			}		
			
			// reset byte count anyway
			ubx_currentMessageCurrentByte = 0;
		}	
	}	
	// read the content of the message
	else if (ubx_state == UBX_STATE_CONTENT)
	{
		// read byte and put it in the structure
		(*ubx_currentMessage)[ubx_currentMessageCurrentByte++] = data;
	
		// checksum
		ubx_currentMessageCkA += data;
		ubx_currentMessageCkB += ubx_currentMessageCkA;
		
		// message is full, go to next state
		if (ubx_currentMessageCurrentByte == ubx_currentMessageSize)
		{
			ubx_state = UBX_STATE_CHECKSUM_A;
		}
	}
	// read the checksum byte A
	else if (ubx_state == UBX_STATE_CHECKSUM_A)	
	{
		// read checksum A
		ubx_currentMessageCkARead = data;
		// go to last state: checksum B
		ubx_state = UBX_STATE_CHECKSUM_B;
	}	
	// read the checksum byte B
	else if (ubx_state == UBX_STATE_CHECKSUM_B)
	{
		// read checksum B
		ubx_currentMessageCkBRead = data;
		
		// checksum is correct ?
		if (ubx_currentMessageCkA == ubx_currentMessageCkARead && ubx_currentMessageCkB == ubx_currentMessageCkBRead)
		{
			// this type of message has been received successfully
			++(*ubx_validMessage);
			// message received, set reset timeout
			ubx_timeout = 0;
			
			// swap message buffers, old message is discarded and becomes incoming buffer, new message become valid message (=old)
			temporaryMessageForSwaping = *ubx_currentMessage;
			*ubx_currentMessage = *ubx_lastMessage;
			*ubx_lastMessage = temporaryMessageForSwaping;
			
		}	
		// checksum error
		else
		{
			++ubx_errorChecksum;
			// keep same buffers as data is corrupted
		}	
		// go back to state header1 for next message
		ubx_state = UBX_STATE_HEADER1;
	}	
	// major error, should never happen, the state is unknown !
	else
	{
		++ubx_errorMajor;
		ubx_state = UBX_STATE_HEADER1;
	}			
}	


/*!
*	This function calculates the number of time that it has been called since the last GPS message was received.
*	If this number goes higher than #UBX_TIMEOUT_CYCLES, then there is a timeout.
*
*	@return 1 if there is a timeout, 0 otherwise.
*/
unsigned char ubx_CheckTimeout()
{
	if (ubx_timeout < UBX_TIMEOUT_CYCLES)
	{
		++ubx_timeout;
	}	
	
	return (ubx_timeout == UBX_TIMEOUT_CYCLES);
}

/*!
*	This function returns if there is a timeout on the GPS.
*	It is passive and does not modifies the timeout value.
*/
unsigned char ubx_GetTimeout()
{
	return (ubx_timeout == UBX_TIMEOUT_CYCLES);
}	


/*!
*	This function gets data using the UBX binary protocol
*	The reference indicated in the data structure returned (itow) is the smallest
*	reference of all messages
*/
void ubx_GetGPSData(gps_Data * data)
{
	ubx_MessagePosllh * gpsPosllh;
	ubx_MessageVelned * gpsVelned;
	ubx_MessageStatus * gpsStatus;
	
	// The reference of each messages
	unsigned long gpsPosllhReference = 0;
	unsigned long gpsVelnedReference = 0;
	unsigned long gpsStatusReference = 0;
	
	unsigned char gpsFix = 0;
 	unsigned long hAcc = 0;
	unsigned long vAcc = 0;
	unsigned long sAcc = 0;
	unsigned long cAcc = 0;

	unsigned char timeout = ubx_GetTimeout();
	
	// Get Posllh message
	gpsPosllh = ubx_GetPosllh();
	if (gpsPosllh)
	{
		gpsPosllhReference = gpsPosllh->itow;
		data->longitude = gpsPosllh->lon;
		data->latitude = gpsPosllh->lat;	
		data->altitude = ((float)gpsPosllh->height) / 1000.;
		hAcc = gpsPosllh->hAcc;
		vAcc = gpsPosllh->vAcc;
	}	

	// Get Velned message	
	gpsVelned = ubx_GetVelned();
	if (gpsVelned)
	{
		gpsVelnedReference = gpsVelned->itow;
		data->speed = ((float)gpsVelned->speed) / 100.;
		data->groundSpeed = ((float)gpsVelned->gSpeed) / 100.;
		data->course = ((float)gpsVelned->heading) / 100000.;
		data->northSpeed = ((float)gpsVelned->velN) / 100.;
		data->eastSpeed = ((float)gpsVelned->velE) / 100.;
		data->verticalSpeed = -((float)gpsVelned->velD) / 100.;
		sAcc = gpsVelned->sAcc;
		cAcc = gpsVelned->cAcc;
	}	
	
	// Get Status message
	gpsStatus = ubx_GetStatus();
	if (gpsStatus)
	{
		gpsStatusReference = gpsStatus->itow;
		gpsFix = gpsStatus->gpsFix; // 2 is ok for 2D, 3 or more means fix is good for 3D
	}	
	// take smallest reference
	data->itow = (gpsPosllhReference <= gpsVelnedReference) ? gpsPosllhReference : gpsVelnedReference;
	data->itow = (data->itow <= gpsStatusReference) ? data->itow : gpsStatusReference;
	
	// update status
	// position and altitude
	if (!gpsPosllh || timeout)
	{
		data->latitudeStatus = TIMEOUT;
		data->longitudeStatus = TIMEOUT;
		data->altitudeStatus = TIMEOUT;
	}
	else
	{
		if (gpsFix >= 2 && hAcc < UBX_POSITION_PRECISION)
		{
			data->latitudeStatus = CORRECT;
			data->longitudeStatus = CORRECT;
		}
		else
		{
			data->latitudeStatus = INACCURATE;
			data->longitudeStatus = INACCURATE;
		}
		
		if (gpsFix >= 3 && vAcc < UBX_ALTITUDE_PRECISION)
		{
			data->altitudeStatus = CORRECT;
		}
		else
		{
			data->altitudeStatus = INACCURATE;
		}		
	}	
	// speed and heading
	if (!gpsVelned || timeout)
	{
		data->speedStatus = TIMEOUT;
		data->groundSpeedStatus = TIMEOUT;
		data->northSpeedStatus = TIMEOUT;
		data->eastSpeedStatus = TIMEOUT;
		data->verticalSpeedStatus = TIMEOUT;
		data->courseStatus = TIMEOUT;
	}
	else 
	{
		if (sAcc < UBX_SPEED_PRECISION)
		{
			data->speedStatus = CORRECT;
			data->groundSpeedStatus = CORRECT;
			data->northSpeedStatus = CORRECT;
			data->eastSpeedStatus = CORRECT;
			data->verticalSpeedStatus = CORRECT;
		}
		else
		{
			data->speedStatus = INACCURATE;
			data->groundSpeedStatus = INACCURATE;
			data->northSpeedStatus = INACCURATE;
			data->eastSpeedStatus = INACCURATE;
			data->verticalSpeedStatus = INACCURATE;
		}
		if (cAcc < UBX_HEADING_PRECISION)
		{
			data->courseStatus = CORRECT;
		}
		else
		{
			data->courseStatus = INACCURATE;
		}		
	}
	
	// FIXME: we write the satelite strength to the buffer at this time instead of in the interrupt
	ubx_MessageSVInfo *svInfo = ubx_GetSVInfo();
	if (svInfo)
	{
		int i;
		ubx_satelliteCount = 0;
		for (i = 0; i < UBX_MAX_CHANNEL; i++)
		{
			ubx_satelliteStrength[i] = svInfo->channelData[i].cno;
			
			if (svInfo->channelData[i].flags & 0x1)
				ubx_satelliteCount++;
		}	
	}	 
}

/*! 
*	This function returns a pointer to the last NAV-POSLLH message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid posllh message, or 0.
*/
ubx_MessagePosllh * ubx_GetPosllh()
{
	if (ubx_numberOfValidPosllhMessage)
		return ubx_lastPosllhMessage;
	else
		return 0;
}
	
/*! 
*	This function returns a pointer to the last NAV-VELNED message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid velned message, or 0.
*/
ubx_MessageVelned * ubx_GetVelned()
{
	if (ubx_numberOfValidVelnedMessage)
		return ubx_lastVelnedMessage;
	else
		return 0;
}

/*!
*	This function returns a pointer to the last NAV-STATUS message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid status message, or 0.
*/
ubx_MessageStatus * ubx_GetStatus()
{
	if (ubx_numberOfValidStatusMessage)
		return ubx_lastStatusMessage;
	else
		return 0;	
}

/*!
*	This function returns a pointer to the last NAV-SVINFO message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid status message, or 0.
*/
ubx_MessageSVInfo * ubx_GetSVInfo()
{
	if (ubx_numberOfValidSVInfoMessage)
		return ubx_lastSVInfoMessage;
	else
		return 0;	
}



#ifdef AEROPIC_ISHTAR
/*!
*	Registers variables to Ishtar.
*/
void ubx_RegisterToIshtar()
{
	ishtar_Variable("ubx.satCount", &ubx_satelliteCount, UCHAR, 1, READ_ONLY);
	ishtar_Variable("ubx.satStrength", ubx_satelliteStrength, UCHAR, UBX_MAX_CHANNEL, READ_ONLY);
	
	ishtar_Variable("ubx.validPosllhMsg", &ubx_numberOfValidPosllhMessage, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.validVelnedMsg", &ubx_numberOfValidVelnedMessage, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.validStatusMsg", &ubx_numberOfValidStatusMessage, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.validSVInfoMsg", &ubx_numberOfValidSVInfoMessage, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorHeader1", &ubx_errorHeader1, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorHeader2", &ubx_errorHeader2, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorChecksum", &ubx_errorChecksum, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorUnknownClass", &ubx_errorUnknownClass, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorUnknownIdSize", &ubx_errorUnknownIdSize, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorWrongId", &ubx_errorWrongId, UCHAR, 1, READ_ONLY);
	ishtar_Variable("ubx.errorWrongSize", &ubx_errorWrongSize, USHORT, 1, READ_ONLY);
	ishtar_Variable("ubx.errorMajor", &ubx_errorMajor, USHORT, 1, READ_ONLY);
}
#endif
	
