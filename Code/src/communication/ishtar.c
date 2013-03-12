/*
	Ishtar Embedded
	Copyright (C) 2008-2010:

		Alexandre Habersaat <alexandre dot habersaat at gmail dot com>
		Antoine Beyeler <abeyeler at ab-ware dot com>
			(http://www.ab-ware.com)
		Stephane Magnenat <stephane at magnenat dot net>
			(http://stephane.magnenat.net)
		
	All rights reserved.

	Ishtar Embedded is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Ishtar Embedded is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with Ishtar Embedded. If not, see <http://www.gnu.org/licenses/>.
*/

/*!
*	\file ishtar.c
*	\brief Main file of the Ishtar Embedded server
*
*	\author Alexandre Habersaat, Antoine Beyeler
*
*	This file contains the core of the Ishtar Embedded server
*/


#include "ishtar.h"

#include "print_util.h"

#ifdef USE_ISHTAR		// only compile code if required

#ifdef __cplusplus
extern "C" {
#endif

// Macro for FAR directive
#if defined(__dsPIC33F__) || defined(__PIC24F__)
	#define ISHTAR_ATTRIBUTE_FAR	__attribute__((far))
#else
	#define ISHTAR_ATTRIBUTE_FAR
#endif


//! The reception buffer of the server
unsigned char ishtar_buffer[ISHTAR_BUFFER_SIZE] ISHTAR_ATTRIBUTE_FAR;

// Statistics
//! The number of overflows that occured in the reception buffer
unsigned long ishtar_numberOfOverflows = 0;
//! The number of bytes dropped in the state Header1
unsigned long ishtar_numberOfDropHeader1 = 0;
//! The number of bytes dropped in the state Header2
unsigned long ishtar_numberOfDropHeader2 = 0;
//! The number of bytes dropped in the state Header3
unsigned long ishtar_numberOfDropHeader3 = 0;
//! The number of bytes dropped in the state Header4
unsigned long ishtar_numberOfDropHeader4 = 0;
//! The number of messages dropped after a size checksum error
unsigned long ishtar_numberOfDropSize = 0;
//! The number of messages dropped after a content checksum error
unsigned long ishtar_numberOfDropContent = 0;
//! The total number of bytes received
unsigned long ishtar_numberOfBytesReceived = 0;
//! The total number of bytes sent
unsigned long ishtar_numberOfBytesSent = 0;
//! The total number of bytes read from memory
unsigned long ishtar_numberOfBytesReadFromMemory = 0;
//! The total number of bytes written to memory
unsigned long ishtar_numberOfBytesWrittenToMemory = 0;
//! The number of messages that were valid
unsigned long ishtar_numberOfValidMessages = 0;
//! The number of messages that were valid but contained invalid data (client error or wrong checksum validated)
unsigned long ishtar_corruptedValidated = 0;
//! The snapshot size
unsigned long ishtar_snapshotSize = 0;
//! If an error occured with the non-volatile memory
unsigned char ishtar_memoryError = NO_ERROR;

//! The number of services / variables registered
ishtar_ServiceId ishtar_numberOfVariables = 0;
//! The registry of the services / variables
ishtar_ServiceDescription ishtar_services[ISHTAR_MAX_NUMBER_OF_SERVICES] ISHTAR_ATTRIBUTE_FAR;

//! The Id of the active snapshot (or the last one if none is active)
ishtar_RequestId ishtar_snapshotId = 0;
//! Snapshot active
unsigned char ishtar_snapshotActive = 0;


// read buffer
//! The start of the reception buffer
unsigned char * ishtar_bufBegin;
//! The end of the reception buffer
unsigned char * ishtar_bufEnd;
//! The position to write in the buffer for the next byte that is received
unsigned char * ishtar_bufPos;

//! The position of the next char to be consumed.
unsigned char * ishtar_consumePos;
//! Begining of the data payload of the current packet
unsigned char* ishtar_dataStartMarker;
//! The start of the first packet waiting to be treated in the buffer
unsigned char * ishtar_packetBegin = 0;

//! The state to read the first byte of message header
#define ISHTAR_STATE_HEADER1 1
//! The state to read the second byte of message header
#define ISHTAR_STATE_HEADER2 2
//! The state to read the third byte of message header
#define ISHTAR_STATE_HEADER3 3
//! The state to read the fourth byte of message header
#define ISHTAR_STATE_HEADER4 4
//! The state to read the size of message
#define ISHTAR_STATE_SIZE 5
//! The state to read the checksum of the size of the message
#define ISHTAR_STATE_CHECKSUM_SIZE 6
//! The state to read the content of the message
#define ISHTAR_STATE_CONTENT 7
//! The state to read the checksum of the content of the message
#define ISHTAR_STATE_CHECKSUM_CONTENT 8

//! The current state of the state machine for reception
unsigned char ishtar_state = ISHTAR_STATE_HEADER1;
//! Detects overflows in the reception buffer
unsigned char ishtar_overFlow = 0;
//! The current byte of the size being read
unsigned char ishtar_sizeByteNum = 0;
//! The current bypte of the checksum being read
unsigned char ishtar_checksumByteNum = 0;

// Checksum read
//! The checksum read for the size
unsigned short ishtar_sizeChecksumRead = 0;
//! The ckecksum read for the content
unsigned short ishtar_contentChecksumRead = 0;
// Checksum calculated
//! Temporary variables to calculate checksums
unsigned char ishtar_ckA, ishtar_ckB;
//! The total size of the current packet
unsigned long ishtar_curPacketSize = 0;
//! The number of bytes read for the current packet
unsigned long ishtar_curSize = 0;

//! At least one client is connected to the server
unsigned char ishtar_connected = 0;

// Non-volatile memory access
//! The lowest memory address that Ishtar is allowed to read/write
unsigned short ishtar_memoryMinAddress = 0;
//! The highest memory address that Ishtar is allowed to read/write
unsigned short ishtar_memoryMaxAddress = 0;
//! The external function to read the non-volatile memory
ishtar_MemoryReadFunc ishtar_memoryReadFunc = 0;
//! The external function to write the non-volatile memory
ishtar_MemoryWriteFunc ishtar_memoryWriteFunc = 0;
//! The checksum A for the non-volatile memory consistency
unsigned char ishtar_memoryChecksumA = 0;
//! The checksum B for the non-volatile memory consistency
unsigned char ishtar_memoryChecksumB = 0;

// Send function
//! The external function to call to send data through communication
ishtar_ExternalSendFunc ishtar_externalSendFunc;
//! The external function to call to send the first heavy messages (blocking function)
ishtar_ExternalSendFunc ishtar_externalSendBlockingFunc;
//! The current function that is used to send data (blocking or non-blocking)
ishtar_ExternalSendFunc ishtar_currentSendFunc;
//! The external function to flush the content of the external output buffer (if used)
ishtar_ExternalFlushFunc ishtar_externalFlushFunc;

//! The intermediate function to send data and calculate the checksum of the sent data
void ishtar_SendData(unsigned char * data, unsigned long len);
//! The function to send the message header
void ishtar_SendHeader();
//! The function to reset the checksums
void ishtar_InitCk();
//! The function to send the current checksum of the message
void ishtar_SendCk();

//! An empty function
void ishtar_DoNothing() { }

//! This function inits the pointers to the reception buffer
void ishtar_InitBuffer();
//! This function is used for debug, to register statistics variables into Ishtar
void ishtar_RegisterDebugVars();

// utils
//! This function is used to calculate the checksum of a size
unsigned short ishtar_CalculateSizeChecksum(unsigned long size);
//! This function normalizes an address (usually old address + offset) to stay inside the reception buffer
inline unsigned char * ishtar_WrapAddr(unsigned char * addr);
//! This function increments the address of the reception buffer
inline void ishtar_IncAddr(unsigned char ** addr, unsigned long size);

//! This function reads the content of an address and copies it into a variable (target)
inline void ishtar_At(unsigned char ** addr, unsigned char * target, unsigned char size);
// This function reads the content of an address, copies it into a variable and increment the address to the next position
inline void ishtar_AtInc(unsigned char ** addr, unsigned char * target, unsigned char size);

//! This function sends a zero-terminated string
void ishtar_SendString(const char * sz);
//! This function sends the value of a variable
void ishtar_SendValue(unsigned char * value, ishtar_ValueVectorType type, unsigned long index);
//! This function reads a value in a buffer, write it into the registry and increments the read buffer
void ishtar_SetValueInc(unsigned char * value, ishtar_ValueVectorType type, unsigned long index, unsigned char ** addr);

//! This function returns the length of a zero-terminated string or ISHTAR_STRING_MAX_SIZE
unsigned long ishtar_Strlen(const char * addr);
//! This function returns the size of a type
unsigned long ishtar_TypeSize(ishtar_ValueVectorType type);

// treat messages
//! This function decodes the type of a packet and call the corresponding dedicated function
void ishtar_HandlePacket(unsigned char * addr, unsigned long len);
//! This function decodes the content of a Hello packet and sends the answer
void ishtar_HandleHelloCall(unsigned char * addr, unsigned long len);
//! This function decodes the content of a GetServiceList packet and sends the answer
void ishtar_HandleGetServiceListCall(unsigned char * addr, unsigned long len);
//! This function decodes the content of a Get packet and sends the answer. It may also set up a snapshot.
void ishtar_HandleGetCall(unsigned char * addr, unsigned long len);
//! This function decodes the content of a Set packet and executes the modification on the local service registry
void ishtar_HandleSetCall(unsigned char * addr, unsigned long len);
//! This function decodes the content of a disconnection packet and stops the snapshot
void ishtar_HandleByeByeCall(unsigned char * addr, unsigned long len);

/*!
*	This function normalizes an address (usually old address + offset) to stay inside the reception buffer
*/
inline unsigned char * ishtar_WrapAddr(unsigned char * addr)
{
	if (addr >= ishtar_bufEnd)
	{
		return (unsigned char*)((unsigned int)ishtar_bufBegin + (unsigned int)addr - (unsigned int)ishtar_bufEnd);
	}
	else
	{
		return addr;
	}
}

/*!
*	This function increments the address of the reception buffer
*/
inline void ishtar_IncAddr(unsigned char ** addr, unsigned long size)
{
	*addr += size;
	*addr = ishtar_WrapAddr(*addr);
}

/*!
*	This function reads the content of an address and copies it into a variable (target)
*/
inline void ishtar_At(unsigned char ** addr, unsigned char * target, unsigned char size)
{
	unsigned char i;

	for (i = 0; i < size; ++i)
	{
		target[i] = *ishtar_WrapAddr(*addr + i);
	}
}

/*!
*	This function reads the content of an address, copies it into a variable and increment the address to the next position
*/
inline void ishtar_AtInc(unsigned char ** addr, unsigned char * target, unsigned char size)
{
	ishtar_At(addr, target, size);
	ishtar_IncAddr(addr, size);
}



/*!
*	This function sends a zero-terminated string
*/
void ishtar_SendString(const char * sz)
{
	// 4 name.type
	ishtar_ValueVectorSize nameSize;
	unsigned long j;

	// 4 name.size
	nameSize = ishtar_Strlen(sz);
	ishtar_SendData((unsigned char*)(&nameSize), sizeof(ishtar_ValueVectorSize));
	// X name
	for (j = 0; j < nameSize; ++j)
	{
		ishtar_SendData((unsigned char*)&(sz[j]), 1);
	}
}

/*!
*	This function sends the value of an element of a local variable
*
*	@param value The pointer to the variable to read
*	@param type The type of the variable
*	@param index The index of the element to read
*/
void ishtar_SendValue(unsigned char * value, ishtar_ValueVectorType type, unsigned long index)
{
	unsigned long size;
	unsigned char * val;

	size = ishtar_TypeSize(type);
	val = (unsigned char*) value;
	ishtar_SendData(&val[size * index], size);
}

/*!
*	This function modifies the value of one element of a local variable
*
*	@param value The pointer to the variable to set
*	@param type The type of the variable to set
*	@param index The index of the element to set (or 0 for a normal variable)
*	@param addr The address from which to read the value to set
*/
void ishtar_SetValueInc(unsigned char * value, ishtar_ValueVectorType type, unsigned long index, unsigned char ** addr)
{
	unsigned long i;
	unsigned long size;

	size = ishtar_TypeSize(type);

	for (i = 0; i < size; ++i)
	{
		value[size*index+i] = **addr;
		*addr = ishtar_WrapAddr(*addr + 1);
	}
}

/*!
*	This function calculates the checksum of a 4-bytes size
*
*	@return The size on which the checksum is calculated
*	@return The calculated checksum
*/
unsigned short ishtar_CalculateSizeChecksum(unsigned long size)
{
	unsigned char i;
	unsigned char * p;
	unsigned char ckA, ckB;

	ckA = 0;
	ckB = 0;
	p = (unsigned char*)(&size);
	for (i = 0; i < 4; ++i)
	{
		ckA += *(p + i);
		ckB += ckA;
	}

	return (ckA << 8 | ckB);
}

/*!
*	This function sends any data. The data sent are added to the current checksums
*
*	@param data A pointer to the data to send
*	@param len The number of bytes to send from the pointer
*/
void ishtar_SendData(unsigned char * data, unsigned long len)
{
	// Add checksums
	 long i;
	for (i = 0; i < len; ++i)
	{
		ishtar_ckA += data[i];
		ishtar_ckB += ishtar_ckA;
	}
	// send data
	ishtar_currentSendFunc(data, len);
	/*
	for (i = len; i >= 0; i--)
	{
		ishtar_currentSendFunc(data[i], 1);
	}
	d*/ 
	ishtar_numberOfBytesSent += len;
}

/*!
*	This function sends the Ishtar headers
*/
void ishtar_SendHeader()
{
	// unsigned long header;
	unsigned char h1, h2, h3, h4;
	h1 = ISHTAR_HEADER1;
	h2 = ISHTAR_HEADER2;
	h3 = ISHTAR_HEADER3;
	h4 = ISHTAR_HEADER4;

	//header = (ISHTAR_HEADER1) | (ISHTAR_HEADER2 << 8) | (ISHTAR_HEADER3 << 16) | (ISHTAR_HEADER4 << 24);
	ishtar_currentSendFunc(&h1, 1);
	ishtar_currentSendFunc(&h2, 1);
	ishtar_currentSendFunc(&h3, 1);
	ishtar_currentSendFunc(&h4, 1);
}

/*!
*	This function resets the current checksum
*/
void ishtar_InitCk()
{
	// initialise checksums to 0
	ishtar_ckA = 0;
	ishtar_ckB = 0;
}

/*!
*	This function sends the current checksum
*/
void ishtar_SendCk()
{
	// compile actual checksums
	unsigned short checksum;
	checksum = (ishtar_ckA << 8) | ishtar_ckB;

	// send the compiled checksum
	//ishtar_currentSendFunc((unsigned char*)(&checksum), 2);
	ishtar_currentSendFunc((unsigned char*)(&ishtar_ckB), 1);
	ishtar_currentSendFunc((unsigned char*)(&ishtar_ckA), 1);
}

/*!
*	This function is provided for convenience if multiple bytes are received at once.
*	It will call ishtar_ReceiveDataByte() for each of these data bytes
*/
void ishtar_ReceiveData(unsigned char * data, unsigned long len)
{
	unsigned long i;
	for (i = 0; i < len; ++i)
	{
		ishtar_ReceiveDataByte(data[i]);
	}
}

/*!
*	This function buffers all incoming bytes without processing them. It drop
*	any byte in case of buffer overflow. 
*/
void ishtar_ReceiveDataByte(unsigned char c)
{
	++ishtar_numberOfBytesReceived;
	
	unsigned char* nextAddr = ishtar_WrapAddr(ishtar_bufPos + 1);
	if (nextAddr == ishtar_packetBegin)
	{
		// we have an overflow
		ishtar_overFlow = 1;
		ishtar_numberOfOverflows++;
	}
	else
	{
		*ishtar_bufPos = c;
		ishtar_bufPos = nextAddr;
	}
}


/*!
*	This function process bytes from the buffer with a state machine. When a packet
*	is completed, it calls #ishtar_HandlePacket().
*
*	@param addr The address of the byte to process.
*/
void ishtar_ConsumeBufferedByte(unsigned char* addr)
{
	unsigned char * p;
	unsigned long i;
	unsigned char ckA, ckB;
	
	unsigned char c = *addr;
	
	//dbg_log_value("ishtar state", ishtar_state, 10);
	dbg_print("ir="); dbg_print_num(c, 16);

	if (ishtar_state == ISHTAR_STATE_CONTENT)
	{
		if (ishtar_curSize == 0)
			ishtar_dataStartMarker = addr;
			
		// Increase size
		++ishtar_curSize;

		// end of packet
		if (ishtar_curSize == ishtar_curPacketSize)
		{
			ishtar_contentChecksumRead = 0;
			ishtar_checksumByteNum = 0;
			ishtar_state = ISHTAR_STATE_CHECKSUM_CONTENT;
		}
	}
	else if (ishtar_state == ISHTAR_STATE_SIZE)
	{
		//p = (unsigned char*)(&ishtar_curPacketSize);
		//*( p + ishtar_sizeByteNum++ ) = c;
		// change: Felix Schill, endian fix
		ishtar_curPacketSize|= c<<(8*ishtar_sizeByteNum);
		ishtar_sizeByteNum++;
		
		if (ishtar_sizeByteNum == ISHTAR_NUMBER_OF_SIZE_BYTES)
		{
			ishtar_sizeChecksumRead = 0;
			ishtar_checksumByteNum = 0;
			ishtar_state = ISHTAR_STATE_CHECKSUM_SIZE;
		}
	}
	else if (ishtar_state == ISHTAR_STATE_HEADER1)
	{
		if (c == ISHTAR_HEADER1)
			ishtar_state = ISHTAR_STATE_HEADER2;
		else
		{
			++ishtar_numberOfDropHeader1;
			ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
		}	
	}
	else if (ishtar_state == ISHTAR_STATE_CHECKSUM_SIZE)
	{
//		p = (unsigned char*)(&ishtar_sizeChecksumRead);
//		*(p + ishtar_checksumByteNum++) = c;
		// change: Felix Schill, endian fix
		ishtar_sizeChecksumRead|=c<<(8*ishtar_checksumByteNum);
		ishtar_checksumByteNum++;
		
		if (ishtar_checksumByteNum == ISHTAR_NUMBER_OF_CHECKSUM_BYTES)
		{
			//p = (unsigned char*)(&ishtar_curPacketSize);
			//dbg_log_value("packsize ", ishtar_curPacketSize, 10);
			// calculate checksum of size
			ckA = 0;
			ckB = 0;
			for (i = 0; i < ISHTAR_NUMBER_OF_SIZE_BYTES; ++i)
			{
				ckA += (ishtar_curPacketSize >> (8 * i)) & 0xFF;
				ckB += ckA;
			}
			//dbg_log_value("ckA ", ckA, 10);
			//dbg_log_value("ckB ", ckB, 10);
			if (ishtar_sizeChecksumRead == (ckA << 8 | ckB))
			{
				ishtar_curSize = 0;
				ishtar_state = ISHTAR_STATE_CONTENT;
			}
			else
			{
				++ishtar_numberOfDropSize;
				ishtar_state = ISHTAR_STATE_HEADER1;
				ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
			}
		}
	}
	else if (ishtar_state == ISHTAR_STATE_CHECKSUM_CONTENT)
	{
//		p = (unsigned char*)(&ishtar_contentChecksumRead);
//		*(p + ishtar_checksumByteNum++) = c;
		// change: Felix Schill, endian fix
		ishtar_contentChecksumRead|=c<<(8*ishtar_checksumByteNum);
		ishtar_checksumByteNum++;

		if (ishtar_checksumByteNum == ISHTAR_NUMBER_OF_CHECKSUM_BYTES)
		{
			// calculate checksum of content, start after the size
			ckA = 0;
			ckB = 0;
			for (i = 0; i < ishtar_curPacketSize; ++i)
			{
				ckA += *(ishtar_WrapAddr(ishtar_dataStartMarker + i));
				ckB += ckA;
			}

			// checksum correct
			if (ishtar_contentChecksumRead == (ckA << 8 | ckB))
			{
				dbg_print("ishtar packet received\n");
				// handle the packet
				ishtar_HandlePacket(ishtar_dataStartMarker, ishtar_curPacketSize);				
				++ishtar_numberOfValidMessages;
			}
			else
			{
				dbg_print("ishtar wrong checksum\n");
				++ishtar_numberOfDropContent;
				ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
			}	
			
			// the begining of the buffer is at the next char in all cases.
			ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  

			// go back for next message
			ishtar_state = ISHTAR_STATE_HEADER1;
		}
	}
	else if (ishtar_state == ISHTAR_STATE_HEADER2)
	{
		if (c == ISHTAR_HEADER2)
			ishtar_state = ISHTAR_STATE_HEADER3;
		else if (c == ISHTAR_STATE_HEADER1)
		{
			ishtar_state = ISHTAR_STATE_HEADER2;
			++ishtar_numberOfDropHeader2;
			ishtar_packetBegin = ishtar_WrapAddr(addr);  
		}
		else
		{
			ishtar_state = ISHTAR_STATE_HEADER1;
			++ishtar_numberOfDropHeader2;
			ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
		}
	}
	else if (ishtar_state == ISHTAR_STATE_HEADER3)
	{
		if (c == ISHTAR_HEADER3)
			ishtar_state = ISHTAR_STATE_HEADER4;
		else if (c == ISHTAR_STATE_HEADER1)
		{
			ishtar_state = ISHTAR_STATE_HEADER2;
			++ishtar_numberOfDropHeader3;
			ishtar_packetBegin = ishtar_WrapAddr(addr);  
		}
		else
		{
			ishtar_state = ISHTAR_STATE_HEADER1;
			++ishtar_numberOfDropHeader3;
			ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
		}
	}
	else if (ishtar_state == ISHTAR_STATE_HEADER4)
	{
		if (c == ISHTAR_HEADER4)
		{
			ishtar_state = ISHTAR_STATE_SIZE;
			ishtar_curPacketSize = 0;
			ishtar_sizeByteNum = 0;
		}
		else if (c == ISHTAR_STATE_HEADER1)
		{
			ishtar_state = ISHTAR_STATE_HEADER2;
			++ishtar_numberOfDropHeader4;
			ishtar_packetBegin = ishtar_WrapAddr(addr);  
		}
		else
		{
			ishtar_state = ISHTAR_STATE_HEADER1;
			++ishtar_numberOfDropHeader4;
			ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
		}
	}
	else
	{
		ishtar_state = ISHTAR_STATE_HEADER1;
		ishtar_packetBegin = ishtar_WrapAddr(addr + 1);  
	}	
	dbg_log_value("\tis=", ishtar_state, 10);
}

// step

/*!
*	This function checks if there are some packets to treat in the input buffer
*	If yes, it calls the decoding functions which send the answers to these packets.
*/
void ishtar_Step()
{
	// atomically read the curent bufPos
	unsigned char* bufPos = ishtar_bufPos;
	if (ishtar_consumePos != bufPos) dbg_print("\n");
	while (ishtar_consumePos != bufPos)
	{
		ishtar_ConsumeBufferedByte(ishtar_consumePos);
		ishtar_consumePos = ishtar_WrapAddr(ishtar_consumePos + 1);
		
	}
}

/*!
*	This function reads the start of a packet and determines it type.
*	It then calls the corresponding dedicated function to decode the packet.
*/
void ishtar_HandlePacket(unsigned char * addr, unsigned long len)
{
	dbg_print("ishtar handle packet:");
	ishtar_MessageId type;

	ishtar_AtInc(&addr, (unsigned char*)&type, sizeof(ishtar_MessageId));

	switch (type)
	{
		case HELLO:
		{
			dbg_print("ishtar Hello!\n");
			ishtar_HandleHelloCall(addr, len - sizeof(ishtar_MessageId));
			break;
		}
		case GET_SERVICE_LIST:
		{
			ishtar_HandleGetServiceListCall(addr, len - sizeof(ishtar_MessageId));
			break;
		}
		case GET_VALUES:
		{
			ishtar_HandleGetCall(addr, len - sizeof(ishtar_MessageId));
			break;
		}
		case SET_VALUES:
		{
			ishtar_HandleSetCall(addr, len - sizeof(ishtar_MessageId));
			break;
		}
		case BYE_BYE:
		{
			ishtar_HandleByeByeCall(addr, len - sizeof(ishtar_MessageId));
			break;
		}
		default:
		{
			return;
		}
	}
}

/*!
*	This function decodes the Hello calls and sends the Hello answer
*/
void ishtar_HandleHelloCall(unsigned char * addr, unsigned long len)
{

	// Stop snapshot when hello received
	ishtar_snapshotActive = 0;

	ishtar_NodeType nodeType;
	ishtar_VersionType version;

	ishtar_MessageId hello = HELLO;
	ishtar_ProtocolType protocol;

	unsigned long answerSize;
	unsigned short sizeChecksum;

	// READ
	ishtar_AtInc(&addr, (unsigned char*)&nodeType, sizeof(ishtar_NodeType));
	ishtar_AtInc(&addr, (unsigned char*)&version, sizeof(ishtar_VersionType));

	// CALCULATE
	if (version == ISHTAR_PROTOCOL_VERSION)
		protocol = EMBEDDED;
	else
		protocol = INCOMPATIBLE;

	// SEND
	answerSize = sizeof(ishtar_MessageId) + sizeof(ishtar_ProtocolType);
	sizeChecksum = ishtar_CalculateSizeChecksum(answerSize);

	ishtar_currentSendFunc = ishtar_externalSendBlockingFunc;
	
	
	ishtar_currentSendFunc = ishtar_externalSendFunc;
	
	
	ishtar_SendHeader();
	// send size and size checksum
	ishtar_SendData((unsigned char*)(&answerSize), sizeof(unsigned long));
	ishtar_SendData((unsigned char*)(&sizeChecksum), sizeof(unsigned short));
	// send content
	ishtar_InitCk();
	ishtar_SendData((unsigned char*)(&hello), sizeof(ishtar_MessageId));
	ishtar_SendData((unsigned char*)(&protocol), sizeof(ishtar_ProtocolType));
	ishtar_SendCk();

	ishtar_externalFlushFunc();
	ishtar_currentSendFunc = ishtar_externalSendFunc;

	ishtar_connected = 1;
}

/*!
*	This function decodes the ServiceList calls and sends the list of all registered variables
*/
void ishtar_HandleGetServiceListCall(unsigned char * addr, unsigned long len)
{
	// Nothing to read
	// Calculate size
	unsigned long answerSize; // AnswerType + numberOfVariables
	ishtar_ServiceId i;

	ishtar_MessageId serviceList;
	unsigned short sizeChecksum;

	// READ - nothing

	// SEND
	answerSize = sizeof(ishtar_MessageId) + sizeof(ishtar_ServiceId);
	serviceList = SERVICE_LIST;
	// ID (4) + name.size(4) + name(X) + type(4) + flags(4) + length(4)
	// Must be in a for loop as the name of services have different sizes
	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		answerSize += sizeof(ishtar_ServiceId) + sizeof(ishtar_ValueVectorSize) + ishtar_Strlen(ishtar_services[i].name) + sizeof(ishtar_ValueVectorType) + sizeof(ishtar_ServiceFlags) + sizeof(ishtar_ValueVectorSize);
	}

	// calculate size checksum
	sizeChecksum = ishtar_CalculateSizeChecksum(answerSize);

	ishtar_currentSendFunc = ishtar_externalSendBlockingFunc;
	ishtar_SendHeader();
	// send size and size checksum
	ishtar_SendData((unsigned char*)(&answerSize), sizeof(unsigned long));
	ishtar_SendData((unsigned char*)(&sizeChecksum), sizeof(unsigned short));

	// send content
	ishtar_InitCk();
	ishtar_SendData((unsigned char*)(&serviceList), sizeof(ishtar_MessageId));
	ishtar_SendData((unsigned char*)(&ishtar_numberOfVariables), sizeof(ishtar_ServiceId));

	// fill table
	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		// 4 service id
		ishtar_SendData((unsigned char*)(&i), sizeof(ishtar_ServiceId));
		// name.size(4) + name(X)
		ishtar_SendString(ishtar_services[i].name);
		// 4 service.type
		ishtar_SendData((unsigned char*)(&(ishtar_services[i].type)), sizeof(ishtar_ValueVectorType));
		// 4 service.flags
		ishtar_SendData((unsigned char*)(&(ishtar_services[i].flags)), sizeof(ishtar_ServiceFlags));
		// 4 service.length
		ishtar_SendData((unsigned char*)(&(ishtar_services[i].length)), sizeof(ishtar_ValueVectorSize));
	}

	ishtar_SendCk();

	ishtar_externalFlushFunc();
	ishtar_currentSendFunc = ishtar_externalSendFunc;
}

/*!
*	This function decodes the Get calls and sends the values for the requested data
*/
void ishtar_HandleGetCall(unsigned char * addr, unsigned long len)
{
	// Read
	ishtar_RequestId requestID;
	ishtar_ServiceId numberOfIds;
	ishtar_ValueVectorSize numberOfIndices;


	unsigned long answerSize;// VALUES(4) + requestID(4) + size(4)
	unsigned short sizeChecksum;
	ishtar_ServiceId i;
	ishtar_ValueVectorSize j;

	ishtar_ServiceId id;
	ishtar_ValueVectorSize indice;
	ishtar_ValueVectorSize length;
	unsigned char snapshot = 0;
	ishtar_MessageId values;
	unsigned char * ids;

	// READ
	ishtar_AtInc(&addr,(unsigned char*)&requestID, sizeof(ishtar_RequestId));
	ishtar_AtInc(&addr,(unsigned char*)&snapshot, sizeof(unsigned char));
	ishtar_AtInc(&addr,(unsigned char*)&numberOfIds, sizeof(ishtar_ServiceId));

	if (snapshot)
	{
		// saves the snapshot id
		ishtar_snapshotId = requestID;
		// snapshot canceled
		if (numberOfIds == 0)
		{
			ishtar_snapshotActive = 0;
		}
		// new snapshot asked
		else
		{
			ishtar_snapshotActive = 1;
		}

		// clear old snapshot
		for (i = 0; i < ishtar_numberOfVariables; ++i)
		{
			ishtar_services[i].inSnapshot = 0;
		}
	}

	ids = addr;

	// Size
	answerSize = sizeof(ishtar_MessageId) + sizeof(ishtar_RequestId);

	values = VALUES;

	for (i = 0; i<numberOfIds; ++i)
	{
		// read id of service
		ishtar_AtInc(&addr,(unsigned char*)&id, sizeof(ishtar_ServiceId));

		// Problem, a corrupted message has been validated
		if ( id >= ishtar_numberOfVariables)
		{
			++ishtar_corruptedValidated;
			return;
		}

		// if the variable is in the snapshot
		if (snapshot)
		{
			ishtar_services[id].inSnapshot = 1;
		}

		// Read the number of indices
		ishtar_AtInc(&addr,(unsigned char*)&numberOfIndices, sizeof(ishtar_ValueVectorSize));
		// Eat these indices, so that the next variable id is correct
		for (j = 0; j < numberOfIndices; ++j)
		{
			ishtar_AtInc(&addr,(unsigned char*)&indice, sizeof(ishtar_ValueVectorSize));
			// Problem, a corrupter message has been validated
			if (indice > ishtar_services[id].length)
			{
				++ishtar_corruptedValidated;
				return;
			}
		}

		// if the size is 0, it means variable size, so we must specify it.
		if (ishtar_services[id].length == 0 && ishtar_services[id].type == UCHAR)
		{
			answerSize += sizeof(ishtar_ValueVectorSize);// size of the size
			answerSize += ishtar_Strlen((char*)(ishtar_services[id].value)); // size of the data
		}
		else
		{
			// we only put the data in the next message
			// here the full variable
			if (numberOfIndices == 0)
			{
				answerSize += ((unsigned long)(ishtar_services[id].length)) * ishtar_TypeSize(ishtar_services[id].type);
			}
			// here only some values
			else
			{
				answerSize += ((unsigned long)(numberOfIndices)) * ishtar_TypeSize(ishtar_services[id].type);
			}
		}
	}

	// answer only if non-snapshot, otherwise it will be done in ishtar_Snapshot()
	if (!snapshot)
	{
		sizeChecksum = ishtar_CalculateSizeChecksum(answerSize);

		// SEND
		ishtar_SendHeader();
		// send size and size checksum
		ishtar_SendData((unsigned char*)(&answerSize), 4);
		ishtar_SendData((unsigned char*)(&sizeChecksum), 2);

		// send content
		ishtar_InitCk();
		ishtar_SendData((unsigned char*)(&values), sizeof(ishtar_MessageId));
		ishtar_SendData((unsigned char*)(&requestID), sizeof(ishtar_RequestId));

		for (i = 0; i < numberOfIds; ++i)
		{
			// service id
			ishtar_AtInc(&ids, (unsigned char*)&id, sizeof(ishtar_ServiceId));
			// number of indices
			ishtar_AtInc(&ids,(unsigned char*)&numberOfIndices, sizeof(ishtar_ValueVectorSize));

			// length == 0 mean variable size, ONLY for char or UCHAR, with ended caracter equal to '\0'
			if (ishtar_services[id].type == UCHAR && ishtar_services[id].length == 0)
			{
				length = ishtar_Strlen((char*)(ishtar_services[id].value));
				ishtar_SendData((unsigned char*)(&length), sizeof(ishtar_ValueVectorSize));

				for (j = 0; j < length; ++j)
				{
					ishtar_SendValue(ishtar_services[id].value, ishtar_services[id].type, j);
				}

			}
			else
			{
				// read the indices
				if (numberOfIndices > 0)
				{
					for (j = 0; j < numberOfIndices; ++j)
					{
						ishtar_AtInc(&ids,(unsigned char*)&indice, sizeof(ishtar_ValueVectorSize));
						ishtar_SendValue(ishtar_services[id].value, ishtar_services[id].type, indice);

					}
				}
				else
				{
					for (j = 0; j < ishtar_services[id].length; ++j)
					{

						ishtar_SendValue(ishtar_services[id].value, ishtar_services[id].type, j);
					}
				}
			}
		}

		ishtar_SendCk();
		ishtar_externalFlushFunc();
	}
}

/*!
*	This function decodes the Set calls and updates local variables
*/
void ishtar_HandleSetCall(unsigned char * addr, unsigned long len)
{
	ishtar_RequestId requestID;
	unsigned char feedback;
	ishtar_ServiceId numberOfIds;
	ishtar_ValueVectorSize numberOfIndices;
	ishtar_ServiceId i;
	ishtar_ValueVectorSize j;
	ishtar_ServiceId id;
	ishtar_ValueVectorSize indice;
	unsigned char * indicesAddr;
	
	unsigned long answerSize;
	unsigned short sizeChecksum;
	ishtar_MessageId setAck;


	// READ
	// request id
	ishtar_AtInc(&addr, (unsigned char*)&requestID, sizeof(ishtar_RequestId));
	// feedback
	ishtar_AtInc(&addr, (unsigned char*)&feedback, 1);
	//numberOfIds
	ishtar_AtInc(&addr, (unsigned char*)&numberOfIds, sizeof(ishtar_ServiceId));

	for (i = 0; i < numberOfIds; ++i)
	{
		//id = ishtar_ulongAtInc(&addr);
		ishtar_AtInc(&addr, (unsigned char*)&id, sizeof(ishtar_ServiceId));

		// Problem, a corrupted message has been validated
		if (id > ishtar_numberOfVariables)
		{
			++ishtar_corruptedValidated;
			return;
		}

		// number of indices
		ishtar_AtInc(&addr, (unsigned char*)&numberOfIndices, sizeof(ishtar_ValueVectorSize));
		indicesAddr = addr;

		// readonly, so skip
		if (ishtar_services[id].flags & READ_ONLY)
		{
			if (numberOfIndices == 0)
			{
				// skip
				ishtar_IncAddr(&addr, ishtar_TypeSize(ishtar_services[id].type) * ((unsigned long)(ishtar_services[id].length)));// values
			}
			else
			{
				// skip
				ishtar_IncAddr(&addr, numberOfIndices * sizeof(ishtar_ValueVectorSize));// indices
				ishtar_IncAddr(&addr, ishtar_TypeSize(ishtar_services[id].type) * numberOfIndices);// values
			}
		}
		else
		{
			if (numberOfIndices == 0)
			{
				for (j = 0; j < ishtar_services[id].length; ++j)
					ishtar_SetValueInc(ishtar_services[id].value,ishtar_services[id].type, j, &addr);
			}
			else
			{
				ishtar_IncAddr(&addr, numberOfIndices * sizeof(ishtar_ValueVectorSize));

				// now addr points to the first value, indicesAddr to the first index

				for (j = 0; j < numberOfIndices; ++j)
				{
					ishtar_AtInc(&indicesAddr, (unsigned char*)&indice, sizeof(ishtar_ValueVectorSize));
					// Problem, a corrupted message has been validated
					if (indice > ishtar_services[id].length)
					{
						++ishtar_corruptedValidated;
						return;
					}
					
					ishtar_SetValueInc(ishtar_services[id].value, ishtar_services[id].type, (unsigned long)indice, &addr);
				}
			}
			
			// call feedback function if existing
			if (ishtar_services[id].feedbackFunc)
				ishtar_services[id].feedbackFunc(&ishtar_services[id]);
		}
	}
	// SEND - the ack if feedback enabled
	if (feedback)
	{
		setAck = SET_ACK;
		answerSize = sizeof(ishtar_MessageId) + sizeof(ishtar_RequestId);
		sizeChecksum = ishtar_CalculateSizeChecksum(answerSize);

		ishtar_SendHeader();
		// send size and size checksum
		ishtar_SendData((unsigned char*)(&answerSize), 4);
		ishtar_SendData((unsigned char*)(&sizeChecksum), 2);

		// send content
		ishtar_InitCk();
		ishtar_SendData((unsigned char*)(&setAck), sizeof(ishtar_MessageId));
		ishtar_SendData((unsigned char*)(&requestID), sizeof(ishtar_RequestId));
		ishtar_SendCk();

		ishtar_externalFlushFunc();
	}
}

/*!
*	This function decodes the content of a disconnection packet and stops the snapshot.
*/
void ishtar_HandleByeByeCall(unsigned char * addr, unsigned long len)
{
	ishtar_snapshotActive = 0;
	ishtar_connected = 0;
}

/*!
*	This function first checks if there is an active snapshot.
*	If yes, it sends the data for this snapshot.
*/
void ishtar_Snapshot()
{
	if (ishtar_snapshotActive && ishtar_connected)
	{
		ishtar_RequestId requestID;

		unsigned long answerSize;// VALUES(4) + requestID(4) + size(4)
		unsigned short sizeChecksum;
		ishtar_ServiceId i;
		ishtar_ValueVectorSize j;
		ishtar_ValueVectorSize length;

		ishtar_MessageId values;

		answerSize = sizeof(ishtar_MessageId) + sizeof(ishtar_RequestId);

		values = VALUES;
		requestID = ishtar_snapshotId;

		// CALCULATE SIZE
		for (i = 0; i < ishtar_numberOfVariables; ++i)
		{
			// if the variable is in the snapshot
			if (ishtar_services[i].inSnapshot == 1)
			{
				// if the size is 0, it mean variable, so we must specify it.
				if (ishtar_services[i].length == 0)
				{
					answerSize += sizeof(ishtar_ValueVectorSize);
					answerSize += ishtar_Strlen((char*)(ishtar_services[i].value));
				}
				else
				{
					// in snapshots, the full variable is sent, cannot save indices in embedded version
					answerSize += ((unsigned long)(ishtar_services[i].length)) * ishtar_TypeSize(ishtar_services[i].type);
				}
			}
		}

		ishtar_snapshotSize = answerSize;
		sizeChecksum = ishtar_CalculateSizeChecksum(answerSize);

		// SEND
		ishtar_SendHeader();
		// send size and size checksum
		ishtar_SendData((unsigned char*)(&answerSize), 4);
		ishtar_SendData((unsigned char*)(&sizeChecksum), 2);

		// send content
		ishtar_InitCk();
		ishtar_SendData((unsigned char*)(&values), sizeof(ishtar_MessageId));
		ishtar_SendData((unsigned char*)(&requestID), sizeof(ishtar_RequestId));

		for (i = 0; i < ishtar_numberOfVariables; ++i)
		{
			if (ishtar_services[i].inSnapshot == 1)
			{
				// service id
				// length == 0 mean variable size, ONLY for char or UCHAR, with ended caracter equal to '\0'
				if (ishtar_services[i].length == 0 && ishtar_services[i].type == UCHAR)
				{
					length = ishtar_Strlen((char*)(ishtar_services[i].value));
					ishtar_SendData((unsigned char*)(&length), sizeof(ishtar_ValueVectorSize));

					for (j = 0; j < length; ++j)
					{
						ishtar_SendValue(ishtar_services[i].value, ishtar_services[i].type, j);
					}

				}
				else
				{
					length = ishtar_services[i].length;
					for (j=0; j < length; ++j)
					{
						ishtar_SendValue(ishtar_services[i].value, ishtar_services[i].type, j);
					}
				}
			}
		}

		ishtar_SendCk();

		ishtar_externalFlushFunc();
	}
}


/*!
*	This function reads the values from the memory and assign them to the Ishtar variables.
*
*	It first read the checksum in the 2 first bytes and checks if it is correct, then it reads and assigns the value.
*	The checksum is used to invalidate memory data when the variable list is changed.
*/
void ishtar_LoadFromMemory()
{
	unsigned short i;
	unsigned short j;
	unsigned short size;
	unsigned short nameSize;
	unsigned short addr = ishtar_memoryMinAddress;
	unsigned char readChecksumA, readChecksumB;

	if (!ishtar_memoryReadFunc)
	{
		ishtar_memoryError = READ_FUNC_NULL;
		return;
	}

	// read checksum first
	if (ishtar_memoryReadFunc(addr++, &readChecksumA, 1) != 1)
	{
		ishtar_memoryError = READ_ERROR;
		return;
	}
	ishtar_numberOfBytesReadFromMemory += 1;
	if (ishtar_memoryReadFunc(addr++, &readChecksumB, 1) != 1)
	{
		ishtar_memoryError = READ_ERROR;
		return;
	}
	ishtar_numberOfBytesReadFromMemory += 1;

	// verify checksums
	ishtar_memoryChecksumA = 0;
	ishtar_memoryChecksumB = 0;

	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		if (ishtar_services[i].flags & PERSISTENT)
		{
			// the id, type, length and name are used in the checksum
			ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].type;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].length;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			ishtar_memoryChecksumA += (unsigned char)i;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;

			nameSize = ishtar_Strlen(ishtar_services[i].name);
			for (j = 0; j < nameSize; ++j)
			{
				// checksum the name
				ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].name[j];
				ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			}
		}
	}
	
	// checksum is wrong
	if (!(readChecksumA == ishtar_memoryChecksumA && readChecksumB == ishtar_memoryChecksumB))
	{
		ishtar_memoryError = CHECKSUM_WRONG;
		return;
	}

	// assign the values
	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		if (ishtar_services[i].flags & PERSISTENT)
		{
			// size of this variable in the memory
			size = (unsigned short)(ishtar_TypeSize(ishtar_services[i].type) * ishtar_services[i].length);
			// write value to memory
			if (size != ishtar_memoryReadFunc(addr, ishtar_services[i].value, size))
			{
				ishtar_memoryError = READ_ERROR;
				return;
			}	
			addr += size;
			ishtar_numberOfBytesReadFromMemory += size;
			
			// call feedback function if existing
			if (ishtar_services[i].feedbackFunc)
				ishtar_services[i].feedbackFunc(&ishtar_services[i]);
		}
	}

	ishtar_memoryError = 0;
}

/*!
*	This function writes the values of the Ishtar variables to the memory.
*
*	It first checks that the is enough space in the memory, then it writes the checksum in the first 2 bytes, and then the values.
*/
void ishtar_SaveToMemory()
{
	unsigned short i;
	unsigned short j;
	unsigned short size = 2;
	unsigned short nameSize;
	unsigned short addr = ishtar_memoryMinAddress + 2;

	if (!ishtar_memoryWriteFunc)
	{
		ishtar_memoryError = WRITE_FUNC_NULL;
		return;
	}

	// check size
	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		if (ishtar_services[i].flags & PERSISTENT)
		{
			size += ishtar_TypeSize(ishtar_services[i].type) * ishtar_services[i].length;
		}
	}

	// checks if there is enough memory available
	if (ishtar_memoryMaxAddress - ishtar_memoryMinAddress < size)
	{
		ishtar_memoryError = NOT_ENOUGH_SPACE;
		return;
	}


	// write and calculate checksum
	ishtar_memoryChecksumA = 0;
	ishtar_memoryChecksumB = 0;

	for (i = 0; i < ishtar_numberOfVariables; ++i)
	{
		if (ishtar_services[i].flags & PERSISTENT)
		{
			// the id, type, length and name are used in the checksum
			ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].type;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].length;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			ishtar_memoryChecksumA += (unsigned char)i;
			ishtar_memoryChecksumB += ishtar_memoryChecksumA;

			nameSize = ishtar_Strlen(ishtar_services[i].name);
			for (j = 0; j < nameSize; ++j)
			{
				// checksum the name
				ishtar_memoryChecksumA += (unsigned char)ishtar_services[i].name[j];
				ishtar_memoryChecksumB += ishtar_memoryChecksumA;
			}

			// size of this variable in the memory
			size = (unsigned short)(ishtar_TypeSize(ishtar_services[i].type) * ishtar_services[i].length);
			// write value to memory
			if (size != ishtar_memoryWriteFunc(addr, ishtar_services[i].value, size))
			{
				ishtar_memoryError = WRITE_ERROR;
				return;	
			}	
			addr += size;
			ishtar_numberOfBytesWrittenToMemory += size;
		}
	}

	// end writing checksum
	if (ishtar_memoryWriteFunc(ishtar_memoryMinAddress, &ishtar_memoryChecksumA, 1) != 1)
	{
		ishtar_memoryError = WRITE_ERROR;
		return;		
	}
	ishtar_numberOfBytesWrittenToMemory += 1;
	if (ishtar_memoryWriteFunc(ishtar_memoryMinAddress + 1, &ishtar_memoryChecksumB, 1) != 1)
	{
		ishtar_memoryError = WRITE_ERROR;
		return;		
	}
	ishtar_numberOfBytesWrittenToMemory += 1;

	ishtar_memoryError = NO_ERROR;
}

// init


/*!
*	This function inits Ishtar Embedded server
*
*	@param externalSendFunc The external function to send data
*	@param externalFlushFunc The external function to flush data output buffer
*
**/
void ishtar_Init(ishtar_ExternalSendFunc externalSendFunc, ishtar_ExternalFlushFunc externalFlushFunc)
{
	ishtar_externalSendFunc = externalSendFunc;
	ishtar_externalSendBlockingFunc = externalSendFunc;
	ishtar_currentSendFunc = externalSendFunc;
	//dbg_log_value("ishtar_init send function:", ishtar_currentSendFunc, 16);
	if (externalFlushFunc)
	{
		ishtar_externalFlushFunc = externalFlushFunc;
	}
	else
	{
		ishtar_externalFlushFunc = &ishtar_DoNothing;
	}
	//ishtar_currentSendFunc(42,1);
	ishtar_InitBuffer();

	// DEBUG
#if ISHTAR_DEBUG
	ishtar_RegisterDebugVars();
#endif

}

/*!
*	This function inits Ishtar Embedded server with the use of
*	a different sending function for big messages.
*	The HELLO and SERVICE_LIST packets use the blocking function, other packets the non-blocking
*
*	WARNING: Using this function may cause the host to stuck for a while when the client asks for
*	the list of all services, as the blocking function could take *several seconds* to send the data
*
*	@param externalSendFunc The external function to send data
*	@param externalSendBlockFunc The external blocking function to send data (big packets)
*	@param externalFlushFunc The external function to flush data output buffer
*
**/
void ishtar_InitWithBlock(ishtar_ExternalSendFunc externalSendFunc, ishtar_ExternalSendFunc externalSendBlockFunc, ishtar_ExternalFlushFunc externalFlushFunc)
{
	ishtar_externalSendFunc = externalSendFunc;
	ishtar_externalSendBlockingFunc = externalSendBlockFunc;
	ishtar_currentSendFunc = externalSendFunc;

	if (externalFlushFunc)
	{
		ishtar_externalFlushFunc = externalFlushFunc;
	}
	else
	{
		ishtar_externalFlushFunc = &ishtar_DoNothing;
	}

	ishtar_InitBuffer();

	// DEBUG
#if ISHTAR_DEBUG
	ishtar_RegisterDebugVars();
#endif
}

/*!
*	This function inits the receive buffers and pointers.
*/
void ishtar_InitBuffer()
{
	ishtar_bufBegin = ishtar_buffer;
	ishtar_bufEnd = ishtar_buffer + ISHTAR_BUFFER_SIZE;
	ishtar_bufPos = ishtar_buffer;

	ishtar_consumePos = ishtar_buffer;

	ishtar_packetBegin = ishtar_buffer;
}

/*!
*	This function directly registers several variables to monitor Ishtar itself
*/
void ishtar_RegisterDebugVars()
{
	ishtar_Variable("ishtar.overflows", (unsigned char*)(&ishtar_numberOfOverflows), UINT, 1, 0);
	ishtar_Variable("ishtar.variables", (unsigned char*)(&ishtar_numberOfVariables), USHORT, 1, 0);
	ishtar_Variable("ishtar.dropHeader1", (unsigned char*)(&ishtar_numberOfDropHeader1), UINT, 1, 0);
	ishtar_Variable("ishtar.dropHeader2", (unsigned char*)(&ishtar_numberOfDropHeader2), UINT, 1, 0);
	ishtar_Variable("ishtar.dropHeader3", (unsigned char*)(&ishtar_numberOfDropHeader3), UINT, 1, 0);
	ishtar_Variable("ishtar.dropHeader4", (unsigned char*)(&ishtar_numberOfDropHeader4), UINT, 1, 0);
	ishtar_Variable("ishtar.dropSize", (unsigned char*)(&ishtar_numberOfDropSize), UINT, 1, 0);
	ishtar_Variable("ishtar.dropContent", (unsigned char*)(&ishtar_numberOfDropContent), UINT, 1, 0);
	ishtar_Variable("ishtar.bytesSent", (unsigned char*)(&ishtar_numberOfBytesSent), UINT, 1, 0);
	ishtar_Variable("ishtar.bytesReceived", (unsigned char*)(&ishtar_numberOfBytesReceived), UINT, 1, 0);
	ishtar_Variable("ishtar.validMessages", (unsigned char*)(&ishtar_numberOfValidMessages), UINT, 1, 0);
	ishtar_Variable("ishtar.memoryBytesRead", (unsigned char*)(&ishtar_numberOfBytesReadFromMemory), UINT, 1, 0);
	ishtar_Variable("ishtar.memoryBytesWritten", (unsigned char*)(&ishtar_numberOfBytesWrittenToMemory), UINT, 1, 0);
	ishtar_Variable("ishtar.memoryError", (unsigned char*)(&ishtar_memoryError), UCHAR, 1, 0);
	ishtar_Variable("ishtar.snapshotSize", (unsigned char*)(&ishtar_snapshotSize), UINT, 1, 0);
}

/*!
*	This function inits the non-volatile memory access functions
*
*	@param memoryReadFunc A pointer to the external function that reads the non-volatile memory
*	@param memoryWriteFunc A pointer to the external function that writes the non-volatile memory
*	@param minAddress The lowest memory address that Ishtar is allowed to write
*	@param maxAddress The highest memory address that Ishtar is allowed to write
*/
void ishtar_InitMemoryAccess(ishtar_MemoryReadFunc memoryReadFunc, ishtar_MemoryWriteFunc memoryWriteFunc, unsigned short minAddress, unsigned short maxAddress)
{
	ishtar_memoryReadFunc = memoryReadFunc;
	ishtar_memoryWriteFunc = memoryWriteFunc;

	ishtar_memoryMinAddress = minAddress;
	ishtar_memoryMaxAddress = maxAddress;
}

/*!
*	This function registers a variable into Ishtar in order to make it accessible to external clients
*
*	@param name The textual name of the variable. Use '.' character to create sub-groups, e.g. "gps.position.longitude"
*	@param var A pointer to the real variable or to the root of the array of variables
*	@param type The type of the variable : BOOL, CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE
*	@param length The number of elements in the array, or 1 for a normal variable
*	@param flags The flags for this variable : 0 or READ_ONLY
*/
ishtar_ServiceId ishtar_Variable(const char* name, void* var, ishtar_ValueVectorType type, ishtar_ValueVectorSize length, ishtar_ServiceFlags flags)
{
	return ishtar_VariableFeedback(name, var, type, length, flags, 0);
}

/*!
*	This function registers a variable into Ishtar in order to make it accessible to external clients
*
*	@param name The textual name of the variable. Use '.' character to create sub-groups, e.g. "gps.position.longitude"
*	@param var A pointer to the real variable or to the root of the array of variables
*	@param type The type of the variable : BOOL, CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE
*	@param length The number of elements in the array, or 1 for a normal variable
*	@param flags The flags for this variable : 0 or READ_ONLY
*	@param feedbackFunc Pointer to a function called whenever the corresponding variable is modified.
*/
ishtar_ServiceId ishtar_VariableFeedback(const char* name, void* var, ishtar_ValueVectorType type, ishtar_ValueVectorSize length, ishtar_ServiceFlags flags, ishtar_VariableModifiedFunc feedbackFunc)
{
	if (ishtar_numberOfVariables < ISHTAR_MAX_NUMBER_OF_SERVICES)
	{
		// If the length == 0, then the variable must be read-only because there is no dynamic allocation
		if (length == 0)
		{
			flags |= READ_ONLY;
		}
		ishtar_services[ishtar_numberOfVariables].name = name;
		ishtar_services[ishtar_numberOfVariables].type = type;
		ishtar_services[ishtar_numberOfVariables].length = length;
		ishtar_services[ishtar_numberOfVariables].flags = flags;
		ishtar_services[ishtar_numberOfVariables].value = var;
		ishtar_services[ishtar_numberOfVariables].inSnapshot = 0;
		ishtar_services[ishtar_numberOfVariables].feedbackFunc = feedbackFunc;
		

		return ishtar_numberOfVariables++;
	}
	return 0;
}


/*!
*	This function calculates the length of a zero-terminated string
*
*	@param The zero-terminated string for which the length will be calculated
*	@return The length of the string or ISHTAR_STRING_MAX_SIZE if the string is too long (to prevent from getting stuck)
*/
unsigned long ishtar_Strlen(const char * addr)
{
	unsigned long l = 0;
	while (*addr != '\0' && l < ISHTAR_STRING_MAX_SIZE)
	{
		++l;
		++addr;
	}
	return l;
}

/*!
*	This function returns the number of bytes taken by a variable of a certain type
*
*	@param type The IShtar-defined type of the variable : BOOL, CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE
*/
unsigned long ishtar_TypeSize(ishtar_ValueVectorType type)
{
	unsigned long size = 0;
	if (type == CHAR || type == UCHAR || type == BOOL)
	{
		size = sizeof(char);
	}
	else if (type == SHORT || type == USHORT)
	{
		size = sizeof(short);
	}
	else if (type == FLOAT || type == INT || type == UINT)
	{
		size = sizeof(long);
	}
	else if (type == DOUBLE)
	{
		size = sizeof(double);
	}
	return size;
}

/*!
*	This function returns true if the server already received a HELLO message since it started, false otherwise
*/
unsigned char ishtar_IsConnected()
{
	return ishtar_connected;
}

unsigned short ishtar_ServiceCount()
{
	return ishtar_numberOfVariables;
}


#ifdef __cplusplus
}
#endif

// end of #if USE_ISHTAR
#endif

