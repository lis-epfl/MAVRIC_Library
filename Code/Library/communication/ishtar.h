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
*	\file ishtar.h
*	\brief Header file of the Ishtar Embedded server
*
*	\author Alexandre Habersaat, Antoine Beyeler
*
*	This file contains the public interface of the Ishtar Embedded server
*/

#ifndef __ISHTAR_H__
#define __ISHTAR_H__
#define USE_ISHTAR 1

#ifdef ISHTAR_PREFIX
	#include ISHTAR_PREFIX
#endif

// #include <stdio.h>

#include "ishtar_def.h"

//-----------------------------
// Module's options
//-----------------------------

#ifndef ISHTAR_DEBUG
	//! Ishtar debug mode
	#define ISHTAR_DEBUG 1
#endif

#ifndef ISHTAR_MAX_NUMBER_OF_SERVICES
	//! The maximum number of services / variables that can be registered in Ishtar
	#define ISHTAR_MAX_NUMBER_OF_SERVICES 250
#endif

#ifndef ISHTAR_BUFFER_SIZE
	//! The size of the reception buffer
	#define ISHTAR_BUFFER_SIZE 300
#endif

#ifndef ISHTAR_STRING_MAX_SIZE
	//! The maximum size of a string in Ishtar Embedded
	#define ISHTAR_STRING_MAX_SIZE 100
#endif


#ifdef __cplusplus
extern "C" {
#endif

#ifndef ISHTAR_MAX_VEC_SIZE
//! Maximum size for vectors
#define ISHTAR_MAX_VEC_SIZE 65535
#endif

#ifndef ISHTAR_MAX_SERVICES_COUNT
// Maximum number of services
#define ISHTAR_MAX_SERVICES_COUNT 65535
#endif

#if ISHTAR_MAX_VEC_SIZE > 65535
	typedef unsigned long ishtar_ValueVectorSize;
#elif ISHTAR_MAX_VEC_SIZE > 255
	typedef unsigned short ishtar_ValueVectorSize;
#else
	typedef unsigned char ishtar_ValueVectorSize;
#endif

typedef unsigned char ishtar_ValueVectorType;

#if ISHTAR_MAX_SERVICES_COUNT > 65535
	typedef unsigned long ishtar_ServiceId;
#else
	typedef unsigned short ishtar_ServiceId;
#endif

typedef unsigned char ishtar_ServiceFlags;
typedef unsigned short ishtar_MessageId;
typedef unsigned short ishtar_RequestId;

typedef unsigned long ishtar_ProtocolType;
typedef unsigned long ishtar_NodeType;
typedef unsigned long ishtar_VersionType;

// forward declaration
//typedef struct ishtar_ServiceDescription ishtar_ServiceDescriptionPtr;
typedef struct ishtar_ServiceDescriptionTag* ishtar_ServiceDescriptionPtr;

//! Type definition for the external function used to send data through communication
typedef void (*ishtar_ExternalSendFunc)(unsigned char *, unsigned long);
//! Type definition for the external function used to flush the data output buffer at the end of each message that is sent
typedef void (*ishtar_ExternalFlushFunc)(void);
//! Type definition for the external function used to write data in non-volatile memory
typedef unsigned short (*ishtar_MemoryWriteFunc)(unsigned short address, void* data, unsigned short length);
//! Type definition for the external function used to read data from non-volatile memory
typedef unsigned short (*ishtar_MemoryReadFunc)(unsigned short address, void* data, unsigned short length);
//! Type definition for the variable modified callback.
typedef void (*ishtar_VariableModifiedFunc)(ishtar_ServiceDescriptionPtr variable);

//! The structure for the description of services / variables
typedef struct ishtar_ServiceDescriptionTag
{
	const char* name;						//!< The name of the variable
	ishtar_ValueVectorType type;			//!< The type of the variable
	ishtar_ValueVectorSize length;			//!< The length of the array
	ishtar_ServiceFlags flags;				//!< The flags for the variable (READ_ONLY)
	void* value;							//!< The pointer to the local real variable
	unsigned char inSnapshot;				//!< If this variable is part of the snapshot
	ishtar_VariableModifiedFunc feedbackFunc;	//!< Function called when the variable is remotely modified
} ishtar_ServiceDescription;


//-----------------------------
// Public functions
//-----------------------------

//! This function inits the Ishtar Embedded server
void ishtar_Init(ishtar_ExternalSendFunc externalSendFunc, ishtar_ExternalFlushFunc externalFlushFunc);
//! This function inits the Ishtar Embedded server, and configures it to work with two different send functions, a non-blocking one and a blocking one
void ishtar_InitWithBlock(ishtar_ExternalSendFunc externalSendFunc, ishtar_ExternalSendFunc externalSendBlockFunc, ishtar_ExternalFlushFunc externalFlushFunc);
//! This function inits the non-volatile memory access functions
void ishtar_InitMemoryAccess(ishtar_MemoryReadFunc memoryReadFunc, ishtar_MemoryWriteFunc memoryWriteFunc, unsigned short minAddress, unsigned short maxAddress);

//! This function registers a local variable in the Ishtar library to make it accessible to external clients connected to the server
ishtar_ServiceId ishtar_Variable(const char* name, void* var, ishtar_ValueVectorType type, ishtar_ValueVectorSize length, ishtar_ServiceFlags flags);
//! This function registers a local variable in the Ishtar library to make it accessible to external clients connected to the server
ishtar_ServiceId ishtar_VariableFeedback(const char* name, void* var, ishtar_ValueVectorType type, ishtar_ValueVectorSize length, ishtar_ServiceFlags flags, ishtar_VariableModifiedFunc feedbackFunc);
//! This function checks treats every packet that arrived since the last call
void ishtar_Step(void);

//! This function should be called each time data bytes are received through communication
void ishtar_ReceiveData(unsigned char * data, unsigned long len);
//! This function should be called each time a data byte is received through communication
void ishtar_ReceiveDataByte(unsigned char c);
//! This function sends the active snapshot if any, otherwise it does nothing
void ishtar_Snapshot(void);

//! This function reads the values from the memory and assign them to the Ishtar variables
void ishtar_LoadFromMemory(void);
//! This function writes the values of the Ishtar variables to the memory
void ishtar_SaveToMemory(void);

//! This function returns true if the server already received a Hello message from a client
unsigned char ishtar_IsConnected(void);

//! This funtion returns the number of currently registered variable.
unsigned short ishtar_ServiceCount(void);

#ifdef __cplusplus
}
#endif

#endif


