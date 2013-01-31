/*
	Ishtar Embedded
	Copyright (C) 2008-2009:

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
*	\file ishtar_def.h
*	\brief Definition file of the Ishtar Embedded server
*
*	\author Alexandre Habersaat, Antoine Beyeler
*	\date 2008-06-12
*
*	This file contains the defintions and enumerations used by the Ishtar protocol
*
*/

#ifndef __ISHTAR_DEF_H__
#define __ISHTAR_DEF_H__




#ifdef ISHTAR_PREFIX
	#include ISHTAR_PREFIX
#endif

#define ISHTAR_PROTOCOL_VERSION 7

#ifdef __cplusplus
extern "C" {
#endif

#define ISHTAR_HEADER1 (0x66)
#define ISHTAR_HEADER2 (0x57)
#define ISHTAR_HEADER3 (0xD9)
#define ISHTAR_HEADER4 (0xF4)

#define ISHTAR_NUMBER_OF_SIZE_BYTES 4
#define ISHTAR_NUMBER_OF_CHECKSUM_BYTES 2

//! Size definition for strings that have a variable size depending of their content
#define ISHTAR_STRING_SIZE 0

//! Enumeration for Answer messages (server to client)
enum IshtarAnswerType
{
	//! Hello answer
	HELLO=0,
	//! OK message
	OK,
	//! Error
	ERROR,
	//! List of services
	SERVICE_LIST,
	//! Values of a given service
	VALUES,
	//! Confirmation of a set
	SET_ACK

};

//! Enumeration for Call messages (client to server)
enum IshtarCallType
{
	//! Hello call for connection
	//HELLO=0,
	//! Service list request
	GET_SERVICE_LIST=1,
	//! Values read request
	GET_VALUES,
	//! Values write request
	SET_VALUES,
	//! Disconnection notification
	BYE_BYE
};

//! Enumeration for node type
enum IshtarNodeType
{
	//! Ishtar server node
	SERVER=0,
	//! Ishtar client node
 	CLIENT
};

//! Enumeration for message type
enum IshtarMessageType
{
	//! Connection was successful
	CONNECTION_SUCCESSFUL = 0
};

//! Enumeration for error types
enum IshtarErrorTypes
{
	//! A request service does not exists any more
	SERVICE_DOES_NOT_EXISTS = 0,
	//! The server and the client are running incompatible protocol version
	INCOMPATIBLE_PROTOCOL_VERSION
};

//! Enumeration for protocol type
enum IshtarProtocolType
{
	REGULAR = 0,	//!< regular protocol should be used
	EMBEDDED,	//!< embedded version of the protocol should be used
	INCOMPATIBLE	//!< this server is not compatible with the request protocol version
};

//! Enumeration for variable types
enum IshtarType
{
	LONG = 0,	//!< 32 bits signed integer, corresponds to signed long
	ULONG,		//!< 32 bits unsigned integer, corresponds to unsigned long
	SHORT,		//!< 16 bits signed integer, corresponds to signed short
	USHORT,		//!< 16 bits unsigned integer, corresponds to unsigned short
	CHAR,		//!< 8 bits signed integer, corresponds to signed char
	UCHAR,		//!< 8 bits unsigned integer, corresponds to unsigned char
	FLOAT,		//!< 32 bits floating point value, corresponds to float
	DOUBLE,		//!< 64 bits double precision floating point value, corresponds to double
	BOOL,		//!< true/false value, serialized on a UCHAR

	//TODO: depracated, remove
	INT=0,		//!< 32 bits signed integer, corresponds to signed long, deprecated alias for LONG
	UINT,		//!< 32 bits unsigned integer, corresponds to unsigned long, deprecated alias for ULONG

};

//! Enumeration for service options (flags)
enum IshtarFlags
{
	//! No flags
	NO_FLAGS=0,
	//! Service can only be read
	READ_ONLY=0x1,
	//! Each value has a name
	NAMED_VALUES=0x2,
	//! Each value is bounded
	CONSTRAINT_VALUES=0x4,
	//! Service can be saved/loaded from memory
	PERSISTENT=0x80
};

//! Enumeration for the memery errors
enum IshtarMemoryErrors
{
	//! No error occured
	NO_ERROR=0,
	//! The write function is not defined
	WRITE_FUNC_NULL,
	//! The read function is not defined
	READ_FUNC_NULL,
	//! The memory space is too small
	NOT_ENOUGH_SPACE,
	//! The variable list has been modified since last save
	CHECKSUM_WRONG,
	//! The external function could not write
	WRITE_ERROR,
	//! The internal function could not read
	READ_ERROR

};


#ifdef __cplusplus
}
#endif

#endif
