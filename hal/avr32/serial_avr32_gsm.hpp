/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file    serial_avr32.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial peripheral for avr32
 *
 ******************************************************************************/

#ifndef SERIAL_AVR32_GSM_HPP_
#define SERIAL_AVR32_GSM_HPP_

#ifndef NULL
#define NULL 0
#endif

#ifndef __SIZE_T
#define __SIZE_T
typedef unsigned int size_t;
#endif

#include "serial_avr32.hpp"
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "drivers/2GCellular/at_commands.h"
#include <string.h>

////////////////////////
// Memory Allocations //
////////////////////////
#define RX_BUFFER_LENGTH 64 // Number of bytes in the SS receive buffer

////////////////////////////
// Timing Characteristics //
////////////////////////////
// All constants in this section defined in milliseconds
//#define POWER_PULSE_DURATION	3000  // 2-5s pulse required to turn on/off
//#define MODULE_OFF_TIME			10000 // Time the module takes to turn off
#define COMMAND_RESPONSE_TIME	500  // Command response timeout on UART
#define MODULE_WARM_UP_TIME		3000  // Time between on and ready

//////////////////////////
// Response Error Codes //
//////////////////////////
enum cmd_response {
	ERROR_OVERRUN_PREVENT = -4,
	ERROR_UNKNOWN_RESPONSE = -3, // Unknown response
	ERROR_FAIL_RESPONSE = -2, // An identified error response (e.g. "ERROR")
	ERROR_TIMEOUT = -1, // No response received withing specified timeout
	SUCCESS_OK = 1 // Good response.
};

/**
 * \brief   Implementation of serial peripheral for avr32 for GSM
 */
class Serial_avr32_gsm: public Serial_avr32
{
public:

		//Serial_avr32_gsm(serial_avr32_conf_t config);

		/// begin([baud]) - Cell shield initialization at specified baud rate
		///
		/// Attempt to initialize the cellular shield at a specific baud rate.
		/// Autobaud will still be attempted. If the set baud rate is different
		/// from the [baud] parameter, it will be changed.
		/// On a successful exit the cell module's echo behavior will be
		/// turned off and the baud rate will be set to TARGET_BAUD_RATE.
		///
		/// Returns: 0 if communication fails, 1 on success.
		bool begin();


    	/////////////////////////
    	// Information Queries //
    	/////////////////////////

    	/// checkSIM() - Check if a SIM card is present
    	/// Sends the AT*TSIMINS command to the MG2639
    	///
    	/// Returns; true if SIM card is present, false if not
    	bool checkSIM();

    	/// getInformation([infoRet]) - Get manufacturer, hardware, software info
    	///
    	/// Sends the ATI command to the MG2639.
    	/// On successful return, infoRet will contain the information string.
    	/// Return: <0 for fail, >0 for success
    	int8_t getInformation(char * infoRet);

    	/// getIMI([cimiRet]) - Get International Mobile Identification #
    	///
    	/// Sends the AT+CIMI command to the MG2639
    	/// On successful return, [imiRet] will contain the results of query.
    	/// Return: <0 for fail, >0 for success
    	int8_t getIMI(char * imiRet);

    	/// getICCID([iccidRet]) - Get ICCID in SIM card
    	///
    	/// Sends the AT+ZGETICCID command to the MG2639
    	/// On successful return, [iccidRet] will contain results of query.
    	/// Return: <0 for fail, >0 for success
    	int8_t getICCID(char * iccidRet);

    	/// getPhoneNumber([phoneRet]) - Get phone number of SIM card
    	///
    	/// Sends the AT+CNUM command to the MG2639
    	/// On successful return, [phoneRet] will contain the owners phone number
    	/// stored in the SIM card.
    	/// Return: <0 for fail, >0 for success
    	int8_t getPhoneNumber(char * phoneRet);

    	/// getIMEI([imeiRet]) - Get the IMEI of the MG2639 module.
    	///
    	/// Sends the AT+GSN command.
    	/// On successful return, [imeiRet] will contain the 15-digit IMEI of the
    	/// MG2639.
    	/// Return: <0 for fail, >0 for success
    	int8_t getIMEI(char * imeiRet);

    	//////////////////////////////////
		// Command and Response Drivers //
		//////////////////////////////////

		/// sendATCommand([command]) - Send an AT command to the module. This
		/// function takes a command WITHOUT the preceding "AT". It will add
		/// the "AT" in the beginning and '\r' at the end.
		/// Ex: sendATCommand("E0"); // Send ATE0\r to turn echo off
		void sendATCommand(const char * command);

		/// readBetween([begin], [end], [rsp], [timeout]) - Read directly from the
		/// UART. Throw away characters before [begin], then store all characters
		/// between that and [end] into the [rsp] array.
		/// If [begin] is not found within [timeout] ms, the function exits.
		/// Returns: 0 on fail, 1 on success
		int8_t readBetween(char begin, char end, char * rsp, unsigned int timeout);

		/// readWaitForResponse([goodRsp], [timeout]) - Read from UART and store in
		/// rxBuffer until [goodRsp] string is received.
		/// If [goodRsp] is not received within [timeout] ms, the function exits.
		/// Returns: ERROR_TIMEOUT if a timeout occurs, >1 if it finds [goodRsp]
		int readWaitForResponse(const char * goodRsp, unsigned int timeout);

		/// readWaitForResponses([goodRsp], [failRsp], [timout] - Read directly
		/// from UART until EITHER [goodRsp] or [failRSP] are found.
		/// This function allows you to check for either a success string
		/// (e.g. "OK") or a fail string (e.g. "ERROR").
		/// Returns:
		///  - ERROR_TIMEOUT (-1) if a timeout
		///  - ERROR_FAIL_RESPONSE (-2) if [failRsp] was found
		///  - ERROR_UNKNOWN_RESPONSE (-3) if response wasn't [failRsp] or [goodRsp]
		///  - >1 if [goodRsp] was found.
		int readWaitForResponses(const char * goodRsp, const char * failRsp, unsigned int timeout);

		/// readUntil([dest], [end], [maxChars], [tiemout]) - This is a multipurpose
		/// function that reads character from the UART to a character array until
		/// either:
		///  - [timeout] - a timeout it reached. [timeout] value is in ms.
		///  - [maxChars] - A set number of characters have been read.
		///  - [end] - An [end] character is read.
		///
		/// Returns:
		///  - ERROR_TIMEOUT (-1) if a timeout
		///  - ERROR_OVERRUN_PREVENT (-4) if maxChars limit was reached
		///  - >1 if [end] character was read.
		int readUntil(char * dest, char end, int maxChars, unsigned int timeout);

		//////////////////////////////////
		// rxBuffer Searching Functions //
		//////////////////////////////////

		/// getSubstringBetween([dest], [src], [in], [out]) - Searches [src] string
		/// for a string between [in] and [out] characters. On exit [dest] contains
		/// string between (not including) requested characters.
		/// Returns: -1 fail, >1 success
		int getSubstringBetween(char * dest, const char * src, char in, char out);

		////////////////////////////
		// Configuration Commands //
		////////////////////////////

		/// setEcho([on]) -- Sets echo on or off.
		/// To limit the data sent back by the MG2639, the library begin's by
		/// turning echo off, then assumes its that way for the rest of the
		/// program.
		/// Returns: <0 if the module didn't respond. >0 on success.
		int setEcho(uint8_t on);

		//////////////////////
		// UART Abstraction //
		//////////////////////

		/// printString([str]) - Send a string of characters out the UART
		void printString(const char * str, uint32_t length);
		void printString(const char * str);

		/// printChar([c]) - Send a single character out the UART
		void printChar(char c);

		/// dataAvailable() - Returns number of characters available in
		/// UART receive buffer.
		int dataAvailable();

		/// uartRead() - UART read char abstraction
		unsigned char uartRead();

		/// readByteToBuffer() - Read first byte from UART receive buffer
		/// and store it in rxBuffer.
		unsigned int readByteToBuffer();

		/// clearSerial() - Empty UART receive buffer
		void clearSerial();

		//////////////////////
		// rxBuffer Control //
		//////////////////////

		/// clearBuffer() - Reset buffer pointer, set all values to 0
		void clearBuffer();

		/// searchBuffer([test]) - Search buffer for string [test]
		/// Success: Returns pointer to beginning of string
		/// Fail: returns NULL
		//! TODO: Fix this function so it searches circularly
		char * searchBuffer(const char * test);

		// Characters received on the software serial uart are stored in rxBuffer.
		// rxBuffer is a circular buffer with no overwrite protection.
		//! TODO: These are also stored in SoftwareSerial buffer (?). Find a way to
		//! share those buffers so we're not wasting twice as much SRAM.
		unsigned char rxBuffer[RX_BUFFER_LENGTH];
		unsigned int bufferHead; // Holds position of latest byte placed in buffer.

		/**
		 * \brief   Write bytes on the serial line
		 *
		 * \param   byte        Outgoing bytes
		 * \param   size        Number of bytes to write
		 *
		 * \return  true        Data successfully written
		 * \return  false       Data not written
		 */
		virtual bool write(const uint8_t* bytes, const uint32_t size = 1);

};


#endif /* SERIAL_AVR32_HPP_ */

