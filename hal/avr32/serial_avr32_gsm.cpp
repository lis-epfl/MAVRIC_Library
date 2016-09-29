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
 * \file    serial_avr32.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial peripheral for avr32
 *
 ******************************************************************************/


#include "serial_avr32_gsm.hpp"

//Serial_avr32_gsm::Serial_avr32_gsm(serial_avr32_conf_t config)
//{
//    config_         = config;
//    irq_callback    = NULL;
//}

///////////////////////////////
// Specific Command Routines //
///////////////////////////////

// Turn Echo ON or OFF
int Serial_avr32_gsm::setEcho(uint8_t on)
{
	int iRetVal;

	if (on) // Send the Echo on command
		sendATCommand(ENABLE_ECHO);
	else	// Or send the echo off command
		sendATCommand(DISABLE_ECHO);

	// Max response for both echo on and off is 11.
	// (Depending on whether echo is already on or off)
	iRetVal = readWaitForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIME);

	return iRetVal;
}


/////////////////////////
// Information Queries //
/////////////////////////

int8_t Serial_avr32_gsm::getInformation(char * infoRet)
{
	int8_t iRetVal;

	sendATCommand(GET_INFORMATION); // Send "ATI"

	// Look for an "OK", which will come at the end of the query response.
	iRetVal = readWaitForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIME);

	// iRetVal will be > 0 if "OK" was received, we can store info into infoRet
	if (iRetVal > 0)
	{
		// Copy contents of rxBuffer from rxBuffer[3] to rxBuffer[end - 8].
		// First 3 bytes are "\r\n" last 8 are "\r\n\r\nOK\r\n"
		strncpy(infoRet, (const char *) rxBuffer+3, strlen((const char*)rxBuffer) - 8);
	}

	return iRetVal;
}

bool Serial_avr32_gsm::begin()
{
	int echoResponse;

	//printChar('\r'); // Print a '\r' to send any possible garbage command
	time_keeper_delay_ms(10); // Wait for a possible "ERROR" response

	// Try turning echo off to see if the module responds with "OK"
	echoResponse = setEcho(0);
	// If setEcho is successful, we found the baud rate! Break this loop
	if (echoResponse > 0)
		return 1;
	else
		return 0; // Otherwise we failed to find it, return 0
}


bool Serial_avr32_gsm::checkSIM()
{
	//char cmd[3]= "AT";
	//Serial_avr32_gsm::write((uint8_t*) cmd,3);
	//int8_t iRetVal;
	print_util_dbg_print("in func\n");
	sendATCommand(CHECK_SIM); // Send "AT*TSIMINS"
	print_util_dbg_print("Command sent\n");
//	iRetVal = readWaitForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIME);
//	print_util_dbg_print("Success");
//	// Example response: *TSIMINS:0, 1/r/n/r/nOK/r/n
//	// First value has no meaning. Second will be 1 if SIM is
//	// present and 0 if there is no SIM.
//	if (iRetVal > 0)
//	{
//		char * ptr;
//		// Look for the comma in the response, closest unique
//		// character up to that point.
//		ptr = strchr((const char *)rxBuffer, ',');
//		ptr += 2; // Move two spots (space, then our character of interest)
//		if (ptr[0] == '1')
//			return true;
//	}
//	print_util_dbg_print("Return\n");
//	return false;
	return true;
}

int8_t Serial_avr32_gsm::getPhoneNumber(char * phoneRet)
{
	int8_t iRetVal;

	sendATCommand(OWNERS_NUMBER); // Send "AT+CNUM"

	// Response will look like "+CNUM: "1234567890",129,7,4\r\nOK\r\n"
	// Read if/until we fund "+CNUM", then we'll do some string work to get the
	// phone number.
	iRetVal = readWaitForResponse("+CNUM:", COMMAND_RESPONSE_TIME);

	if (iRetVal > 0)
	{
		// Read between the quotes ("), that's where our phone will be
		//! TODO: What if the phone number isn't in the string.
		//! Should return some value based on the return of readBetween
		readBetween('\"', '\"', phoneRet, COMMAND_RESPONSE_TIME);
	}

	return iRetVal;
}

int8_t Serial_avr32_gsm::getICCID(char * iccidRet)
{
	int iRetVal;

	sendATCommand(GET_ICCID); // Send "AT+ZGETICCID"

	// Response will be e.g.: "+ZGETICCID: 89860042190733578148\r\nOK\r\n" or
	// "ERROR" if a SIM card is not inserted
	iRetVal = readWaitForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIME);

	if (iRetVal > 0)
	{
		// Get the substring between the first space and the first \r
		// store it in [iccidRet].
		getSubstringBetween(iccidRet, (const char *) rxBuffer, ' ', '\r');
	}

	return iRetVal;
}

int8_t Serial_avr32_gsm::getIMI(char * imiRet)
{
	int iRetVal;

	sendATCommand(READ_IMI); // Send "AT+CIMI"

	// Successful response e.g.: AT+CIMI\r\n460030916875923\r\nOK\r\n
	// Fail response e.g.: ERROR
	iRetVal = readWaitForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIME);

	if (iRetVal > 0)
	{
		// Get the substring between the first \n and second \r:
		getSubstringBetween(imiRet, (const char *) rxBuffer, '\n', '\r');
	}

	return iRetVal;
}

int8_t Serial_avr32_gsm::getIMEI(char * imeiRet)
{
	int iRetVal;

	sendATCommand(GET_IMEI); // Send "AT+CIMI"

	// Successful response e.g.: "AT+GSN\r\n8640490246nnnnn\r\n\r\nOK\r\n
	// Fail response e.g.: ERROR
	iRetVal = readWaitForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIME);

	if (iRetVal > 0)
	{
		// Get the substring between the first \n and second \r:
		getSubstringBetween(imeiRet, (const char *) rxBuffer, '\n', '\r');
	}

	return iRetVal;

}


/////////////////////
// Command Drivers //
/////////////////////

void Serial_avr32_gsm::sendATCommand(const char * command)
{
	//print_util_dbg_print("going to clear\n");
	clearSerial();	// Empty the UART receive buffer
	// Send the command:
	printString("\r");
	printString("AT"); // Print "AT"
	printString(command); // Print the command
	printString("\r"); // Print a carriage return to end command
}

int8_t Serial_avr32_gsm::readBetween(char begin, char end, char * rsp,
                                 unsigned int timeout)
{
	unsigned long timeIn = time_keeper_get_ms(); // timeIn stores timestamp where we enter
	bool inString = false; // Flag to keep track of whether we've seen [begin]
	char c = 0; // Char to store currently read character
	int index = 0;

	//clearBuffer();
	while (timeIn + timeout > time_keeper_get_ms()) // While we haven't timed out
	{
		if (inString == false) // Waiting for beginning character
		{
			if (dataAvailable()) // If data is available on UART RX
			{
				c = uartRead();  // Read from UART RX
				if (c == begin)  // If c is the [begin] character
					inString = true; // Set inString to true
			}
		}
		else // in the string
		{
			if (dataAvailable()) // If data is available on UART RX
			{
				c = uartRead();  // Read data in from UART RX
				if (c == end)    // If c is the [end] char, we're done
				{
					rsp[index] = 0; // Terminate the string
					return 1;      // Return success
				}
				else
				{	// Else we add to the response string
					rsp[index++] = c;
				}
			}
		}
	}

	// Return fail if we timed out
	//! TODO: Could be a more verbose response. Did we see [begin]? Timeout?
	return 0;
}

int Serial_avr32_gsm::readUntil(char * dest, char end, int maxChars,
                            unsigned int timeout)
{
	unsigned long timeIn = time_keeper_get_ms();
	char c = 0;
	int index = 0;

	while ((timeIn + timeout > time_keeper_get_ms()) && (index < maxChars))
	{
		if (dataAvailable())
		{
			c = uartRead();
			if (c == end)
				return index;
			else
				dest[index++] = c;
		}
	}

	if (index >= maxChars)
		return ERROR_OVERRUN_PREVENT;

	return ERROR_TIMEOUT;
}

int Serial_avr32_gsm::readWaitForResponse(const char *goodRsp, unsigned int timeout)
{
	unsigned long timeIn = time_keeper_get_ms();	// Timestamp coming into function
	unsigned int received = 0; // received keeps track of number of chars read

	clearBuffer();	// Clear the class receive buffer (rxBuffer)
	while (timeIn + timeout > time_keeper_get_ms()) // While we haven't timed out
	{
		if (dataAvailable()) // If data is available on UART RX
		{
			print_util_dbg_print("Something Available: I am readWaitForResponse \n ");
			received += readByteToBuffer();

			if (searchBuffer(goodRsp))	// Search the buffer for goodRsp
				return received;	// Return how number of chars read
		}
	}

	print_util_dbg_print("No data received \n ");
	if (received > 0) // If we received any characters
		return ERROR_UNKNOWN_RESPONSE; // Return unkown response error code
	else // If we haven't received any characters
		return ERROR_TIMEOUT; // Return the timeout error code
}

int Serial_avr32_gsm::readWaitForResponses(const char * goodRsp,
                                      const char * failRsp, unsigned int timeout)
{
	unsigned long timeIn = time_keeper_get_ms(); // Timestamp coming into function
	unsigned int received = 0; // received keeps track of number of chars read

	clearBuffer(); // Clear the class receive buffer (rxBuffer)
	while (timeIn + timeout > time_keeper_get_ms()) // While we haven't timed out
	{
		if (dataAvailable()) // If data is available on UART RX
		{
			// Increment received count & read byte to buffer
			received += readByteToBuffer();
			if (searchBuffer(goodRsp))
				return received; // If we've received [goodRsp], return received
			if (searchBuffer(failRsp)) // If we've received [failRsp]
				return ERROR_FAIL_RESPONSE; // return FAIL response error code
		}
	}

	if (received > 0) // If we received any characters
		return ERROR_UNKNOWN_RESPONSE;	// Return unkown response error code
	else // If we haven't received any characters
		return ERROR_TIMEOUT; // Return the timeout error code
}

int Serial_avr32_gsm::getSubstringBetween(char * dest, const char * src, char in, char out)
{
	char * start;
	char * end;
	unsigned int length;

	start = strchr(src, in); // Find the first occurence of [in] in the [src] string
	if (start == NULL) // If it's not there
		return -1; // Return -1 error
	start += 1;	// Increment by 1 to point to start of string
	end = strchr(start, out); // Starting at [start], find [out] character
	if (end == NULL) // If it's not there
		return -1; // Return -1 error
	end -= 1; // Change [end] to point to last char in string before [out]
	length = strlen(start) - strlen(end); // Find the length from start to end
	strncpy(dest, start, length); // Copy that string into [dest]

	return length; // Return the size of our string on success
}



////////////////////
// UART Functions //
////////////////////

//void GSMShield::initializeUART(long baud)
//{
	// End any previously started UART
	// (maybe it was begin()'s at a different baud rate)
	//uart0.end();
	// Start our UART at the requested baud rate
	//uart0.begin(baud);
//}

void Serial_avr32_gsm::printString(const char * str)
{
	//print_util_dbg_print("String to print: ");
	//print_util_dbg_print(str);
	uint32_t size;
	for(size=0; str[size]!='\0'; ++size);

	//TODO: Check sizeof(str) returns the proper size
	printString(str,size);
	//uart0.print(str); // Abstracting a UART print char array
}

void Serial_avr32_gsm::printString(const char * str, uint32_t length)
{
	//TODO: check this function
	Serial_avr32::write((uint8_t*) str,length);
}

void Serial_avr32_gsm::printChar(char c)
{
	Serial_avr32::write(( uint8_t*) &c,1);
	//uart0.print(c); // Abstracting a UART print char
}

unsigned char Serial_avr32_gsm::uartRead()
{
	uint8_t byte;

	if(Serial_avr32::read(&byte, 1)){
		return byte; // Abstracting UART read
	}
	else
		return NULL;

}

unsigned int Serial_avr32_gsm::readByteToBuffer()
{
	// Read the data in
	char c = uartRead();	// uart0.read();
	print_util_dbg_print("Reading:");
	print_util_dbg_print(&c);
	print_util_dbg_print("\n");
	// Store the data in the buffer
	rxBuffer[bufferHead] = c;
	//! TODO: Don't care if we overflow. Should we? Set a flag or something?
	bufferHead = (bufferHead + 1) % RX_BUFFER_LENGTH;

	return 1;
}

void Serial_avr32_gsm::clearBuffer()
{
	memset(rxBuffer, '\0', RX_BUFFER_LENGTH);
	bufferHead = 0;
}

char * Serial_avr32_gsm::searchBuffer(const char * test)
{
	int bufferLen = strlen((const char *)rxBuffer);
	// If our buffer isn't full, just do an strstr
	if (bufferLen < RX_BUFFER_LENGTH)
		return strstr((const char *)rxBuffer, test);
	else
	{	//! TODO
		// If the buffer is full, we need to search from the end of the
		// buffer back to the beginning.
		//int testLen = strlen(test);
		//for (int i=0; i<RX_BUFFER_LENGTH; i++)
		//{

		//}
		print_util_dbg_print("Buffer full");
		return 0;
	}
}

int Serial_avr32_gsm::dataAvailable()
{
	//print_util_dbg_print("Available Bytes:");
	//print_util_dbg_print_num(Serial_avr32::readable(),10);
	//print_util_dbg_print("\n");


	return Serial_avr32::readable();
}

void Serial_avr32_gsm::clearSerial()
{
	while (dataAvailable()){
		char c = uartRead();
		print_util_dbg_print("Clearing: ");
		print_util_dbg_print(&c);
		print_util_dbg_print("\n");
	}
}

bool Serial_avr32_gsm::write(const uint8_t* bytes, const uint32_t size){
	Serial_avr32::write(bytes,size);
	return true;
}
//// Improve this later
//bool Serial_avr32::read(uint8_t* bytes, const uint32_t size)
//{
//    bool ret = false;
//
//    if (readable() >= size)
//    {
//        ret = true;
//        for (uint32_t i = 0; i < size; ++i)
//        {
//            ret &= rx_buffer_.get(bytes[i]);
//        }
//    }
//
//    return ret;
//}
