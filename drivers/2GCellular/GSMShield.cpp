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
 * \file SFE_MG2639_CellShield.cpp
 *
 * \author MAV'RIC Team
 * \author Anand B
 *
 * \brief Abstract class for accelerometers
 *
 *This file is originally derived from a SparkFun project available at https://github.com/sparkfun/MG2639_Cellular_Shield
 *This file contains all the functions to successfully communicate with the cellular shield.
 ******************************************************************************/


#include "at_commands.h"
#include "GSMShield.h"
#include "util/print_util.h"

#include "hal/common/time_keeper.hpp"

#define BAUD_COUNT 7 // Number of possible baud rates the MG2639 can be set to
unsigned long baudRates[BAUD_COUNT] = {2400, 4800, 9600, 19200, 38400, 
										57600, 115200};

#define DEBUG 1
#if defined(DEBUG) && DEBUG > 0
# define DEBUG_PRINT(x) Serial.print(x)
# define DEBUG_PRINTLN(x) Serial.println(x)
# define DEBUG_WRITE(x) Serial.write(x)
#else
# define DEBUG_PRINT(x) do {} while (0)
# define DEBUG_PRINTLN(x) do {} while (0)
# define DEBUG_WRITE(x) do {} while (0)
#endif
	
/////////////////
// Constructor //
/////////////////
GSMShield::GSMShield(Serial& uart)
{
	IPAddress myIP; // IP address to store local IP

	IPAddress serverIP(128, 178, 46, 22);

	DEBUG_PRINTLN("Checking SIM...");
	  int simCheck = cell.checkSIM();
	  // Call cell.begin() to turn the module on and verify
	  // communication.
	  DEBUG_PRINTLN("Starting communication with the mobile shield");

	  int beginStatus = cell.begin();
	  while (beginStatus <= 0)
	  {
	    DEBUG_PRINTLN(F("Unable to communicate with shield. Looping"));
	    beginStatus = cell.begin();
	  }
	  DEBUG_PRINTLN(F("Communication established with the shield. "));
	  DEBUG_PRINTLN(F("Attempting to open mobile network and GPRS "));

	  // gprs.open() enables GPRS. This function should be called
	  // before doing any TCP stuff. It should only be necessary
	  // to call this function once.
	  // gprs.open() can take up to 30 seconds to successfully
	  // return. It'll return a positive value upon success.
	  int openStatus = gprs.open();
	  while (openStatus <= 0)
	  {
	    DEBUG_PRINTLN(F("Unable to open GPRS. Looping"));
	    openStatus = gprs.open();
	  }
	   DEBUG_PRINTLN(F("Opening GPRS Connection"));
	  // gprs.status() returns the current GPRS connection status.
	  // This function will either return a 1 if a connection is
	  // established, or a 0 if the module is disconnected.
	  int GPRSStatus = gprs.status();
	  if (GPRSStatus == 1)
	    DEBUG_PRINTLN(F("GPRS Connection Established"));
	  else
	    DEBUG_PRINTLN(F("GPRS disconnected"));

	  // gprs.localIP() gets and returns the local IP address
	  // assigned to the MG2639 during this session.
	 // myIP = gprs.localIP();
	 // DEBUG_PRINT(F("My IP address is: "));
	 // DEBUG_PRINTLN(myIP);

	  // gprsHostByName(char * domain, IPAddress * ipRet) looks
	  // up the DNS value of a domain name. The pertinent return
	  // value is passed back by reference in the second
	  // parameter. The return value is an negative error code or
	  // positive integer if successful.
	 // int DNSStatus = gprs.hostByName(server, &serverIP);
	//  if (DNSStatus <= 0)
	//  {
	//    Serial.println(F("Couldn't find the server IP. Looping."));
	//    while (1)
	//      ;
	 //  }
	//  DEBUG_PRINT(F("Server IP is: "));
	//  DEBUG_PRINTLN(serverIP);

	  // gprs.connect(IPAddress remoteIP, port) establishes a TCP
	  // connection to the desired host on the desired port.
	  // We looked up serverIP in the previous step. Port 80 is
	  // the standard HTTP port.
	  int connectStatus = gprs.connectUDP(serverIP, 14550);
	  // a gprs.connect(char * domain, port) is also defined.
	  // It takes care of the DNS lookup for you. For example:
	  //gprs.connect(server, 80); // Connect to a domain
	  if (connectStatus <= 0)
	  {
	    DEBUG_PRINTLN(F("Unable to connect. Looping."));
	    while (1)
	      ;
	  }
	  DEBUG_PRINTLN(F("Connected!"));
}

//////////////////////////////
// Initialization Functions //
//////////////////////////////

//bool Spektrum_satellite::init(void)
//{
//	// Attach interrupt handler function to UART if necessary
//	return true;
//
//}

  
uint8_t GSMShield::begin(unsigned long baud)
{
	unsigned long setBaud = 0;
	uint8_t tries = 0;
	
	//initializePins(); // Set up power and UART pin direction
	initializeUART(baud); // Initialize UART to requested baud
	
	// Try turning echo off first (send ATE0 command). If it succeeds,
	// we're already at the target baud and the module's on.
	if (setEcho(0) <= 0)
	{	
		// If setEcho fails, we can't communicate with the shield. The baud
		// may be incorrect, or the shield may be off. First time in, we'll
		// assume the module is on.
		// tries should be at least > 2 - one time assuming module is on,
		// another time assuming module is off
		while (setBaud <= 0 && tries < 4)
		{
			setBaud = autoBaud(); // Try to find the baud rate
			
			if (setBaud <= 0)
			{
				// If autoBaud fails to find anything, the module must be off.
				powerPulse(); // Send a power pulse (hold PWRKEY for 3s)
				time_keeper_delay_ms(MODULE_WARM_UP_TIME); // Delay ~3s for module to warm up
			}
			tries++; // Increment tries and go again
		}
		
		// If we still can't find the baud rate, or communicate with the shield
		// we give up. Return a fail.
		if (setBaud <= 0){
			return 0;
		}
		
		// If autoBaud succeeds, the module is on, we just need to 
		// change the baud rate.
		if (setBaud != baud)
		{
			int baudRsp;
	#if (ARDUINO >= 10601) 
			// Software serial after 1.6.1 works much better, no need for brute force
			baudRsp = changeBaud(setBaud, baud);
	#else
			baudRsp = bruteForceBaudChange(setBaud, baud, 100);
	#endif
			// Look for any error _except_ ERROR_UNKOWN_RESPONSE -- a change from 
			// 115200 will result in that error, even though the change baud 
			// worked. (SoftwareSerial can't read reliably at that high rate.)
			if ((baudRsp == 0) || (baudRsp == ERROR_FAIL_RESPONSE) || 
				(baudRsp == ERROR_TIMEOUT))
			{
				return 0;
			}
		}
		time_keeper_delay_ms(COMMAND_RESPONSE_TIME);
	}
	return 1;
}

uint8_t GSMShield::begin()
{
	// begin() works just like begin([baud]), but we'll call
	// TARGET_BAUD_RATE defined in the h file.
	return begin(TARGET_BAUD_RATE);
}

//////////////////////
// Hardware Control //
//////////////////////
	
//void MG2639_Cell::initializePins()
//{
	// Set ON/OFF and RESET pins as OUTPUTs:
	// To avoid complexity, no connection has been given to this pin
	//pinMode(CELL_ON_OFF, OUTPUT);	// Set CELL_ON_OFF as an OUTPUT
	//digitalWrite(CELL_ON_OFF, LOW);
//}

void GSMShield::powerPulse()
{
	//digitalWrite(CELL_ON_OFF, HIGH);	// Writing high will initiate
	//delay(POWER_PULSE_DURATION);		// Delay 2 to 5 seconds to turn on/off
	//digitalWrite(CELL_ON_OFF, LOW);		// Writing low will end the pulse
}

///////////////////////////////
// Specific Command Routines //
///////////////////////////////

// Turn Echo ON or OFF
int GSMShield::setEcho(uint8_t on)
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

///////////////////////
// Baud Rate Control //
///////////////////////

unsigned long GSMShield::autoBaud()
{
	int i;
	int echoResponse;
	for (i=0; i<BAUD_COUNT; i++)
	{
		initializeUART(baudRates[i]); // Set UART to baud rate

		printChar('\r'); // Print a '\r' to send any possible garbage command
		time_keeper_delay_ms(10); // Wait for a possible "ERROR" response

		// Try turning echo off to see if the module responds with "OK"
		echoResponse = setEcho(0);
		// If setEcho is successful, we found the baud rate! Break this loop
		if (echoResponse > 0)
			break;
	}
	// If we found the baud rate, return the matching value
	if (echoResponse > 0)
		return baudRates[i];
	else
		return 0; // Otherwise we failed to find it, return 0
}

int GSMShield::changeBaud(unsigned long from, unsigned long to)
{
	int iRetVal;
	char changeBaudCmd[14];
	
	memset(changeBaudCmd, 0, 14);
	// Print something like "AT+IPR=9600" to changeBaudCmd string
	sprintf(changeBaudCmd, "%s=%lu", SET_BAUD_RATE, to);
	
	initializeUART(from); // Set UART baud to [from] baud	
	sendATCommand((const char *)changeBaudCmd); // Send the baud change command

	// SoftwareSerial included with Arduino 1.6.1+ is drastically improved.
	// We can reliably send strings at 115200
#if (ARDUINO >= 10601) 	
	iRetVal = readWaitForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIME);
	initializeUART(to); // Set SS UART to [to] baud rate
	return iRetVal;
#else
	// Earlier versions of SS can't reliably read, but we should be able
	// to tell the difference between "ERROR" and "OK" due to the size
	// of the received string.
	unsigned long timeIn = time_keeper_get_ms();
	unsigned long timeout = 10; // Set timeout to 10ms
	int received = 0;
	
	clearBuffer(); // Clear rxBuffer
	while (timeIn + timeout > time_keeper_get_ms()) // Check for a timeout
	{
		if (dataAvailable()) // If data available on UART
		{
			received += readByteToBuffer(); // Read it into rxBuffer, inc received
		}
	}
	
	initializeUART(to); // Set SS UART to [to] baud rate
	
	return received; // Return number of characters received
#endif
}

// KLUDGE ALERT: We can't read reliably at 115200, BUT we can tell the 
// difference between "ERROR" and "OK" by the length of the response. 
// An OK response is usually 5 characters, ERROR is 8 (string + \r\n).
// If the response exists and is short, assume we got an OK.
uint8_t GSMShield::bruteForceBaudChange(unsigned long from, unsigned long to,
											unsigned int tries)
{
	uint8_t baudRsp;
	for (unsigned int i=0; i<tries; i++)
	{
		baudRsp = changeBaud(from, to);
		if ((baudRsp <= 5) && (baudRsp > 0))
		{
			return 1;
		}
	}
	return 0;
}

/////////////////////////
// Information Queries //
/////////////////////////

int8_t GSMShield::getInformation(char * infoRet)
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

bool GSMShield::checkSIM()
{
	int8_t iRetVal;
	
	sendATCommand(CHECK_SIM); // Send "AT*TSIMINS"
	
	iRetVal = readWaitForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIME);
	// Example response: *TSIMINS:0, 1/r/n/r/nOK/r/n
	// First value has no meaning. Second will be 1 if SIM is
	// present and 0 if there is no SIM.
	if (iRetVal > 0)
	{
		char * ptr;
		// Look for the comma in the response, closest unique
		// character up to that point.
		ptr = strchr((const char *)rxBuffer, ',');
		ptr += 2; // Move two spots (space, then our character of interest)
		if (ptr[0] == '1')
			return true;
	}
	
	return false;
}

int8_t GSMShield::getPhoneNumber(char * phoneRet)
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

int8_t GSMShield::getICCID(char * iccidRet)
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

int8_t GSMShield::getIMI(char * imiRet)
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

int8_t GSMShield::getIMEI(char * imeiRet)
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

void GSMShield::sendATCommand(const char * command)
{	
	clearSerial();	// Empty the UART receive buffer
	// Send the command:
	printString("AT"); // Print "AT"
	printString(command); // Print the command
	printChar('\r'); // Print a carriage return to end command
}

int8_t GSMShield::readBetween(char begin, char end, char * rsp,
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

int GSMShield::readUntil(char * dest, char end, int maxChars,
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

int GSMShield::readWaitForResponse(const char *goodRsp, unsigned int timeout)
{
	unsigned long timeIn = time_keeper_get_ms();	// Timestamp coming into function
	unsigned int received = 0; // received keeps track of number of chars read
	
	clearBuffer();	// Clear the class receive buffer (rxBuffer)
	while (timeIn + timeout > time_keeper_get_ms()) // While we haven't timed out
	{
		if (dataAvailable()) // If data is available on UART RX
		{
			received += readByteToBuffer();
			if (searchBuffer(goodRsp))	// Search the buffer for goodRsp
				return received;	// Return how number of chars read
		}
	}
	
	if (received > 0) // If we received any characters
		return ERROR_UNKNOWN_RESPONSE; // Return unkown response error code
	else // If we haven't received any characters
		return ERROR_TIMEOUT; // Return the timeout error code
}

int GSMShield::readWaitForResponses(const char * goodRsp,
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

int GSMShield::getSubstringBetween(char * dest, const char * src, char in, char out)
{
	char * start;
	char * end;
	size_t length;
	
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

void GSMShield::printString(const char * str)
{
	uart0.print(str); // Abstracting a UART print char array
}

void GSMShield::printString(const char * str, size_t length)
{
	unsigned int i;
	for (i = 0; i < length; i++)
	{
		uart0.write(str[i]);
	}
}

void GSMShield::printChar(char c)
{
	uart0.print(c); // Abstracting a UART print char
}

unsigned char GSMShield::uartRead()
{
	return uart0.read(); // Abstracting UART read
}

unsigned int GSMShield::readByteToBuffer()
{
	// Read the data in
	char c = uartRead();	// uart0.read();
	
	// Store the data in the buffer
	rxBuffer[bufferHead] = c;
	//! TODO: Don't care if we overflow. Should we? Set a flag or something?
	bufferHead = (bufferHead + 1) % RX_BUFFER_LENGTH;
	
	return 1;
}

void GSMShield::clearBuffer()
{
	memset(rxBuffer, '\0', RX_BUFFER_LENGTH);
	bufferHead = 0;
}

char * GSMShield::searchBuffer(const char * test)
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
		return 0;
	}
}

int GSMShield::dataAvailable()
{
	return uart0.available();
}

void GSMShield::clearSerial()
{
	while (uart0.available())
		uart0.read();
}

GSMShield cell;
