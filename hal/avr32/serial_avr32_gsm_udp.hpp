/*
 * serial_avr32_gsm_udp.hpp
 *
 *  Created on: Sep 21, 2016
 *      Author: cri
 */

#ifndef HAL_AVR32_SERIAL_AVR32_GSM_UDP_HPP_
#define HAL_AVR32_SERIAL_AVR32_GSM_UDP_HPP_

#define DEFAULT_CHANNEL 0

enum connection_status {
	GPRS_DISCONNECTED = 0,
	GPRS_ESTABLISHED
};

#define WEB_RESPONSE_TIMEOUT	30000	// 30 second timeout on web response
#define IP_ADDRESS_LENGTH 15
#define MAX_DOMAIN_LENGTH 269

#include "serial_avr32_gsm.hpp"
//#include "drivers/2GCellular/headers/IPAddress.h"
//#include "drivers/2GCellular/headers/printf.h"
//This can be improved
#include<stdio.h>

extern "C"
{
#include "util/print_util.h"
}

const char ipCharSet[] = "0123456789.";

class Serial_avr32_gsm_udp : public Serial_avr32_gsm
{
private:
	// Keep track of the active channel - the last value specified in [channel]
	// of connect([ip], [port], [channel])
	int8_t _activeChannel;
	union IPAddress {
		uint8_t bytes[4];  // IPv4 address
		uint32_t dword;
	    };



	// Helper function to convert a char array to IPAddress object
	bool charToIPAddress(char * ipChar, union IPAddress* ipRet);

	bool initcheck;
	int count;

public:
	Serial_avr32_gsm_udp(serial_avr32_conf_t config);

	void initialize(uint8_t retry);


	//////////////////////////////
	// GPRS Connection Commands //
	//////////////////////////////

	/// open() - Sends AT+ZPPPOPEN to enable GPRS
	/// Before using any TCP commands, this function must be called.
	/// This function can take a long time to complete. WEB_RESPONSE_TIMEOUT is
	/// set to 30s, which is the high end of what it might take.
	///
	/// Returns: >0 on success, <0 on fail
	int open();

	/// close() - Sends AT+ZPPPCLOSE to disable GPRS
	///
	/// Returns >0 on success, <0 on fail
	int close();

	/// localIP() - Returns the IP address assigned to the MG2639.
	/// Sends the AT+ZIPGETIP function.
	/// An object of type IPAddress is returned. This class is included with
	/// all Arduino cores - used for WiFi and Ethernet classes.
	///
	/// Returns: >0 on success, <0 on fail
	uint32_t localIP();

	/// hostByName([domain], [ipRet]) - Gets the IP address of the requested
	/// remote domain.
	/// e.g.: hostByName("sparkfun.com", returnIPhere);
	///
	/// Returns: >0 on success, <0 on fail
	int hostByName(const char * domain, IPAddress* ipRet);


	///////////////////////
	// TCP/UDP Link Commands //
	///////////////////////

	/// connect([ip], [port], [channel]) - Open a TCP connection to
	/// a specified [ip] address on a specific [port].
	/// [channel] defaults to 0. If you only need one connection open at a time,
	/// this variable can be ignored.
	/// e.g.: connect("204.144.132.37", 80);
	///
	/// Returns: >0 on success, <0 on fail
	int connectTCP(IPAddress ip, unsigned int port, uint8_t channel = DEFAULT_CHANNEL); // AT+ZIPSETUP
	int connectUDP(IPAddress ip, unsigned int port, uint8_t channel = DEFAULT_CHANNEL); // AT+ZIPSETUP

	/// connect([domain], [port], [channel]) - Open a TCP connection to
	/// a specified [domain] on a specific [port].
	/// This function calls connect(ip, port, channel) after looking up an IP.
	/// [channel] defaults to 0. If you only need one connection open at a time,
	/// this variable can be ignored.
	/// e.g.: connect("sparkfun.com", 80);
	///
	/// Returns: >0 on success, <0 on fail
	int connectTCP(const char * domain, unsigned int port, uint8_t channel = DEFAULT_CHANNEL);
	int connectUDP(const char * domain, unsigned int port, uint8_t channel = DEFAULT_CHANNEL);


	/// status() - Checks the GPRS connection status
	/// This function returns the result of "AT+ZPPPSTATUS". If it responds
	/// "ESTABLISHED", GPRS_ESTABLISHED (1) is returned.
	/// If it responds "DISCONNECTED" 0 is returned.
	int8_t statusTCP();
	int8_t statusUDP();

	/// available() - Returns number of characters available in the UART RX buffer.
	int available();

	/// read() - Reads the first character out of the UART RX buffer. Then
	/// removes it from the buffer.
	int readTCP();
	int readUDP();

	/// peek() - Looks at the first character in the UART RX buffer, but
	/// the character is left at the top of the buffer.
	//! TODO: not yet implemented
	int peek();

	//! TODO: not yet implemented
	void flush();

	/// write(b) - Send a single byte over a TCP link
	/// This function sends the "AT+ZIPSEND" command, then the requested byte.
	/// +ZIPSEND requires a TCP channel number be sent along with the data.
	/// This function uses the channel used in the last connect() function.
	///
	/// Returns: >0 on success, <0 on fail.
	size_t writeTCP(uint8_t b);
	size_t writeUDP(uint8_t b);

	/// write([buf], size) - Send a character buffer (of length [size]) over
	/// a TCP link.
	/// +ZIPSEND requires a TCP channel number be sent along with the data.
	/// This function uses the channel used in the last connect() function.
	///
	/// Returns: >0 on success, <0 on fail.
	size_t writeTCP(const uint8_t *buf, size_t size);
	size_t writeUDP(const uint8_t *buf, size_t size);

	//using Print::write;

	/**
	 * \brief   Write bytes on the serial line
	 *
	 * \param   byte        Outgoing bytes
	 * \param   size        Number of bytes to write
	 *
	 * \return  true        Data successfully written
	 * \return  false       Data not written
	 */
	bool write(const uint8_t* bytes, const uint32_t size = 1);


	/**
	 * \brief   Read bytes from the serial line
	 *
	 * \param   bytes       Incoming bytes
	 * \param   size        Number of bytes to read
	 *
	 * \return  true        Data successfully read
	 * \return  false       Data not read
	 */
	//bool read(uint8_t* bytes, const uint32_t size = 1);

};

#endif /* HAL_AVR32_SERIAL_AVR32_GSM_UDP_HPP_ */
