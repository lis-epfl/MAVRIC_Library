/*
 * serial_avr32_gsm_udp.cpp
 *
 *  Created on: Sep 21, 2016
 *      Author: cri
 */

#include "serial_avr32_gsm_udp.hpp"

#define DEBUG 1
#if defined(DEBUG) && DEBUG > 0
# define PRINT(x) print_util_dbg_print(x)
# define PRINTLN(x) print_util_dbg_print(x); print_util_dbg_sep('-');
# define WRITE(x) print_util_dbg_print(x)
#else
# define PRINT(x) do {} while (0)
# define PRINTLN(x) do {} while (0)
# define WRITE(x) do {} while (0)
#endif

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Serial_avr32_gsm_udp::Serial_avr32_gsm_udp(serial_avr32_conf_t config) {
	config_ = config;
	irq_callback = NULL;
	_activeChannel = -1;
	initcheck = false;
	temp = 1;
	//print_util_dbg_print("initializing init check");
	//initialize();
}

void Serial_avr32_gsm_udp::initialize(uint8_t retry) {

	//Serial_avr32::init();

	print_util_dbg_sep('-');
	print_util_dbg_print("GSM Initialization\n");
	print_util_dbg_sep('-');
	union IPAddress myIP; // IP address to store local IP

	union IPAddress serverIP;
	serverIP.bytes= {128, 178, 46, 22};

	//PRINTLN("Checking SIM 2...");
	//bool simCheck = checkSIM();
//	// Call  begin() to turn the module on and verify
//	// communication.
	//if(simCheck){
	//	print_util_dbg_print("Starting communication with the mobile shield");
	//}else{
	//	print_util_dbg_print("Simcheck failed");
	//}

	bool beginStatus =  begin();
	uint8_t retry_iter = retry;
	while (beginStatus <= 0 && retry_iter>0 ) {
		PRINT("Attempting to communicate with shield :");
		print_util_dbg_print_num(retry-retry_iter+1,10);
		PRINT("\n");
		beginStatus =  begin();
		retry_iter--;
	}
	if(beginStatus){
		PRINTLN("Communication established with the shield. ");
		PRINTLN("Attempting to open mobile network and GPRS ");
	}
	else{
		PRINTLN("Communication failed ");
	}

//
//	 // gprs.open() enables GPRS. This function should be called
//	  // before doing any TCP stuff. It should only be necessary
//	  // to call this function once.
//	  // gprs.open() can take up to 30 seconds to successfully
//	  // return. It'll return a positive value upon success.
//	  int openStatus = open();
//	  while (openStatus <= 0)
//	  {
//		PRINTLN("Unable to open GPRS. Looping");
//		openStatus = open();
//	  }
//	  PRINTLN("Opening GPRS Connection");
//
//	  // gprs.status() returns the current GPRS connection status.
//	  // This function will either return a 1 if a connection is
//	  // established, or a 0 if the module is disconnected.
//	  int GPRSStatus = statusUDP();
//	  if (GPRSStatus == 1){
//		PRINTLN("GPRS Connection Established");
//	  }
//	  else{
//		PRINTLN("GPRS disconnected");
//	  }
//
//	  // gprs.connect(IPAddress remoteIP, port) establishes a TCP
//	  // connection to the desired host on the desired port.
//	  // We looked up serverIP in the previous step. Port 80 is
//	  // the standard HTTP port.
//	  int connectStatus = connectUDP(serverIP, 14550);
//	  // a gprs.connect(char * domain, port) is also defined.
//	  // It takes care of the DNS lookup for you. For example:
//	  //gprs.connect(server, 80); // Connect to a domain
//	  if (connectStatus <= 0)
//	  {
//		PRINTLN("Unable to connect. Looping.");
//		while (1)
//		  ;
//	  }
//	  PRINTLN("Connected!");
	  initcheck = false;
}
int Serial_avr32_gsm_udp::open() // AT+ZPPPOPEN
{
	int iRetVal;
	 sendATCommand(OPEN_GPRS);
	// Should respond "+ZPPPOPEN:CONNECTED\r\n\r\nOK\r\n\r\n" or
	//				  "+ZPPPOPEN:ESTABLISHED\r\n\r\nOK\r\n\r\n"
	// Bad response is "+ZPPPOPEN:FAIL\r\n\r\nERROR\r\n"
	// bad response can take ~20 seconds to occur
	iRetVal =  readWaitForResponses(RESPONSE_OK, RESPONSE_ERROR,
			WEB_RESPONSE_TIMEOUT);

	return iRetVal;
}

int Serial_avr32_gsm_udp::close() //AT+ZPPPCLOSE
{
	int iRetVal;
	 sendATCommand(CLOSE_GPRS);
	// Should respond "+ZPPCLOSE:OK\r\n\r\nOK\r\n\r\n"
	iRetVal =  readWaitForResponses(RESPONSE_OK, RESPONSE_ERROR,
			WEB_RESPONSE_TIMEOUT);

	return iRetVal;
}

uint32_t Serial_avr32_gsm_udp::localIP() // AT+ZIPGETIP
{
	int iRetVal;
	 sendATCommand(GET_IP);
	iRetVal =  readWaitForResponse(RESPONSE_OK, WEB_RESPONSE_TIMEOUT);
	if (iRetVal < 0) {
		return iRetVal;
	}

	// Response looks like: +ZIPGETIP:nnn.nnn.nnn.nnn\r\n\r\nOK\r\n\r\n
	// We need to copy the middle, IP address portion of that to ipRet
	// Find the first occurence of numbers or .'s:
	char * start;
	int len = 0;
	char tempIP[IP_ADDRESS_LENGTH];
	memset(tempIP, 0, IP_ADDRESS_LENGTH);
	start = strpbrk((const char *)  rxBuffer, ipCharSet);
	len = strspn(start, ipCharSet);
	// Copy the string
	if ((len > 0) && (len <= IP_ADDRESS_LENGTH))
		strncpy(tempIP, start, len);

	// Little extra work to convert the "nnn.nnn.nnn.nnn" string to four
	// octet values required for the IPAdress type.
	//IPAddress* ipRet;
	union IPAddress* ipRet;
	charToIPAddress(tempIP, ipRet);
	return ipRet->dword;
}

int Serial_avr32_gsm_udp::hostByName(const char * domain, union IPAddress ipRet) // AT+ZDNSGETIP
		{
	int iRetVal;
	char dnsCommand[MAX_DOMAIN_LENGTH];
	memset(dnsCommand, '\0', 269);
	sprintf(dnsCommand, "%s=\"%s\"", DNS_GET_IP, domain);
	 sendATCommand((const char *) dnsCommand);
	iRetVal =  readWaitForResponse(RESPONSE_OK, WEB_RESPONSE_TIMEOUT);
	if (iRetVal < 0) {
		return iRetVal;
	}

	// Response looks like +ZDNSGETIP:nnn.nnn.nnn.nnn\r\n\r\nOK\r\n\r\n"
	// We need to copy the middle, IP address portion of that to ipRet
	// Find the first occurence of numbers or .'s:
	char * start;
	int len = 0;
	char tempIP[IP_ADDRESS_LENGTH];
	start = strpbrk((const char *)  rxBuffer, ipCharSet);
	// !!! TO DO Check if we're at the edge of the ring buffer
	// Find the length of that string:
	len = strspn(start, ipCharSet);
	// Copy the string
	if ((len > 0) && (len <= IP_ADDRESS_LENGTH))
		strncpy(tempIP, start, len);

	// Little extra work to convert the "nnn.nnn.nnn.nnn" string to four
	// octet values required for the IPAdress type.
	charToIPAddress(tempIP, &ipRet);

	return iRetVal;
}

int Serial_avr32_gsm_udp::connectTCP(const char * domain, unsigned int port,
		uint8_t channel) {
	union IPAddress destIP;
	hostByName(domain, destIP);
	return connectTCP(destIP, port, channel);
}
int Serial_avr32_gsm_udp::connectUDP(const char * domain, unsigned int port,
		uint8_t channel) {
	union IPAddress destIP;
	hostByName(domain, destIP);
	return connectUDP(destIP, port, channel);
}

int Serial_avr32_gsm_udp::connectTCP(union IPAddress ip, unsigned int port,
		uint8_t channel) {
	int iRetVal;
	// Maximum is 15 for IP + 5 for port + 1 for channel + 12 for cmd, = and ,'s
	char ipSetupCmd[33];
	memset(ipSetupCmd, '\0', 33);
	sprintf(ipSetupCmd, "%s=%d,%d.%d.%d.%d,%d", TCP_SETUP, channel, ip.bytes[0],
			ip.bytes[1], ip.bytes[2], ip.bytes[3], port);
	//sprintf(ipSetupCmd, "%s=%i,%s,%i", TCP_SETUP, channel, ip, port);
	 sendATCommand((const char *) ipSetupCmd);

	iRetVal =  readWaitForResponse(RESPONSE_OK, WEB_RESPONSE_TIMEOUT);
	if (iRetVal < 0)	// If nothing was received return timeout error
			{
		return iRetVal;
	}

	_activeChannel = channel;

	return iRetVal;
}
int Serial_avr32_gsm_udp::connectUDP(union IPAddress ip, unsigned int port,
		uint8_t channel) {
	int iRetVal;
	// Maximum is 15 for IP + 5 for port + 1 for channel + 12 for cmd, = and ,'s
	char ipSetupCmd[33];
	memset(ipSetupCmd, '\0', 33);
	sprintf(ipSetupCmd, "%s=%d,%d.%d.%d.%d,%d", UDP_SETUP, channel, ip.bytes[0],
			ip.bytes[1], ip.bytes[2], ip.bytes[3], port);
	//sprintf(ipSetupCmd, "%s=%i,%s,%i", TCP_SETUP, channel, ip, port);
	 sendATCommand((const char *) ipSetupCmd);

	iRetVal =  readWaitForResponse(RESPONSE_OK, WEB_RESPONSE_TIMEOUT);
	if (iRetVal < 0)	// If nothing was received return timeout error
			{
		return iRetVal;
	}

	_activeChannel = channel;

	return iRetVal;
}

int8_t Serial_avr32_gsm_udp::statusTCP() {
	int iRetVal;
	 sendATCommand(TCP_STATUS);
	iRetVal =  readWaitForResponses("ESTABLISHED", "DISCONNECTED",
			WEB_RESPONSE_TIMEOUT);

	if (iRetVal > 0)
		return GPRS_ESTABLISHED;
	else if (iRetVal == ERROR_FAIL_RESPONSE)
		return GPRS_DISCONNECTED;

	return iRetVal;
}
int8_t Serial_avr32_gsm_udp::statusUDP() {
	int iRetVal;
	 sendATCommand(UDP_STATUS);
	iRetVal =  readWaitForResponses("ESTABLISHED", "DISCONNECTED",
			WEB_RESPONSE_TIMEOUT);

	if (iRetVal > 0)
		return GPRS_ESTABLISHED;
	else if (iRetVal == ERROR_FAIL_RESPONSE)
		return GPRS_DISCONNECTED;

	return iRetVal;
}

int Serial_avr32_gsm_udp::available() {
	// Should check if we're connected & within a +ZIPRECV
	return  dataAvailable();
}

int Serial_avr32_gsm_udp::readTCP() {
	if (!available())
		return -1;
	return  uartRead();
}
int Serial_avr32_gsm_udp::readUDP() {
	if (!available())
		return -1;
	return  uartRead();
}

int Serial_avr32_gsm_udp::peek() {
	return 0;
}

void Serial_avr32_gsm_udp::flush() {
}

size_t Serial_avr32_gsm_udp::writeTCP(uint8_t b) {
	return writeTCP(&b, 1);
}

size_t Serial_avr32_gsm_udp::writeUDP(uint8_t b) {
	return writeUDP(&b, 1);
}

size_t Serial_avr32_gsm_udp::writeTCP(const uint8_t *buf, size_t size) {
	int iRetVal;
	// Maximum is 10 for command ',' and '=', 5 for port, 4 for length
	char sendCmd[19];
	memset(sendCmd, '\0', 19);
	sprintf(sendCmd, "%s=%i,%i", TCP_SEND, _activeChannel, size);
	 sendATCommand((const char *) sendCmd);
	iRetVal =  readWaitForResponse(">", WEB_RESPONSE_TIMEOUT);
	if (iRetVal <= 0)
		return -1;

	 clearSerial();		// Clear out the serial rx buffer
	 printString((const char *) buf);// Send the data string to cell module
	iRetVal =  readWaitForResponse("+ZIPSEND: OK", WEB_RESPONSE_TIMEOUT);
	if (iRetVal <= 0)
		return -1;

	return size;
}

size_t Serial_avr32_gsm_udp::writeUDP(const uint8_t *buf, size_t size) {
	int iRetVal;
	// Maximum is 10 for command ',' and '=', 5 for port, 4 for length
	char sendCmd[19];
	memset(sendCmd, '\0', 19);
	sprintf(sendCmd, "%s=%i,%i", UDP_SEND, _activeChannel, size);
	 sendATCommand((const char *) sendCmd);
	iRetVal =  readWaitForResponse(">", WEB_RESPONSE_TIMEOUT);
	if (iRetVal <= 0)
		return -1;

	 clearSerial();		// Clear out the serial rx buffer
	 printString((const char *) buf);// Send the data string to cell module
	iRetVal =  readWaitForResponse("+ZIPSENDU: OK", WEB_RESPONSE_TIMEOUT);
	if (iRetVal <= 0)
		return -1;

	return size;
}

bool Serial_avr32_gsm_udp::write(const uint8_t* bytes, const uint32_t size){

//	Serial_avr32::write((uint8_t*) bytes,size);
	//print_util_dbg_print("Writing something");
//	if(initcheck){
//		print_util_dbg_print("Actually Writing");
//		writeUDP((uint8_t*)bytes ,(uint32_t)size);
//	}else{
		//if(temp==1){
			initialize(3);
		//}

		//temp = 0;

//	}

}
bool Serial_avr32_gsm_udp::charToIPAddress(char * ipChar,
		union IPAddress* ipRet) {
	int a, b, c, d;
	int scan = sscanf(ipChar, "%d.%d.%d.%d", &a, &b, &c, &d);
	if (scan == 4) {
		ipRet->bytes[0] = (uint8_t) a;
		ipRet->bytes[1] = (uint8_t) b;
		ipRet->bytes[2] = (uint8_t) c;
		ipRet->bytes[3] = (uint8_t) d;

		//ipRet = {(uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d};
		return 1;
	}

	return 0;
}
