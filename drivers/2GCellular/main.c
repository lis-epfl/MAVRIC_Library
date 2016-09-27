/************************************************************
MG2639_GPRS_Client.h
MG2639 Cellular Shield library - GPRS TCP Client Example
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 3, 2015
https://github.com/sparkfun/MG2639_Cellular_Shield

Development environment specifics:
  IDE: Arduino 1.6.3
  Hardware Platform: Arduino Uno
  MG2639 Cellular Shield Version: 1.0

This code is beerware; if you see me (or any other SparkFun
employee) at the local, and you've found our code helpful,
please buy us a round!

Distributed as-is; no warranty is given.
************************************************************/
// The SparkFun MG2639 Cellular Shield uses SoftwareSerial
// to communicate with the MG2639 module. Include that
// library first:
#include <SoftwareSerial.h>
#include <headers/IPAddress.h>
// Include the MG2639 Cellular Shield library
#include "GSMShield.h"
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

#define MAX_PACKET_SIZE 100
#define MIN_PACKET_SIZE 2 // MAVLINK heartbeat is 19 byte

IPAddress myIP; // IP address to store local IP

// To define your destination server, you can either set the
// IP address explicitly:
IPAddress serverIP(128, 178, 46, 22);
// Or define the domain of the server and use DNS to look up
// the IP for you:
//const char server[] = "lispc1.epfl.ch";
//unsigned long timeSend = 0;
//int sent = 0;

void setup()
{
  // USB serial connection is used to print the information
  // we find.
  Serial.begin(9600);

  // serialTrigger() halts execution of the program until
  // any value is received over the serial link. Cell data
  // costs $, so we don't want to use it unless it's visible!
  serialTrigger();
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

  // Time to send an HTTP request to the server we connected
  // to. gprs.print() and gprs.println() can be used to send
  // data of just about any variable type.
  UDPWrite();

  // You can do this to send a GET string..
  /*gprs.println("GET / HTTP/1.1");
  gprs.print("Host: ");
  gprs.print(server);
  gprs.println();
  gprs.println();*/
  // ...but it's a whole lot faster if you do this:
  //gprs.print("GET / HTTP/1.1\nHost: example.com\n\n");
}
void UDPWrite(){


  byte buffer[MAX_PACKET_SIZE];

  //This will not be needed when buffer is MAVRIC buffer
  int length = 0;
  if(Serial.available()<MAX_PACKET_SIZE-1 && Serial.available()>MIN_PACKET_SIZE ){
      length = Serial.available();
  }
  // Delete till this

  if(length!=0){
    Serial.readBytes(buffer, length); //Read the messages from the buffer and write into GPRS
    int UDPWrite = gprs.writeUDP(buffer, length);
    if(UDPWrite == -1){
      DEBUG_PRINT("Error in UDP Write:");
    }else{
      DEBUG_PRINT("UDP Write Success: ");
      DEBUG_PRINTLN(length);
    }
  }
}
int checkCommand(int* length){
  uint8_t textCheck[] = "ZIPRECVU:";
  uint8_t read = gprs.readUDP(); // Reads a single character from gprs
  int i=0;
  PACKETREAD:
  while(i<9){
      //Many blank characters are being sent by the cellular module
      if(read==255){//If blank read the next
        read = gprs.readUDP();
      }

      //Keep validating each character
      if(read==textCheck[i]){
        read = gprs.readUDP();
        i++;
        continue;
      }
      else{
          //Read the next byte and check- Many time Junk pops-up
          //TODO: The else part should be improved after integration
          while(read == 255){
            read = gprs.readUDP();
            if(read==textCheck[i]){
              read = gprs.readUDP();
              i++;
              goto PACKETREAD;
            }
          }
         // DEBUG_PRINT("Retry Expecting:");
        //  DEBUG_WRITE(textCheck[i]);
        //  DEBUG_PRINT("Retry Received:");
        //  DEBUG_WRITE(read);

          DEBUG_PRINT("\n - Wrong message");
          while (gprs.available()) {
            read = gprs.readUDP();
            Serial.write(read);//See the excess
          }
          i=10;// bad message
          break;
        }

      }
      if(i==9){
        //After ZIPRECVU:,  ignore till next ','
        while(read!=','){
          read = gprs.readUDP();
        }
       int len = 0;
       do{
          read = gprs.readUDP(); // Reads the numbers as ascii
          if(read!=','){// Loop till next ','
            if(read!=255){
              len = 10 * len + (read - '0');
            }
            continue;
          }
          else{
            break;// Stop reading when ',' appears
          }
       } while(1);
       //Set the length in the parameter variable and return success
       *length = len;
        return 1;
      }
      //Set length as 0 and return failure
      *length = 0;
      return 0;
}


//This function is not required after integration
char c2h(char c)
{  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}
//This function is not required after integration
void serialPrintByte(uint8_t val){
  Serial.print(c2h(val>>4));
  Serial.println(c2h(val));
}

void loop()
{
  // gprs.available() returns the number of bytes available
  // sent back from the server. If it's 0 there is no data
  // available.
  uint8_t read;
  while (gprs.available()) {
    read = gprs.readUDP();
    if(read == '+'){

      int len = 0;
      int valid = checkCommand(&len); // On reading + expect the command ZIPRECVU:xx,length, packet
      //When the function returns valid, the buffer's conent should be the first character(FE) of the mavlink packet. Eliminate all junks before that
      while(valid){
        read = gprs.readUDP();
        if(read==254){
          break;// 254 (or) FE is the srart of the mavlink message
        }
       }
       DEBUG_PRINT("Packet Length:");
       DEBUG_PRINTLN(len);
       //Read the packet till its end
      for(int i=0;i<len;i++){
        Serial.print(read,HEX);
        Serial.print(" ");
        serialPrintByte(read);
        read = gprs.readUDP();
      }
      DEBUG_PRINTLN("end of receiving");
    }
    else{
      // DEBUG_PRINT("Excess: ");
    //   Serial.write(read);//See the excess
    }
  }

  //Remove after integration
  if (Serial.available()){
    UDPWrite();
  }

}

void serialTrigger()
{
  DEBUG_PRINTLN("Send some serial to start");
  while (!Serial.available()){
  }
  DEBUG_PRINTLN(Serial.available());
  DEBUG_PRINTLN(gprs.available());
  Serial.read();
}
