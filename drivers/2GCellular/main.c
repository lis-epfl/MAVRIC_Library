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
// Include the MG2639 Cellular Shield library
#include <SFE_MG2639_CellShield.h>
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


IPAddress myIP; // IP address to store local IP

// To define your destination server, you can either set the
// IP address explicitly:
IPAddress serverIP(128, 178, 46, 22);
// Or define the domain of the server and use DNS to look up
// the IP for you:
const char server[] = "lispc1.epfl.ch";
unsigned long timeSend = 0;
int sent = 0;

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
  myIP = gprs.localIP();
  DEBUG_PRINT(F("My IP address is: "));
  DEBUG_PRINTLN(myIP);

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
  DEBUG_PRINT(F("Server IP is: "));
  DEBUG_PRINTLN(serverIP);

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
  uint8_t textToSend[] = "and";
  byte buffer[100];
  int length = 98; // Max payload of one packet
  if(Serial.available()<98){
      length = Serial.available();
  }

  if(length!=0){
    Serial.readBytes(buffer, length);
    int UDPWrite = gprs.writeUDP(buffer, length);
    if(UDPWrite == -1){
      DEBUG_PRINT("Error in UDP Write:");
    }else{

      DEBUG_PRINT("UDP Write Success");
     // timeSend = micros();
      //sent = 1;
    }
  }
}
int checkCommand(){
  uint8_t textCheck[] = "ZIPRECVU:";
  //Two blank messages will be sent after + before U
  uint8_t read = gprs.readUDP();
    int i;
    for(i=0;i<9;i++){
        read = gprs.readUDP();
       // Serial.write(read);//See the excess
        if(read==textCheck[i]){

          continue;
        }
        else{
          if(read == 255){
            i-=1;// Reset the count
            continue;
          }
          DEBUG_PRINT("Wrong message");
          i=10;// bad message
          break;
        }
      }
      if(i==9){
        //ignore till next ','
        while(read!=','){
          read = gprs.readUDP();
        }
        //Read next char
       // Serial.println("lecnt calc");
       int len = 0;
       do{
          read = gprs.readUDP(); // Reads the next char and gets its integer number
          if(read!=','){
            if(read!=255){
              len = 10 * len + (read - '0');
            }
            continue;
          }
          else{
            break;
          }
       } while(1);
       // DEBUG_PRINT(len);
        return len;
      }
      return 0;
}

char c2h(char c)
{  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

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
  //Serial.println(gprs.available());
  while (gprs.available()) {
    //DEBUG_PRINT("Loop...");
    read = gprs.readUDP();
    if(read == '+'){
      int len = 0;
      len = checkCommand();
      while(1){
        read = gprs.readUDP();
        if(read==254){
          break;// 254 (or) FE is the srart of the mavlink message
        }
       }
      for(int i=0;i<len;i++){
        Serial.print(read,HEX);
        Serial.print(" ");
        serialPrintByte(read);
        read = gprs.readUDP();


        //Serial.print(i);
      }
      DEBUG_PRINT("\n");
      Serial.print("\n");
    // DEBUG_PRINTLN(Serial.available());
     //  UDPWrite();
    }
    else{
      // DEBUG_PRINT("Excess: ");
    //   Serial.write(read);//See the excess
    }
  }
  if (Serial.available()){
    DEBUG_PRINT("Available");
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
