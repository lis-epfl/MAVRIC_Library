/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file gps_ublox.c
 *
 * This file decodes the messages from the UBLOX GPS
 */


#include "gps_ublox.h"

#include "central_data.h"
#include "print_util.h"
#include "buffer.h"

central_data_t *centralData;


unsigned char **ubx_currentMessage = 0;		///<  The pointer to the pointer to the structure of the current message to fill
unsigned char ** ubx_lastMessage = 0;		///<  The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
unsigned short * ubx_validMessage = 0;		///<  The pointer to the number to increment when a message of the type has been received

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
ubx_nav_posllh ubx_posllhMessage[2];			///<  The Posllh message buffer
ubx_nav_status ubx_statusMessage[2];			///<  The Status message buffer
ubx_nav_solution ubx_solutionMessage[2];		///<  The Solution message buffer
ubx_nav_velned ubx_velnedMessage[2];			///<  The Velned message buffer
ubx_nav_SVInfo ubx_svInfoMessage[2];			///<  The SVInfo message buffer
ubx_cfg_nav_settings ubx_NavSettingsMessage[2]; ///<  The Nav Settings message buffer
ubx_cfg_nav_rate ubx_CFGRateMessage[2];			///<  The CFG Rate message buffer
ubx_cfg_msg_rate ubx_CFGSetGetRateMessage[2];	///<  The CFG Set/get Rate message buffer
ubx_mon_rxr_struct ubx_MONRXRMessage[2];		///<  The MON RXR message buffer
ubx_tim_tp ubx_TimTPMessage[2];					///<  The TIM TP message buffer
ubx_tim_vrfy ubx_TimVRFYMessage[2];				///<  The TIM VRFY message buffer

// NAV-POSLLH
ubx_nav_posllh * ubx_currentPosllhMessage = &ubx_posllhMessage[0];	///<  The pointer to the Posllh message that is being filled (not usable)
ubx_nav_posllh * ubx_lastPosllhMessage = &ubx_posllhMessage[1];		///<  The pointer to the last Posllh message that was completed
unsigned short ubx_numberOfValidPosllhMessage = 0;					///<  Number of valid Posllh message received

// NAV-STATUS
ubx_nav_status *ubx_currentStatusMessage = &ubx_statusMessage[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_status *ubx_lastStatusMessage = &ubx_statusMessage[1];		///<  The pointer to the last Status message that was completed
unsigned short ubx_numberOfValidStatusMessage = 0;					///<  Number of valid Status message received

// NAV-Sol
ubx_nav_solution *ubx_currentSolutionMessage = &ubx_solutionMessage[0]; ///<  The pointer to the Solution message that is being filled (not usable)
ubx_nav_solution *ubx_lastSolutionMessage = &ubx_solutionMessage[1];	///<  The pointer to the last Status message that was completed
unsigned short ubx_numberOfValidSolutionMessage = 0;					///<  Number of valid Status message received

// NAV-VELNED
ubx_nav_velned *ubx_currentVelnedMessage = &ubx_velnedMessage[0];	///<  The pointer to the Velned message that is being filled (not usable)
ubx_nav_velned *ubx_lastVelnedMessage = &ubx_velnedMessage[1];		///<  The pointer to the last Velned message that was completed
unsigned short ubx_numberOfValidVelnedMessage = 0;					///<  Number of valid Velned message received

// NAV-SVINFO
ubx_nav_SVInfo *ubx_currentSVInfoMessage = &ubx_svInfoMessage[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_SVInfo *ubx_lastSVInfoMessage = &ubx_svInfoMessage[1];		///<  The pointer to the last Status message that was completed
unsigned short ubx_numberOfValidSVInfoMessage = 0;					///<  Number of valid Status message received

// NAV-Settings
ubx_cfg_nav_settings *ubx_currentNavSettingsMessage = &ubx_NavSettingsMessage[0];	///<  The pointer to the Nav Settings message that is being filled (not usable)
ubx_cfg_nav_settings *ubx_lastNavSettingsMessage = &ubx_NavSettingsMessage[1];		///<  The pointer to the last Nav Settings message that was completed
unsigned short ubx_numberOfValidNavSettingsMessage = 0;								///<  Number of valid Nav Settings message received

// CFG message rate
ubx_cfg_nav_rate *ubx_currentCFGRateMessage = &ubx_CFGRateMessage[0];	///<  The pointer to the CFG Rate message that is being filled (not usable)
ubx_cfg_nav_rate *ubx_lastCFGRateMessage = &ubx_CFGRateMessage[1];		///<  The pointer to the last CFG Rate message that was completed
unsigned short ubx_numberOfValidCFGRateMessage = 0;						///<  Number of valid CFG Rate message received

// CFG Set/Get message rate
ubx_cfg_msg_rate *ubx_currentCFGSetGetRateMessage = &ubx_CFGSetGetRateMessage[0];	///<  The pointer to the CFG Set/get Rate message that is being filled (not usable)
ubx_cfg_msg_rate *ubx_lastCFGSetGetRateMessage = &ubx_CFGSetGetRateMessage[1];		///<  The pointer to the last CFG Set/get Rate message that was completed
unsigned short ubx_numberOfValidCFGSetGetRateMessage = 0;							///<  Number of valid CFG Set/get Rate message received

// MON RXR message
ubx_mon_rxr_struct *ubx_currentMONRXRMessage = &ubx_MONRXRMessage[0];	///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_mon_rxr_struct *ubx_lastMONRXRMessage = &ubx_MONRXRMessage[1];		///<  The pointer to the last MON RXR message that was completed
unsigned short ubx_numberOfValidMONRXRMessage = 0;						///<  Number of valid MON RXR message received

// TIM TP message
ubx_tim_tp *ubx_currentTimTPMessage = &ubx_TimTPMessage[0];		///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_tim_tp *ubx_lastTimTPMessage = &ubx_TimTPMessage[1];		///<  The pointer to the last TIM TP message that was completed
unsigned short ubx_numberOfValidTimTPMessage = 0;				///<  Number of valid TIM TP message received


// TIM VRFY message
ubx_tim_vrfy *ubx_currentTimVRFYMessage = &ubx_TimVRFYMessage[0];	///<  The pointer to the TIM VRFY message that is being filled (not usable)
ubx_tim_vrfy *ubx_lastTimVRFYMessage = &ubx_TimVRFYMessage[1];		///<  The pointer to the last TIM VRFY message that was completed
unsigned short ubx_numberOfValidTimVRFYMessage = 0;					///<  Number of valid TIM VRFY message received

// Set to true to print all data
bool printNavOnDebug = false;

uint8_t loopPosllh = 0, loopVelned = 0, loopStatus = 0, loopSolution = 0, loopTimTp = 0, loopTimVrfy = 0;
uint8_t numSkippedMsg = 10;

/**
 * \brief	Checksum update
 *
 * \param	data	pointer to the data to update the checksum
 * \param	len		length of the data to update the checksum
 * \param	ck_a	checksum a: sum of all the data
 * \param	ck_b	checksum b: sum of checksum a
 *
 * \return	true if new velocity and new position message
 */
void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);

/**
 * \brief	To send the lower bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint16 uint
 */
uint8_t endian_lower_bytes_uint16(uint16_t bytes);

/**
 * \brief	To send the higher bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint16 uint
 */
uint8_t endian_higher_bytes_uint16(uint16_t bytes);

/**
 * \brief	To send the lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint32_t uint
 */
uint8_t endian_lower_bytes_uint32(uint32_t bytes);

/**
 * \brief	To send the mid lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the mid lower 8 bytes of the uint32_t uint
 */
uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes);

/**
 * \brief	To send the mid higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the mid higher 8 bytes of the uint32_t uint
 */
uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes);

/**
 * \brief	To send the higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint32_t uint
 */
uint8_t endian_higher_bytes_uint32(uint32_t bytes);

/**
 * \brief	To send the UBX header of all messages
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	_msg_id		the U-Blox message ID
 * \param	size		the size of the U-Blox following message
 */
void ubx_send_header(uint8_t msg_class, uint8_t _msg_id, uint16_t size);

/**
 * \brief	To send the checksum of every message
 *
 * \param	ck_sum_a	the checksum a
 * \param	ck_sum_b	the checksum b
 */
void ubx_send_cksum(uint8_t ck_sum_a, uint8_t ck_sum_b);

/**
 * \brief	To send a CFG NAV RATE message
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x08	MSG_CFG_RATE
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	_msg_id		the U-Blox message ID
 * \param	msg			the CFG_NAV_RATE message
 * \param	size		the size of the U-Blox following message
 */
void ubx_send_message_CFG_nav_rate(uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_rate_send msg, uint16_t size);

/**
 * \brief	To send the NAV settings message
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x24	MSG_CFG_NAV_SETTINGS
 *
 *
 * \warning	This function sends wrong element
 *
 * \param	msg_class			the U-Blox class of the message
 * \param	_msg_id				the U-Blox message ID
 * \param	engine_settings		the engine_settings sent
 * \param	size				the size of the U-Blox following message
 */
void ubx_send_message_nav_settings(uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_settings *engine_settings, uint16_t size);

/**
 * \brief	To send the NAV messages that we want to receive
 *
 * Class:	0x06	UBX_CLASS_CFG
 * Msg_id:	0x01	MSG_CFG_SET_RATE
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	msg_id		the U-Blox message ID
 * \param	rate		the rate of the CFG message
 */
void ubx_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

/**
 * \brief	This function returns a pointer to the last NAV-POSLLH message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid posllh message, or 0.
 */
ubx_nav_posllh * ubx_GetPosllh(void);

/**
 * \brief	This function returns a pointer to the last NAV-STATUS message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid status message, or 0.
 */
ubx_nav_status * ubx_GetStatus(void);
/**
 * \brief	This function returns a pointer to the last NAV-SOL message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid NAV-SOL message, or 0.
 */
ubx_nav_solution * ubx_GetSolution(void);

/**
* \brief	This function returns a pointer to the last NAV-VELNED message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid velned message, or 0.
*/
ubx_nav_velned * ubx_GetVelned(void);

/**
* \brief	This function returns a pointer to the last NAV-SVINFO message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_nav_SVInfo * ubx_GetSVInfo(void);

/**
* \brief	This function returns a pointer to the last NAV-Settings message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_cfg_nav_settings * ubx_GetNavSettings(void);

/**
* \brief	This function returns a pointer to the last CFG set/get rate message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_cfg_msg_rate * ubx_GetMsgRate(void);

/**
* \brief	This function returns a pointer to the last MON RXR message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_mon_rxr_struct * ubx_GetMonRXR(void);

/**
* \brief	This function returns a pointer to the last TIM TP message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_tim_tp * ubx_GetTimTP(void);

/**
* \brief	This function returns a pointer to the last TIM VRFY message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
ubx_tim_vrfy * ubx_GetTimVRFY(void);

/**
* \brief	This function transforms a float angle in degree in a float angle in radians
*
* \return	The value in radians
*/
float ToRad(float numdeg);

void gps_ublox_init(GPS_Engine_Setting _engine_nav_setting)
{
	centralData = central_data_get_pointer_to_struct();
	
	// uint8_t epoch = TIME_OF_WEEK;
	idleTimeout = 1200;
	
	gps_ublox_configure_gps();
	
	engine_nav_setting = _engine_nav_setting;
	
	centralData->GPS_data.status = NO_FIX;
	centralData->GPS_data.num_sats = 0;
	
	next_fix = false;
	have_raw_velocity = false;
	
	last_fix_time = 0;
	
	new_position = false;
	new_speed = false;
	
	step = 0;
}

bool gps_ublox_message_decode(void)
{
	uint8_t data;
	bool msg_ok = false;
	
	unsigned char * temporaryMessageForSwaping;
	
	while(buffer_bytes_available(&(centralData->gps_buffer)))
	{
		data = buffer_get(&(centralData->gps_buffer));
		reset:
		
		switch (step)
		{
			// Message preamble detection
			//
			// If we fail to match any of the expected bytes, we reset
			// the state machine and re-consider the failed byte as
			// the first byte of the preamble.  This improves our
			// chances of recovering from a mismatch and makes it less
			// likely that we will be fooled by the preamble appearing
			// as data in some other message.
			//
		case 1:
			if (data == UBX_PREAMBLE2)
			{
				step++;
				break;
			}
			step = 0;
		case 0:
			if (data == UBX_PREAMBLE1)
			{
				step++;
				break;
			}
			step = 0;
			break;
		// Message header processing
		//
		// We sniff the class and message ID to decide whether we
		// are going to gather the message bytes or just discard
		// them.
		//
		// We always collect the length so that we can avoid being
		// fooled by preamble bytes in messages.
		//
		case 2:
			step++;
			ubxclass = data;
			cksum_a = data;
			cksum_b = cksum_a; // reset the checksum accumulators
			break;
			
		case 3:
			step++;
			cksum_a += data;
			cksum_b += cksum_a; // checksum byte
			msg_id = data;
			break;
			
		case 4:
			step++;
			cksum_a += data;
			cksum_b += cksum_a; // checksum byte
			payload_length = data;
			break;
			
		case 5:
			step++;
			payload_length |= data<<8;
			cksum_a += data;
			cksum_b += cksum_a; // checksum byte
			
			if (payload_length > 512)
			{
				// we assume very large payloads are line noise
				dbg_print("large payload: ");
				dbg_print_num(payload_length,10);
				dbg_print("\n");
				payload_length = 0;
				step = 0;
				goto reset;
			}
			payload_counter = 0; // prepare to receive payload
			
			if(ubxclass == UBX_CLASS_NAV)
			{
				switch(msg_id)
				{
				case MSG_NAV_POSLLH:
					if(payload_length == UBX_SIZE_NAV_POSLLH)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentPosllhMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastPosllhMessage;
						ubx_validMessage = &ubx_numberOfValidPosllhMessage;
					}
					else
					{
						dbg_print("Wrong Posllh message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_POSLLH,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_NAV_STATUS:
					if(payload_length == UBX_SIZE_NAV_STATUS)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentStatusMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastStatusMessage;
						ubx_validMessage = &ubx_numberOfValidStatusMessage;
					}
					else
					{
						dbg_print("Wrong Nav Status message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_STATUS,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_NAV_SOL:
					if(payload_length == UBX_SIZE_NAV_SOL)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentSolutionMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastSolutionMessage;
						ubx_validMessage = &ubx_numberOfValidSolutionMessage;;
					}
					else
					{
						dbg_print("Wrong Solution message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_SOL,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_NAV_VELNED:
					if(payload_length == UBX_SIZE_NAV_VELNED)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentVelnedMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastVelnedMessage;
						ubx_validMessage = &ubx_numberOfValidVelnedMessage;
					}
					else
					{
						dbg_print("Wrong Velned message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_VELNED,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_NAV_SVINFO:
					if(payload_length == UBX_SIZE_NAV_SVINFO)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentSVInfoMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastSVInfoMessage;
						ubx_validMessage = &ubx_numberOfValidSVInfoMessage;
					}
					else
					{
						dbg_print("Wrong SV Info message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_SVINFO,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				default:
					step = 0;
					dbg_print("Unexpected NAV message, Class: 0x");
					dbg_print_num(ubxclass,16);
					dbg_print(", msg id: 0x");
					dbg_print_num(msg_id,16);
					dbg_print(" of size ");
					dbg_print_num(payload_length,10);
					dbg_print("\n");
					goto reset;
				}	
			}
			else if(ubxclass == UBX_CLASS_CFG)
			{
				
				switch(msg_id)
				{			
				case MSG_CFG_NAV_SETTINGS:
					if(payload_length == UBX_SIZE_NAV_SETTINGS)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentNavSettingsMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastNavSettingsMessage;
						ubx_validMessage = &ubx_numberOfValidNavSettingsMessage;
					}
					else
					{
						dbg_print("Wrong Nav Settings message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_NAV_SETTINGS,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
						
				case MSG_CFG_RATE:
					if(payload_length == UBX_SIZE_CFG_RATE)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentCFGRateMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastCFGRateMessage;
						ubx_validMessage = &ubx_numberOfValidCFGRateMessage;
					}
					else
					{
						dbg_print("Wrong CFG Rate message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_CFG_RATE,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_CFG_SET_RATE:
					if (payload_length == UBX_SIZE_CFG_GETSET_RATE)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentCFGSetGetRateMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastCFGSetGetRateMessage;
						ubx_validMessage = &ubx_numberOfValidCFGSetGetRateMessage;
					}
					else
					{
						dbg_print("Wrong CFG Set/get message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_CFG_GETSET_RATE,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				default:
					step = 0;
					dbg_print("Unexpected CFG message, Class: 0x");
					dbg_print_num(ubxclass,16);
					dbg_print(", msg id: 0x");
					dbg_print_num(msg_id,16);
					dbg_print(" of size ");
					dbg_print_num(payload_length,10);
					dbg_print("\n");
					goto reset;
				}
			} else if (ubxclass == UBX_CLASS_MON)
			{
 				switch (msg_id)
 				{
 				case MSG_MON_RXR:
 					if(payload_length == UBX_SIZE_MON_RXR)
 					{
 						ubx_currentMessage = (unsigned char**)&ubx_currentMONRXRMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastMONRXRMessage;
						ubx_validMessage = &ubx_numberOfValidMONRXRMessage;
 					}
					 else
					 {
 						dbg_print("Wrong MON RXR message 0x");
 						dbg_print_num(ubxclass,16);
 						dbg_print(" Msg id: 0x");
 						dbg_print_num(msg_id,16);
 						dbg_print(" Received size:");
 						dbg_print_num(payload_length,10);
 						dbg_print(" should be:");
 						dbg_print_num(UBX_SIZE_MON_RXR,10);
 						dbg_print("\n");
 						step = 0;
 						goto reset;
 					}
					break;
					
 				default:
 					step = 0;
 					dbg_print("Unexpected TIM message, Class: 0x");
 					dbg_print_num(ubxclass,16);
 					dbg_print(", msg id: 0x");
 					dbg_print_num(msg_id,16);
 					dbg_print(" of size ");
 					dbg_print_num(payload_length,10);
					dbg_print(" should be :");
					dbg_print_num(MSG_MON_RXR,16);
 					dbg_print("\n");
 					goto reset;
 				}
				
			}
			else if(ubxclass == UBX_CLASS_TIM)
			{
				switch(msg_id)
				{
				case MSG_TIM_TP:
					if (payload_length == UBX_SIZE_TIM_TP)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentTimTPMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastTimTPMessage;
						ubx_validMessage = &ubx_numberOfValidTimTPMessage;
					}
					else
					{
						dbg_print("Wrong TIM TP message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_TIM_TP,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				case MSG_TIM_VRFY:
					if (payload_length == UBX_SIZE_TIM_VRFY)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentTimVRFYMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastTimVRFYMessage;
						ubx_validMessage = &ubx_numberOfValidTimVRFYMessage;
						}
						else
						{
						dbg_print("Wrong TIM VRFY message 0x");
						dbg_print_num(ubxclass,16);
						dbg_print(" Msg id: 0x");
						dbg_print_num(msg_id,16);
						dbg_print(" Received size:");
						dbg_print_num(payload_length,10);
						dbg_print(" should be:");
						dbg_print_num(UBX_SIZE_TIM_VRFY,10);
						dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
				default:
					step = 0;
					dbg_print("Unexpected TIM message, Class: 0x");
					dbg_print_num(ubxclass,16);
					dbg_print(", msg id: 0x");
					dbg_print_num(msg_id,16);
					dbg_print(" of size ");
					dbg_print_num(payload_length,10);
					dbg_print(" should be :");
					dbg_print_num(MSG_TIM_TP,16);
					dbg_print("\n");
					goto reset;
				}
			}
			else
			{
				step = 0;
				dbg_print("Unexpected message, Class: 0x");
				dbg_print_num(ubxclass,16);
				dbg_print(", msg id: 0x");
				dbg_print_num(msg_id,16);
				dbg_print(" of size ");
				dbg_print_num(payload_length,10);
				dbg_print("\n");
				goto reset;	
			}		
			break;
			
		case 6:
			cksum_a += data;
			cksum_b += cksum_a; // checksum byte
			
			#ifdef BIG_ENDIAN
				(*ubx_currentMessage)[payload_length - 1 - payload_counter] = data;
			#else
				(*ubx_currentMessage)[payload_counter] = data;
			#endif
			
			payload_counter++;
			
			if (payload_counter == payload_length)
			{
				step++;
			}
			break;
			
		case 7:
			step++;
			if (cksum_a != data)
			{
				dbg_print("bad cksum_a ");
				dbg_print_num(data,16);
				dbg_print(" should be ");
				dbg_print_num(cksum_a,16);
				dbg_print(" class : 0x");
				dbg_print_num(ubxclass,16);
				dbg_print(" msg_id : 0x");
				dbg_print_num(msg_id,16);
				dbg_print("\n");
				step = 0;
				goto reset;
			}
			break;
			
		case 8:
			step=0;
			if (cksum_b != data)
			{
				dbg_print("bad cksum_b ");
				dbg_print_num(data,16);
				dbg_print(" should be ");
				dbg_print_num(cksum_b,16);
				dbg_print("\n");
				break;
			}
			++(*ubx_validMessage);
			//dbg_print("Valid message");
			
			// swap message buffers, old message is discarded and becomes incoming buffer, new message become valid message (=old)
			temporaryMessageForSwaping = *ubx_currentMessage;
			*ubx_currentMessage = *ubx_lastMessage;
			*ubx_lastMessage = temporaryMessageForSwaping;
			
			if (gps_ublox_process_data())
			{
				msg_ok = true;
			}
		}
	}
	return msg_ok;
}

bool gps_ublox_process_data(void)
{
	ubx_nav_posllh *gpsPosllh; 
	ubx_nav_status *gpsStatus;
	ubx_nav_solution *gpsSolution;
	ubx_nav_velned *gpsVelned;
	ubx_nav_SVInfo *gpsSVInfo;
	
	if (ubxclass == UBX_CLASS_ACK)
	{
		dbg_print_num(msg_id,10);
		dbg_print("\n");
		return false;
	}
 	if (ubxclass == UBX_CLASS_MON)
 	{
 		ubx_mon_rxr_struct *gpsRXR = ubx_GetMonRXR();
 		if (gpsRXR)
 		{
 			dbg_print("MSG_MON GPS awake\n");
 		}
		 return false;
 	}
	if (ubxclass == UBX_CLASS_TIM)
	{
		ubx_tim_tp *gpsTimTP = ubx_GetTimTP();
		if (gpsTimTP)
		{
			loopTimTp = ++loopTimTp % numSkippedMsg;
			if((printNavOnDebug)&&(loopTimTp == 0))
			{
				dbg_print("MSG_TIM_TP GPS awake\n");
			}
		}
		ubx_tim_vrfy *gpsTimVrfy = ubx_GetTimVRFY();
		if (gpsTimVrfy)
		{
			loopTimVrfy = ++loopTimVrfy % numSkippedMsg;
			if((printNavOnDebug)&&(loopTimVrfy == 0))
			{
				dbg_print("MSG_TIM_VRFY");
				dbg_print(" itow :");
				dbg_print_num(gpsTimVrfy->itow,10);
				dbg_print(" frac :");
				dbg_print_num(gpsTimVrfy->frac,10);
				dbg_print(" deltaMs :");
				dbg_print_num(gpsTimVrfy->deltaMs,10);
				dbg_print(" deltaNs :");
				dbg_print_num(gpsTimVrfy->deltaNs,10);
			}
		}
		return false;
	}
	 
	if (ubxclass == UBX_CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS)
	{
		ubx_cfg_nav_settings *gpsNavSettings = ubx_GetNavSettings();
		
		/*
		Dynamic Platform model:
		- 0 Portable
		- 2 Stationary
		- 3 Pedestrian
		- 4 Automotive
		- 5 Sea
		- 6 Airborne with <1g Acceleration
		- 7 Airborne with <2g Acceleration
		- 8 Airborne with <4g Acceleration
		*/
		if(gpsNavSettings)
		{
			dbg_print("Got engine settings ");
			dbg_print_num(gpsNavSettings->dynModel,16);
			dbg_print("\n");
		}
		else
		{
			if (engine_nav_setting != GPS_ENGINE_NONE && !gpsNavSettings)
			{
				if(gpsNavSettings->dynModel != engine_nav_setting)
				{
					nav_settings.dynModel = engine_nav_setting;
					dbg_print("Send Nav settings");
					ubx_send_message_nav_settings(UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, &nav_settings, sizeof(nav_settings));
				}				
			}
			dbg_print("No engine settings received ");
			dbg_print_num(msg_id,16);
			dbg_print("\n");
		}
		return false;
	}
	
	if (ubxclass == UBX_CLASS_CFG && msg_id == MSG_CFG_SET_RATE)
	{
		ubx_cfg_msg_rate *gpsMsgRate;
		gpsMsgRate = ubx_GetMsgRate();
		
		if (gpsMsgRate)
		{
			dbg_print("Message CFG Rate 0x");
			dbg_print_num(gpsMsgRate->msg_class,16);
			dbg_print_num(gpsMsgRate->msg_id_rate,16);
			dbg_print_num(gpsMsgRate->rate,10);
			dbg_print("\n");
		}		
		return false;
	}
	
	if (ubxclass != UBX_CLASS_NAV)
	{
		dbg_print("Unexpected message 0x");
		dbg_print_num(ubxclass,16);
		dbg_print("02x 0x");
		dbg_print_num(msg_id,10);
		dbg_print("02x\n");
		if (++disable_counter == 0)
		{
			// disable future sends of this message id, but
			// only do this every 256 messages, as some
			// message types can't be disabled and we don't
			// want to get into an ack war
			
			dbg_print("Disabling message 0x");
			dbg_print_num(ubxclass,16);
			dbg_print("02x 0x");
			dbg_print_num(msg_id,16);
			dbg_print("02x\n");
			ubx_configure_message_rate(ubxclass, msg_id, 0);
		}
		return false;
	}
	
	// Class NAV:
	switch (msg_id)
	{
	case MSG_NAV_POSLLH:
		gpsPosllh = ubx_GetPosllh();
		if (gpsPosllh)
		{
			loopPosllh = ++loopPosllh % numSkippedMsg;
			if (printNavOnDebug && (loopPosllh == 0))
			{
				dbg_print("MSG_NAV_POSLLH");
				dbg_print(" itow :");
				dbg_print_num(gpsPosllh->itow,10);
				dbg_print(" longitude :");
				dbg_print_num(gpsPosllh->longitude,10);
				dbg_print(" latitude :");
				dbg_print_num(gpsPosllh->latitude,10);
				dbg_print(" alt_ellips :");
				dbg_print_num(gpsPosllh->altitude_ellipsoid,10);
				dbg_print(" alt_msl :");
				dbg_print_num(gpsPosllh->altitude_msl,10);
				dbg_print(" horz_acc :");
				dbg_print_num(gpsPosllh->horizontal_accuracy,10);
				dbg_print(" vert_acc :");
				dbg_print_num(gpsPosllh->vertical_accuracy,10);
				dbg_print("\n");
			}
			
			centralData->GPS_data.timegps = gpsPosllh->itow;
			centralData->GPS_data.longitude = gpsPosllh->longitude / 10000000.0;
			centralData->GPS_data.latitude = gpsPosllh->latitude / 10000000.0;
			centralData->GPS_data.alt_elips = ((float)gpsPosllh->altitude_ellipsoid) / 1000.0;
			centralData->GPS_data.altitude = ((float)gpsPosllh->altitude_msl) / 1000.0;
			centralData->GPS_data.horizontalAccuracy = ((float)gpsPosllh->horizontal_accuracy) / 1000.0;
			centralData->GPS_data.verticalAccuracy = ((float)gpsPosllh->vertical_accuracy) / 1000.0;
			
			new_position = true;
		}
		break;
		
	case MSG_NAV_STATUS:
		gpsStatus = ubx_GetStatus();
		
		if (gpsStatus)
		{
			loopStatus = ++loopStatus % numSkippedMsg;
			if (printNavOnDebug && (loopStatus == 0))
			{
				dbg_print("MSG_STATUS fix_type = 0x");
				dbg_print_num(gpsStatus->fix_type,16);
				dbg_print(", uptime =");
				dbg_print_num(gpsStatus->uptime,10);
				dbg_print("\n");
			}
			next_fix = (gpsStatus->fix_type == GPS_FIX_TYPE_3DFIX);
			if (!next_fix)
			{
				centralData->GPS_data.status = NO_FIX;
			}
			else
			{
				centralData->GPS_data.status = GPS_OK;
			}
		}
		break;
		
	case MSG_NAV_SOL:
		gpsSolution = ubx_GetSolution();
		
		if (gpsSolution)
		{
			loopSolution = ++loopSolution % numSkippedMsg;
			if (printNavOnDebug && (loopSolution == 0))
			{
				dbg_print("MSG_SOL ");
				dbg_print("itow :");
				dbg_print_num(gpsSolution->itow,10);
				dbg_print(" week :");
				dbg_print_num(gpsSolution->week,10);
				dbg_print(" fix_type = 0x0");
				dbg_print_num(gpsSolution->fix_type,16);
				dbg_print(" pos_acc_3d :");
				dbg_print_num(gpsSolution->position_accuracy_3d,10);
				dbg_print(" ecefx :");
				dbg_print_num(gpsSolution->ecef_x,10);
				dbg_print(" ecefy :");
				dbg_print_num(gpsSolution->ecef_y,10);
				dbg_print(" ecefz :");
				dbg_print_num(gpsSolution->ecef_z,10);
				dbg_print(" pos_DOP :");
				dbg_print_num(gpsSolution->position_DOP,10);
				dbg_print(" num sat :");
				dbg_print_num(gpsSolution->satellites,10);
				dbg_print("\n");
			}
			next_fix = (gpsSolution->fix_type == GPS_FIX_TYPE_3DFIX);
			if (!next_fix)
			{
				centralData->GPS_data.status = NO_FIX;
			}
			else
			{
				centralData->GPS_data.status = GPS_OK;
			}
		
			centralData->GPS_data.num_sats = gpsSolution->satellites;
			centralData->GPS_data.hdop = gpsSolution->position_DOP;
		}
		break;
		
	case MSG_NAV_VELNED:
		gpsVelned = ubx_GetVelned();
		
		if (gpsVelned)
		{
			loopVelned = ++loopVelned % numSkippedMsg;
			if (printNavOnDebug && (loopVelned == 0))
			{
			
				dbg_print("MSG_NAV_VELNED ");
		
				dbg_print("itow :");
				dbg_print_num(gpsVelned->itow,10);
				dbg_print(" ned_north :");
				dbg_print_num(gpsVelned->ned_north,10);
				dbg_print(" ned_east :");
				dbg_print_num(gpsVelned->ned_east,10);
				dbg_print(" ned_down :");
				dbg_print_num(gpsVelned->ned_down,10);
				dbg_print(" speed_3d :");
				dbg_print_num(gpsVelned->speed_3d,10);
				dbg_print(" heading_2d :");
				dbg_print_num(gpsVelned->heading_2d,10);
				dbg_print(" speed_accuracy :");
				dbg_print_num(gpsVelned->speed_accuracy,10);
				dbg_print(" heading_accuracy :");
				dbg_print_num(gpsVelned->heading_accuracy,10);
				dbg_print("\n");
			}
			centralData->GPS_data.timegps = gpsVelned->itow;
			centralData->GPS_data.speed        = ((float)gpsVelned->speed_3d)/100.; // m/s
			centralData->GPS_data.groundSpeed = ((float)gpsVelned->groundSpeed_2d) / 100.; // m/s
			centralData->GPS_data.course = ((float)gpsVelned->heading_2d) / 100000.; // Heading 2D deg * 100000 rescaled to deg * 100
			have_raw_velocity = true;
			centralData->GPS_data.northSpeed  = ((float)gpsVelned->ned_north) / 100.0;
			centralData->GPS_data.eastSpeed   = ((float)gpsVelned->ned_east) / 100.;
			centralData->GPS_data.verticalSpeed   = ((float)gpsVelned->ned_down) / 100.;
			centralData->GPS_data.speedAccuracy = ((float)gpsVelned->speed_accuracy)/100.;
			centralData->GPS_data.headingAccuracy = gpsVelned->heading_accuracy;
			new_speed = true;
		}
		break;
		
	case MSG_NAV_SVINFO:
		gpsSVInfo = ubx_GetSVInfo();
		
		if (gpsSVInfo)
		{
			if (printNavOnDebug)
			{
				dbg_print("MSG_NAV_SVINFO, numChannel:");
				dbg_print_num(gpsSVInfo->numCh,10);
				dbg_print("\n");
			}
		}
		
	default:
		dbg_print("Unexpected NAV message 0x");
		dbg_print_num(msg_id,10);
		dbg_print("\n");
		
		if (++disable_counter == 0)
		{
			dbg_print("Disabling NAV message 0x");
			dbg_print_num(msg_id,16);
			dbg_print("\n");
			ubx_configure_message_rate(UBX_CLASS_NAV, msg_id, 0);
		}
		return false;
	}
	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (new_position && new_speed)
	{
		new_speed = false;
		new_position = false;
		return true;
	}
	return false;
}

void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	while (len--)
	{
		*ck_a += *data;
		*ck_b += *ck_a;
		data++;
	}
}

uint8_t endian_lower_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0x00FF);
}

uint8_t endian_higher_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0xFF00)>>8;
}

uint8_t endian_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x000000FF);
}

uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x0000FF00)>>8;
}

uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x00FF0000)>>16;
}

uint8_t endian_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0xFF000000)>>24;
}

void ubx_send_header(uint8_t msg_class, uint8_t _msg_id, uint16_t size)
{
	ubx_header header;
	header.preamble1		= UBX_PREAMBLE1;
	header.preamble2		= UBX_PREAMBLE2;
	header.msg_class		= msg_class;
	header.msg_id_header    = _msg_id;
	header.length			= size;
	
	putnum(&centralData->gps_stream_out,header.preamble1,16);
	putnum(&centralData->gps_stream_out,header.preamble2,16);
	putnum(&centralData->gps_stream_out,header.msg_class,10);
	putnum(&centralData->gps_stream_out,header.msg_id_header,16);
	
	putnum(&centralData->gps_stream_out,(uint8_t) endian_lower_bytes_uint16(header.length),16);
	putnum(&centralData->gps_stream_out,(uint8_t) endian_higher_bytes_uint16(header.length),16);
	
}

void ubx_send_cksum(uint8_t ck_sum_a, uint8_t ck_sum_b)
{
	putnum(&centralData->gps_stream_out,ck_sum_a,16);
	putnum(&centralData->gps_stream_out,ck_sum_b,16);
}

void ubx_send_message_CFG_nav_rate(uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_rate_send msg, uint16_t size)
{
	uint8_t ck_a=0, ck_b=0;
	
	uint8_t data;
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	// update_checksum((uint8_t *)&msg, size, &ck_a, &ck_b); Wrong!
	
	ubx_send_header(msg_class,_msg_id,size);
	
	data = endian_lower_bytes_uint16(msg.measure_rate_ms);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.measure_rate_ms);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	data = endian_lower_bytes_uint16(msg.nav_rate);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.nav_rate);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	data = endian_lower_bytes_uint16(msg.timeref);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.timeref);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	putnum(&centralData->gps_stream_out, data, 16);
	
	ubx_send_cksum(ck_a,ck_b);
}

void ubx_send_message_nav_settings(uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_settings *engine_settings, uint16_t size)
{
	uint8_t ck_a=0, ck_b=0;
	uint8_t data;
	
	ubx_send_header(msg_class,_msg_id,size);
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	if (engine_settings != NULL)
	{
		//update_checksum((uint8_t *)engine_settings, size, &ck_a, &ck_b);
		
		data = endian_lower_bytes_uint16(engine_settings->mask);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->mask);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->dynModel;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->fixMode;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->minElev;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->drLimit;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->pDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->pDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->tDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->tDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->pAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->pAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->tAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->tAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->staticHoldThresh;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = engine_settings->dgpsTimeOut;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		putnum(&centralData->gps_stream_out, data, 16);
	}
	
	ubx_send_cksum(ck_a,ck_b);
}

void ubx_configure_message_rate(uint8_t msg_class, uint8_t _msg_id, uint8_t rate)
{
	uint8_t ck_a = 0, ck_b = 0;
	ubx_cfg_msg_rate_send msg;
	msg.msg_class = msg_class;
	msg.msg_id_rate    = _msg_id;
	msg.rate          = rate;
	
	uint8_t size = sizeof(msg);
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	update_checksum((uint8_t *)&msg, size, &ck_a, &ck_b);
	
	ubx_send_header(UBX_CLASS_CFG,MSG_CFG_SET_RATE,sizeof(msg));
	
	putnum(&centralData->gps_stream_out,msg.msg_class,16);
	putnum(&centralData->gps_stream_out,msg.msg_id_rate,16);
	putnum(&centralData->gps_stream_out,msg.rate,16);
	
	ubx_send_cksum(ck_a,ck_b);
}

void gps_ublox_configure_gps(void)
{
	ubx_cfg_nav_rate_send msg;
	// const unsigned baudrates[4] = {9600U, 19200U, 38400U, 57600U};

	const char *set_binary = UBLOX_SET_BINARY;
	// the GPS may be setup for a different baud rate. This ensures
	// it gets configured correctly
// 	for (uint8_t i=0; i<4; i++)
// 	{
	
	//dbg_print("Set to binary mode ");
	//dbg_print(set_binary);
	putstring(&(centralData->gps_stream_out),set_binary);
	//centralData->gps_stream_out.put(centralData->gps_stream_out.data,set_binary);
	//centralData->gps_stream_out.flush(&(centralData->gps_stream_out.data));
	//}

	// ask for navigation solutions every 200ms
	msg.measure_rate_ms = 200;		// ms
	msg.nav_rate        = 1;		// constant equal to 1
	msg.timeref         = 0;		// 0:UTC time, 1:GPS time
	
	ubx_send_message_CFG_nav_rate(UBX_CLASS_CFG, MSG_CFG_RATE, msg, sizeof(msg));

	// ask for the messages we parse to be sent on every navigation solution
	//dbg_print("Set navigation messages\n");
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_POSLLH, 4);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_STATUS, 4);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_SOL, 4);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_VELNED, 4);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_SVINFO, 4);

	// ask for the current navigation settings
	//dbg_print("Asking for engine setting\n");
	ubx_send_message_nav_settings(UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}

void gps_ublox_update(void)
{
	bool result;
	uint32_t tnow;
	
	result = gps_ublox_message_decode();
	
	tnow = get_millis();
	
	if (! result)
	{
		if ((tnow - idleTimer) > idleTimeout)
		{
			centralData->GPS_data.status = NO_GPS;
			
			gps_ublox_init(engine_nav_setting);
			idleTimer = tnow;
		}
		
	}
	else
	{
		// reset the idle timer
		idleTimer = tnow;
		
		centralData->GPS_data.timeLastMsg = tnow;
		
		if(centralData->GPS_data.status == GPS_OK)
		{
			// Check for horizontal accuracy
			if (centralData->GPS_data.horizontalAccuracy < UBX_POSITION_PRECISION)
			{
				centralData->GPS_data.horizontalStatus = 1;
			}
			else
			{  
				centralData->GPS_data.horizontalStatus = 0;
			}
			// Check for vertical accuracy
			if (centralData->GPS_data.verticalAccuracy < UBX_ALTITUDE_PRECISION)
			{
				centralData->GPS_data.altitudeStatus = 1;
			}
			else
			{
				centralData->GPS_data.altitudeStatus = 0;
			}
			// Check for speed accuracy
			if (centralData->GPS_data.speedAccuracy < UBX_SPEED_PRECISION)
			{
				centralData->GPS_data.speedStatus = 1;
			}
			else
			{
				centralData->GPS_data.speedStatus = 0;
			}
			// Check for heading accuracy
			if (centralData->GPS_data.headingAccuracy < UBX_HEADING_PRECISION)
			{
				centralData->GPS_data.courseStatus = 1;
			}
			else
			{
				centralData->GPS_data.courseStatus = 0;
			}
			
			centralData->GPS_data.accuracyStatus = centralData->GPS_data.horizontalStatus & centralData->GPS_data.altitudeStatus & centralData->GPS_data.speedStatus & centralData->GPS_data.courseStatus;
			
			// speed approximation with the 
// 			if (!have_raw_velocity)
// 			{
// 				float gps_heading = ToRad(centralData->GPS_data.course);
// 				float cos_heading,sin_heading;
// 				
// 				cos_heading = cosf(gps_heading);
// 				sin_heading = sinf(gps_heading);
// 				
// 				centralData->GPS_data.northspeed = centralData->GPS_data.groundSpeed * cos_heading;
// 				centralData->GPS_data.eastspeed = centralData->GPS_data.groundSpeed * sin_heading;
// 				
// 				// no good way to get descent rate
// 				centralData->GPS_data.verticalSpeed = 0;
// 			}
		}
		else
		{
			centralData->GPS_data.horizontalStatus = 0;
			centralData->GPS_data.altitudeStatus = 0;
			centralData->GPS_data.speedStatus = 0;
			centralData->GPS_data.courseStatus = 0;
			
			centralData->GPS_data.accuracyStatus = 0;
		}
	}
}

bool gps_ublox_newValidGpsMsg(uint32_t *prevGpsMsgTime)
{
	
	
	if((*prevGpsMsgTime != centralData->GPS_data.timeLastMsg)&&(centralData->GPS_data.status == GPS_OK)//&&(centralData->GPS_data.accuracyStatus == 1)
	)
	{
		*prevGpsMsgTime = centralData->GPS_data.timeLastMsg;
		return true;
	}
	else
	{
		return false;
	}
}

ubx_nav_posllh * ubx_GetPosllh()
{
	if (ubx_numberOfValidPosllhMessage)
	{
		return ubx_lastPosllhMessage;
	}
	else
	{
		return 0;
	}
}

ubx_nav_status * ubx_GetStatus()
{
	if (ubx_numberOfValidStatusMessage)
	{
		return ubx_lastStatusMessage;
	}
	else
	{
		return 0;
	}
}

ubx_nav_solution * ubx_GetSolution()
{
	if (ubx_numberOfValidSolutionMessage)
	{
		return ubx_lastSolutionMessage;
	}
	else
	{
		return 0;
	}
}

ubx_nav_velned * ubx_GetVelned()
{
	if (ubx_numberOfValidVelnedMessage)
	{
		return ubx_lastVelnedMessage;
	}
	else
	{
		return 0;
	}
}

ubx_nav_SVInfo * ubx_GetSVInfo()
{
	if (ubx_numberOfValidSVInfoMessage)
	{
		return ubx_lastSVInfoMessage;
	}
	else
	{
		return 0;
	}
}

ubx_cfg_nav_settings * ubx_GetNavSettings()
{
	if (ubx_numberOfValidNavSettingsMessage)
	{
		return ubx_lastNavSettingsMessage;
	}
	else
	{
		return 0;
	}
}

ubx_cfg_msg_rate * ubx_GetMsgRate()
{
	if (ubx_numberOfValidCFGSetGetRateMessage)
	{
		return ubx_lastCFGSetGetRateMessage;
	}
	else
	{
		return 0;
	}
}

ubx_mon_rxr_struct * ubx_GetMonRXR()
{
	if (ubx_numberOfValidMONRXRMessage)
	{
		return ubx_lastMONRXRMessage;
	}
	else
	{
		return 0;
	}
}

ubx_tim_tp * ubx_GetTimTP()
{
	if(ubx_numberOfValidTimTPMessage)
	{
		return ubx_lastTimTPMessage;
	}
	else
	{
		return 0;
	}
}

ubx_tim_vrfy * ubx_GetTimVRFY()
{
	if(ubx_numberOfValidTimVRFYMessage)
	{
		return ubx_lastTimVRFYMessage;
	}
	else
	{
		return 0;
	}
}

float ToRad(float numdeg)
{
	return numdeg * DEG2RAD;
}