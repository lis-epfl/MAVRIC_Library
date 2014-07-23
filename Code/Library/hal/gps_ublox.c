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

#include "print_util.h"
#include "uart_int.h"
#include "buffer.h"
#include "time_keeper.h"
#include "mavlink_communication.h"

uint8_t  **ubx_currentMessage = 0;		///<  The pointer to the pointer to the structure of the current message to fill
uint8_t  ** ubx_lastMessage = 0;		///<  The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
uint16_t * ubx_validMessage = 0;		///<  The pointer to the number to increment when a message of the type has been received

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
ubx_nav_posllh_t ubx_posllhMessage[2];			///<  The Posllh message buffer
ubx_nav_status_t ubx_statusMessage[2];			///<  The Status message buffer
ubx_nav_solution_t ubx_solutionMessage[2];		///<  The Solution message buffer
ubx_nav_velned_t ubx_velnedMessage[2];			///<  The Velned message buffer
ubx_nav_SVInfo_t ubx_svInfoMessage[2];			///<  The SVInfo message buffer
ubx_cfg_nav_settings_t ubx_NavSettingsMessage[2]; ///<  The Nav Settings message buffer
ubx_cfg_nav_rate_t ubx_CFGRateMessage[2];			///<  The CFG Rate message buffer
ubx_cfg_msg_rate_t ubx_CFGSetGetRateMessage[2];	///<  The CFG Set/get Rate message buffer
ubx_mon_rxr_struct_t ubx_MONRXRMessage[2];		///<  The MON RXR message buffer
ubx_tim_tp_t ubx_TimTPMessage[2];					///<  The TIM TP message buffer
ubx_tim_vrfy_t ubx_TimVRFYMessage[2];				///<  The TIM VRFY message buffer

// NAV-POSLLH
ubx_nav_posllh_t * ubx_currentPosllhMessage = &ubx_posllhMessage[0];	///<  The pointer to the Posllh message that is being filled (not usable)
ubx_nav_posllh_t * ubx_lastPosllhMessage = &ubx_posllhMessage[1];		///<  The pointer to the last Posllh message that was completed
uint16_t ubx_numberOfValidPosllhMessage = 0;					///<  Number of valid Posllh message received

// NAV-STATUS
ubx_nav_status_t *ubx_currentStatusMessage = &ubx_statusMessage[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_status_t *ubx_lastStatusMessage = &ubx_statusMessage[1];		///<  The pointer to the last Status message that was completed
uint16_t ubx_numberOfValidStatusMessage = 0;					///<  Number of valid Status message received

// NAV-Sol
ubx_nav_solution_t *ubx_currentSolutionMessage = &ubx_solutionMessage[0]; ///<  The pointer to the Solution message that is being filled (not usable)
ubx_nav_solution_t *ubx_lastSolutionMessage = &ubx_solutionMessage[1];	///<  The pointer to the last Status message that was completed
uint16_t ubx_numberOfValidSolutionMessage = 0;					///<  Number of valid Status message received

// NAV-VELNED
ubx_nav_velned_t *ubx_currentVelnedMessage = &ubx_velnedMessage[0];	///<  The pointer to the Velned message that is being filled (not usable)
ubx_nav_velned_t *ubx_lastVelnedMessage = &ubx_velnedMessage[1];		///<  The pointer to the last Velned message that was completed
uint16_t ubx_numberOfValidVelnedMessage = 0;					///<  Number of valid Velned message received

// NAV-SVINFO
ubx_nav_SVInfo_t *ubx_currentSVInfoMessage = &ubx_svInfoMessage[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_SVInfo_t *ubx_lastSVInfoMessage = &ubx_svInfoMessage[1];		///<  The pointer to the last Status message that was completed
uint16_t ubx_numberOfValidSVInfoMessage = 0;					///<  Number of valid Status message received

// NAV-Settings
ubx_cfg_nav_settings_t *ubx_currentNavSettingsMessage = &ubx_NavSettingsMessage[0];	///<  The pointer to the Nav Settings message that is being filled (not usable)
ubx_cfg_nav_settings_t *ubx_lastNavSettingsMessage = &ubx_NavSettingsMessage[1];		///<  The pointer to the last Nav Settings message that was completed
uint16_t ubx_numberOfValidNavSettingsMessage = 0;								///<  Number of valid Nav Settings message received

// CFG message rate
ubx_cfg_nav_rate_t *ubx_currentCFGRateMessage = &ubx_CFGRateMessage[0];	///<  The pointer to the CFG Rate message that is being filled (not usable)
ubx_cfg_nav_rate_t *ubx_lastCFGRateMessage = &ubx_CFGRateMessage[1];		///<  The pointer to the last CFG Rate message that was completed
uint16_t ubx_numberOfValidCFGRateMessage = 0;						///<  Number of valid CFG Rate message received

// CFG Set/Get message rate
ubx_cfg_msg_rate_t *ubx_currentCFGSetGetRateMessage = &ubx_CFGSetGetRateMessage[0];	///<  The pointer to the CFG Set/get Rate message that is being filled (not usable)
ubx_cfg_msg_rate_t *ubx_lastCFGSetGetRateMessage = &ubx_CFGSetGetRateMessage[1];		///<  The pointer to the last CFG Set/get Rate message that was completed
uint16_t ubx_numberOfValidCFGSetGetRateMessage = 0;							///<  Number of valid CFG Set/get Rate message received

// MON RXR message
ubx_mon_rxr_struct_t *ubx_currentMONRXRMessage = &ubx_MONRXRMessage[0];	///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_mon_rxr_struct_t *ubx_lastMONRXRMessage = &ubx_MONRXRMessage[1];		///<  The pointer to the last MON RXR message that was completed
uint16_t ubx_numberOfValidMONRXRMessage = 0;						///<  Number of valid MON RXR message received

// TIM TP message
ubx_tim_tp_t *ubx_currentTimTPMessage = &ubx_TimTPMessage[0];		///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_tim_tp_t *ubx_lastTimTPMessage = &ubx_TimTPMessage[1];		///<  The pointer to the last TIM TP message that was completed
uint16_t ubx_numberOfValidTimTPMessage = 0;				///<  Number of valid TIM TP message received


// TIM VRFY message
ubx_tim_vrfy_t *ubx_currentTimVRFYMessage = &ubx_TimVRFYMessage[0];	///<  The pointer to the TIM VRFY message that is being filled (not usable)
ubx_tim_vrfy_t *ubx_lastTimVRFYMessage = &ubx_TimVRFYMessage[1];		///<  The pointer to the last TIM VRFY message that was completed
uint16_t ubx_numberOfValidTimVRFYMessage = 0;					///<  Number of valid TIM VRFY message received

// Set to true to print all data
bool printNavOnDebug = false;

uint8_t loopPosllh = 0, loopVelned = 0, loopStatus = 0, loopSolution = 0, loopTimTp = 0, loopTimVrfy = 0;
uint8_t numSkippedMsg = 10;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Reset the gps U-Blox module
 *
 * \param	_engine_nav_setting		the GPS Nav settings 
 */
static void gps_ublox_reset(gps_t *gps, GPS_Engine_Setting _engine_nav_setting);


/**
 * \brief	Process bytes available from the stream
 *
 * The stream is assumed to contain only messages we recognise.  If it
 * contains other messages, and those messages contain the preamble
 * bytes, it is possible for this code to fail to synchronise to the
 * stream immediately.  Without buffering the entire message and
 * re-processing it from the top, this is unavoidable. The parser
 * attempts to avoid this when possible.
 *
 * \return	true if new velocity and new position message
 */
static bool gps_ublox_message_decode(gps_t *gps);


/**
 * \brief	Process the new received message, class by class
 *
 * \return	true if new velocity and new position message
 */
static bool gps_ublox_process_data(gps_t *gps);


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
static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b);


/**
 * \brief	To send the lower bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint16 uint
 */
static uint8_t endian_lower_bytes_uint16(uint16_t bytes);


/**
 * \brief	To send the higher bytes of an uint16_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint16 uint
 */
static uint8_t endian_higher_bytes_uint16(uint16_t bytes);


/**
 * \brief	To send the lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the lower 8 bytes of the uint32_t uint
 */
static uint8_t endian_lower_bytes_uint32(uint32_t bytes);


/**
 * \brief	To send the mid lower bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the mid lower 8 bytes of the uint32_t uint
 */
static uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes);


/**
 * \brief	To send the mid higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the mid higher 8 bytes of the uint32_t uint
 */
static uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes);


/**
 * \brief	To send the higher bytes of an uint32_t in the Little Endian format
 *
 * \param	bytes	the uint16 bytes to be transformed
 *
 * \return	the higher 8 bytes of the uint32_t uint
 */
static uint8_t endian_higher_bytes_uint32(uint32_t bytes);


/**
 * \brief	To send the UBX header of all messages
 *
 * \param	msg_class	the U-Blox class of the message
 * \param	_msg_id		the U-Blox message ID
 * \param	size		the size of the U-Blox following message
 */
static void ubx_send_header(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, uint16_t size);


/**
 * \brief	To send the checksum of every message
 *
 * \param	ck_sum_a	the checksum a
 * \param	ck_sum_b	the checksum b
 */
static void ubx_send_cksum(gps_t *gps, uint8_t ck_sum_a, uint8_t ck_sum_b);


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
static void ubx_send_message_CFG_nav_rate(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_rate_send_t msg, uint16_t size);


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
static void ubx_send_message_nav_settings(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_settings_t *engine_settings, uint16_t size);


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
static void ubx_configure_message_rate(gps_t *gps, uint8_t msg_class, uint8_t msg_id, uint8_t rate);


/**
 * \brief	This function returns a pointer to the last NAV-POSLLH message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid posllh message, or 0.
 */
static ubx_nav_posllh_t * ubx_GetPosllh(void);


/**
 * \brief	This function returns a pointer to the last NAV-STATUS message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid status message, or 0.
 */
static ubx_nav_status_t * ubx_GetStatus(void);


/**
 * \brief	This function returns a pointer to the last NAV-SOL message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid NAV-SOL message, or 0.
 */
static ubx_nav_solution_t * ubx_GetSolution(void);


/**
* \brief	This function returns a pointer to the last NAV-VELNED message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid velned message, or 0.
*/
static ubx_nav_velned_t * ubx_GetVelned(void);


/**
* \brief	This function returns a pointer to the last NAV-SVINFO message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_nav_SVInfo_t * ubx_GetSVInfo(void);


/**
* \brief	This function returns a pointer to the last NAV-Settings message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_nav_settings_t * ubx_GetNavSettings(void);


/**
* \brief	This function returns a pointer to the last CFG set/get rate message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_msg_rate_t * ubx_GetMsgRate(void);


/**
* \brief	This function returns a pointer to the last MON RXR message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_mon_rxr_struct_t * ubx_GetMonRXR(void);


/**
* \brief	This function returns a pointer to the last TIM TP message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_tp_t * ubx_GetTimTP(void);


/**
* \brief	This function returns a pointer to the last TIM VRFY message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_vrfy_t * ubx_GetTimVRFY(void);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void gps_ublox_reset(gps_t *gps, GPS_Engine_Setting _engine_nav_setting)
{
	// uint8_t epoch = TIME_OF_WEEK;
	idleTimeout = 1200;
	
	gps_ublox_configure_gps(gps);
	
	engine_nav_setting = _engine_nav_setting;
	
	gps->status = NO_FIX;
	gps->num_sats = 0;
	
	next_fix = false;
	have_raw_velocity = false;
	
	last_fix_time = 0;
	
	new_position = false;
	new_speed = false;
	
	step = 0;
}


static bool gps_ublox_message_decode(gps_t *gps)
{
	uint8_t data;
	bool msg_ok = false;
	
	uint8_t  * temporaryMessageForSwaping;
	
	while(buffer_bytes_available(&(gps->gps_buffer)))
	{
		data = buffer_get(&(gps->gps_buffer));
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
				print_util_dbg_print("large payload: ");
				print_util_dbg_print_num(payload_length,10);
				print_util_dbg_print("\n");
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
						ubx_currentMessage = (uint8_t **)&ubx_currentPosllhMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastPosllhMessage;
						ubx_validMessage = &ubx_numberOfValidPosllhMessage;
					}
					else
					{
						print_util_dbg_print("Wrong Posllh message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_POSLLH,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_STATUS:
					if(payload_length == UBX_SIZE_NAV_STATUS)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentStatusMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastStatusMessage;
						ubx_validMessage = &ubx_numberOfValidStatusMessage;
					}
					else
					{
						print_util_dbg_print("Wrong Nav Status message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_STATUS,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_SOL:
					if(payload_length == UBX_SIZE_NAV_SOL)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentSolutionMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastSolutionMessage;
						ubx_validMessage = &ubx_numberOfValidSolutionMessage;;
					}
					else
					{
						print_util_dbg_print("Wrong Solution message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SOL,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_VELNED:
					if(payload_length == UBX_SIZE_NAV_VELNED)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentVelnedMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastVelnedMessage;
						ubx_validMessage = &ubx_numberOfValidVelnedMessage;
					}
					else
					{
						print_util_dbg_print("Wrong Velned message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_VELNED,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_SVINFO:
					if(payload_length == UBX_SIZE_NAV_SVINFO)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentSVInfoMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastSVInfoMessage;
						ubx_validMessage = &ubx_numberOfValidSVInfoMessage;
					}
					else
					{
						print_util_dbg_print("Wrong SV Info message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SVINFO,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected NAV message, Class: 0x");
					print_util_dbg_print_num(ubxclass,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print("\n");
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
						ubx_currentMessage = (uint8_t **)&ubx_currentNavSettingsMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastNavSettingsMessage;
						ubx_validMessage = &ubx_numberOfValidNavSettingsMessage;
					}
					else
					{
						print_util_dbg_print("Wrong Nav Settings message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SETTINGS,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_CFG_RATE:
					if(payload_length == UBX_SIZE_CFG_RATE)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentCFGRateMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastCFGRateMessage;
						ubx_validMessage = &ubx_numberOfValidCFGRateMessage;
					}
					else
					{
						print_util_dbg_print("Wrong CFG Rate message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_CFG_RATE,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_CFG_SET_RATE:
					if (payload_length == UBX_SIZE_CFG_GETSET_RATE)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentCFGSetGetRateMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastCFGSetGetRateMessage;
						ubx_validMessage = &ubx_numberOfValidCFGSetGetRateMessage;
					}
					else
					{
						print_util_dbg_print("Wrong CFG Set/get message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_CFG_GETSET_RATE,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected CFG message, Class: 0x");
					print_util_dbg_print_num(ubxclass,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print("\n");
					goto reset;
				}
			} else if (ubxclass == UBX_CLASS_MON)
			{
				switch (msg_id)
				{
					case MSG_MON_RXR:
					if(payload_length == UBX_SIZE_MON_RXR)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentMONRXRMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastMONRXRMessage;
						ubx_validMessage = &ubx_numberOfValidMONRXRMessage;
					}
					else
					{
						print_util_dbg_print("Wrong MON RXR message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_MON_RXR,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected TIM message, Class: 0x");
					print_util_dbg_print_num(ubxclass,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print(" should be :");
					print_util_dbg_print_num(MSG_MON_RXR,16);
					print_util_dbg_print("\n");
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
						ubx_currentMessage = (uint8_t **)&ubx_currentTimTPMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastTimTPMessage;
						ubx_validMessage = &ubx_numberOfValidTimTPMessage;
					}
					else
					{
						print_util_dbg_print("Wrong TIM TP message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_TIM_TP,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_TIM_VRFY:
					if (payload_length == UBX_SIZE_TIM_VRFY)
					{
						ubx_currentMessage = (uint8_t **)&ubx_currentTimVRFYMessage;
						ubx_lastMessage = (uint8_t **)&ubx_lastTimVRFYMessage;
						ubx_validMessage = &ubx_numberOfValidTimVRFYMessage;
					}
					else
					{
						print_util_dbg_print("Wrong TIM VRFY message 0x");
						print_util_dbg_print_num(ubxclass,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_TIM_VRFY,10);
						print_util_dbg_print("\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected TIM message, Class: 0x");
					print_util_dbg_print_num(ubxclass,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print(" should be :");
					print_util_dbg_print_num(MSG_TIM_TP,16);
					print_util_dbg_print("\n");
					goto reset;
				}
			}
			else
			{
				step = 0;
				print_util_dbg_print("Unexpected message, Class: 0x");
				print_util_dbg_print_num(ubxclass,16);
				print_util_dbg_print(", msg id: 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print(" of size ");
				print_util_dbg_print_num(payload_length,10);
				print_util_dbg_print("\n");
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
				print_util_dbg_print("bad cksum_a ");
				print_util_dbg_print_num(data,16);
				print_util_dbg_print(" should be ");
				print_util_dbg_print_num(cksum_a,16);
				print_util_dbg_print(" class : 0x");
				print_util_dbg_print_num(ubxclass,16);
				print_util_dbg_print(" msg_id : 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print("\n");
				step = 0;
				goto reset;
			}
			break;
			
			case 8:
			step = 0;
			if (cksum_b != data)
			{
				print_util_dbg_print("bad cksum_b ");
				print_util_dbg_print_num(data,16);
				print_util_dbg_print(" should be ");
				print_util_dbg_print_num(cksum_b,16);
				print_util_dbg_print("\n");
				break;
			}
			++(*ubx_validMessage);
			//print_util_dbg_print("Valid message");
			
			// swap message buffers, old message is discarded and becomes incoming buffer, new message become valid message (=old)
			temporaryMessageForSwaping = *ubx_currentMessage;
			*ubx_currentMessage = *ubx_lastMessage;
			*ubx_lastMessage = temporaryMessageForSwaping;
			
			if (gps_ublox_process_data(gps))
			{
				msg_ok = true;
			}
		}
	}
	return msg_ok;
}


static bool gps_ublox_process_data(gps_t *gps)
{
	ubx_nav_posllh_t *gpsPosllh; 
	ubx_nav_status_t *gpsStatus;
	ubx_nav_solution_t *gpsSolution;
	ubx_nav_velned_t *gpsVelned;
	ubx_nav_SVInfo_t *gpsSVInfo;
	
	if (ubxclass == UBX_CLASS_ACK)
	{
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("\n");
		return false;
	}
 	if (ubxclass == UBX_CLASS_MON)
 	{
 		ubx_mon_rxr_struct_t *gpsRXR = ubx_GetMonRXR();
 		if (gpsRXR)
 		{
 			print_util_dbg_print("MSG_MON GPS awake\n");
 		}
		 return false;
 	}
	if (ubxclass == UBX_CLASS_TIM)
	{
		ubx_tim_tp_t *gpsTimTP = ubx_GetTimTP();
		if (gpsTimTP)
		{
			++loopTimTp;
			loopTimTp %= numSkippedMsg;
			if((printNavOnDebug)&&(loopTimTp == 0))
			{
				print_util_dbg_print("MSG_TIM_TP GPS awake\n");
			}
		}
		ubx_tim_vrfy_t *gpsTimVrfy = ubx_GetTimVRFY();
		if (gpsTimVrfy)
		{
			++loopTimVrfy;
			loopTimVrfy %= numSkippedMsg;
			if((printNavOnDebug)&&(loopTimVrfy == 0))
			{
				print_util_dbg_print("MSG_TIM_VRFY");
				print_util_dbg_print(" itow :");
				print_util_dbg_print_num(gpsTimVrfy->itow,10);
				print_util_dbg_print(" frac :");
				print_util_dbg_print_num(gpsTimVrfy->frac,10);
				print_util_dbg_print(" deltaMs :");
				print_util_dbg_print_num(gpsTimVrfy->deltaMs,10);
				print_util_dbg_print(" deltaNs :");
				print_util_dbg_print_num(gpsTimVrfy->deltaNs,10);
			}
		}
		return false;
	}
	 
	if (ubxclass == UBX_CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS)
	{
		ubx_cfg_nav_settings_t *gpsNavSettings = ubx_GetNavSettings();
		
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
			print_util_dbg_print("Got engine settings ");
			print_util_dbg_print_num(gpsNavSettings->dynModel,16);
			print_util_dbg_print("\n");
		}
		else
		{
			if (engine_nav_setting != GPS_ENGINE_NONE && !gpsNavSettings)
			{
				if(gpsNavSettings->dynModel != engine_nav_setting)
				{
					nav_settings.dynModel = engine_nav_setting;
					print_util_dbg_print("Send Nav settings");
					ubx_send_message_nav_settings(gps, UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, &nav_settings, sizeof(nav_settings));
				}				
			}
			print_util_dbg_print("No engine settings received ");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("\n");
		}
		return false;
	}
	
	if (ubxclass == UBX_CLASS_CFG && msg_id == MSG_CFG_SET_RATE)
	{
		ubx_cfg_msg_rate_t *gpsMsgRate;
		gpsMsgRate = ubx_GetMsgRate();
		
		if (gpsMsgRate)
		{
			print_util_dbg_print("Message CFG Rate 0x");
			print_util_dbg_print_num(gpsMsgRate->msg_class,16);
			print_util_dbg_print_num(gpsMsgRate->msg_id_rate,16);
			print_util_dbg_print_num(gpsMsgRate->rate,10);
			print_util_dbg_print("\n");
		}		
		return false;
	}
	
	if (ubxclass != UBX_CLASS_NAV)
	{
		print_util_dbg_print("Unexpected message 0x");
		print_util_dbg_print_num(ubxclass,16);
		print_util_dbg_print("02x 0x");
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("02x\n");
		if (++disable_counter == 0)
		{
			// disable future sends of this message id, but
			// only do this every 256 messages, as some
			// message types can't be disabled and we don't
			// want to get into an ack war
			
			print_util_dbg_print("Disabling message 0x");
			print_util_dbg_print_num(ubxclass,16);
			print_util_dbg_print("02x 0x");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("02x\n");
			ubx_configure_message_rate(gps,ubxclass, msg_id, 0);
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
			++loopPosllh;
			loopPosllh %= numSkippedMsg;
			if (printNavOnDebug && (loopPosllh == 0))
			{
				print_util_dbg_print("MSG_NAV_POSLLH");
				print_util_dbg_print(" itow :");
				print_util_dbg_print_num(gpsPosllh->itow,10);
				print_util_dbg_print(" longitude :");
				print_util_dbg_print_num(gpsPosllh->longitude,10);
				print_util_dbg_print(" latitude :");
				print_util_dbg_print_num(gpsPosllh->latitude,10);
				print_util_dbg_print(" alt_ellips :");
				print_util_dbg_print_num(gpsPosllh->altitude_ellipsoid,10);
				print_util_dbg_print(" alt_msl :");
				print_util_dbg_print_num(gpsPosllh->altitude_msl,10);
				print_util_dbg_print(" horz_acc :");
				print_util_dbg_print_num(gpsPosllh->horizontal_accuracy,10);
				print_util_dbg_print(" vert_acc :");
				print_util_dbg_print_num(gpsPosllh->vertical_accuracy,10);
				print_util_dbg_print("\n");
			}
			
			gps->timegps = gpsPosllh->itow;
			gps->longitude = gpsPosllh->longitude / 10000000.0f;
			gps->latitude = gpsPosllh->latitude / 10000000.0f;
			gps->alt_elips = ((float)gpsPosllh->altitude_ellipsoid) / 1000.0f;
			gps->altitude = ((float)gpsPosllh->altitude_msl) / 1000.0f;
			gps->horizontalAccuracy = ((float)gpsPosllh->horizontal_accuracy) / 1000.0f;
			gps->verticalAccuracy = ((float)gpsPosllh->vertical_accuracy) / 1000.0f;
			
			new_position = true;
		}
		break;
		
	case MSG_NAV_STATUS:
		gpsStatus = ubx_GetStatus();
		
		if (gpsStatus)
		{
			++loopStatus;
			loopStatus %= numSkippedMsg;
			if (printNavOnDebug && (loopStatus == 0))
			{
				print_util_dbg_print("MSG_STATUS fix_type = 0x");
				print_util_dbg_print_num(gpsStatus->fix_type,16);
				print_util_dbg_print(", uptime =");
				print_util_dbg_print_num(gpsStatus->uptime,10);
				print_util_dbg_print("\n");
			}
			next_fix = (gpsStatus->fix_type == GPS_FIX_TYPE_3DFIX);
			if (!next_fix)
			{
				gps->status = NO_FIX;
			}
			else
			{
				gps->status = GPS_OK;
			}
		}
		break;
		
	case MSG_NAV_SOL:
		gpsSolution = ubx_GetSolution();
		
		if (gpsSolution)
		{
			++loopSolution;
			loopSolution %= numSkippedMsg;
			if (printNavOnDebug && (loopSolution == 0))
			{
				print_util_dbg_print("MSG_SOL ");
				print_util_dbg_print("itow :");
				print_util_dbg_print_num(gpsSolution->itow,10);
				print_util_dbg_print(" week :");
				print_util_dbg_print_num(gpsSolution->week,10);
				print_util_dbg_print(" fix_type = 0x0");
				print_util_dbg_print_num(gpsSolution->fix_type,16);
				print_util_dbg_print(" pos_acc_3d :");
				print_util_dbg_print_num(gpsSolution->position_accuracy_3d,10);
				print_util_dbg_print(" ecefx :");
				print_util_dbg_print_num(gpsSolution->ecef_x,10);
				print_util_dbg_print(" ecefy :");
				print_util_dbg_print_num(gpsSolution->ecef_y,10);
				print_util_dbg_print(" ecefz :");
				print_util_dbg_print_num(gpsSolution->ecef_z,10);
				print_util_dbg_print(" pos_DOP :");
				print_util_dbg_print_num(gpsSolution->position_DOP,10);
				print_util_dbg_print(" num sat :");
				print_util_dbg_print_num(gpsSolution->satellites,10);
				print_util_dbg_print("\n");
			}
			next_fix = (gpsSolution->fix_type == GPS_FIX_TYPE_3DFIX);
			if (!next_fix)
			{
				gps->status = NO_FIX;
			}
			else
			{
				gps->status = GPS_OK;
			}
		
			gps->num_sats = gpsSolution->satellites;
			gps->hdop = gpsSolution->position_DOP;
		}
		break;
		
	case MSG_NAV_VELNED:
		gpsVelned = ubx_GetVelned();
		
		if (gpsVelned)
		{
			++loopVelned;
			loopVelned %= numSkippedMsg;
			if (printNavOnDebug && (loopVelned == 0))
			{
			
				print_util_dbg_print("MSG_NAV_VELNED ");
		
				print_util_dbg_print("itow :");
				print_util_dbg_print_num(gpsVelned->itow,10);
				print_util_dbg_print(" ned_north :");
				print_util_dbg_print_num(gpsVelned->ned_north,10);
				print_util_dbg_print(" ned_east :");
				print_util_dbg_print_num(gpsVelned->ned_east,10);
				print_util_dbg_print(" ned_down :");
				print_util_dbg_print_num(gpsVelned->ned_down,10);
				print_util_dbg_print(" speed_3d :");
				print_util_dbg_print_num(gpsVelned->speed_3d,10);
				print_util_dbg_print(" heading_2d :");
				print_util_dbg_print_num(gpsVelned->heading_2d,10);
				print_util_dbg_print(" speed_accuracy :");
				print_util_dbg_print_num(gpsVelned->speed_accuracy,10);
				print_util_dbg_print(" heading_accuracy :");
				print_util_dbg_print_num(gpsVelned->heading_accuracy,10);
				print_util_dbg_print("\n");
			}
			gps->timegps = gpsVelned->itow;
			gps->speed        = ((float)gpsVelned->speed_3d) / 100.; // m/s
			gps->groundSpeed = ((float)gpsVelned->groundSpeed_2d) / 100.; // m/s
			gps->course = ((float)gpsVelned->heading_2d) / 100000.; // Heading 2D deg * 100000 rescaled to deg * 100
			have_raw_velocity = true;
			gps->northSpeed  = ((float)gpsVelned->ned_north) / 100.0f;
			gps->eastSpeed   = ((float)gpsVelned->ned_east) / 100.;
			gps->verticalSpeed   = ((float)gpsVelned->ned_down) / 100.;
			gps->speedAccuracy = ((float)gpsVelned->speed_accuracy) / 100.;
			gps->headingAccuracy = gpsVelned->heading_accuracy;
			new_speed = true;
		}
		break;
		
	case MSG_NAV_SVINFO:
		gpsSVInfo = ubx_GetSVInfo();
		
		if (gpsSVInfo)
		{
			if (printNavOnDebug)
			{
				print_util_dbg_print("MSG_NAV_SVINFO, numChannel:");
				print_util_dbg_print_num(gpsSVInfo->numCh,10);
				print_util_dbg_print("\n");
			}
		}
		
	default:
		print_util_dbg_print("Unexpected NAV message 0x");
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("\n");
		
		if (++disable_counter == 0)
		{
			print_util_dbg_print("Disabling NAV message 0x");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("\n");
			ubx_configure_message_rate(gps, UBX_CLASS_NAV, msg_id, 0);
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


static void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	while (len--)
	{
		*ck_a += *data;
		*ck_b += *ck_a;
		data++;
	}
}


static uint8_t endian_lower_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0x00FF);
}


static uint8_t endian_higher_bytes_uint16(uint16_t bytes)
{
	return (bytes & 0xFF00)>>8;
}


static uint8_t endian_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x000000FF);
}


static uint8_t endian_mid_lower_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x0000FF00)>>8;
}


static uint8_t endian_mid_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0x00FF0000)>>16;
}


static uint8_t endian_higher_bytes_uint32(uint32_t bytes)
{
	return (bytes & 0xFF000000)>>24;
}


static void ubx_send_header(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, uint16_t size)
{
	ubx_header_t header;
	header.preamble1		= UBX_PREAMBLE1;
	header.preamble2		= UBX_PREAMBLE2;
	header.msg_class		= msg_class;
	header.msg_id_header    = _msg_id;
	header.length			= size;
	
	print_util_putnum(&gps->gps_stream_out,header.preamble1,16);
	print_util_putnum(&gps->gps_stream_out,header.preamble2,16);
	print_util_putnum(&gps->gps_stream_out,header.msg_class,10);
	print_util_putnum(&gps->gps_stream_out,header.msg_id_header,16);
	
	print_util_putnum(&gps->gps_stream_out,(uint8_t) endian_lower_bytes_uint16(header.length),16);
	print_util_putnum(&gps->gps_stream_out,(uint8_t) endian_higher_bytes_uint16(header.length),16);
	
}


static void ubx_send_cksum(gps_t *gps, uint8_t ck_sum_a, uint8_t ck_sum_b)
{
	print_util_putnum(&gps->gps_stream_out,ck_sum_a,16);
	print_util_putnum(&gps->gps_stream_out,ck_sum_b,16);
}


static void ubx_send_message_CFG_nav_rate(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_rate_send_t msg, uint16_t size)
{
	uint8_t ck_a = 0, ck_b = 0;
	
	uint8_t data;
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	// update_checksum((uint8_t *)&msg, size, &ck_a, &ck_b); Wrong!
	
	ubx_send_header(gps, msg_class,_msg_id,size);
	
	data = endian_lower_bytes_uint16(msg.measure_rate_ms);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.measure_rate_ms);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	data = endian_lower_bytes_uint16(msg.nav_rate);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.nav_rate);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	data = endian_lower_bytes_uint16(msg.timeref);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	data = endian_higher_bytes_uint16(msg.timeref);
	update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
	print_util_putnum(&gps->gps_stream_out, data, 16);
	
	ubx_send_cksum(gps, ck_a,ck_b);
}


static void ubx_send_message_nav_settings(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, ubx_cfg_nav_settings_t *engine_settings, uint16_t size)
{
	uint8_t ck_a = 0, ck_b = 0;
	uint8_t data;
	
	ubx_send_header(gps, msg_class,_msg_id,size);
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	if (engine_settings != NULL)
	{
		//update_checksum((uint8_t *)engine_settings, size, &ck_a, &ck_b);
		
		data = endian_lower_bytes_uint16(engine_settings->mask);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->mask);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->dynModel;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->fixMode;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixedAlt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixedAltVar);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->minElev;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->drLimit;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->pDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->pDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->tDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->tDop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->pAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->pAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->tAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->tAcc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->staticHoldThresh;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->dgpsTimeOut;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res2);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res3);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->res4);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
	}
	
	ubx_send_cksum(gps, ck_a,ck_b);
}


static void ubx_configure_message_rate(gps_t *gps, uint8_t msg_class, uint8_t _msg_id, uint8_t rate)
{
	uint8_t ck_a = 0, ck_b = 0;
	ubx_cfg_msg_rate_send_t msg;
	msg.msg_class = msg_class;
	msg.msg_id_rate    = _msg_id;
	msg.rate          = rate;
	
	uint8_t size = sizeof(msg);
	
	update_checksum((uint8_t *)&msg_class, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&_msg_id, 1, &ck_a, &ck_b);
	update_checksum((uint8_t *)&size, 1, &ck_a, &ck_b);

	update_checksum((uint8_t *)&msg, size, &ck_a, &ck_b);
	
	ubx_send_header(gps, UBX_CLASS_CFG,MSG_CFG_SET_RATE,sizeof(msg));
	
	print_util_putnum(&gps->gps_stream_out,msg.msg_class,16);
	print_util_putnum(&gps->gps_stream_out,msg.msg_id_rate,16);
	print_util_putnum(&gps->gps_stream_out,msg.rate,16);
	
	ubx_send_cksum(gps, ck_a,ck_b);
}


static ubx_nav_posllh_t * ubx_GetPosllh()
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


static ubx_nav_status_t * ubx_GetStatus()
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


static ubx_nav_solution_t * ubx_GetSolution()
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

static ubx_nav_velned_t * ubx_GetVelned()
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


static ubx_nav_SVInfo_t * ubx_GetSVInfo()
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


static ubx_cfg_nav_settings_t * ubx_GetNavSettings()
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


static ubx_cfg_msg_rate_t * ubx_GetMsgRate()
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


static ubx_mon_rxr_struct_t * ubx_GetMonRXR()
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


static ubx_tim_tp_t * ubx_GetTimTP()
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


static ubx_tim_vrfy_t * ubx_GetTimVRFY()
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


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void gps_ublox_init(gps_t *gps, int32_t UID, mavlink_stream_t* mavlink_stream)
{
	// uart setting
	usart_config_t usart_conf_gps =
	{
		.mode=UART_IN_OUT,
		.uart_device.uart=(avr32_usart_t *)&AVR32_USART3,
		.uart_device.IRQ=AVR32_USART3_IRQ,
		.uart_device.receive_stream=NULL,
		.options={
			.baudrate     = 38400,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE },
		.rx_pin_map= {AVR32_USART3_RXD_0_0_PIN, AVR32_USART3_RXD_0_0_FUNCTION},
		.tx_pin_map= {AVR32_USART3_TXD_0_0_PIN, AVR32_USART3_TXD_0_0_FUNCTION}
	};
	uart_int_set_usart_conf(UID, &usart_conf_gps);
	
	uart_int_init(UID);
	buffer_make_buffered_stream(&(gps->gps_buffer), &(gps->gps_stream_in));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(gps->gps_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(gps->gps_stream_out));
	
	gps->mavlink_stream = mavlink_stream;
}


void gps_ublox_configure_gps(gps_t *gps)
{
	ubx_cfg_nav_rate_send_t msg;
	// const uint32_t audrates[4] = {9600U, 19200U, 38400U, 57600U};

	const char  *set_binary = UBLOX_SET_BINARY;
	// the GPS may be setup for a different baud rate. This ensures
	// it gets configured correctly
// 	for (uint8_t i = 0; i < 4; i++)
// 	{
	
	//print_util_dbg_print("Set to binary mode ");
	//print_util_dbg_print(set_binary);
	print_util_putstring(&(gps->gps_stream_out), set_binary);
	//gps->gps_stream_out.put(gps->gps_stream_out.data,set_binary);
	//gps->gps_stream_out.flush(&(gps->gps_stream_out.data));
	//}

	// ask for navigation solutions every 200ms
	msg.measure_rate_ms = 200;		// ms
	msg.nav_rate        = 1;		// constant equal to 1
	msg.timeref         = 0;		// 0:UTC time, 1:GPS time
	
	ubx_send_message_CFG_nav_rate(gps, UBX_CLASS_CFG, MSG_CFG_RATE, msg, sizeof(msg));

	// ask for the messages we parse to be sent on every navigation solution
	//print_util_dbg_print("Set navigation messages\n");
	ubx_configure_message_rate(gps,UBX_CLASS_NAV, MSG_NAV_POSLLH, 4);
	ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_STATUS, 4);
	ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_SOL, 4);
	ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_VELNED, 4);
	ubx_configure_message_rate(gps, UBX_CLASS_NAV, MSG_NAV_SVINFO, 4);

	// ask for the current navigation settings
	//print_util_dbg_print("Asking for engine setting\n");
	ubx_send_message_nav_settings(gps, UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}


void gps_ublox_update(gps_t *gps)
{
	bool result;
	uint32_t tnow;
	
	result = gps_ublox_message_decode(gps);
	
	tnow = time_keeper_get_millis();
	
	if (! result)
	{
		if ((tnow - idleTimer) > idleTimeout)
		{
			gps->status = NO_GPS;
			
			gps_ublox_reset(gps, engine_nav_setting);
			idleTimer = tnow;
		}
		
	}
	else
	{
		// reset the idle timer
		idleTimer = tnow;
		
		gps->time_last_msg = tnow;
		
		if(gps->status == GPS_OK)
		{
			// Check for horizontal accuracy
			if (gps->horizontalAccuracy < UBX_POSITION_PRECISION)
			{
				gps->horizontalStatus = 1;
			}
			else
			{  
				gps->horizontalStatus = 0;
			}
			// Check for vertical accuracy
			if (gps->verticalAccuracy < UBX_ALTITUDE_PRECISION)
			{
				gps->altitudeStatus = 1;
			}
			else
			{
				gps->altitudeStatus = 0;
			}
			// Check for speed accuracy
			if (gps->speedAccuracy < UBX_SPEED_PRECISION)
			{
				gps->speedStatus = 1;
			}
			else
			{
				gps->speedStatus = 0;
			}
			// Check for heading accuracy
			if (gps->headingAccuracy < UBX_HEADING_PRECISION)
			{
				gps->courseStatus = 1;
			}
			else
			{
				gps->courseStatus = 0;
			}
			
			gps->accuracyStatus = gps->horizontalStatus & gps->altitudeStatus & gps->speedStatus & gps->courseStatus;
			
			// speed approximation with the 
// 			if (!have_raw_velocity)
// 			{
// 				float gps_heading = ToRad(gps->course);
// 				float cos_heading,sin_heading;
// 				
// 				cos_heading = cosf(gps_heading);
// 				sin_heading = sinf(gps_heading);
// 				
// 				gps->northspeed = gps->groundSpeed * cos_heading;
// 				gps->eastspeed = gps->groundSpeed * sin_heading;
// 				
// 				// no good way to get descent rate
// 				gps->verticalSpeed = 0;
// 			}
		}
		else
		{
			gps->horizontalStatus = 0;
			gps->altitudeStatus = 0;
			gps->speedStatus = 0;
			gps->courseStatus = 0;
			
			gps->accuracyStatus = 0;
		}
	}
}


task_return_t gps_ublox_send_raw(gps_t* gps)
{
	mavlink_message_t msg;
	if (gps->status == GPS_OK)
	{
		mavlink_msg_gps_raw_int_pack(	gps->mavlink_stream->sysid,
										gps->mavlink_stream->compid,
										&msg,
										1000 * gps->time_last_msg,
										gps->status,
										gps->latitude * 10000000.0f,
										gps->longitude * 10000000.0f,
										gps->altitude * 1000.0f,
										gps->hdop * 100.0f,
										gps->speedAccuracy * 100.0f,
										gps->groundSpeed * 100.0f,
										gps->course,
										gps->num_sats	);
		mavlink_stream_send(gps->mavlink_stream,&msg);
	}
	else
	{
		mavlink_msg_gps_raw_int_pack(	gps->mavlink_stream->sysid,
										gps->mavlink_stream->compid,
										&msg,
										time_keeper_get_micros(),
										gps->status,
										46.5193f * 10000000,
										6.56507f * 10000000,
										400 * 1000,
										0,
										0,
										0,
										0,
										gps->num_sats);		// TODO: return TASK_RUN_ERROR here?
		mavlink_stream_send(gps->mavlink_stream,&msg);
	}

	return TASK_RUN_SUCCESS;
}