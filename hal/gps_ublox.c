/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file gps_ublox.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file decodes the messages from the UBLOX GPS
 * 
 ******************************************************************************/


#include "gps_ublox.h"

#include "print_util.h"
#include "uart_int.h"
#include "buffer.h"
#include "time_keeper.h"

uint8_t  **ubx_current_message = 0;		///<  The pointer to the pointer to the structure of the current message to fill
uint8_t  ** ubx_last_message = 0;		///<  The pointer to the pointer to the structure of the last message received of the same type than the current one being received (for exchange at the end)
uint16_t * ubx_valid_message = 0;		///<  The pointer to the number to increment when a message of the type has been received

// We are using two buffers for each message, one for the last message received, the other for the message being received (not complete)
ubx_nav_pos_llh_t ubx_pos_llh_message[2];			///<  The Posllh message buffer
ubx_nav_status_t ubx_status_message[2];			///<  The Status message buffer
ubx_nav_solution_t ubx_solution_message[2];		///<  The Solution message buffer
ubx_nav_vel_ned_t ubx_vel_ned_message[2];			///<  The Velned message buffer
ubx_nav_sv_info_t ubx_sv_info_message[2];			///<  The SVInfo message buffer
ubx_cfg_nav_settings_t ubx_nav_settings_message[2]; ///<  The Nav Settings message buffer
ubx_cfg_nav_rate_t ubx_cfg_rate_message[2];			///<  The CFG Rate message buffer
ubx_cfg_msg_rate_t ubx_cfg_set_get_rate_message[2];	///<  The CFG Set/get Rate message buffer
ubx_mon_rxr_struct_t ubx_mon_rxr_message[2];		///<  The MON RXR message buffer
ubx_tim_tp_t ubx_tim_tp_message[2];					///<  The TIM TP message buffer
ubx_tim_vrfy_t ubx_tim_vrfy_message[2];				///<  The TIM VRFY message buffer

// NAV-POSLLH
ubx_nav_pos_llh_t * ubx_current_pos_llh_message = &ubx_pos_llh_message[0];	///<  The pointer to the Posllh message that is being filled (not usable)
ubx_nav_pos_llh_t * ubx_last_pos_llh_message = &ubx_pos_llh_message[1];		///<  The pointer to the last Posllh message that was completed
uint16_t ubx_number_of_valid_pos_llh_message = 0;					///<  Number of valid Posllh message received

// NAV-STATUS
ubx_nav_status_t *ubx_current_status_message = &ubx_status_message[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_status_t *ubx_last_status_message = &ubx_status_message[1];		///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_status_message = 0;					///<  Number of valid Status message received

// NAV-Sol
ubx_nav_solution_t *ubx_current_solution_message = &ubx_solution_message[0]; ///<  The pointer to the Solution message that is being filled (not usable)
ubx_nav_solution_t *ubx_last_solution_message = &ubx_solution_message[1];	///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_solution_message = 0;					///<  Number of valid Status message received

// NAV-VELNED
ubx_nav_vel_ned_t *ubx_current_vel_ned_message = &ubx_vel_ned_message[0];	///<  The pointer to the Velned message that is being filled (not usable)
ubx_nav_vel_ned_t *ubx_last_vel_ned_message = &ubx_vel_ned_message[1];		///<  The pointer to the last Velned message that was completed
uint16_t ubx_number_of_valid_vel_ned_message = 0;					///<  Number of valid Velned message received

// NAV-SVINFO
ubx_nav_sv_info_t *ubx_current_sv_info_message = &ubx_sv_info_message[0];	///<  The pointer to the Status message that is being filled (not usable)
ubx_nav_sv_info_t *ubx_last_sv_info_message = &ubx_sv_info_message[1];		///<  The pointer to the last Status message that was completed
uint16_t ubx_number_of_valid_sv_info_message = 0;					///<  Number of valid Status message received

// NAV-Settings
ubx_cfg_nav_settings_t *ubx_current_nav_settings_message = &ubx_nav_settings_message[0];	///<  The pointer to the Nav Settings message that is being filled (not usable)
ubx_cfg_nav_settings_t *ubx_last_nav_settings_message = &ubx_nav_settings_message[1];		///<  The pointer to the last Nav Settings message that was completed
uint16_t ubx_number_of_valid_nav_settings_message = 0;								///<  Number of valid Nav Settings message received

// CFG message rate
ubx_cfg_nav_rate_t *ubx_current_cfg_rate_message = &ubx_cfg_rate_message[0];	///<  The pointer to the CFG Rate message that is being filled (not usable)
ubx_cfg_nav_rate_t *ubx_last_cfg_rate_message = &ubx_cfg_rate_message[1];		///<  The pointer to the last CFG Rate message that was completed
uint16_t ubx_number_of_valid_cfg_rate_message = 0;						///<  Number of valid CFG Rate message received

// CFG Set/Get message rate
ubx_cfg_msg_rate_t *ubx_current_cfg_set_get_rate_message = &ubx_cfg_set_get_rate_message[0];	///<  The pointer to the CFG Set/get Rate message that is being filled (not usable)
ubx_cfg_msg_rate_t *ubx_last_cfg_set_get_rate_message = &ubx_cfg_set_get_rate_message[1];		///<  The pointer to the last CFG Set/get Rate message that was completed
uint16_t ubx_number_of_valid_cfg_set_get_rate_message = 0;							///<  Number of valid CFG Set/get Rate message received

// MON RXR message
ubx_mon_rxr_struct_t *ubx_current_mon_rxr_message = &ubx_mon_rxr_message[0];	///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_mon_rxr_struct_t *ubx_last_mon_rxr_message = &ubx_mon_rxr_message[1];		///<  The pointer to the last MON RXR message that was completed
uint16_t ubx_number_of_valid_mon_rxr_message = 0;						///<  Number of valid MON RXR message received

// TIM TP message
ubx_tim_tp_t *ubx_current_tim_tp_message = &ubx_tim_tp_message[0];		///<  The pointer to the MON RXR message that is being filled (not usable)
ubx_tim_tp_t *ubx_last_tim_tp_message = &ubx_tim_tp_message[1];		///<  The pointer to the last TIM TP message that was completed
uint16_t ubx_number_of_valid_tim_tp_message = 0;				///<  Number of valid TIM TP message received


// TIM VRFY message
ubx_tim_vrfy_t *ubx_current_tim_vrfy_message = &ubx_tim_vrfy_message[0];	///<  The pointer to the TIM VRFY message that is being filled (not usable)
ubx_tim_vrfy_t *ubx_last_tim_vrfy_message = &ubx_tim_vrfy_message[1];		///<  The pointer to the last TIM VRFY message that was completed
uint16_t ubx_number_of_valid_tim_vrfy_message = 0;					///<  Number of valid TIM VRFY message received

// Set to true to print all data
bool print_nav_on_debug = false;

uint8_t loop_pos_llh = 0, loop_vel_ned = 0, loop_status = 0, loop_solution = 0, loop_tim_tp = 0, loop_tim_vrfy = 0;
uint8_t num_skipped_msg = 10;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Reset the gps U-Blox module
 *
 * \param	gps						Pointer to the GPS structure
 * \param	_engine_nav_setting		the GPS Nav settings 
 */
static void gps_ublox_reset(gps_t *gps, gps_engine_setting_t _engine_nav_setting);


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
 * \param gps	Pointer to the GPS structure
 *
 * \return	true if new velocity and new position message
 */
static bool gps_ublox_message_decode(gps_t *gps);


/**
 * \brief	Process the new received message, class by class
 *
 * \param gps	Pointer to the GPS structure
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
 * \param	gps			Pointer to the GPS structure
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
 * \param	gps			Pointer to the GPS structure
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
 * \param	gps					Pointer to the GPS structure
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
 * \param	gps			Pointer to the GPS structure
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
static ubx_nav_pos_llh_t * ubx_get_pos_llh(void);


/**
 * \brief	This function returns a pointer to the last NAV-STATUS message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid status message, or 0.
 */
static ubx_nav_status_t * ubx_get_status(void);


/**
 * \brief	This function returns a pointer to the last NAV-SOL message that was received
 *
 * Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
 *
 * \return	A pointer to the last valid NAV-SOL message, or 0.
 */
static ubx_nav_solution_t * ubx_get_solution(void);


/**
* \brief	This function returns a pointer to the last NAV-VELNED message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid velned message, or 0.
*/
static ubx_nav_vel_ned_t * ubx_get_vel_ned(void);


/**
* \brief	This function returns a pointer to the last NAV-SVINFO message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_nav_sv_info_t * ubx_get_sv_info(void);


/**
* \brief	This function returns a pointer to the last NAV-Settings message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_nav_settings_t * ubx_get_nav_settings(void);


/**
* \brief	This function returns a pointer to the last CFG set/get rate message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_cfg_msg_rate_t * ubx_get_msg_rate(void);


/**
* \brief	This function returns a pointer to the last MON RXR message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_mon_rxr_struct_t * ubx_get_mon_rxr(void);


/**
* \brief	This function returns a pointer to the last TIM TP message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_tp_t * ubx_get_tim_tp(void);


/**
* \brief	This function returns a pointer to the last TIM VRFY message that was received
* Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
* \return	A pointer to the last valid status message, or 0.
*/
static ubx_tim_vrfy_t * ubx_get_tim_vrfy(void);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void gps_ublox_reset(gps_t *gps, gps_engine_setting_t _engine_nav_setting)
{
	// uint8_t epoch = TIME_OF_WEEK;
	idle_timeout = 1200;
	
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
	
	uint8_t  * temporary_message_for_swaping;
	
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
				ubx_class = data;
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
				print_util_dbg_print("\r\n");
				payload_length = 0;
				step = 0;
				goto reset;
			}
			payload_counter = 0; // prepare to receive payload
			
			if(ubx_class == UBX_CLASS_NAV)
			{
				switch(msg_id)
				{
					case MSG_NAV_POSLLH:
					if(payload_length == UBX_SIZE_NAV_POSLLH)
					{
						ubx_current_message = (uint8_t **)&ubx_current_pos_llh_message;
						ubx_last_message = (uint8_t **)&ubx_last_pos_llh_message;
						ubx_valid_message = &ubx_number_of_valid_pos_llh_message;
					}
					else
					{
						print_util_dbg_print("Wrong Posllh message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_POSLLH,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_STATUS:
					if(payload_length == UBX_SIZE_NAV_STATUS)
					{
						ubx_current_message = (uint8_t **)&ubx_current_status_message;
						ubx_last_message = (uint8_t **)&ubx_last_status_message;
						ubx_valid_message = &ubx_number_of_valid_status_message;
					}
					else
					{
						print_util_dbg_print("Wrong Nav Status message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_STATUS,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_SOL:
					if(payload_length == UBX_SIZE_NAV_SOL)
					{
						ubx_current_message = (uint8_t **)&ubx_current_solution_message;
						ubx_last_message = (uint8_t **)&ubx_last_solution_message;
						ubx_valid_message = &ubx_number_of_valid_solution_message;;
					}
					else
					{
						print_util_dbg_print("Wrong Solution message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SOL,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_VELNED:
					if(payload_length == UBX_SIZE_NAV_VELNED)
					{
						ubx_current_message = (uint8_t **)&ubx_current_vel_ned_message;
						ubx_last_message = (uint8_t **)&ubx_last_vel_ned_message;
						ubx_valid_message = &ubx_number_of_valid_vel_ned_message;
					}
					else
					{
						print_util_dbg_print("Wrong Velned message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_VELNED,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_NAV_SVINFO:
					if(payload_length == UBX_SIZE_NAV_SVINFO)
					{
						ubx_current_message = (uint8_t **)&ubx_current_sv_info_message;
						ubx_last_message = (uint8_t **)&ubx_last_sv_info_message;
						ubx_valid_message = &ubx_number_of_valid_sv_info_message;
					}
					else
					{
						print_util_dbg_print("Wrong SV Info message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SVINFO,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected NAV message, Class: 0x");
					print_util_dbg_print_num(ubx_class,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print("\r\n");
					goto reset;
				}
			}
			else if(ubx_class == UBX_CLASS_CFG)
			{
				
				switch(msg_id)
				{
					case MSG_CFG_NAV_SETTINGS:
					if(payload_length == UBX_SIZE_NAV_SETTINGS)
					{
						ubx_current_message = (uint8_t **)&ubx_current_nav_settings_message;
						ubx_last_message = (uint8_t **)&ubx_last_nav_settings_message;
						ubx_valid_message = &ubx_number_of_valid_nav_settings_message;
					}
					else
					{
						print_util_dbg_print("Wrong Nav Settings message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_NAV_SETTINGS,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_CFG_RATE:
					if(payload_length == UBX_SIZE_CFG_RATE)
					{
						ubx_current_message = (uint8_t **)&ubx_current_cfg_rate_message;
						ubx_last_message = (uint8_t **)&ubx_last_cfg_rate_message;
						ubx_valid_message = &ubx_number_of_valid_cfg_rate_message;
					}
					else
					{
						print_util_dbg_print("Wrong CFG Rate message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_CFG_RATE,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_CFG_SET_RATE:
					if (payload_length == UBX_SIZE_CFG_GETSET_RATE)
					{
						ubx_current_message = (uint8_t **)&ubx_current_cfg_set_get_rate_message;
						ubx_last_message = (uint8_t **)&ubx_last_cfg_set_get_rate_message;
						ubx_valid_message = &ubx_number_of_valid_cfg_set_get_rate_message;
					}
					else
					{
						print_util_dbg_print("Wrong CFG Set/get message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_CFG_GETSET_RATE,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected CFG message, Class: 0x");
					print_util_dbg_print_num(ubx_class,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print("\r\n");
					goto reset;
				}
			} else if (ubx_class == UBX_CLASS_MON)
			{
				switch (msg_id)
				{
					case MSG_MON_RXR:
					if(payload_length == UBX_SIZE_MON_RXR)
					{
						ubx_current_message = (uint8_t **)&ubx_current_mon_rxr_message;
						ubx_last_message = (uint8_t **)&ubx_last_mon_rxr_message;
						ubx_valid_message = &ubx_number_of_valid_mon_rxr_message;
					}
					else
					{
						print_util_dbg_print("Wrong MON RXR message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_MON_RXR,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected TIM message, Class: 0x");
					print_util_dbg_print_num(ubx_class,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print(" should be :");
					print_util_dbg_print_num(MSG_MON_RXR,16);
					print_util_dbg_print("\r\n");
					goto reset;
				}
				
			}
			else if(ubx_class == UBX_CLASS_TIM)
			{
				switch(msg_id)
				{
					case MSG_TIM_TP:
					if (payload_length == UBX_SIZE_TIM_TP)
					{
						ubx_current_message = (uint8_t **)&ubx_current_tim_tp_message;
						ubx_last_message = (uint8_t **)&ubx_last_tim_tp_message;
						ubx_valid_message = &ubx_number_of_valid_tim_tp_message;
					}
					else
					{
						print_util_dbg_print("Wrong TIM TP message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_TIM_TP,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					case MSG_TIM_VRFY:
					if (payload_length == UBX_SIZE_TIM_VRFY)
					{
						ubx_current_message = (uint8_t **)&ubx_current_tim_vrfy_message;
						ubx_last_message = (uint8_t **)&ubx_last_tim_vrfy_message;
						ubx_valid_message = &ubx_number_of_valid_tim_vrfy_message;
					}
					else
					{
						print_util_dbg_print("Wrong TIM VRFY message 0x");
						print_util_dbg_print_num(ubx_class,16);
						print_util_dbg_print(" Msg id: 0x");
						print_util_dbg_print_num(msg_id,16);
						print_util_dbg_print(" Received size:");
						print_util_dbg_print_num(payload_length,10);
						print_util_dbg_print(" should be:");
						print_util_dbg_print_num(UBX_SIZE_TIM_VRFY,10);
						print_util_dbg_print("\r\n");
						step = 0;
						goto reset;
					}
					break;
					
					default:
					step = 0;
					print_util_dbg_print("Unexpected TIM message, Class: 0x");
					print_util_dbg_print_num(ubx_class,16);
					print_util_dbg_print(", msg id: 0x");
					print_util_dbg_print_num(msg_id,16);
					print_util_dbg_print(" of size ");
					print_util_dbg_print_num(payload_length,10);
					print_util_dbg_print(" should be :");
					print_util_dbg_print_num(MSG_TIM_TP,16);
					print_util_dbg_print("\r\n");
					goto reset;
				}
			}
			else
			{
				step = 0;
				print_util_dbg_print("Unexpected message, Class: 0x");
				print_util_dbg_print_num(ubx_class,16);
				print_util_dbg_print(", msg id: 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print(" of size ");
				print_util_dbg_print_num(payload_length,10);
				print_util_dbg_print("\r\n");
				goto reset;
			}
			break;
			
			case 6:
			cksum_a += data;
			cksum_b += cksum_a; // checksum byte
			
			#ifdef BIG_ENDIAN
			(*ubx_current_message)[payload_length - 1 - payload_counter] = data;
			#else
			(*ubx_current_message)[payload_counter] = data;
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
				print_util_dbg_print_num(ubx_class,16);
				print_util_dbg_print(" msg_id : 0x");
				print_util_dbg_print_num(msg_id,16);
				print_util_dbg_print("\r\n");
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
				print_util_dbg_print("\r\n");
				break;
			}
			++(*ubx_valid_message);
			//print_util_dbg_print("Valid message");
			
			// swap message buffers, old message is discarded and becomes incoming buffer, new message become valid message (=old)
			temporary_message_for_swaping = *ubx_current_message;
			*ubx_current_message = *ubx_last_message;
			*ubx_last_message = temporary_message_for_swaping;
			
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
	ubx_nav_pos_llh_t *gps_pos_llh; 
	ubx_nav_status_t *gps_status;
	ubx_nav_solution_t *gps_solution;
	ubx_nav_vel_ned_t *gps_vel_ned;
	ubx_nav_sv_info_t *gps_sv_info;
	
	if (ubx_class == UBX_CLASS_ACK)
	{
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("\r\n");
		return false;
	}
 	if (ubx_class == UBX_CLASS_MON)
 	{
 		ubx_mon_rxr_struct_t *gps_rxr = ubx_get_mon_rxr();
 		if (gps_rxr)
 		{
 			print_util_dbg_print("MSG_MON GPS awake\r\n");
 		}
		 return false;
 	}
	if (ubx_class == UBX_CLASS_TIM)
	{
		ubx_tim_tp_t *gps_tim_tp = ubx_get_tim_tp();
		if (gps_tim_tp)
		{
			++loop_tim_tp;
			loop_tim_tp %= num_skipped_msg;
			if((print_nav_on_debug)&&(loop_tim_tp == 0))
			{
				print_util_dbg_print("MSG_TIM_TP GPS awake\r\n");
			}
		}
		ubx_tim_vrfy_t *gps_tim_vrfy = ubx_get_tim_vrfy();
		if (gps_tim_vrfy)
		{
			++loop_tim_vrfy;
			loop_tim_vrfy %= num_skipped_msg;
			if((print_nav_on_debug)&&(loop_tim_vrfy == 0))
			{
				print_util_dbg_print("MSG_TIM_VRFY");
				print_util_dbg_print(" itow :");
				print_util_dbg_print_num(gps_tim_vrfy->itow,10);
				print_util_dbg_print(" frac :");
				print_util_dbg_print_num(gps_tim_vrfy->frac,10);
				print_util_dbg_print(" delta_ms :");
				print_util_dbg_print_num(gps_tim_vrfy->delta_ms,10);
				print_util_dbg_print(" delta_ns :");
				print_util_dbg_print_num(gps_tim_vrfy->delta_ns,10);
			}
		}
		return false;
	}
	 
	if (ubx_class == UBX_CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS)
	{
		ubx_cfg_nav_settings_t *gps_nav_settings = ubx_get_nav_settings();
		
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
		if(gps_nav_settings)
		{
			print_util_dbg_print("Got engine settings ");
			print_util_dbg_print_num(gps_nav_settings->dyn_model,16);
			print_util_dbg_print("\r\n");
		}
		else
		{
			if (engine_nav_setting != GPS_ENGINE_NONE && !gps_nav_settings)
			{
				if(gps_nav_settings->dyn_model != engine_nav_setting)
				{
					nav_settings.dyn_model = engine_nav_setting;
					print_util_dbg_print("Send Nav settings");
					ubx_send_message_nav_settings(gps, UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, &nav_settings, sizeof(nav_settings));
				}				
			}
			print_util_dbg_print("No engine settings received ");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("\r\n");
		}
		return false;
	}
	
	if (ubx_class == UBX_CLASS_CFG && msg_id == MSG_CFG_SET_RATE)
	{
		ubx_cfg_msg_rate_t *gps_msg_rate;
		gps_msg_rate = ubx_get_msg_rate();
		
		if (gps_msg_rate)
		{
			print_util_dbg_print("Message CFG Rate 0x");
			print_util_dbg_print_num(gps_msg_rate->msg_class,16);
			print_util_dbg_print_num(gps_msg_rate->msg_id_rate,16);
			print_util_dbg_print_num(gps_msg_rate->rate,10);
			print_util_dbg_print("\r\n");
		}		
		return false;
	}
	
	if (ubx_class != UBX_CLASS_NAV)
	{
		print_util_dbg_print("Unexpected message 0x");
		print_util_dbg_print_num(ubx_class,16);
		print_util_dbg_print("02x 0x");
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("02x\r\n");
		if (++disable_counter == 0)
		{
			// disable future sends of this message id, but
			// only do this every 256 messages, as some
			// message types can't be disabled and we don't
			// want to get into an ack war
			
			print_util_dbg_print("Disabling message 0x");
			print_util_dbg_print_num(ubx_class,16);
			print_util_dbg_print("02x 0x");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("02x\r\n");
			ubx_configure_message_rate(gps,ubx_class, msg_id, 0);
		}
		return false;
	}
	
	// Class NAV:
	switch (msg_id)
	{
	case MSG_NAV_POSLLH:
		gps_pos_llh = ubx_get_pos_llh();
		if (gps_pos_llh)
		{
			++loop_pos_llh;
			loop_pos_llh %= num_skipped_msg;
			if (print_nav_on_debug && (loop_pos_llh == 0))
			{
				print_util_dbg_print("MSG_NAV_POSLLH");
				print_util_dbg_print(" itow :");
				print_util_dbg_print_num(gps_pos_llh->itow,10);
				print_util_dbg_print(" longitude :");
				print_util_dbg_print_num(gps_pos_llh->longitude,10);
				print_util_dbg_print(" latitude :");
				print_util_dbg_print_num(gps_pos_llh->latitude,10);
				print_util_dbg_print(" alt_ellips :");
				print_util_dbg_print_num(gps_pos_llh->altitude_ellipsoid,10);
				print_util_dbg_print(" alt_msl :");
				print_util_dbg_print_num(gps_pos_llh->altitude_msl,10);
				print_util_dbg_print(" horz_acc :");
				print_util_dbg_print_num(gps_pos_llh->horizontal_accuracy,10);
				print_util_dbg_print(" vert_acc :");
				print_util_dbg_print_num(gps_pos_llh->vertical_accuracy,10);
				print_util_dbg_print("\r\n");
			}
			
			gps->time_gps = gps_pos_llh->itow;
			gps->longitude = gps_pos_llh->longitude / 10000000.0f;
			gps->latitude = gps_pos_llh->latitude / 10000000.0f;
			gps->alt_elips = ((float)gps_pos_llh->altitude_ellipsoid) / 1000.0f;
			gps->altitude = ((float)gps_pos_llh->altitude_msl) / 1000.0f;
			gps->horizontal_accuracy = ((float)gps_pos_llh->horizontal_accuracy) / 1000.0f;
			gps->vertical_accuracy = ((float)gps_pos_llh->vertical_accuracy) / 1000.0f;
			
			new_position = true;
		}
		break;
		
	case MSG_NAV_STATUS:
		gps_status = ubx_get_status();
		
		if (gps_status)
		{
			++loop_status;
			loop_status %= num_skipped_msg;
			if (print_nav_on_debug && (loop_status == 0))
			{
				print_util_dbg_print("MSG_STATUS fix_type = 0x");
				print_util_dbg_print_num(gps_status->fix_type,16);
				print_util_dbg_print(", uptime =");
				print_util_dbg_print_num(gps_status->uptime,10);
				print_util_dbg_print("\r\n");
			}
			next_fix = (gps_status->fix_type == GPS_FIX_TYPE_3DFIX);
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
		gps_solution = ubx_get_solution();
		
		if (gps_solution)
		{
			++loop_solution;
			loop_solution %= num_skipped_msg;
			if (print_nav_on_debug && (loop_solution == 0))
			{
				print_util_dbg_print("MSG_SOL ");
				print_util_dbg_print("itow :");
				print_util_dbg_print_num(gps_solution->itow,10);
				print_util_dbg_print(" week :");
				print_util_dbg_print_num(gps_solution->week,10);
				print_util_dbg_print(" fix_type = 0x0");
				print_util_dbg_print_num(gps_solution->fix_type,16);
				print_util_dbg_print(" pos_acc_3d :");
				print_util_dbg_print_num(gps_solution->position_accuracy_3d,10);
				print_util_dbg_print(" ecefx :");
				print_util_dbg_print_num(gps_solution->ecef_x,10);
				print_util_dbg_print(" ecefy :");
				print_util_dbg_print_num(gps_solution->ecef_y,10);
				print_util_dbg_print(" ecefz :");
				print_util_dbg_print_num(gps_solution->ecef_z,10);
				print_util_dbg_print(" pos_DOP :");
				print_util_dbg_print_num(gps_solution->position_DOP,10);
				print_util_dbg_print(" num sat :");
				print_util_dbg_print_num(gps_solution->satellites,10);
				print_util_dbg_print("\r\n");
			}
			next_fix = (gps_solution->fix_type == GPS_FIX_TYPE_3DFIX);
			if (!next_fix)
			{
				gps->status = NO_FIX;
			}
			else
			{
				gps->status = GPS_OK;
			}
		
			gps->num_sats = gps_solution->satellites;
			gps->hdop = gps_solution->position_DOP;
		}
		break;
		
	case MSG_NAV_VELNED:
		gps_vel_ned = ubx_get_vel_ned();
		
		if (gps_vel_ned)
		{
			++loop_vel_ned;
			loop_vel_ned %= num_skipped_msg;
			if (print_nav_on_debug && (loop_vel_ned == 0))
			{
			
				print_util_dbg_print("MSG_NAV_VELNED ");
		
				print_util_dbg_print("itow :");
				print_util_dbg_print_num(gps_vel_ned->itow,10);
				print_util_dbg_print(" ned_north :");
				print_util_dbg_print_num(gps_vel_ned->ned_north,10);
				print_util_dbg_print(" ned_east :");
				print_util_dbg_print_num(gps_vel_ned->ned_east,10);
				print_util_dbg_print(" ned_down :");
				print_util_dbg_print_num(gps_vel_ned->ned_down,10);
				print_util_dbg_print(" speed_3d :");
				print_util_dbg_print_num(gps_vel_ned->speed_3d,10);
				print_util_dbg_print(" heading_2d :");
				print_util_dbg_print_num(gps_vel_ned->heading_2d,10);
				print_util_dbg_print(" speed_accuracy :");
				print_util_dbg_print_num(gps_vel_ned->speed_accuracy,10);
				print_util_dbg_print(" heading_accuracy :");
				print_util_dbg_print_num(gps_vel_ned->heading_accuracy,10);
				print_util_dbg_print("\r\n");
			}
			gps->time_gps         = gps_vel_ned->itow;
			gps->speed           = ((float)gps_vel_ned->speed_3d) / 100.; // m/s
			gps->ground_speed     = ((float)gps_vel_ned->ground_speed_2d) / 100.; // m/s
			gps->course          = ((float)gps_vel_ned->heading_2d) / 100000.; // Heading 2D deg * 100000 rescaled to deg * 100
			have_raw_velocity    = true;
			gps->north_speed      = ((float)gps_vel_ned->ned_north) / 100.0f;
			gps->east_speed       = ((float)gps_vel_ned->ned_east) / 100.;
			gps->vertical_speed   = ((float)gps_vel_ned->ned_down) / 100.;
			gps->speed_accuracy   = ((float)gps_vel_ned->speed_accuracy) / 100.;
			gps->heading_accuracy = gps_vel_ned->heading_accuracy;
			new_speed            = true;
		}
		break;
		
	case MSG_NAV_SVINFO:
		gps_sv_info = ubx_get_sv_info();
		
		if (gps_sv_info)
		{
			if (print_nav_on_debug)
			{
				print_util_dbg_print("MSG_NAV_SVINFO, num_channel:");
				print_util_dbg_print_num(gps_sv_info->num_ch,10);
				print_util_dbg_print("\r\n");
			}
		}
		
	default:
		print_util_dbg_print("Unexpected NAV message 0x");
		print_util_dbg_print_num(msg_id,10);
		print_util_dbg_print("\r\n");
		
		if (++disable_counter == 0)
		{
			print_util_dbg_print("Disabling NAV message 0x");
			print_util_dbg_print_num(msg_id,16);
			print_util_dbg_print("\r\n");
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
		data = engine_settings->dyn_model;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->fix_mode;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixed_alt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixed_alt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixed_alt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixed_alt);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint32(engine_settings->fixed_alt_var);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_lower_bytes_uint32(engine_settings->fixed_alt_var);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_mid_higher_bytes_uint32(engine_settings->fixed_alt_var);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint32(engine_settings->fixed_alt_var);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->min_elev;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->dr_limit;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->p_dop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->p_dop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->t_dop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->t_dop);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->p_acc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->p_acc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_lower_bytes_uint16(engine_settings->t_acc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = endian_higher_bytes_uint16(engine_settings->t_acc);
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->static_hold_thresh;
		update_checksum((uint8_t *)&data, 1, &ck_a, &ck_b);
		print_util_putnum(&gps->gps_stream_out, data, 16);
		data = engine_settings->dgps_timeout;
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


static ubx_nav_pos_llh_t * ubx_get_pos_llh()
{
	if (ubx_number_of_valid_pos_llh_message)
	{
		return ubx_last_pos_llh_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_status_t * ubx_get_status()
{
	if (ubx_number_of_valid_status_message)
	{
		return ubx_last_status_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_solution_t * ubx_get_solution()
{
	if (ubx_number_of_valid_solution_message)
	{
		return ubx_last_solution_message;
	}
	else
	{
		return 0;
	}
}

static ubx_nav_vel_ned_t * ubx_get_vel_ned()
 {
	if (ubx_number_of_valid_vel_ned_message)
	{
		return ubx_last_vel_ned_message;
	}
	else
	{
		return 0;
	}
}


static ubx_nav_sv_info_t * ubx_get_sv_info()
{
	if (ubx_number_of_valid_sv_info_message)
	{
		return ubx_last_sv_info_message;
	}
	else
	{
		return 0;
	}
}


static ubx_cfg_nav_settings_t * ubx_get_nav_settings()
{
	if (ubx_number_of_valid_nav_settings_message)
	{
		return ubx_last_nav_settings_message;
	}
	else
	{
		return 0;
	}
}


static ubx_cfg_msg_rate_t * ubx_get_msg_rate()
{
	if (ubx_number_of_valid_cfg_set_get_rate_message)
	{
		return ubx_last_cfg_set_get_rate_message;
	}
	else
	{
		return 0;
	}
}


static ubx_mon_rxr_struct_t * ubx_get_mon_rxr()
{
	if (ubx_number_of_valid_mon_rxr_message)
	{
		return ubx_last_mon_rxr_message;
	}
	else
	{
		return 0;
	}
}


static ubx_tim_tp_t * ubx_get_tim_tp()
{
	if(ubx_number_of_valid_tim_tp_message)
	{
		return ubx_last_tim_tp_message;
	}
	else
	{
		return 0;
	}
}


static ubx_tim_vrfy_t * ubx_get_tim_vrfy()
{
	if(ubx_number_of_valid_tim_vrfy_message)
	{
		return ubx_last_tim_vrfy_message;
	}
	else
	{
		return 0;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void gps_ublox_init(gps_t *gps, int32_t UID, usart_config_t usart_conf_gps)
{
	uart_int_set_usart_conf(UID, &usart_conf_gps);
	
	uart_int_init(UID);
	buffer_make_buffered_stream(&(gps->gps_buffer), &(gps->gps_stream_in));
	uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(gps->gps_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(gps->gps_stream_out));
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
		if ((tnow - idle_timer) > idle_timeout)
		{
			gps->status = NO_GPS;
			
			gps_ublox_reset(gps, engine_nav_setting);
			idle_timer = tnow;
		}
		
	}
	else
	{
		// reset the idle timer
		idle_timer = tnow;
		
		gps->time_last_msg = tnow;
		
		if(gps->status == GPS_OK)
		{
			// Check for horizontal accuracy
			if (gps->horizontal_accuracy < UBX_POSITION_PRECISION)
			{
				gps->horizontal_status = 1;
			}
			else
			{  
				gps->horizontal_status = 0;
			}
			// Check for vertical accuracy
			if (gps->vertical_accuracy < UBX_ALTITUDE_PRECISION)
			{
				gps->altitude_status = 1;
			}
			else
			{
				gps->altitude_status = 0;
			}
			// Check for speed accuracy
			if (gps->speed_accuracy < UBX_SPEED_PRECISION)
			{
				gps->speed_status = 1;
			}
			else
			{
				gps->speed_status = 0;
			}
			// Check for heading accuracy
			if (gps->heading_accuracy < UBX_HEADING_PRECISION)
			{
				gps->course_status = 1;
			}
			else
			{
				gps->course_status = 0;
			}
			
			gps->accuracy_status = gps->horizontal_status & gps->altitude_status & gps->speed_status & gps->course_status;
			
			// speed approximation with the 
// 			if (!have_raw_velocity)
// 			{
// 				float gps_heading = to_rad(gps->course);
// 				float cos_heading,sin_heading;
// 				
// 				cos_heading = cosf(gps_heading);
// 				sin_heading = sinf(gps_heading);
// 				
// 				gps->northspeed = gps->ground_speed * cos_heading;
// 				gps->eastspeed = gps->ground_speed * sin_heading;
// 				
// 				// no good way to get descent rate
// 				gps->vertical_speed = 0;
// 			}
		}
		else
		{
			gps->horizontal_status = 0;
			gps->altitude_status = 0;
			gps->speed_status = 0;
			gps->course_status = 0;
			
			gps->accuracy_status = 0;
		}
	}
}