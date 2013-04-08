/*
* gps_ublox.c
* 
* created on March 29 2013
* author: N. Dousse
*/

#include "gps_ublox.h"

#include "boardsupport.h"



board_hardware_t *board;

void init_gps_ubx(enum GPS_Engine_Setting _nav_setting)
{
	board = get_board_hardware();
	
	board->gps_stream_in.flush(board->gps_stream_in.data);
	board->gps_stream_out.flush(board->gps_stream_out.data);
	
	uint8_t epoch = TIME_OF_WEEK;
	idleTimeout = 1200;
	
	//configure_gps();
	
	nav_setting = _nav_setting;
	
	board->GPS_data.status = NO_FIX;
	board->GPS_data.num_sats = 0;
	
	next_fix = false;
	new_data = false;
	valid_read = false;
	have_raw_velocity = false;
	fix = false;
	
	last_fix_time = 0;
	
	
	
	step = 0;
	}


// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool ubx_read(void)
{
	uint8_t data;
	bool msg_ok = false;
	
	while(buffer_bytes_available(&(board->gps_buffer)))
	{
		data = buffer_get(&(board->gps_buffer));
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
			cksum_b = cksum_a = data; // reset the checksum accumulators
			break;
		case 3:
			step++;
			cksum_b += (cksum_a += data); // checksum byte
			msg_id = data;
			break;
		case 4:
			step++;
			cksum_b += (cksum_a += data); // checksum byte
			payload_length = data;
			switch(msg_id)
			{
				case MSG_NAV_POSLLH:
					if(payload_length == UBX_SIZE_NAV_POSLLH)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentPosllhMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastPosllhMessage;
						ubx_validMessage = &ubx_numberOfValidPosllhMessage;
					}
					break;
				case MSG_NAV_STATUS:
					if(payload_length == UBX_SIZE_NAV_STATUS)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentStatusMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastStatusMessage;
						ubx_validMessage = &ubx_numberOfValidStatusMessage;
					}
					break;
				case MSG_NAV_SOL:
					if(payload_length == UBX_SIZE_NAV_SOL)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentSolutionMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastSolutionMessage;
						ubx_validMessage = &ubx_numberOfValidSolutionMessage;
					}
					break;
				case MSG_NAV_VELNED:
					if(payload_length == UBX_SIZE_NAV_VELNED)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentVelnedMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastVelnedMessage;
						ubx_validMessage = &ubx_numberOfValidVelnedMessage;
					}
					break;
				case MSG_NAV_SVINFO:
					if(payload_length == UBX_SIZE_NAV_SVINFO)
					{
						ubx_currentMessage = (unsigned char**)&ubx_currentSVInfoMessage;
						ubx_lastMessage = (unsigned char**)&ubx_lastSVInfoMessage;
						ubx_validMessage = &ubx_numberOfValidSVInfoMessage;
					}
					break;
				default:
					step = 0;
					dbg_print("Unexpected NAV message 0x");
					dbg_print_num(msg_id,16);
					dbg_print("02x\n");
					goto reset;
			}
			
		case 5:
			step++;
			cksum_b += (cksum_a += data); // checksum byte
			payload_length += (uint16_t)(data<<8);
			if (payload_length > 512)
			{
				// we assume very large payloads are line noise
				dbg_print("large payload");
				payload_length = 0;
				step = 0;
				goto reset;
			}
			payload_counter = 0; // prepare to receive payload
			break;
		case 6:
			cksum_b += (cksum_a += data); // checksum byte
			//if (payload_counter < sizeof(buffergps))
			//{
				(*ubx_currentMessage)[payload_counter++] = data;
				//buffergps.bytes[payload_counter] = data;
				
			//}
			//if (++payload_counter == payload_length)
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
			if (ubx_process_data())
			{
				msg_ok = true;
			}
		}
		
	}
	return msg_ok;
}

bool ubx_process_data(void)
{
	ubx_nav_posllh *gpsPosllh;
	ubx_nav_velned *gpsVelned;
	ubx_nav_status *gpsStatus;
	ubx_nav_solution *gpsSolution;
	ubx_nav_SVInfo *gpsSVInfo;
	
	
	if (ubxclass == UBX_CLASS_ACK)
	{
		dbg_print_num(msg_id,16);
		return false;
	}
	if (ubxclass == UBX_CLASS_CFG && msg_id == MSG_CFG_NAV_SETTINGS)
	{
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
		dbg_print("Got engine settings ");
		dbg_print_num(nav_settings.dynModel,10);
		
		if (nav_setting != GPS_ENGINE_NONE && nav_settings.dynModel != nav_setting)
		{
			nav_settings.dynModel = nav_setting;
			ubx_send_message(UBX_CLASS_CFG,MSG_CFG_NAV_SETTINGS, &nav_settings,sizeof(nav_settings));
		}
		return false;
	}
	
	if (ubxclass != UBX_CLASS_NAV)
	{
		dbg_print("Unexpected message 0x");
		dbg_print_num(ubxclass,16);
		dbg_print("02x 0x");
		dbg_print_num(msg_id,16);
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
			dbg_print("02x");
			ubx_configure_message_rate(ubxclass, msg_id, 0);
		}
		return false;
	}
	
	switch (msg_id)
	{
	case MSG_NAV_POSLLH:
		gpsPosllh = ubx_GetPosllh();
		dbg_print("MSG_NAV_POSLLH");
		board->GPS_data.timegps = gpsPosllh->itow;
		board->GPS_data.longitude = gpsPosllh->longitude;
		board->GPS_data.latitude = gpsPosllh->latitude;
		board->GPS_data.alt_elips = gpsPosllh->altitude_ellipsoid;
		board->GPS_data.altitude = gpsPosllh->altitude_msl;
		board->GPS_data.status = next_fix;
		
		new_position = true;
		break;
	case MSG_NAV_STATUS:
		gpsStatus = ubx_GetStatus();
		dbg_print("MSG_STATUS fix_status=");
		dbg_print_num(gpsStatus->fix_status,16);
		dbg_print(" fix_type =");
		dbg_print_num(gpsStatus->fix_type,16);
		dbg_print(", uptime =");
		dbg_print_num(gpsStatus->uptime,16);
		dbg_print("\n");
		
		next_fix = (gpsStatus->fix_status & NAV_STATUS_FIX_VALID) && (gpsStatus->fix_type == GPS_FIX_TYPE_3DFIX);
		if (!next_fix)
		{
			board->GPS_data.status = NO_FIX;
		}
		break;
	case MSG_NAV_SOL:
		gpsSolution = ubx_GetSolution();
		dbg_print("MSG_SOL fix status = ");
		dbg_print_num(gpsSolution->fix_status,16);
		dbg_print(" fix_type = ");
		dbg_print_num(gpsSolution->fix_type,16);
		dbg_print("\n");

		next_fix = (gpsSolution->fix_status & NAV_STATUS_FIX_VALID) && (gpsSolution->fix_type == GPS_FIX_TYPE_3DFIX);
		if (!next_fix)
		{
			board->GPS_data.status = NO_FIX;
		}
		
		board->GPS_data.num_sats = gpsSolution->satellites;
		board->GPS_data.hdop = gpsSolution->position_DOP;
		break;
	case MSG_NAV_VELNED:
		gpsVelned = ubx_GetVelned();
		dbg_print("MSG_NAV_VELNED");
		board->GPS_data.timegps = gpsVelned->itow;
		board->GPS_data.speed        = gpsVelned->speed_3d; // cm/s
		board->GPS_data.groundSpeed = gpsVelned->groundSpeed_2d; // cm/s
		board->GPS_data.course = gpsVelned->heading_2d / 1000; // Heading 2D deg * 100000 rescaled to deg * 100
		have_raw_velocity = true;
		board->GPS_data.northSpeed  = gpsVelned->ned_north;
		board->GPS_data.eastSpeed   = gpsVelned->ned_east;
		board->GPS_data.verticalSpeed   = gpsVelned->ned_down;
		board->GPS_data.speedAccuracy = gpsVelned->speed_accuracy;
		board->GPS_data.headingAccuracy = gpsVelned->heading_accuracy;
		new_speed = true;
		break;
	case MSG_NAV_SVINFO:
		gpsSVInfo = ubx_GetSVInfo();
		dbg_print("MSG_NAV_SVINFO");
		
		
		
	default:
		dbg_print("Unexpected NAV message 0x");
		dbg_print_num(msg_id,16);
		dbg_print("02x\n");
		
		if (++disable_counter == 0) {
			//Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
			dbg_print("Disabling NAV message 0x");
			dbg_print_num(msg_id,16);
			dbg_print("02x\n");
			ubx_configure_message_rate(UBX_CLASS_NAV, msg_id, 0);
		}
		return false;
	}
	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (new_position && new_speed) {
		new_speed = new_position = false;
		fix_count++;
		if (fix_count == 100) {
			// ask for nav settings every 20 seconds
			dbg_print("Asking for engine setting\n");
			ubx_send_message(UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
		}
		return true;
	}
	return false;
}

void update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	while (len--) {
		*ck_a += *data;
		*ck_b += *ck_a;
		data++;
	}
}

void ubx_send_message(uint8_t msg_class, uint8_t _msg_id, void *msg, uint8_t size)
{
	ubx_header header;
	uint8_t ck_a=0, ck_b=0;
	header.preamble1 = UBX_PREAMBLE1;
	header.preamble2 = UBX_PREAMBLE2;
	header.msg_class = msg_class;
	header.msg_id_header    = _msg_id;
	header.length    = size;

	update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, &ck_a, &ck_b);
	update_checksum((uint8_t *)msg, size, &ck_a, &ck_b);

	board->gps_stream_out.put(board->gps_stream_out.data,&header);
	board->gps_stream_out.put(board->gps_stream_out.data,msg);
	board->gps_stream_out.put(board->gps_stream_out.data,&ck_a);
	board->gps_stream_out.put(board->gps_stream_out.data,&ck_b);
}

void ubx_configure_message_rate(uint8_t msg_class, uint8_t _msg_id, uint8_t rate)
{
	ubx_cfg_msg_rate msg;
	msg.msg_class = msg_class;
	msg.msg_id_rate    = _msg_id;
	msg.rate          = rate;
	ubx_send_message(UBX_CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
}

void configure_gps(void)
{
	ubx_cfg_nav_rate msg;
	const unsigned baudrates[4] = {9600U, 19200U, 38400U, 57600U};

	// the GPS may be setup for a different baud rate. This ensures
	// it gets configured correctly
// 	for (uint8_t i=0; i<4; i++) {
// 		
// 		_write_progstr_block(board, _ublox_set_binary, _ublox_set_binary_size);
// 		
// 		while (board->tx_pending()) {
// 			
// 		}
// 	}
	
	//_port->begin(38400U);

	// ask for navigation solutions every 200ms
	msg.measure_rate_ms = 200;
	msg.nav_rate        = 1;
	msg.timeref         = 0;     // UTC time
	ubx_send_message(UBX_CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));

	// ask for the messages we parse to be sent on every navigation solution
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_POSLLH, 1);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_STATUS, 1);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_SOL, 1);
	ubx_configure_message_rate(UBX_CLASS_NAV, MSG_NAV_VELNED, 1);

	// ask for the current navigation settings
	//Debug("Asking for engine setting\n");
	dbg_print("Asking for engine setting\n");
	ubx_send_message(UBX_CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}

bool ubx_detect(uint8_t data)
{
	static uint8_t payload_length, payload_counter;
	static uint8_t step;
	static uint8_t ck_a, ck_b;

	reset:
	switch (step) {
		case 1:
		if (UBX_PREAMBLE2 == data) {
			step++;
			break;
		}
		step = 0;
		case 0:
		if (UBX_PREAMBLE1 == data)
		step++;
		break;
		case 2:
		step++;
		ck_b = ck_a = data;
		break;
		case 3:
		step++;
		ck_b += (ck_a += data);
		break;
		case 4:
		step++;
		ck_b += (ck_a += data);
		payload_length = data;
		break;
		case 5:
		step++;
		ck_b += (ck_a += data);
		payload_counter = 0;
		break;
		case 6:
		ck_b += (ck_a += data);
		if (++payload_counter == payload_length)
		step++;
		break;
		case 7:
		step++;
		if (ck_a != data) {
			step = 0;
			goto reset;
		}
		break;
		case 8:
		step = 0;
		if (ck_b == data) {
			// a valid UBlox packet
			return true;
			} else {
			goto reset;
		}
	}
	return false;
}

void gps_update(void)
{
	bool result;
	uint32_t tnow;
	
	result = ubx_read();
	
	tnow = get_millis();
	
	if (! result)
	{
		if ((tnow - idleTimer) > idleTimeout)
		{
			dbg_print("gps read timeout ");
			dbg_print_num(tnow,16);
			
			board->GPS_data.status = NO_GPS;
			
			init_gps_ubx(nav_setting);
			idleTimer = tnow;
		}
		
		} else {
		board->GPS_data.status = board->GPS_data.status ? GPS_OK : NO_FIX;
		
		valid_read = true;
		new_data = true;

		// reset the idle timer
		idleTimer = tnow;
		
		if (board->GPS_data.status == GPS_OK)
		{
			last_fix_time = idleTimer;
			last_ground_speed_cm = board->GPS_data.groundSpeed;
			
			if (have_raw_velocity)
			{
				// the GPS is able to give us velocity numbers directly
				velocity_north = board->GPS_data.northSpeed * 0.01f;
				velocity_east  = board->GPS_data.eastSpeed * 0.01f;
				velocity_down  = board->GPS_data.verticalSpeed * 0.01f;
			} else {
				float gps_heading = ToRad(board->GPS_data.course * 0.01f);
				float gps_speed   = board->GPS_data.groundSpeed * 0.01f;
				float sin_heading, cos_heading;

				cos_heading = cosf(gps_heading);
				sin_heading = sinf(gps_heading);

				velocity_north = gps_speed * cos_heading;
				velocity_east  = gps_speed * sin_heading;

				// no good way to get descent rate
				velocity_down  = 0;
			}
		}
	}
}


/*!
*	This function returns a pointer to the last NAV-POSLLH message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid posllh message, or 0.
*/
ubx_nav_posllh * ubx_GetPosllh()
{
	if (ubx_numberOfValidPosllhMessage)
	return ubx_lastPosllhMessage;
	else
	return 0;
}

/*!
*	This function returns a pointer to the last NAV-STATUS message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid status message, or 0.
*/
ubx_nav_status * ubx_GetStatus()
{
	if (ubx_numberOfValidStatusMessage)
	return ubx_lastStatusMessage;
	else
	return 0;
}

/*!
*	This function returns a pointer to the last NAV-VELNED message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid velned message, or 0.
*/
ubx_nav_solution * ubx_GetSolution()
{
	if (ubx_numberOfValidSolutionMessage)
	return ubx_lastSolutionMessage;
	else
	return 0;
}

/*!
*	This function returns a pointer to the last NAV-VELNED message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid velned message, or 0.
*/
ubx_nav_velned * ubx_GetVelned()
{
	if (ubx_numberOfValidVelnedMessage)
	return ubx_lastVelnedMessage;
	else
	return 0;
}

/*!
*	This function returns a pointer to the last NAV-SVINFO message that was received
*	Warning: the values of the message must be read very quickly after the call to this function as buffer may be swapped in an interruption
*
*	@return A pointer to the last valid status message, or 0.
*/
ubx_nav_SVInfo * ubx_GetSVInfo()
{
	if (ubx_numberOfValidSVInfoMessage)
	return ubx_lastSVInfoMessage;
	else
	return 0;
}

float ToRad(float numdeg)
{
	return numdeg * DEG2RAD;
}