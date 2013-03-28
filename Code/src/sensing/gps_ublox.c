

#include "gps_ublox.h"

#include "boardsupport.h"



board_hardware_t *board;

void init_gps_ubx(GPS_Engine_Setting _nav_setting)
{
	board = get_board_hardware();
	
	board->gps_stream_in.flush(board->gps_stream_in.data);
	board->gps_stream_out.flush(board->gps_stream_out.data);
	
	epoch = TIME_OF_WEEK;
	idleTimeout = 1200;
	
	//configure_gps();
	
	nav_setting = _nav_setting;
	
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
			class = data;
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
			break;
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
			if (payload_counter < sizeof(buffergps))
			{
				buffergps.bytes[payload_counter] = data;
			}
			if (++payload_counter == payload_length)
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
			if (read_msg())
			{
				msg_ok = true;
			}
		}
		
	}
	return msg_ok;
}

bool ubx_process_data(void)
{
	if (ubxclass == UBX_CLASS_ACK)
	{
		dbg_print_num(msg_id);
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
		dbg_print_num(buffergps.nav_settings.dynModel,10);
		
		if (nav_setting != GPS_ENGINE_NONE && buffergps.nav_settings.dynModel != nav_setting)
		{
			buffergps.nav_settings.dynModel = nav_setting;
			ubx_send_message(UBX_CLASS_CFG,MSG_CFG_NAV_SETTINGS, &buffergps.nav_settings,sizeof(buffergps.nav_settings));
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
			
			//Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
			dbg_print("Disabling message 0x");
			dbg_print_num(ubxclass,16);
			dbg_print("02x 0x");
			dbg_print_num(_msg_id);
			dbg_print("02x");
			ubxconfigure_message_rate(_class, _msg_id, 0);
		}
		return false;
	}
	
	switch (msg_id)
	{
	case MSG_NAV_POSLLH:
		board->GPS_data.itow = buffergps.posllh.time;
		board->GPS_data.longitude = buffergps.posllh.longitude;
		board->GPS_data.latitude = buffergps.posllh.latitude;
		board->GPS_data.altitude = buffergps.posllh.altitude_msl;
		board->GPS_data.itow = buffergps.posllh.time;
		board->GPS_data.fix = next_fix;
		
		new_position = true;
		break;
	case MSG_NAV_STATUS:
		dbg_print("MSG_STATUS fix_status=");
		dbg_print_num(_buffer.status.fix_status);
		dbg_print(" fix_type =");
		dbg_print_num(_buffer.status.fix_type);
		dbg_print("\n");
		
		next_fix = (buffergps.status.fix_status & NAV_STATUS_FIX_VALID) && (buffergps.status.fix_type == GPS_FIX_TYPE_3DFIX)
		if (!next_fix)
		{
			board->GPS_data.fix = false;
		}
		break;
	case MSG_NAV_SOL:
		dbg_print("MSG_SOL fix status = ");
		dbg_print_num(buffergps.solution.fix_status,16);
		dbg_print(" fix_type = ");
		dbg_print_num(_buffer.solution.fix_type,16);
		dbg_print("\n");

		next_fix = (buffergps.solution.fix_status & NAV_STATUS_FIX_VALID) && (buffergps.solution.fix_type == GPS_FIX_TYPE_3DFIX);
		if (!next_fix)
		{
			board->GPS_data.fix = false;
		}
		
		board->GPS_data.num_sats = buffergps.solution.satellites;
		board->GPS_data.hdop = buffergps.solution.position_DOP;
		break;
	case MSG_VELNED:
		dbg_print("MSG_VELNED");
		board->GPS_data.speed        = _buffer.velned.speed_3d; // cm/s
		board->GPS_data.groundSpeed = _buffer.velned.speed_2d; // cm/s
		board->GPS_data.course = _buffer.velned.heading_2d / 1000; // Heading 2D deg * 100000 rescaled to deg * 100
		have_raw_velocity = true;
		board->GPS_data.northSpeed  = _buffer.velned.ned_north;
		board->GPS_data.eastSpeed   = _buffer.velned.ned_east;
		board->GPS_data.verticalSpeed   = _buffer.velned.ned_down;
		new_speed = true;
		break;
	default:
		dbg_print("Unexpected NAV message 0x");
		dbg_print_num(msg_id,16);
		dbg_print("02x\n");
		
		if (++_disable_counter == 0) {
			//Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
			dbg_print("Disabling NAV message 0x");
			dbg_print_num(msg_id,16);
			dbg_print("02x\n");
			ubx_configure_message_rate(CLASS_NAV, msg_id, 0);
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
			ubx_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
		}
		return true;
	}
	return false;
}

void update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

void ubx_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
	ubx_header header;
	uint8_t ck_a=0, ck_b=0;
	header.preamble1 = PREAMBLE1;
	header.preamble2 = PREAMBLE2;
	header.msg_class = msg_class;
	header.msg_id    = msg_id;
	header.length    = size;

	update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
	update_checksum((uint8_t *)msg, size, ck_a, ck_b);

	board->gps_stream_out.put(board->gps_stream_out.data,&header);
	board->gps_stream_out.put(board->gps_stream_out.data,msg);
	board->gps_stream_out.put(board->gps_stream_out.data,&ck_a);
	board->gps_stream_out.put(board->gps_stream_out.data,&ck_b);
}

void ubx_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
	ubx_cfg_msg_rate msg;
	msg.msg_class = msg_class;
	msg.msg_id    = msg_id;
	msg.rate          = rate;
	ubx_send_message(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
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
	ubx_send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));

	// ask for the messages we parse to be sent on every navigation solution
	ubx_configure_message_rate(CLASS_NAV, MSG_POSLLH, 1);
	ubx_configure_message_rate(CLASS_NAV, MSG_STATUS, 1);
	ubx_configure_message_rate(CLASS_NAV, MSG_SOL, 1);
	ubx_configure_message_rate(CLASS_NAV, MSG_VELNED, 1);

	// ask for the current navigation settings
	//Debug("Asking for engine setting\n");
	dbg_print("Asking for engine setting\n");
	ubx_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}

bool ubx_detect(uint8_t data)
{
	static uint8_t payload_length, payload_counter;
	static uint8_t step;
	static uint8_t ck_a, ck_b;

	reset:
	switch (step) {
		case 1:
		if (PREAMBLE2 == data) {
			step++;
			break;
		}
		step = 0;
		case 0:
		if (PREAMBLE1 == data)
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