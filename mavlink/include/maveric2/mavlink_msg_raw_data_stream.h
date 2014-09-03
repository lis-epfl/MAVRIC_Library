// MESSAGE RAW_DATA_STREAM PACKING

#define MAVLINK_MSG_ID_RAW_DATA_STREAM 152

typedef struct __mavlink_raw_data_stream_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int16_t values[64]; ///< raw sample values
 uint8_t stream_id; ///< Stream ID
 uint8_t packets_per_block; ///< Number of packets per block (0 if continuous stream)
 uint8_t packet_id; ///< packet counter
 uint8_t sample_count; ///< Number of valid samples in this packet
} mavlink_raw_data_stream_t;

#define MAVLINK_MSG_ID_RAW_DATA_STREAM_LEN 136
#define MAVLINK_MSG_ID_152_LEN 136

#define MAVLINK_MSG_RAW_DATA_STREAM_FIELD_VALUES_LEN 64

#define MAVLINK_MESSAGE_INFO_RAW_DATA_STREAM { \
	"RAW_DATA_STREAM", \
	6, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_raw_data_stream_t, time_boot_ms) }, \
         { "values", NULL, MAVLINK_TYPE_INT16_T, 64, 4, offsetof(mavlink_raw_data_stream_t, values) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_raw_data_stream_t, stream_id) }, \
         { "packets_per_block", NULL, MAVLINK_TYPE_UINT8_T, 0, 133, offsetof(mavlink_raw_data_stream_t, packets_per_block) }, \
         { "packet_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 134, offsetof(mavlink_raw_data_stream_t, packet_id) }, \
         { "sample_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 135, offsetof(mavlink_raw_data_stream_t, sample_count) }, \
         } \
}


/**
 * @brief Pack a raw_data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param packet_id packet counter
 * @param sample_count Number of valid samples in this packet
 * @param values raw sample values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t stream_id, uint8_t packets_per_block, uint8_t packet_id, uint8_t sample_count, const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, stream_id);
	_mav_put_uint8_t(buf, 133, packets_per_block);
	_mav_put_uint8_t(buf, 134, packet_id);
	_mav_put_uint8_t(buf, 135, sample_count);
	_mav_put_int16_t_array(buf, 4, values, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 136);
#else
	mavlink_raw_data_stream_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	packet.packet_id = packet_id;
	packet.sample_count = sample_count;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 136);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_DATA_STREAM;
	return mavlink_finalize_message(msg, system_id, component_id, 136, 43);
}

/**
 * @brief Pack a raw_data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param packet_id packet counter
 * @param sample_count Number of valid samples in this packet
 * @param values raw sample values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t stream_id,uint8_t packets_per_block,uint8_t packet_id,uint8_t sample_count,const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, stream_id);
	_mav_put_uint8_t(buf, 133, packets_per_block);
	_mav_put_uint8_t(buf, 134, packet_id);
	_mav_put_uint8_t(buf, 135, sample_count);
	_mav_put_int16_t_array(buf, 4, values, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 136);
#else
	mavlink_raw_data_stream_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	packet.packet_id = packet_id;
	packet.sample_count = sample_count;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 136);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_DATA_STREAM;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 136, 43);
}

/**
 * @brief Encode a raw_data_stream struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_data_stream_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_data_stream_t* raw_data_stream)
{
	return mavlink_msg_raw_data_stream_pack(system_id, component_id, msg, raw_data_stream->time_boot_ms, raw_data_stream->stream_id, raw_data_stream->packets_per_block, raw_data_stream->packet_id, raw_data_stream->sample_count, raw_data_stream->values);
}

/**
 * @brief Send a raw_data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param packet_id packet counter
 * @param sample_count Number of valid samples in this packet
 * @param values raw sample values
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_data_stream_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t stream_id, uint8_t packets_per_block, uint8_t packet_id, uint8_t sample_count, const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, stream_id);
	_mav_put_uint8_t(buf, 133, packets_per_block);
	_mav_put_uint8_t(buf, 134, packet_id);
	_mav_put_uint8_t(buf, 135, sample_count);
	_mav_put_int16_t_array(buf, 4, values, 64);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_DATA_STREAM, buf, 136, 43);
#else
	mavlink_raw_data_stream_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	packet.packet_id = packet_id;
	packet.sample_count = sample_count;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_DATA_STREAM, (const char *)&packet, 136, 43);
#endif
}

#endif

// MESSAGE RAW_DATA_STREAM UNPACKING


/**
 * @brief Get field time_boot_ms from raw_data_stream message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_raw_data_stream_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field stream_id from raw_data_stream message
 *
 * @return Stream ID
 */
static inline uint8_t mavlink_msg_raw_data_stream_get_stream_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  132);
}

/**
 * @brief Get field packets_per_block from raw_data_stream message
 *
 * @return Number of packets per block (0 if continuous stream)
 */
static inline uint8_t mavlink_msg_raw_data_stream_get_packets_per_block(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  133);
}

/**
 * @brief Get field packet_id from raw_data_stream message
 *
 * @return packet counter
 */
static inline uint8_t mavlink_msg_raw_data_stream_get_packet_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  134);
}

/**
 * @brief Get field sample_count from raw_data_stream message
 *
 * @return Number of valid samples in this packet
 */
static inline uint8_t mavlink_msg_raw_data_stream_get_sample_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  135);
}

/**
 * @brief Get field values from raw_data_stream message
 *
 * @return raw sample values
 */
static inline uint16_t mavlink_msg_raw_data_stream_get_values(const mavlink_message_t* msg, int16_t *values)
{
	return _MAV_RETURN_int16_t_array(msg, values, 64,  4);
}

/**
 * @brief Decode a raw_data_stream message into a struct
 *
 * @param msg The message to decode
 * @param raw_data_stream C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_data_stream_decode(const mavlink_message_t* msg, mavlink_raw_data_stream_t* raw_data_stream)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_data_stream->time_boot_ms = mavlink_msg_raw_data_stream_get_time_boot_ms(msg);
	mavlink_msg_raw_data_stream_get_values(msg, raw_data_stream->values);
	raw_data_stream->stream_id = mavlink_msg_raw_data_stream_get_stream_id(msg);
	raw_data_stream->packets_per_block = mavlink_msg_raw_data_stream_get_packets_per_block(msg);
	raw_data_stream->packet_id = mavlink_msg_raw_data_stream_get_packet_id(msg);
	raw_data_stream->sample_count = mavlink_msg_raw_data_stream_get_sample_count(msg);
#else
	memcpy(raw_data_stream, _MAV_PAYLOAD(msg), 136);
#endif
}
