// MESSAGE RAW_DATA_STREAM_DESCRIPTOR PACKING

#define MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR 153

typedef struct __mavlink_raw_data_stream_descriptor_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 uint32_t sample_count; ///< Total Number of samples
 uint8_t stream_id; ///< Stream ID
 uint8_t packets_per_block; ///< Number of packets per block (0 if continuous stream)
 char name[10]; ///< Name
} mavlink_raw_data_stream_descriptor_t;

#define MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR_LEN 20
#define MAVLINK_MSG_ID_153_LEN 20

#define MAVLINK_MSG_RAW_DATA_STREAM_DESCRIPTOR_FIELD_NAME_LEN 10

#define MAVLINK_MESSAGE_INFO_RAW_DATA_STREAM_DESCRIPTOR { \
	"RAW_DATA_STREAM_DESCRIPTOR", \
	5, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_raw_data_stream_descriptor_t, time_boot_ms) }, \
         { "sample_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_raw_data_stream_descriptor_t, sample_count) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_raw_data_stream_descriptor_t, stream_id) }, \
         { "packets_per_block", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_raw_data_stream_descriptor_t, packets_per_block) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 10, offsetof(mavlink_raw_data_stream_descriptor_t, name) }, \
         } \
}


/**
 * @brief Pack a raw_data_stream_descriptor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param sample_count Total Number of samples
 * @param name Name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_data_stream_descriptor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t stream_id, uint8_t packets_per_block, uint32_t sample_count, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint32_t(buf, 4, sample_count);
	_mav_put_uint8_t(buf, 8, stream_id);
	_mav_put_uint8_t(buf, 9, packets_per_block);
	_mav_put_char_array(buf, 10, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_raw_data_stream_descriptor_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sample_count = sample_count;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR;
	return mavlink_finalize_message(msg, system_id, component_id, 20, 225);
}

/**
 * @brief Pack a raw_data_stream_descriptor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param sample_count Total Number of samples
 * @param name Name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_data_stream_descriptor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t stream_id,uint8_t packets_per_block,uint32_t sample_count,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint32_t(buf, 4, sample_count);
	_mav_put_uint8_t(buf, 8, stream_id);
	_mav_put_uint8_t(buf, 9, packets_per_block);
	_mav_put_char_array(buf, 10, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_raw_data_stream_descriptor_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sample_count = sample_count;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 225);
}

/**
 * @brief Encode a raw_data_stream_descriptor struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_data_stream_descriptor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_data_stream_descriptor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_data_stream_descriptor_t* raw_data_stream_descriptor)
{
	return mavlink_msg_raw_data_stream_descriptor_pack(system_id, component_id, msg, raw_data_stream_descriptor->time_boot_ms, raw_data_stream_descriptor->stream_id, raw_data_stream_descriptor->packets_per_block, raw_data_stream_descriptor->sample_count, raw_data_stream_descriptor->name);
}

/**
 * @brief Send a raw_data_stream_descriptor message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param stream_id Stream ID
 * @param packets_per_block Number of packets per block (0 if continuous stream)
 * @param sample_count Total Number of samples
 * @param name Name
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_data_stream_descriptor_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t stream_id, uint8_t packets_per_block, uint32_t sample_count, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint32_t(buf, 4, sample_count);
	_mav_put_uint8_t(buf, 8, stream_id);
	_mav_put_uint8_t(buf, 9, packets_per_block);
	_mav_put_char_array(buf, 10, name, 10);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR, buf, 20, 225);
#else
	mavlink_raw_data_stream_descriptor_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sample_count = sample_count;
	packet.stream_id = stream_id;
	packet.packets_per_block = packets_per_block;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_DATA_STREAM_DESCRIPTOR, (const char *)&packet, 20, 225);
#endif
}

#endif

// MESSAGE RAW_DATA_STREAM_DESCRIPTOR UNPACKING


/**
 * @brief Get field time_boot_ms from raw_data_stream_descriptor message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_raw_data_stream_descriptor_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field stream_id from raw_data_stream_descriptor message
 *
 * @return Stream ID
 */
static inline uint8_t mavlink_msg_raw_data_stream_descriptor_get_stream_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field packets_per_block from raw_data_stream_descriptor message
 *
 * @return Number of packets per block (0 if continuous stream)
 */
static inline uint8_t mavlink_msg_raw_data_stream_descriptor_get_packets_per_block(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field sample_count from raw_data_stream_descriptor message
 *
 * @return Total Number of samples
 */
static inline uint32_t mavlink_msg_raw_data_stream_descriptor_get_sample_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field name from raw_data_stream_descriptor message
 *
 * @return Name
 */
static inline uint16_t mavlink_msg_raw_data_stream_descriptor_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 10,  10);
}

/**
 * @brief Decode a raw_data_stream_descriptor message into a struct
 *
 * @param msg The message to decode
 * @param raw_data_stream_descriptor C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_data_stream_descriptor_decode(const mavlink_message_t* msg, mavlink_raw_data_stream_descriptor_t* raw_data_stream_descriptor)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_data_stream_descriptor->time_boot_ms = mavlink_msg_raw_data_stream_descriptor_get_time_boot_ms(msg);
	raw_data_stream_descriptor->sample_count = mavlink_msg_raw_data_stream_descriptor_get_sample_count(msg);
	raw_data_stream_descriptor->stream_id = mavlink_msg_raw_data_stream_descriptor_get_stream_id(msg);
	raw_data_stream_descriptor->packets_per_block = mavlink_msg_raw_data_stream_descriptor_get_packets_per_block(msg);
	mavlink_msg_raw_data_stream_descriptor_get_name(msg, raw_data_stream_descriptor->name);
#else
	memcpy(raw_data_stream_descriptor, _MAV_PAYLOAD(msg), 20);
#endif
}
