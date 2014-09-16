// MESSAGE RADAR_RAW_DATA PACKING

#define MAVLINK_MSG_ID_RADAR_RAW_DATA 152

typedef struct __mavlink_radar_raw_data_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int16_t values[64]; ///< raw sample values
 uint8_t sensor_id; ///< Sensor ID
} mavlink_radar_raw_data_t;

#define MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN 133
#define MAVLINK_MSG_ID_152_LEN 133

#define MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC 213
#define MAVLINK_MSG_ID_152_CRC 213

#define MAVLINK_MSG_RADAR_RAW_DATA_FIELD_VALUES_LEN 64

#define MAVLINK_MESSAGE_INFO_RADAR_RAW_DATA { \
	"RADAR_RAW_DATA", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_raw_data_t, time_boot_ms) }, \
         { "values", NULL, MAVLINK_TYPE_INT16_T, 64, 4, offsetof(mavlink_radar_raw_data_t, values) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 132, offsetof(mavlink_radar_raw_data_t, sensor_id) }, \
         } \
}


/**
 * @brief Pack a radar_raw_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param values raw sample values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_raw_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t sensor_id, const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, sensor_id);
	_mav_put_int16_t_array(buf, 4, values, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#else
	mavlink_radar_raw_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_RAW_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
}

/**
 * @brief Pack a radar_raw_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param values raw sample values
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_raw_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t sensor_id,const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, sensor_id);
	_mav_put_int16_t_array(buf, 4, values, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#else
	mavlink_radar_raw_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_RAW_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
}

/**
 * @brief Encode a radar_raw_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_raw_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_raw_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_raw_data_t* radar_raw_data)
{
	return mavlink_msg_radar_raw_data_pack(system_id, component_id, msg, radar_raw_data->time_boot_ms, radar_raw_data->sensor_id, radar_raw_data->values);
}

/**
 * @brief Encode a radar_raw_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_raw_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_raw_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_raw_data_t* radar_raw_data)
{
	return mavlink_msg_radar_raw_data_pack_chan(system_id, component_id, chan, msg, radar_raw_data->time_boot_ms, radar_raw_data->sensor_id, radar_raw_data->values);
}

/**
 * @brief Send a radar_raw_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param values raw sample values
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_raw_data_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t sensor_id, const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, sensor_id);
	_mav_put_int16_t_array(buf, 4, values, 64);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
#else
	mavlink_radar_raw_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.values, values, sizeof(int16_t)*64);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, (const char *)&packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, (const char *)&packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_raw_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t sensor_id, const int16_t *values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 132, sensor_id);
	_mav_put_int16_t_array(buf, 4, values, 64);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, buf, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
#else
	mavlink_radar_raw_data_t *packet = (mavlink_radar_raw_data_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->sensor_id = sensor_id;
	mav_array_memcpy(packet->values, values, sizeof(int16_t)*64);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, (const char *)packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN, MAVLINK_MSG_ID_RADAR_RAW_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_RAW_DATA, (const char *)packet, MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RADAR_RAW_DATA UNPACKING


/**
 * @brief Get field time_boot_ms from radar_raw_data message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_radar_raw_data_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_id from radar_raw_data message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_radar_raw_data_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  132);
}

/**
 * @brief Get field values from radar_raw_data message
 *
 * @return raw sample values
 */
static inline uint16_t mavlink_msg_radar_raw_data_get_values(const mavlink_message_t* msg, int16_t *values)
{
	return _MAV_RETURN_int16_t_array(msg, values, 64,  4);
}

/**
 * @brief Decode a radar_raw_data message into a struct
 *
 * @param msg The message to decode
 * @param radar_raw_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_raw_data_decode(const mavlink_message_t* msg, mavlink_radar_raw_data_t* radar_raw_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	radar_raw_data->time_boot_ms = mavlink_msg_radar_raw_data_get_time_boot_ms(msg);
	mavlink_msg_radar_raw_data_get_values(msg, radar_raw_data->values);
	radar_raw_data->sensor_id = mavlink_msg_radar_raw_data_get_sensor_id(msg);
#else
	memcpy(radar_raw_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RADAR_RAW_DATA_LEN);
#endif
}
