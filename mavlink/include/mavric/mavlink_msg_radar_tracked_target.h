// MESSAGE RADAR_TRACKED_TARGET PACKING

#define MAVLINK_MSG_ID_RADAR_TRACKED_TARGET 150

typedef struct __mavlink_radar_tracked_target_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float velocity; ///< Velocity estimate
 float amplitude; ///< Amplitude
 float distance; ///< Distance estimate
 uint8_t sensor_id; ///< Sensor ID
 uint8_t target_id; ///< Temporary enumeration of target
} mavlink_radar_tracked_target_t;

#define MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN 18
#define MAVLINK_MSG_ID_150_LEN 18

#define MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC 117
#define MAVLINK_MSG_ID_150_CRC 117



#define MAVLINK_MESSAGE_INFO_RADAR_TRACKED_TARGET { \
	"RADAR_TRACKED_TARGET", \
	6, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_tracked_target_t, time_boot_ms) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_radar_tracked_target_t, velocity) }, \
         { "amplitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_radar_tracked_target_t, amplitude) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_radar_tracked_target_t, distance) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_radar_tracked_target_t, sensor_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_radar_tracked_target_t, target_id) }, \
         } \
}


/**
 * @brief Pack a radar_tracked_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param target_id Temporary enumeration of target
 * @param velocity Velocity estimate
 * @param amplitude Amplitude
 * @param distance Distance estimate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_tracked_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t sensor_id, uint8_t target_id, float velocity, float amplitude, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_uint8_t(buf, 16, sensor_id);
	_mav_put_uint8_t(buf, 17, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_TRACKED_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
}

/**
 * @brief Pack a radar_tracked_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param target_id Temporary enumeration of target
 * @param velocity Velocity estimate
 * @param amplitude Amplitude
 * @param distance Distance estimate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_tracked_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t sensor_id,uint8_t target_id,float velocity,float amplitude,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_uint8_t(buf, 16, sensor_id);
	_mav_put_uint8_t(buf, 17, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_TRACKED_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
}

/**
 * @brief Encode a radar_tracked_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_tracked_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_tracked_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_tracked_target_t* radar_tracked_target)
{
	return mavlink_msg_radar_tracked_target_pack(system_id, component_id, msg, radar_tracked_target->time_boot_ms, radar_tracked_target->sensor_id, radar_tracked_target->target_id, radar_tracked_target->velocity, radar_tracked_target->amplitude, radar_tracked_target->distance);
}

/**
 * @brief Encode a radar_tracked_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radar_tracked_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_tracked_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radar_tracked_target_t* radar_tracked_target)
{
	return mavlink_msg_radar_tracked_target_pack_chan(system_id, component_id, chan, msg, radar_tracked_target->time_boot_ms, radar_tracked_target->sensor_id, radar_tracked_target->target_id, radar_tracked_target->velocity, radar_tracked_target->amplitude, radar_tracked_target->distance);
}

/**
 * @brief Send a radar_tracked_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param target_id Temporary enumeration of target
 * @param velocity Velocity estimate
 * @param amplitude Amplitude
 * @param distance Distance estimate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_tracked_target_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t sensor_id, uint8_t target_id, float velocity, float amplitude, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_uint8_t(buf, 16, sensor_id);
	_mav_put_uint8_t(buf, 17, target_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, (const char *)&packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, (const char *)&packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radar_tracked_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t sensor_id, uint8_t target_id, float velocity, float amplitude, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_uint8_t(buf, 16, sensor_id);
	_mav_put_uint8_t(buf, 17, target_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, buf, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
#else
	mavlink_radar_tracked_target_t *packet = (mavlink_radar_tracked_target_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->velocity = velocity;
	packet->amplitude = amplitude;
	packet->distance = distance;
	packet->sensor_id = sensor_id;
	packet->target_id = target_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, (const char *)packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, (const char *)packet, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RADAR_TRACKED_TARGET UNPACKING


/**
 * @brief Get field time_boot_ms from radar_tracked_target message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_radar_tracked_target_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_id from radar_tracked_target message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_radar_tracked_target_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_id from radar_tracked_target message
 *
 * @return Temporary enumeration of target
 */
static inline uint8_t mavlink_msg_radar_tracked_target_get_target_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field velocity from radar_tracked_target message
 *
 * @return Velocity estimate
 */
static inline float mavlink_msg_radar_tracked_target_get_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field amplitude from radar_tracked_target message
 *
 * @return Amplitude
 */
static inline float mavlink_msg_radar_tracked_target_get_amplitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field distance from radar_tracked_target message
 *
 * @return Distance estimate
 */
static inline float mavlink_msg_radar_tracked_target_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a radar_tracked_target message into a struct
 *
 * @param msg The message to decode
 * @param radar_tracked_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_tracked_target_decode(const mavlink_message_t* msg, mavlink_radar_tracked_target_t* radar_tracked_target)
{
#if MAVLINK_NEED_BYTE_SWAP
	radar_tracked_target->time_boot_ms = mavlink_msg_radar_tracked_target_get_time_boot_ms(msg);
	radar_tracked_target->velocity = mavlink_msg_radar_tracked_target_get_velocity(msg);
	radar_tracked_target->amplitude = mavlink_msg_radar_tracked_target_get_amplitude(msg);
	radar_tracked_target->distance = mavlink_msg_radar_tracked_target_get_distance(msg);
	radar_tracked_target->sensor_id = mavlink_msg_radar_tracked_target_get_sensor_id(msg);
	radar_tracked_target->target_id = mavlink_msg_radar_tracked_target_get_target_id(msg);
#else
	memcpy(radar_tracked_target, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN);
#endif
}
