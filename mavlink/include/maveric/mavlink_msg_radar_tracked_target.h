// MESSAGE RADAR_TRACKED_TARGET PACKING

#define MAVLINK_MSG_ID_RADAR_TRACKED_TARGET 150

typedef struct __mavlink_radar_tracked_target_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float velocity; ///< Velocity estimate
 float amplitude; ///< Amplitude (signal strength)
 float distance; ///< Distance estimate
 float azimuth; ///< Azimuth
 float elevation; ///< Elevation
 float uncertainty; ///< Uncertainty
 uint8_t sensor_id; ///< Sensor ID
 uint8_t target_id; ///< Temporary enumeration of target
} mavlink_radar_tracked_target_t;

#define MAVLINK_MSG_ID_RADAR_TRACKED_TARGET_LEN 30
#define MAVLINK_MSG_ID_150_LEN 30



#define MAVLINK_MESSAGE_INFO_RADAR_TRACKED_TARGET { \
	"RADAR_TRACKED_TARGET", \
	9, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_tracked_target_t, time_boot_ms) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_radar_tracked_target_t, velocity) }, \
         { "amplitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_radar_tracked_target_t, amplitude) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_radar_tracked_target_t, distance) }, \
         { "azimuth", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_radar_tracked_target_t, azimuth) }, \
         { "elevation", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_radar_tracked_target_t, elevation) }, \
         { "uncertainty", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_radar_tracked_target_t, uncertainty) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_radar_tracked_target_t, sensor_id) }, \
         { "target_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_radar_tracked_target_t, target_id) }, \
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
 * @param amplitude Amplitude (signal strength)
 * @param distance Distance estimate
 * @param azimuth Azimuth
 * @param elevation Elevation
 * @param uncertainty Uncertainty
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_tracked_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t sensor_id, uint8_t target_id, float velocity, float amplitude, float distance, float azimuth, float elevation, float uncertainty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_float(buf, 16, azimuth);
	_mav_put_float(buf, 20, elevation);
	_mav_put_float(buf, 24, uncertainty);
	_mav_put_uint8_t(buf, 28, sensor_id);
	_mav_put_uint8_t(buf, 29, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.azimuth = azimuth;
	packet.elevation = elevation;
	packet.uncertainty = uncertainty;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_TRACKED_TARGET;
	return mavlink_finalize_message(msg, system_id, component_id, 30, 185);
}

/**
 * @brief Pack a radar_tracked_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param target_id Temporary enumeration of target
 * @param velocity Velocity estimate
 * @param amplitude Amplitude (signal strength)
 * @param distance Distance estimate
 * @param azimuth Azimuth
 * @param elevation Elevation
 * @param uncertainty Uncertainty
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_tracked_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t sensor_id,uint8_t target_id,float velocity,float amplitude,float distance,float azimuth,float elevation,float uncertainty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_float(buf, 16, azimuth);
	_mav_put_float(buf, 20, elevation);
	_mav_put_float(buf, 24, uncertainty);
	_mav_put_uint8_t(buf, 28, sensor_id);
	_mav_put_uint8_t(buf, 29, target_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.azimuth = azimuth;
	packet.elevation = elevation;
	packet.uncertainty = uncertainty;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_TRACKED_TARGET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30, 185);
}

/**
 * @brief Encode a radar_tracked_target struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_tracked_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_tracked_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_tracked_target_t* radar_tracked_target)
{
	return mavlink_msg_radar_tracked_target_pack(system_id, component_id, msg, radar_tracked_target->time_boot_ms, radar_tracked_target->sensor_id, radar_tracked_target->target_id, radar_tracked_target->velocity, radar_tracked_target->amplitude, radar_tracked_target->distance, radar_tracked_target->azimuth, radar_tracked_target->elevation, radar_tracked_target->uncertainty);
}

/**
 * @brief Send a radar_tracked_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param target_id Temporary enumeration of target
 * @param velocity Velocity estimate
 * @param amplitude Amplitude (signal strength)
 * @param distance Distance estimate
 * @param azimuth Azimuth
 * @param elevation Elevation
 * @param uncertainty Uncertainty
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_tracked_target_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t sensor_id, uint8_t target_id, float velocity, float amplitude, float distance, float azimuth, float elevation, float uncertainty)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, velocity);
	_mav_put_float(buf, 8, amplitude);
	_mav_put_float(buf, 12, distance);
	_mav_put_float(buf, 16, azimuth);
	_mav_put_float(buf, 20, elevation);
	_mav_put_float(buf, 24, uncertainty);
	_mav_put_uint8_t(buf, 28, sensor_id);
	_mav_put_uint8_t(buf, 29, target_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, buf, 30, 185);
#else
	mavlink_radar_tracked_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.velocity = velocity;
	packet.amplitude = amplitude;
	packet.distance = distance;
	packet.azimuth = azimuth;
	packet.elevation = elevation;
	packet.uncertainty = uncertainty;
	packet.sensor_id = sensor_id;
	packet.target_id = target_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_TRACKED_TARGET, (const char *)&packet, 30, 185);
#endif
}

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
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field target_id from radar_tracked_target message
 *
 * @return Temporary enumeration of target
 */
static inline uint8_t mavlink_msg_radar_tracked_target_get_target_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
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
 * @return Amplitude (signal strength)
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
 * @brief Get field azimuth from radar_tracked_target message
 *
 * @return Azimuth
 */
static inline float mavlink_msg_radar_tracked_target_get_azimuth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field elevation from radar_tracked_target message
 *
 * @return Elevation
 */
static inline float mavlink_msg_radar_tracked_target_get_elevation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field uncertainty from radar_tracked_target message
 *
 * @return Uncertainty
 */
static inline float mavlink_msg_radar_tracked_target_get_uncertainty(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
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
	radar_tracked_target->azimuth = mavlink_msg_radar_tracked_target_get_azimuth(msg);
	radar_tracked_target->elevation = mavlink_msg_radar_tracked_target_get_elevation(msg);
	radar_tracked_target->uncertainty = mavlink_msg_radar_tracked_target_get_uncertainty(msg);
	radar_tracked_target->sensor_id = mavlink_msg_radar_tracked_target_get_sensor_id(msg);
	radar_tracked_target->target_id = mavlink_msg_radar_tracked_target_get_target_id(msg);
#else
	memcpy(radar_tracked_target, _MAV_PAYLOAD(msg), 30);
#endif
}
