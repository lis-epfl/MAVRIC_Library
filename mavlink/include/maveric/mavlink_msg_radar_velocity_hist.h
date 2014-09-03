// MESSAGE RADAR_VELOCITY_HIST PACKING

#define MAVLINK_MSG_ID_RADAR_VELOCITY_HIST 151

typedef struct __mavlink_radar_velocity_hist_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 uint8_t sensor_id; ///< Sensor ID
 uint8_t velocity[64]; ///< Velocity field
} mavlink_radar_velocity_hist_t;

#define MAVLINK_MSG_ID_RADAR_VELOCITY_HIST_LEN 69
#define MAVLINK_MSG_ID_151_LEN 69

#define MAVLINK_MSG_RADAR_VELOCITY_HIST_FIELD_VELOCITY_LEN 64

#define MAVLINK_MESSAGE_INFO_RADAR_VELOCITY_HIST { \
	"RADAR_VELOCITY_HIST", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radar_velocity_hist_t, time_boot_ms) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_radar_velocity_hist_t, sensor_id) }, \
         { "velocity", NULL, MAVLINK_TYPE_UINT8_T, 64, 5, offsetof(mavlink_radar_velocity_hist_t, velocity) }, \
         } \
}


/**
 * @brief Pack a radar_velocity_hist message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param velocity Velocity field
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_velocity_hist_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t sensor_id, const uint8_t *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[69];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, sensor_id);
	_mav_put_uint8_t_array(buf, 5, velocity, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 69);
#else
	mavlink_radar_velocity_hist_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.velocity, velocity, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 69);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_VELOCITY_HIST;
	return mavlink_finalize_message(msg, system_id, component_id, 69, 37);
}

/**
 * @brief Pack a radar_velocity_hist message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param velocity Velocity field
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radar_velocity_hist_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t sensor_id,const uint8_t *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[69];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, sensor_id);
	_mav_put_uint8_t_array(buf, 5, velocity, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 69);
#else
	mavlink_radar_velocity_hist_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.velocity, velocity, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 69);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADAR_VELOCITY_HIST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 69, 37);
}

/**
 * @brief Encode a radar_velocity_hist struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radar_velocity_hist C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radar_velocity_hist_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radar_velocity_hist_t* radar_velocity_hist)
{
	return mavlink_msg_radar_velocity_hist_pack(system_id, component_id, msg, radar_velocity_hist->time_boot_ms, radar_velocity_hist->sensor_id, radar_velocity_hist->velocity);
}

/**
 * @brief Send a radar_velocity_hist message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param sensor_id Sensor ID
 * @param velocity Velocity field
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radar_velocity_hist_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t sensor_id, const uint8_t *velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[69];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint8_t(buf, 4, sensor_id);
	_mav_put_uint8_t_array(buf, 5, velocity, 64);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_VELOCITY_HIST, buf, 69, 37);
#else
	mavlink_radar_velocity_hist_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.sensor_id = sensor_id;
	mav_array_memcpy(packet.velocity, velocity, sizeof(uint8_t)*64);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADAR_VELOCITY_HIST, (const char *)&packet, 69, 37);
#endif
}

#endif

// MESSAGE RADAR_VELOCITY_HIST UNPACKING


/**
 * @brief Get field time_boot_ms from radar_velocity_hist message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_radar_velocity_hist_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_id from radar_velocity_hist message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_radar_velocity_hist_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field velocity from radar_velocity_hist message
 *
 * @return Velocity field
 */
static inline uint16_t mavlink_msg_radar_velocity_hist_get_velocity(const mavlink_message_t* msg, uint8_t *velocity)
{
	return _MAV_RETURN_uint8_t_array(msg, velocity, 64,  5);
}

/**
 * @brief Decode a radar_velocity_hist message into a struct
 *
 * @param msg The message to decode
 * @param radar_velocity_hist C-struct to decode the message contents into
 */
static inline void mavlink_msg_radar_velocity_hist_decode(const mavlink_message_t* msg, mavlink_radar_velocity_hist_t* radar_velocity_hist)
{
#if MAVLINK_NEED_BYTE_SWAP
	radar_velocity_hist->time_boot_ms = mavlink_msg_radar_velocity_hist_get_time_boot_ms(msg);
	radar_velocity_hist->sensor_id = mavlink_msg_radar_velocity_hist_get_sensor_id(msg);
	mavlink_msg_radar_velocity_hist_get_velocity(msg, radar_velocity_hist->velocity);
#else
	memcpy(radar_velocity_hist, _MAV_PAYLOAD(msg), 69);
#endif
}
