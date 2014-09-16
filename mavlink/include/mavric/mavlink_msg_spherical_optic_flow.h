// MESSAGE SPHERICAL_OPTIC_FLOW PACKING

#define MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW 166

typedef struct __mavlink_spherical_optic_flow_t
{
 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 int16_t of_azimuth[18]; ///< Azimuthal component of optic flow vectors, in millirad /sec
 int16_t of_elevation[18]; ///< Elevation component of optic flow vectors, in millirad /sec
 int16_t azimuth[18]; ///< Azimuth of regions used for optic flow computation, in millirad
 int16_t elevation[18]; ///< Elevation of regions used for optic flow computation, in millirad
 uint8_t id_sensor; ///< Sensor ID
 uint8_t nb_sensors; ///< Number of sensors sending data, this can also be used to indicate in how many messages the data from a single sensor was splitted
 uint8_t nb_of; ///< Number of Optic Flow vectors sent in this message (between 0 and 16)
 uint8_t status; ///< Status of the sensor
 uint8_t of_info[18]; ///< Information on the optic flow vectors
} mavlink_spherical_optic_flow_t;

#define MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN 174
#define MAVLINK_MSG_ID_166_LEN 174

#define MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC 23
#define MAVLINK_MSG_ID_166_CRC 23

#define MAVLINK_MSG_SPHERICAL_OPTIC_FLOW_FIELD_OF_AZIMUTH_LEN 18
#define MAVLINK_MSG_SPHERICAL_OPTIC_FLOW_FIELD_OF_ELEVATION_LEN 18
#define MAVLINK_MSG_SPHERICAL_OPTIC_FLOW_FIELD_AZIMUTH_LEN 18
#define MAVLINK_MSG_SPHERICAL_OPTIC_FLOW_FIELD_ELEVATION_LEN 18
#define MAVLINK_MSG_SPHERICAL_OPTIC_FLOW_FIELD_OF_INFO_LEN 18

#define MAVLINK_MESSAGE_INFO_SPHERICAL_OPTIC_FLOW { \
	"SPHERICAL_OPTIC_FLOW", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_spherical_optic_flow_t, time_usec) }, \
         { "of_azimuth", NULL, MAVLINK_TYPE_INT16_T, 18, 8, offsetof(mavlink_spherical_optic_flow_t, of_azimuth) }, \
         { "of_elevation", NULL, MAVLINK_TYPE_INT16_T, 18, 44, offsetof(mavlink_spherical_optic_flow_t, of_elevation) }, \
         { "azimuth", NULL, MAVLINK_TYPE_INT16_T, 18, 80, offsetof(mavlink_spherical_optic_flow_t, azimuth) }, \
         { "elevation", NULL, MAVLINK_TYPE_INT16_T, 18, 116, offsetof(mavlink_spherical_optic_flow_t, elevation) }, \
         { "id_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 152, offsetof(mavlink_spherical_optic_flow_t, id_sensor) }, \
         { "nb_sensors", NULL, MAVLINK_TYPE_UINT8_T, 0, 153, offsetof(mavlink_spherical_optic_flow_t, nb_sensors) }, \
         { "nb_of", NULL, MAVLINK_TYPE_UINT8_T, 0, 154, offsetof(mavlink_spherical_optic_flow_t, nb_of) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 155, offsetof(mavlink_spherical_optic_flow_t, status) }, \
         { "of_info", NULL, MAVLINK_TYPE_UINT8_T, 18, 156, offsetof(mavlink_spherical_optic_flow_t, of_info) }, \
         } \
}


/**
 * @brief Pack a spherical_optic_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param id_sensor Sensor ID
 * @param nb_sensors Number of sensors sending data, this can also be used to indicate in how many messages the data from a single sensor was splitted
 * @param nb_of Number of Optic Flow vectors sent in this message (between 0 and 16)
 * @param status Status of the sensor
 * @param of_azimuth Azimuthal component of optic flow vectors, in millirad /sec
 * @param of_elevation Elevation component of optic flow vectors, in millirad /sec
 * @param azimuth Azimuth of regions used for optic flow computation, in millirad
 * @param elevation Elevation of regions used for optic flow computation, in millirad
 * @param of_info Information on the optic flow vectors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t id_sensor, uint8_t nb_sensors, uint8_t nb_of, uint8_t status, const int16_t *of_azimuth, const int16_t *of_elevation, const int16_t *azimuth, const int16_t *elevation, const uint8_t *of_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 152, id_sensor);
	_mav_put_uint8_t(buf, 153, nb_sensors);
	_mav_put_uint8_t(buf, 154, nb_of);
	_mav_put_uint8_t(buf, 155, status);
	_mav_put_int16_t_array(buf, 8, of_azimuth, 18);
	_mav_put_int16_t_array(buf, 44, of_elevation, 18);
	_mav_put_int16_t_array(buf, 80, azimuth, 18);
	_mav_put_int16_t_array(buf, 116, elevation, 18);
	_mav_put_uint8_t_array(buf, 156, of_info, 18);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#else
	mavlink_spherical_optic_flow_t packet;
	packet.time_usec = time_usec;
	packet.id_sensor = id_sensor;
	packet.nb_sensors = nb_sensors;
	packet.nb_of = nb_of;
	packet.status = status;
	mav_array_memcpy(packet.of_azimuth, of_azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_elevation, of_elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.elevation, elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_info, of_info, sizeof(uint8_t)*18);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
}

/**
 * @brief Pack a spherical_optic_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param id_sensor Sensor ID
 * @param nb_sensors Number of sensors sending data, this can also be used to indicate in how many messages the data from a single sensor was splitted
 * @param nb_of Number of Optic Flow vectors sent in this message (between 0 and 16)
 * @param status Status of the sensor
 * @param of_azimuth Azimuthal component of optic flow vectors, in millirad /sec
 * @param of_elevation Elevation component of optic flow vectors, in millirad /sec
 * @param azimuth Azimuth of regions used for optic flow computation, in millirad
 * @param elevation Elevation of regions used for optic flow computation, in millirad
 * @param of_info Information on the optic flow vectors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t id_sensor,uint8_t nb_sensors,uint8_t nb_of,uint8_t status,const int16_t *of_azimuth,const int16_t *of_elevation,const int16_t *azimuth,const int16_t *elevation,const uint8_t *of_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 152, id_sensor);
	_mav_put_uint8_t(buf, 153, nb_sensors);
	_mav_put_uint8_t(buf, 154, nb_of);
	_mav_put_uint8_t(buf, 155, status);
	_mav_put_int16_t_array(buf, 8, of_azimuth, 18);
	_mav_put_int16_t_array(buf, 44, of_elevation, 18);
	_mav_put_int16_t_array(buf, 80, azimuth, 18);
	_mav_put_int16_t_array(buf, 116, elevation, 18);
	_mav_put_uint8_t_array(buf, 156, of_info, 18);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#else
	mavlink_spherical_optic_flow_t packet;
	packet.time_usec = time_usec;
	packet.id_sensor = id_sensor;
	packet.nb_sensors = nb_sensors;
	packet.nb_of = nb_of;
	packet.status = status;
	mav_array_memcpy(packet.of_azimuth, of_azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_elevation, of_elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.elevation, elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_info, of_info, sizeof(uint8_t)*18);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
}

/**
 * @brief Encode a spherical_optic_flow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param spherical_optic_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_spherical_optic_flow_t* spherical_optic_flow)
{
	return mavlink_msg_spherical_optic_flow_pack(system_id, component_id, msg, spherical_optic_flow->time_usec, spherical_optic_flow->id_sensor, spherical_optic_flow->nb_sensors, spherical_optic_flow->nb_of, spherical_optic_flow->status, spherical_optic_flow->of_azimuth, spherical_optic_flow->of_elevation, spherical_optic_flow->azimuth, spherical_optic_flow->elevation, spherical_optic_flow->of_info);
}

/**
 * @brief Encode a spherical_optic_flow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param spherical_optic_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_spherical_optic_flow_t* spherical_optic_flow)
{
	return mavlink_msg_spherical_optic_flow_pack_chan(system_id, component_id, chan, msg, spherical_optic_flow->time_usec, spherical_optic_flow->id_sensor, spherical_optic_flow->nb_sensors, spherical_optic_flow->nb_of, spherical_optic_flow->status, spherical_optic_flow->of_azimuth, spherical_optic_flow->of_elevation, spherical_optic_flow->azimuth, spherical_optic_flow->elevation, spherical_optic_flow->of_info);
}

/**
 * @brief Send a spherical_optic_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param id_sensor Sensor ID
 * @param nb_sensors Number of sensors sending data, this can also be used to indicate in how many messages the data from a single sensor was splitted
 * @param nb_of Number of Optic Flow vectors sent in this message (between 0 and 16)
 * @param status Status of the sensor
 * @param of_azimuth Azimuthal component of optic flow vectors, in millirad /sec
 * @param of_elevation Elevation component of optic flow vectors, in millirad /sec
 * @param azimuth Azimuth of regions used for optic flow computation, in millirad
 * @param elevation Elevation of regions used for optic flow computation, in millirad
 * @param of_info Information on the optic flow vectors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_spherical_optic_flow_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t id_sensor, uint8_t nb_sensors, uint8_t nb_of, uint8_t status, const int16_t *of_azimuth, const int16_t *of_elevation, const int16_t *azimuth, const int16_t *elevation, const uint8_t *of_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 152, id_sensor);
	_mav_put_uint8_t(buf, 153, nb_sensors);
	_mav_put_uint8_t(buf, 154, nb_of);
	_mav_put_uint8_t(buf, 155, status);
	_mav_put_int16_t_array(buf, 8, of_azimuth, 18);
	_mav_put_int16_t_array(buf, 44, of_elevation, 18);
	_mav_put_int16_t_array(buf, 80, azimuth, 18);
	_mav_put_int16_t_array(buf, 116, elevation, 18);
	_mav_put_uint8_t_array(buf, 156, of_info, 18);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
#else
	mavlink_spherical_optic_flow_t packet;
	packet.time_usec = time_usec;
	packet.id_sensor = id_sensor;
	packet.nb_sensors = nb_sensors;
	packet.nb_of = nb_of;
	packet.status = status;
	mav_array_memcpy(packet.of_azimuth, of_azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_elevation, of_elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.azimuth, azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet.elevation, elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet.of_info, of_info, sizeof(uint8_t)*18);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, (const char *)&packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, (const char *)&packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_spherical_optic_flow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t id_sensor, uint8_t nb_sensors, uint8_t nb_of, uint8_t status, const int16_t *of_azimuth, const int16_t *of_elevation, const int16_t *azimuth, const int16_t *elevation, const uint8_t *of_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 152, id_sensor);
	_mav_put_uint8_t(buf, 153, nb_sensors);
	_mav_put_uint8_t(buf, 154, nb_of);
	_mav_put_uint8_t(buf, 155, status);
	_mav_put_int16_t_array(buf, 8, of_azimuth, 18);
	_mav_put_int16_t_array(buf, 44, of_elevation, 18);
	_mav_put_int16_t_array(buf, 80, azimuth, 18);
	_mav_put_int16_t_array(buf, 116, elevation, 18);
	_mav_put_uint8_t_array(buf, 156, of_info, 18);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, buf, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
#else
	mavlink_spherical_optic_flow_t *packet = (mavlink_spherical_optic_flow_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->id_sensor = id_sensor;
	packet->nb_sensors = nb_sensors;
	packet->nb_of = nb_of;
	packet->status = status;
	mav_array_memcpy(packet->of_azimuth, of_azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet->of_elevation, of_elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet->azimuth, azimuth, sizeof(int16_t)*18);
	mav_array_memcpy(packet->elevation, elevation, sizeof(int16_t)*18);
	mav_array_memcpy(packet->of_info, of_info, sizeof(uint8_t)*18);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, (const char *)packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW, (const char *)packet, MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SPHERICAL_OPTIC_FLOW UNPACKING


/**
 * @brief Get field time_usec from spherical_optic_flow message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_spherical_optic_flow_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id_sensor from spherical_optic_flow message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_spherical_optic_flow_get_id_sensor(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  152);
}

/**
 * @brief Get field nb_sensors from spherical_optic_flow message
 *
 * @return Number of sensors sending data, this can also be used to indicate in how many messages the data from a single sensor was splitted
 */
static inline uint8_t mavlink_msg_spherical_optic_flow_get_nb_sensors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  153);
}

/**
 * @brief Get field nb_of from spherical_optic_flow message
 *
 * @return Number of Optic Flow vectors sent in this message (between 0 and 16)
 */
static inline uint8_t mavlink_msg_spherical_optic_flow_get_nb_of(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  154);
}

/**
 * @brief Get field status from spherical_optic_flow message
 *
 * @return Status of the sensor
 */
static inline uint8_t mavlink_msg_spherical_optic_flow_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  155);
}

/**
 * @brief Get field of_azimuth from spherical_optic_flow message
 *
 * @return Azimuthal component of optic flow vectors, in millirad /sec
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_get_of_azimuth(const mavlink_message_t* msg, int16_t *of_azimuth)
{
	return _MAV_RETURN_int16_t_array(msg, of_azimuth, 18,  8);
}

/**
 * @brief Get field of_elevation from spherical_optic_flow message
 *
 * @return Elevation component of optic flow vectors, in millirad /sec
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_get_of_elevation(const mavlink_message_t* msg, int16_t *of_elevation)
{
	return _MAV_RETURN_int16_t_array(msg, of_elevation, 18,  44);
}

/**
 * @brief Get field azimuth from spherical_optic_flow message
 *
 * @return Azimuth of regions used for optic flow computation, in millirad
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_get_azimuth(const mavlink_message_t* msg, int16_t *azimuth)
{
	return _MAV_RETURN_int16_t_array(msg, azimuth, 18,  80);
}

/**
 * @brief Get field elevation from spherical_optic_flow message
 *
 * @return Elevation of regions used for optic flow computation, in millirad
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_get_elevation(const mavlink_message_t* msg, int16_t *elevation)
{
	return _MAV_RETURN_int16_t_array(msg, elevation, 18,  116);
}

/**
 * @brief Get field of_info from spherical_optic_flow message
 *
 * @return Information on the optic flow vectors
 */
static inline uint16_t mavlink_msg_spherical_optic_flow_get_of_info(const mavlink_message_t* msg, uint8_t *of_info)
{
	return _MAV_RETURN_uint8_t_array(msg, of_info, 18,  156);
}

/**
 * @brief Decode a spherical_optic_flow message into a struct
 *
 * @param msg The message to decode
 * @param spherical_optic_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_spherical_optic_flow_decode(const mavlink_message_t* msg, mavlink_spherical_optic_flow_t* spherical_optic_flow)
{
#if MAVLINK_NEED_BYTE_SWAP
	spherical_optic_flow->time_usec = mavlink_msg_spherical_optic_flow_get_time_usec(msg);
	mavlink_msg_spherical_optic_flow_get_of_azimuth(msg, spherical_optic_flow->of_azimuth);
	mavlink_msg_spherical_optic_flow_get_of_elevation(msg, spherical_optic_flow->of_elevation);
	mavlink_msg_spherical_optic_flow_get_azimuth(msg, spherical_optic_flow->azimuth);
	mavlink_msg_spherical_optic_flow_get_elevation(msg, spherical_optic_flow->elevation);
	spherical_optic_flow->id_sensor = mavlink_msg_spherical_optic_flow_get_id_sensor(msg);
	spherical_optic_flow->nb_sensors = mavlink_msg_spherical_optic_flow_get_nb_sensors(msg);
	spherical_optic_flow->nb_of = mavlink_msg_spherical_optic_flow_get_nb_of(msg);
	spherical_optic_flow->status = mavlink_msg_spherical_optic_flow_get_status(msg);
	mavlink_msg_spherical_optic_flow_get_of_info(msg, spherical_optic_flow->of_info);
#else
	memcpy(spherical_optic_flow, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SPHERICAL_OPTIC_FLOW_LEN);
#endif
}
