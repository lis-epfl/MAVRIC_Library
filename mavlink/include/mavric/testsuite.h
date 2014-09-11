/** @file
 *	@brief MAVLink comm protocol testsuite generated from mavric.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef MAVRIC_TESTSUITE_H
#define MAVRIC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_mavric(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_mavric(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_radar_tracked_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radar_tracked_target_t packet_in = {
		963497464,45.0,73.0,101.0,53,120
    };
	mavlink_radar_tracked_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.velocity = packet_in.velocity;
        	packet1.amplitude = packet_in.amplitude;
        	packet1.distance = packet_in.distance;
        	packet1.sensor_id = packet_in.sensor_id;
        	packet1.target_id = packet_in.target_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_tracked_target_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radar_tracked_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_tracked_target_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.target_id , packet1.velocity , packet1.amplitude , packet1.distance );
	mavlink_msg_radar_tracked_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_tracked_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.target_id , packet1.velocity , packet1.amplitude , packet1.distance );
	mavlink_msg_radar_tracked_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radar_tracked_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_tracked_target_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.sensor_id , packet1.target_id , packet1.velocity , packet1.amplitude , packet1.distance );
	mavlink_msg_radar_tracked_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radar_velocity_hist(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radar_velocity_hist_t packet_in = {
		963497464,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147 }
    };
	mavlink_radar_velocity_hist_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.sensor_id = packet_in.sensor_id;
        
        	mav_array_memcpy(packet1.velocity, packet_in.velocity, sizeof(uint8_t)*64);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_velocity_hist_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radar_velocity_hist_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_velocity_hist_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.velocity );
	mavlink_msg_radar_velocity_hist_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_velocity_hist_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.velocity );
	mavlink_msg_radar_velocity_hist_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radar_velocity_hist_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_velocity_hist_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.sensor_id , packet1.velocity );
	mavlink_msg_radar_velocity_hist_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radar_raw_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radar_raw_data_t packet_in = {
		963497464,{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450, 17451, 17452, 17453, 17454, 17455, 17456, 17457, 17458, 17459, 17460, 17461, 17462, 17463, 17464, 17465, 17466, 17467, 17468, 17469, 17470, 17471, 17472, 17473, 17474, 17475, 17476, 17477, 17478, 17479, 17480, 17481, 17482, 17483, 17484, 17485, 17486, 17487, 17488, 17489, 17490, 17491, 17492, 17493, 17494, 17495, 17496, 17497, 17498, 17499, 17500, 17501, 17502, 17503, 17504, 17505, 17506 },145
    };
	mavlink_radar_raw_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.sensor_id = packet_in.sensor_id;
        
        	mav_array_memcpy(packet1.values, packet_in.values, sizeof(int16_t)*64);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_raw_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radar_raw_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_raw_data_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.values );
	mavlink_msg_radar_raw_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_raw_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.sensor_id , packet1.values );
	mavlink_msg_radar_raw_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radar_raw_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radar_raw_data_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.sensor_id , packet1.values );
	mavlink_msg_radar_raw_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_local_position_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_local_position_setpoint_t packet_in = {
		17.0,45.0,73.0,101.0,53,120,187
    };
	mavlink_set_local_position_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_setpoint_t packet_in = {
		17.0,45.0,73.0,101.0,53
    };
	mavlink_local_position_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_position_setpoint_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_position_setpoint_int_t packet_in = {
		963497464,963497672,963497880,17859,175
    };
	mavlink_global_position_setpoint_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_global_position_setpoint_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_global_position_setpoint_int_t packet_in = {
		963497464,963497672,963497880,17859,175
    };
	mavlink_set_global_position_setpoint_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_roll_pitch_yaw_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_roll_pitch_yaw_thrust_t packet_in = {
		17.0,45.0,73.0,101.0,53,120
    };
	mavlink_set_roll_pitch_yaw_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_roll_pitch_yaw_speed_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet_in = {
		17.0,45.0,73.0,101.0,53,120
    };
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll_speed = packet_in.roll_speed;
        	packet1.pitch_speed = packet_in.pitch_speed;
        	packet1.yaw_speed = packet_in.yaw_speed;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yaw_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_pitch_yaw_thrust_setpoint_t packet_in = {
		963497464,45.0,73.0,101.0,129.0
    };
	mavlink_roll_pitch_yaw_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yaw_speed_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet_in = {
		963497464,45.0,73.0,101.0,129.0
    };
	mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll_speed = packet_in.roll_speed;
        	packet1.pitch_speed = packet_in.pitch_speed;
        	packet1.yaw_speed = packet_in.yaw_speed;
        	packet1.thrust = packet_in.thrust;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_quad_motors_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_quad_motors_setpoint_t packet_in = {
		17235,17339,17443,17547,29
    };
	mavlink_set_quad_motors_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.motor_front_nw = packet_in.motor_front_nw;
        	packet1.motor_right_ne = packet_in.motor_right_ne;
        	packet1.motor_back_se = packet_in.motor_back_se;
        	packet1.motor_left_sw = packet_in.motor_left_sw;
        	packet1.target_system = packet_in.target_system;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_pack(system_id, component_id, &msg , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_quad_motors_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_send(MAVLINK_COMM_1 , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_quad_swarm_roll_pitch_yaw_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet_in = {
		{ 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },101,168
    };
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.group = packet_in.group;
        	packet1.mode = packet_in.mode;
        
        	mav_array_memcpy(packet1.roll, packet_in.roll, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.pitch, packet_in.pitch, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.yaw, packet_in.yaw, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.thrust, packet_in.thrust, sizeof(uint16_t)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(system_id, component_id, &msg , packet1.group , packet1.mode , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.group , packet1.mode , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_send(MAVLINK_COMM_1 , packet1.group , packet1.mode , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_quad_swarm_led_roll_pitch_yaw_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_quad_swarm_led_roll_pitch_yaw_thrust_t packet_in = {
		{ 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },101,168,{ 235, 236, 237, 238 },{ 247, 248, 249, 250 },{ 3, 4, 5, 6 }
    };
	mavlink_set_quad_swarm_led_roll_pitch_yaw_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.group = packet_in.group;
        	packet1.mode = packet_in.mode;
        
        	mav_array_memcpy(packet1.roll, packet_in.roll, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.pitch, packet_in.pitch, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.yaw, packet_in.yaw, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.thrust, packet_in.thrust, sizeof(uint16_t)*4);
        	mav_array_memcpy(packet1.led_red, packet_in.led_red, sizeof(uint8_t)*4);
        	mav_array_memcpy(packet1.led_blue, packet_in.led_blue, sizeof(uint8_t)*4);
        	mav_array_memcpy(packet1.led_green, packet_in.led_green, sizeof(uint8_t)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_pack(system_id, component_id, &msg , packet1.group , packet1.mode , packet1.led_red , packet1.led_blue , packet1.led_green , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.group , packet1.mode , packet1.led_red , packet1.led_blue , packet1.led_green , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_send(MAVLINK_COMM_1 , packet1.group , packet1.mode , packet1.led_red , packet1.led_blue , packet1.led_green , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_state_correction(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_state_correction_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
	mavlink_state_correction_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xErr = packet_in.xErr;
        	packet1.yErr = packet_in.yErr;
        	packet1.zErr = packet_in.zErr;
        	packet1.rollErr = packet_in.rollErr;
        	packet1.pitchErr = packet_in.pitchErr;
        	packet1.yawErr = packet_in.yawErr;
        	packet1.vxErr = packet_in.vxErr;
        	packet1.vyErr = packet_in.vyErr;
        	packet1.vzErr = packet_in.vzErr;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_pack(system_id, component_id, &msg , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_state_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_send(MAVLINK_COMM_1 , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yaw_rates_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_pitch_yaw_rates_thrust_setpoint_t packet_in = {
		963497464,45.0,73.0,101.0,129.0
    };
	mavlink_roll_pitch_yaw_rates_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll_rate = packet_in.roll_rate;
        	packet1.pitch_rate = packet_in.pitch_rate;
        	packet1.yaw_rate = packet_in.yaw_rate;
        	packet1.thrust = packet_in.thrust;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_spherical_optic_flow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_spherical_optic_flow_t packet_in = {
		93372036854775807ULL,{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668 },{ 19523, 19524, 19525, 19526, 19527, 19528, 19529, 19530, 19531, 19532, 19533, 19534, 19535, 19536, 19537, 19538, 19539, 19540 },{ 21395, 21396, 21397, 21398, 21399, 21400, 21401, 21402, 21403, 21404, 21405, 21406, 21407, 21408, 21409, 21410, 21411, 21412 },{ 23267, 23268, 23269, 23270, 23271, 23272, 23273, 23274, 23275, 23276, 23277, 23278, 23279, 23280, 23281, 23282, 23283, 23284 },205,16,83,150,{ 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234 }
    };
	mavlink_spherical_optic_flow_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.id_sensor = packet_in.id_sensor;
        	packet1.nb_sensors = packet_in.nb_sensors;
        	packet1.nb_of = packet_in.nb_of;
        	packet1.status = packet_in.status;
        
        	mav_array_memcpy(packet1.of_azimuth, packet_in.of_azimuth, sizeof(int16_t)*18);
        	mav_array_memcpy(packet1.of_elevation, packet_in.of_elevation, sizeof(int16_t)*18);
        	mav_array_memcpy(packet1.azimuth, packet_in.azimuth, sizeof(int16_t)*18);
        	mav_array_memcpy(packet1.elevation, packet_in.elevation, sizeof(int16_t)*18);
        	mav_array_memcpy(packet1.of_info, packet_in.of_info, sizeof(uint8_t)*18);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_spherical_optic_flow_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_spherical_optic_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_spherical_optic_flow_pack(system_id, component_id, &msg , packet1.time_usec , packet1.id_sensor , packet1.nb_sensors , packet1.nb_of , packet1.status , packet1.of_azimuth , packet1.of_elevation , packet1.azimuth , packet1.elevation , packet1.of_info );
	mavlink_msg_spherical_optic_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_spherical_optic_flow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.id_sensor , packet1.nb_sensors , packet1.nb_of , packet1.status , packet1.of_azimuth , packet1.of_elevation , packet1.azimuth , packet1.elevation , packet1.of_info );
	mavlink_msg_spherical_optic_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_spherical_optic_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_spherical_optic_flow_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.id_sensor , packet1.nb_sensors , packet1.nb_of , packet1.status , packet1.of_azimuth , packet1.of_elevation , packet1.azimuth , packet1.elevation , packet1.of_info );
	mavlink_msg_spherical_optic_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mavric(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_radar_tracked_target(system_id, component_id, last_msg);
	mavlink_test_radar_velocity_hist(system_id, component_id, last_msg);
	mavlink_test_radar_raw_data(system_id, component_id, last_msg);
	mavlink_test_set_local_position_setpoint(system_id, component_id, last_msg);
	mavlink_test_local_position_setpoint(system_id, component_id, last_msg);
	mavlink_test_global_position_setpoint_int(system_id, component_id, last_msg);
	mavlink_test_set_global_position_setpoint_int(system_id, component_id, last_msg);
	mavlink_test_set_roll_pitch_yaw_thrust(system_id, component_id, last_msg);
	mavlink_test_set_roll_pitch_yaw_speed_thrust(system_id, component_id, last_msg);
	mavlink_test_roll_pitch_yaw_thrust_setpoint(system_id, component_id, last_msg);
	mavlink_test_roll_pitch_yaw_speed_thrust_setpoint(system_id, component_id, last_msg);
	mavlink_test_set_quad_motors_setpoint(system_id, component_id, last_msg);
	mavlink_test_set_quad_swarm_roll_pitch_yaw_thrust(system_id, component_id, last_msg);
	mavlink_test_set_quad_swarm_led_roll_pitch_yaw_thrust(system_id, component_id, last_msg);
	mavlink_test_state_correction(system_id, component_id, last_msg);
	mavlink_test_roll_pitch_yaw_rates_thrust_setpoint(system_id, component_id, last_msg);
	mavlink_test_spherical_optic_flow(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVRIC_TESTSUITE_H
