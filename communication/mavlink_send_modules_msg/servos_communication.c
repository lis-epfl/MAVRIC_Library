

#include "servos_communication.h"
#include "time_keeper.h"

void servos_communication_init(servos_communication_t* servos_communication, const servos_t* servos, const mavlink_stream_t* mavlink_stream)
{
	servos_communication->servos = servos;
	servos_communication->mavlink_stream = mavlink_stream;
}

task_return_t servos_communication_mavlink_send(servos_communication_t* servos_communication)
{
	mavlink_message_t msg;
	mavlink_msg_servo_output_raw_pack(	servos_communication->mavlink_stream->sysid,
										servos_communication->mavlink_stream->compid,
										&msg,
										time_keeper_get_micros(),
										0,
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[0].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[1].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[2].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[3].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[4].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[5].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[6].value ),
										(uint16_t)( 1500 + 500 * servos_communication->servos->servo[7].value )	);
	mavlink_stream_send( servos_communication->mavlink_stream, &msg );

	return TASK_RUN_SUCCESS;
}