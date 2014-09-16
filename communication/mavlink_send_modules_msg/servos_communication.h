

#ifndef SERVOS_COMMUNICATION_H_
#define SERVOS_COMMUNICATION_H_

#include "servos.h"
#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct  
{
	const servos_t* servos;
	const mavlink_stream_t* mavlink_stream;
}servos_communication_t;


void servos_communication_init(servos_communication_t* servos_communication, const servos_t* servos, const mavlink_stream_t* mavlink_stream);

task_return_t servos_communication_mavlink_send(servos_communication_t* servos);


#ifdef __cplusplus
}
#endif

#endif /* SERVOS_COMMUNICATION_H_ */