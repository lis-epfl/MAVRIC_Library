



#ifndef REMOTE_COMMUNICATION_H_
#define REMOTE_COMMUNICATION_H_

#include "mavlink_stream.h"
#include "mavlink_communication.h"
#include "remote.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct  
{
	const remote_t* remote;
	const mavlink_stream_t* mavlink_stream;
}remote_communication_t;

void remote_communication_init(remote_communication_t* remote_communication, remote_t* remote, mavlink_stream_t* mavlink_stream, mavlink_message_handler_t *mavlink_handler);

/**
 * \brief	Sends the raw remote values via MAVLink
 * 
 * \param	remote				The pointer to the remote structure
 *
 * \return	The result of the task
 */
task_return_t remote_send_raw(const remote_communication_t* remote_communication);

/**
 * \brief	Sends the scaled remote values via MAVLink
 * 
 * \param	remote				The pointer to the remote structure
 *
 * \return	The result of the task
 */
task_return_t remote_send_scaled(const remote_communication_t* remote_communication);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_COMMUNICATION_H_ */