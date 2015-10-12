/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file sma.h
 * 
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief A Simple Moving Average utility (SMA)
 *
 ******************************************************************************/

 #ifndef SMA_H_
 #define SMA_H_

#include <stdint.h>

#include "mavlink_stream.h"
#include "mavlink_message_handler.h"

 #ifdef __cplusplus
extern "C" 
{
#endif

#define SAMPLING_PERIOD 10

/**
 * \brief 		Simple Moving Average (SMA) structure
 */
typedef struct
{
	int16_t buffer[SAMPLING_PERIOD];	//< Stores the values used to compute the moving average (size is equal to period)
	int16_t current_avg; 				//< Stores the current average value
	int32_t sum;						//< Stores the sum of previous values
	uint16_t nb_samples;				//< Total number of samples read
	uint16_t period;					//< Period for moving average (number of samples used for computation)
} sma_t;

/**
 * \brief        		Simple Moving Average initialisation
 * 
 * \param sma 			Pointer the moving average struct
 * \param period 		Period for moving average (number of samples used for computation)
 */
void sma_init(sma_t * sma, uint16_t period);


/**
 * \brief        		Compute new moving average
 *
 * \param sma 			Pointer the moving average struct
 * \param sample 		New sample to consider for moving average
 */
void sma_update(sma_t * sma, int16_t sample);

// TODO !
/**
 * \brief	Function to send the MAVLink current average message
 * 
 * \param	sma						The pointer to the SMA structure
 * \param	mavlink_stream			The pointer to the MAVLink stream structure
 * \param	msg						The pointer to the MAVLink message
 */
// void sma_telemetry_send_current_avg(const sma_t * sma, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);
// TODO !

/**
 * \brief        		Return a positive value of i%n
 *
 * \param i 			Value to compute modulo for
 * \param n				Modulo
 */
static inline int positive_modulo(int i, int n) {
    return (i % n + n) % n;
}

#ifdef __cplusplus
}
#endif

#endif /* SMA_H_ */