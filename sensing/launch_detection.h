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
 * \file launch_detection.h
 * 
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief	Detect launches
 *
 ******************************************************************************/

 #ifndef LAUNCH_DETECTION_H_
 #define LAUNCH_DETECTION_H_

 #ifdef __cplusplus
extern "C" 
{
#endif

#include <stdbool.h>

#include "sma.h"
#include "tasks.h"

#define C_IDLE 20

/**
 * \brief 		Launch status states
 */
typedef enum { 
				IDLE = 0,		//< Before launch
			   	LAUNCHING = 1, 	//< During launch
			   	IN_FLIGHT = 2 	//< After launch
			 } launch_status_t;

/**
 * \brief 		Launch detection structure
 */
typedef struct
{
	sma_t * sma; 						//< Simple moving average of acceleration norm
	int16_t t_launch; 					//< Launch detection threshold
	const int16_t c_idle = C_IDLE;		//< Norm of idle thrust value
	launch_status_t status; 			//< Launch status
	float acc[3];						//< Acceleration values
	bool enabled;						//< 1 if launch detection is enabled, 0 otherwise
} launch_detection_t;

/**
 * \brief        		Launch detection initialisation
 * 
 * \param t_launch 		Threshold value for launch detection
 * \param ld 			Pointer to the launch detection struct
 */
void launch_detection_init(launch_detection_t * ld, int16_t t_launch);


/**
 * \brief        		Update launch detection parameters
 *
 * \param acc 			Acceleration values measured by the IMU
 * \param ld 			Pointer to the launch detection struct
 *
 * \return 				Task status
 */
task_return_t launch_detection_update(launch_detection_t * ld, float acc[3]);


#ifdef __cplusplus
}
#endif

#endif /* LAUNCH_DETECTION_H_ */