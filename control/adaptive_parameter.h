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
 * \file adaptive_parameter.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This module automatically adapts the existing parameters based on 
 * given control variable
 *
 ******************************************************************************/


#ifndef ADAPTIVE_PARAMETER_H_
#define ADAPTIVE_PARAMETER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define MAX_ADAPT_PARAM_SETPOINTS 5
#define MAX_ADAPT_PARAM_COUNT 10


/**
 * \brief	Adaptive parameter structure
 */
typedef struct 
{
	float* control_variable;					///< Control Variable
	float* parameter;							///< Parameter
	int32_t nb_setpoints;						///< Number of setpoints
	float setpoints[MAX_ADAPT_PARAM_SETPOINTS];	///< set points
	float setvalues[MAX_ADAPT_PARAM_SETPOINTS];	///< set values
} adaptive_parameter_t;


/**
 * \brief	Adaptive parameter set structure
 */
typedef struct
{
	int32_t param_count;										///< Number of Parameters
	adaptive_parameter_t parameters[MAX_ADAPT_PARAM_COUNT];		///< Parameter set
} adaptive_parameter_set_t;

/**
 * \brief	Returns a pointer to the adaptive parameter set
 *
 * \return	The adaptive parameter set structure
 */
adaptive_parameter_set_t* adaptive_parameter_get_param_set(void);

/**
 * \brief	Initializes the adaptive parameter set
 */
void adaptive_parameter_init(void);

/**
 * \brief					Add an adaptive parameter to the parameter set
 *
 * \param control_variable	Pointer to the control variable
 * \param parameter			Pointer to the parameter
 * \param nb_setpoints		Number of setpoints
 * \param setpoints			Pointer to the setpoints
 * \param setvalues			Pointer to the setvalues
 *
 * \return					Returns 1 if successfully added and 0 otherwise
 */
int32_t adaptive_parameter_add(	float* control_variable, 
								float* parameter, 
								int32_t nb_setpoints, 
								float* setpoints, 
								float* setvalues);
/**
 * \brief					Update adaptive parameter
 *
 * \param	param			Adaptive parameter to be updated  
 */							
void adaptive_parameter_update(adaptive_parameter_t param);

/**
 * \brief					Update all adaptive parameters
 */	
void adaptive_parameter_update_all(void);

#ifdef __cplusplus
}
#endif

#endif /* ADAPTIVE_PARAMETER_H_ */
