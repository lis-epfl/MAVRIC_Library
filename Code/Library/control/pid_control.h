/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file pid_control.h
 *
 * PID controller
 */


#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include <math.h>

typedef struct {
	float pregain, postgain;			///< pregain and postgain 
	float accumulator;					///< accumulator
	float maths_clip;							///< clipping value
	float leakiness;					///< leakiness
} Integrator_t;							///< integrator parameters

typedef struct {
	float gain;							///< gain 
	float previous;						///< Previous input to the differentiator 
	float LPF;							///< Low pass filter
	float maths_clip;							///< clipping value
} Differentiator_t;						///< differentiator parameters

typedef struct {
	float p_gain;						///< proportional gain
	float clip_min, clip_max;			///< min and max clipping values
	Integrator_t integrator;			///< integrator parameters
	Differentiator_t differentiator;	///< differentiator parameters
	float output;						///< output
	float error;						///< error
	uint32_t last_update;				///< last update time in timer tick
	float dt;							///< time step
	float soft_zone_width;				///< approximate width of a "soft zone" on the error input, i.e. a region of low gain around the target point. Value 0 -> switched off
}PID_Controller_t;						///< PID controller parameters

/**
 * \brief				Passing through the controller
 */
PID_Controller_t pid_control_passthroughController(void);

/**
 * \brief				Initialize integrator parameters
 *
 * \param	integrator	Pointer to an integrator structure
 * \param	pregain
 * \param	postgain
 * \param	clip_val	Clipping value
 */
void pid_control_init_integrator(Integrator_t *integrator, float pregain, float postgain, float clip_val);

/**
 * \brief				Reset integrator
 */
void pid_control_reset_integrator(Integrator_t *integrator);

/**
 * \brief				Initialize Differentiator parameters
 *
 * \param	diff		Pointer to differentiator structure
 * \param	gain
 * \param	LPF			Low pass filter
 * \param	clip_val	Clipping value
 */
void pid_control_init_differenciator(Differentiator_t *diff, float gain, float LPF, float clip_val);

/**
 * \brief				Integrating
 *
 * \param	integrator	Pointer to integrator parameters
 * \param	input
 * \param	dt			Timestep
 *
 * \return				Result
 */
float pid_control_integrate(Integrator_t *integrator, float input, float dt);

/**
 * \brief Differentiating
 *
 * \param	diff		Pointer to differentiator parameters
 * \param	input
 * \param	dt			Timestep
 *
 * \return				Result
 */
float pid_control_differentiate(Differentiator_t *diff, float input,  float dt);

/**
 * \brief				Update the PID controller
 *
 * \param	controller	Pointer to the PID controller structure
 * \param	error		Error in the controlled variable
 *
 * \return				Controller output
 */
float pid_control_update(PID_Controller_t* controller, float error);

/**
 * \brief				Update the PID controller for a given time step
 *
 * \param	controller	Pointer to the PID controller structure
 * \param	error		Error in the controlled variable
 * \param	dt			Timestep
 *
 * \return				Controller output
 */
float pid_control_update_dt(PID_Controller_t* controller, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_H_ */
