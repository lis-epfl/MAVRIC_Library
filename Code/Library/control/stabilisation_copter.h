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
 * \file stabilisation_copter.h
 *
 * This file handles the stabilization of the platform
 */


#ifndef STABILISATION_COPTER_H_
#define STABILISATION_COPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation.h"

typedef struct 
{
	Stabiliser_t rate_stabiliser;
	Stabiliser_t attitude_stabiliser;
	Stabiliser_t velocity_stabiliser;
	float yaw_coordination_velocity;
} Stabiliser_Stack_copter_t;

/**
 * \brief						Initialize module stabilization
 *
 * \param	stabiliser_stack	Pointer to structure with all controllers 
 */
void stabilisation_copter_init(Stabiliser_Stack_copter_t* stabiliser_stack);

/**
 * \brief						Gets the velocity vector from the remote
 *
 * \param	tvel				Pointer to array for storing the velocity vector 
 */
void stabilisation_copter_get_velocity_vector_from_remote(float tvel[]);

/**
 * \brief						Main Controller for controlling and stabilizing the quad
 *
 * \param	imu					Pointer to Imu
 * \param	pos_est				Pointer to position estimator
 * \param	control_input		Pointer to the controlling inputs
 */
void stabilisation_copter_cascade_stabilise(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input);

/**
 * \brief						Mix to servo for quad configuration diagonal
 *
 * \param	control				Pointer to controlling inputs
 */
void stabilisation_copter_mix_to_servos_diag_quad(Control_Command_t *control);

/**
 * \brief						Mix to servo for quad configuration cross
 *
 * \param	control				Pointer to controlling inputs
 */
void stabilisation_copter_mix_to_servos_cross_quad(Control_Command_t *control);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_COPTER_H_ */