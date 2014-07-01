/**
 *  This file handles the stabilization of the platform
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */


#ifndef STABILISATION_COPTER_H_
#define STABILISATION_COPTER_H_

#include "stabilisation.h"


typedef struct {
	Stabiliser_t rate_stabiliser;
	Stabiliser_t attitude_stabiliser;
	Stabiliser_t velocity_stabiliser;
	float yaw_coordination_velocity;
} Stabiliser_Stack_copter_t;

/**
 * \brief Initialize module stabilization
 * \param stabiliser_stack pointer to structure with all controllers 
 */
void init_stabilisation_copter(Stabiliser_Stack_copter_t* stabiliser_stack);

/**
 * \brief gets the velocity vector from the remote
 * \param tvel pointer to array for storing the velocity vector 
 */
void get_velocity_vector_from_remote(float tvel[]);

/**
 * \brief Main Controller for controlling and stabilizing the quad
 * \param imu Pointer to Imu
 * \param pos_est pointer to position estimator
 * \param control_input pointer to the controlling inputs
 */
void cascade_stabilise_copter(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input);

/**
 * \brief mix to servo for quad configuration diagonal
 * \param control pointer to controlling inputs
 */
void mix_to_servos_diag_quad(Control_Command_t *control);

/**
 * \brief mix to servo for quad configuration cross
 * \param control pointer to controlling inputs
 */
void mix_to_servos_cross_quad(Control_Command_t *control);

#endif /* STABILISATION_COPTER_H_ */