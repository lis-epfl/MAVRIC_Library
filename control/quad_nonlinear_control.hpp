#ifndef QUAD_NONLINEAR_CONTROL_H_
#define QUAD_NONLINEAR_CONTROL_H_

#include <math.h>
#include "sensing/position_estimation.hpp"
#include "util/matrix.hpp"

extern "C"
{
#include "control/control_command.h"
}

typedef struct
{
    const Position_estimation* pos_est;                         ///< The pointer to the position estimation structure
    const ahrs_t* ahrs;                                         ///< The pointer to the attitude estimation structure
    thrust_command_t* thrust_command;                           ///< The pointer to the thrust command structure
    torque_command_t* torque_command;                           ///< The pointer to the torque command structure
} quad_nonlinear_control_struct;

bool quad_nonlinear_control_init(quad_nonlinear_control_struct* quad_nonlinear_control, const Position_estimation* pos_est, const ahrs_t* ahrs, thrust_command_t* thrust, torque_command_t* torque);

void quad_nonlinear_control_law(quad_nonlinear_control_struct* quad_nonlinear_control);

void quat_to_rot(quat_t quaternion, Mat<3,3>& res);

#endif