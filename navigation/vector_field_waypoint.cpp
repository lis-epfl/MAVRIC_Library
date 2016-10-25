/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file vector_field_waypoint.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Vector field navigation using repulsors and attractors set through GPS waypoints
 *
 ******************************************************************************/


#include "navigation/vector_field_waypoint.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "cstdint"
#include "util/vectors.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


/**
 * \brief       Vector field for floor avoidance
 *
 * \details     Computes a 3D velocity vector keeping the MAV from colliding with the floor
 *
 * \param       pos_mav     Current position of the MAV (input)
 * \param       altitude    Threshold altitude (>0) under which the vector field is active (input)
 * \param       vector      Velocity command vector (output)
 */
static void vector_field_floor(const float pos_mav[3], const float altitude, float vector[3]);


/**
 * \brief       Vector field for attractor object
 *
 * \details     Computes a 3D velocity vector attracting the MAV towards a 3D location
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the attractor (input)
 * \param       attractiveness  Weight given to this object
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_attractor(const float pos_mav[3], const float pos_obj[3], const float attractiveness, float vector[3]);


/**
 * \brief       Vector field for repulsive cylinder
 *
 * \details     Computes a 3D velocity vector pushing the MAV away from a vertical cylinder
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the repulsor (input)
 * \param       repulsiveness   Weight given to this object (input)
 * \param       max_range       Range of action (in m)
 * \param       safety_radius   Minimum distance the object can be approached (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_repulsor_cylinder(const float pos_mav[3], const float pos_obj[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3]);


/**
 * \brief       Vector field for repulsive object
 *
 * \details     Computes a 3D velocity vector pushing the MAV away from a 3D location
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the repulsor (input)
 * \param       repulsiveness   Weight given to this object
 * \param       max_range       Range of action (in m)
 * \param       safety_radius   Minimum distance the object can be approached (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_repulsor_sphere(const float pos_mav[3], const float pos_obj[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3]);


/**
 * \brief       Vector field for circular waypoint
 *
 * \details     Computes a 3D velocity vector guiding the MAV around a circular waypoint
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the center of the circle (input)
 * \param       attractiveness  Weight given to this object (input)
 * \param       cruise_speed    Nominal speed around the circle in m/s (input)
 * \param       radius          Radius of the circle to follow (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_circular_waypoint(const float pos_mav[3], const float pos_obj[3], const float attractiveness, const float cruise_speed, const float radius, float vector[3]);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


/**
 * \brief       Vector field for floor avoidance
 *
 * \details     Computes a 3D velocity vector keeping the MAV from colliding with the floor
 *
 * \param       pos_mav     Current position of the MAV (input)
 * \param       altitude    Threshold altitude (>0) under which the vector field is active (input)
 * \param       vector      Velocity command vector (output)
 */
static void vector_field_floor(const float pos_mav[3], const float altitude, float vector[3])
{
    // The horizontal and vertical components are always null
    vector[X] = 0.0f;
    vector[Y] = 0.0f;

    if (pos_mav[2] >= 0.0f)
    {
        // Avoid division by 0
        vector[Z] = 0.0f;
    }
    else
    {
        /**
         *  Student code Here
         */

        if (pos_mav[2] < -altitude)
        {
            // High altitude
            vector[Z] = 0.0f;
        }
        else
        {
            // Low altitude
            vector[Z] = (1.0f / pos_mav[2]) * (altitude - pos_mav[2]);
        }

        /**
         *  End of Student code
         */

    }
}


/**
 * \brief       Vector field for attractor object
 *
 * \details     Computes a 3D velocity vector attracting the MAV towards a 3D location
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the attractor (input)
 * \param       attractiveness  Weight given to this object (input)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_attractor(const float pos_mav[3], const float pos_obj[3], const float attractiveness, float vector[3])
{
    float direction[3] = {0.0f, 0.0f, 0.0f};    // desired direction (unit vector)
    float speed = 0.0f;                         // desired speed

    /**
     *  Student code Here
     */

    // Compute vector from MAV to goal
    float mav_to_obj[3] = { pos_obj[X] - pos_mav[X],
                            pos_obj[Y] - pos_mav[Y],
                            pos_obj[Z] - pos_mav[Z]
                          };

    // Compute norm of this vector
    float dist_to_object = vectors_norm(mav_to_obj);

    // Get desired direction
    direction[X] = mav_to_obj[X] / dist_to_object;
    direction[Y] = mav_to_obj[Y] / dist_to_object;
    direction[Z] = mav_to_obj[Z] / dist_to_object;

    // Compute desired speed
    speed = attractiveness * dist_to_object;

    /**
     *  End of Student code
     */

    vector[X] = direction[X] * speed;
    vector[Y] = direction[Y] * speed;
    vector[Z] = direction[Z] * speed;
}


/**
 * \brief       Vector field for repulsive cylinder
 *
 * \details     Computes a 3D velocity vector pushing the MAV away from a vertical cylinder
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the repulsor (input)
 * \param       repulsiveness   Weight given to this object (input)
 * \param       max_range       Range of action (in m)
 * \param       safety_radius   Minimum distance the object can be approached (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_repulsor_cylinder(const float pos_mav[3], const float pos_obj[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3])
{
    float direction[3] = {0.0f, 0.0f, 0.0f};    // desired direction (unit vector)
    float speed = 0.0f;                         // desired speed

    /**
     *  Student code Here
     */

    // Compute vector from MAV to goal
    float mav_to_obj[3] = { pos_obj[X] - pos_mav[X],
                            pos_obj[Y] - pos_mav[Y],
                            0.0f
                          };

    // Compute norm of this vector
    float dist_to_object = vectors_norm(mav_to_obj);

    // Get desired direction
    direction[X] = mav_to_obj[X] / dist_to_object;
    direction[Y] = mav_to_obj[Y] / dist_to_object;
    direction[Z] = mav_to_obj[Z] / dist_to_object;

    // Compute desired speed
    if (dist_to_object <= safety_radius)
    {
        speed = 0.0f;
    }
    else if (dist_to_object > max_range)
    {
        speed = 0.0f;
    }
    else
    {
        speed = - repulsiveness  / (dist_to_object - safety_radius) * (max_range - dist_to_object);
    }

    /**
     *  End of Student code
     */

    vector[X] = direction[X] * speed;
    vector[Y] = direction[Y] * speed;
    vector[Z] = direction[Z] * speed;
}


/**
 * \brief       Vector field for repulsive object
 *
 * \details     Computes a 3D velocity vector pushing the MAV away from a 3D location
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the repulsor (input)
 * \param       repulsiveness   Weight given to this object (input)
 * \param       max_range       Range of action (in m)
 * \param       safety_radius   Minimum distance the object can be approached (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_repulsor_sphere(const float pos_mav[3], const float pos_obj[3], const float repulsiveness, const float max_range, const float safety_radius, float vector[3])
{
    float direction[3] = {0.0f, 0.0f, 0.0f};    // desired direction (unit vector)
    float speed = 0.0f;                         // desired speed

    /**
     *  Student code Here
     */

    // Compute vector from MAV to goal
    float mav_to_obj[3] = { pos_obj[X] - pos_mav[X],
                            pos_obj[Y] - pos_mav[Y],
                            pos_obj[Z] - pos_mav[Z]
                          };

    // Compute norm of this vector
    float dist_to_object = vectors_norm(mav_to_obj);

    // Get desired direction
    direction[X] = mav_to_obj[X] / dist_to_object;
    direction[Y] = mav_to_obj[Y] / dist_to_object;
    direction[Z] = mav_to_obj[Z] / dist_to_object;

    // Compute desired speed
    if (dist_to_object <= safety_radius)
    {
        speed = 0.0f;
    }
    else if (dist_to_object > max_range)
    {
        speed = 0.0f;
    }
    else
    {
        speed = - repulsiveness  / (dist_to_object - safety_radius) * (max_range - dist_to_object);
    }

    /**
     *  End of Student code
     */

    vector[X] = direction[X] * speed;
    vector[Y] = direction[Y] * speed;
    vector[Z] = direction[Z] * speed;
}


/**
 * \brief       Vector field for circular waypoint
 *
 * \details     Computes a 3D velocity vector guiding the MAV around a circular waypoint
 *
 * \param       pos_mav         Current position of the MAV (input)
 * \param       pos_obj         Position of the center of the circle (input)
 * \param       attractiveness  Weight given to this object (input)
 * \param       cruise_speed    Nominal speed around the circle in m/s (input)
 * \param       radius          Radius of the circle to follow (in m)
 * \param       vector          Velocity command vector (output)
 */
static void vector_field_circular_waypoint(const float pos_mav[3], const float pos_obj[3], const float attractiveness, const float cruise_speed, const float radius, float vector[3])
{
    float direction_radial[3]       = {0.0f, 0.0f, 0.0f};   // desired direction (unit vector) toward the closest point of the circle
    float direction_tangential[3]   = {0.0f, 0.0f, 0.0f};   // desired direction (unit vector) tangential to the circle (circular motion)
    float speed_radial              = 0.0f;                 // desired radial speed
    float speed_tangential          = 0.0f;                 // desired tangential speed

    /**
     *  Student code Here
     */
    (void)pos_mav; // unused
    (void)pos_obj; // unused
    (void)attractiveness; // unused
    (void)cruise_speed; // unused
    (void)radius; // unused

    /**
     *  End of Student code
     */

    vector[X] = direction_radial[X]     * speed_radial +
                direction_tangential[X] * speed_tangential;
    vector[Y] = direction_radial[Y]     * speed_radial +
                direction_tangential[Y] * speed_tangential;
    vector[Z] = direction_radial[Z]     * speed_radial +
                direction_tangential[Z] * speed_tangential;
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool vector_field_waypoint_init(vector_field_waypoint_t* vector_field, Mavlink_waypoint_handler* waypoint_handler, const INS* ins, velocity_command_t* velocity_command, const vector_field_waypoint_conf_t* config)
{
    // Init dependencies
    vector_field->waypoint_handler  = waypoint_handler;
    vector_field->ins               = ins;
    vector_field->velocity_command  = velocity_command;

    return true;
}


bool vector_field_waypoint_update(vector_field_waypoint_t* vector_field)
{
    float tmp_vector[3];
    float pos_obj[3];

    // Re-init velocity command
    vector_field->velocity_command->xyz[X]  = 0.0f;
    vector_field->velocity_command->xyz[Y]  = 0.0f;
    vector_field->velocity_command->xyz[Z]  = 0.0f;

    // Compute vector field for floor avoidance
    vector_field_floor(vector_field->ins->position_lf().data(),
                       20,
                       tmp_vector);

    // Add floor vector field to the velocity command
    vector_field->velocity_command->xyz[X] += tmp_vector[X];
    vector_field->velocity_command->xyz[Y] += tmp_vector[Y];
    vector_field->velocity_command->xyz[Z] += tmp_vector[Z];

    // Go through waypoint list
    for (uint16_t i = 0; i < vector_field->waypoint_handler->waypoint_count(); ++i)
    {
        const Waypoint& waypoint = vector_field->waypoint_handler->waypoint_from_index(i);
        local_position_t local_wpt = waypoint.local_pos();

        // Get object position
        pos_obj[X] = local_wpt[X];
        pos_obj[Y] = local_wpt[Y];
        pos_obj[Z] = local_wpt[Z];

        switch (waypoint.command())
        {
            // Attractive object
            case 22:
                vector_field_attractor(vector_field->ins->position_lf().data(),
                                       pos_obj,
                                       waypoint.param1(),     // attractiveness
                                       tmp_vector);
                break;

            // Cylindrical repulsive object
            case 17:
                vector_field_repulsor_cylinder(vector_field->ins->position_lf().data(),
                                               pos_obj,
                                               waypoint.param1(),     // repulsiveness
                                               waypoint.param2(),     // max_range
                                               waypoint.param3(),     // safety_radius
                                               tmp_vector);
                break;

            // Spherical repulsive object
            case 18:
                vector_field_repulsor_sphere(vector_field->ins->position_lf().data(),
                                             pos_obj,
                                             waypoint.param1(),   // repulsiveness
                                             waypoint.param2(),   // max_range
                                             waypoint.param3(),   // safety_radius
                                             tmp_vector);
                break;

            // Circular waypoint
          case 16:
                vector_field_circular_waypoint(vector_field->ins->position_lf().data(),
                                               pos_obj,
                                               waypoint.param1(),     // attractiveness
                                               waypoint.param2(),     // cruise_speed
                                               waypoint.param3(), // radius
                                               tmp_vector);
                break;

            // Unknown object
            default:
                tmp_vector[X] = 0.0f;
                tmp_vector[Y] = 0.0f;
                tmp_vector[Z] = 0.0f;
                break;
        }

        // Add vector field to the velocity command
        vector_field->velocity_command->xyz[X] += tmp_vector[X];
        vector_field->velocity_command->xyz[Y] += tmp_vector[Y];
        vector_field->velocity_command->xyz[Z] += tmp_vector[Z];
    }

    // Limit velocity
    float vel_norm = vectors_norm(vector_field->velocity_command->xyz.data());
    if (vel_norm > 5.0f)
    {
        vector_field->velocity_command->xyz[X] = 5.0f * vector_field->velocity_command->xyz[X] / vel_norm;
        vector_field->velocity_command->xyz[Y] = 5.0f * vector_field->velocity_command->xyz[Y] / vel_norm;
        vector_field->velocity_command->xyz[Z] = 5.0f * vector_field->velocity_command->xyz[Z] / vel_norm;
    }

    return true;
}
