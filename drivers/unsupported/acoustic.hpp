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
 * \file acoustic.h
 *
 * \author MAV'RIC Team
 * \author Meysam Basiri
 * \author Gregoire Heitz
 *
 * \brief Acoustic communication and processing functions
 *
 ******************************************************************************/

#ifndef ACOUSTIC_H_
#define ACOUSTIC_H_

#include <stdint.h>
#include <stdbool.h>

#include "communication/mavlink_waypoint_handler.hpp"
#include "control/stabilisation_copter.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/remote.hpp"

extern "C"
{
#include "util/streams.h"
#include "buffer.h"
#include "sensing/ahrs.h"
#include "control/stabilisation.h"
}

#define STORE_SIZE          4       ///< number of azimuth/elevation values stored for reliability test
#define RELIABILITY_ARC     0.25f   ///< the threshold to consider a measurement as reliable (compared with previous 3 measurements)
#define WAIT_LIMIT          6       ///< wait for WAITLIMIT*ACOUSTIC_TASK_ITERATION ms to recieve the new measurement, else reset reliability
#define MAX_DETECTION_RANGE 100     ///< set the maximum range


/**
 * \brief   Structure of the acoustic data.
 */
typedef struct
{
    int16_t azimuth;                                    ///< The azimuth angle corresponds to the horizontal orientation of the sound
    int16_t elevation;                                  ///< The elevation angle corresponds to the vertical orientation of the sound
    bool    new_data;                                   ///< The flag to tell that new data are available
    bool    reliabe_data;                               ///< The flag to tell that new data are reliable
    float   reliabe_az;                                 ///< Number of reliable azimuth measurement
    float   reliabe_el;                                 ///< Number of reliable elevation measurement

    buffer_t audio_buffer;                              ///< Acoustic buffer
    byte_stream_t audio_stream_in;                      ///< Acoustic in coming stream
    //byte_stream_t* audio_stream_out;                  ///< Acoustic out coming stream

    ahrs_t*                     ahrs;                   ///< The pointer to the attitude estimation structure
    position_estimation_t*      position_estimation;    ///< The pointer to the position estimation structure
    remote_t*                   remote;                 ///< The pointer to the remote structure
    navigation_t*               navigation;             ///< The pointer to the navigation control structure
    stabilisation_copter_t*     stabilisation_copter;   ///< The pointer to the stabilization copter structure
    control_command_t*          controls_nav;           ///< The pointer to the control structure
    mavlink_waypoint_handler_t* waypoint_handler;       ///< The pointer to the waypoint handler structure
    byte_stream_t*              telemetry_down_stream;  ///< The pointer to the down coming telemetry byte stream
} audio_t;


/**
 * \brief   Initialize the acoustic structure and communication with the audio board
 *
 * \param   audio_data              The pointer to the acoustic structure
 * \param   UID                     The UART identification number
 * \param   ahrs                    The pointer to the attitude estimation structure
 * \param   position_estimation     The pointer to the position estimation structure
 * \param   remote                  The pointer to the remote structure
 * \param   navigation              The pointer to the navigation structure
 * \param   stabilisation_copter    The pointer to the stabilisation_copter structure
 * \param   controls_nav            The pointer to the controls_nav structure
 * \param   waypoint_handler        The pointer to the waypoint_handler structure
 * \param   telemetry_down_stream   The pointer to the telemetry_down_stream structure
 */
void acoustic_init(audio_t*         audio_data,
                   int32_t                      UID,
                   ahrs_t*                      ahrs,
                   position_estimation_t*       position_estimation,
                   remote_t*                    remote,
                   navigation_t*                navigation,
                   stabilisation_copter_t*      stabilisation_copter,
                   control_command_t*           controls_nav,
                   mavlink_waypoint_handler_t*  waypoint_handler,
                   byte_stream_t*               telemetry_down_stream);


/**
 * \brief   Run the acoustic communication and process the data.
 *
 * \param   audio_data  The pointer to the acoustic structure
 *
 * \return  Success
 */
bool acoustic_update(audio_t* audio_data);


/**
 * \brief   Send command to turn off the siren
 * \detail  Should be declared static in private function declaration part
 *          in .c file but move hear to remove "defined but not used" warning
 *
 * \param   out_stream  Pointer to the out stream
 */
void turn_off_siren(byte_stream_t* out_stream);

/**
 * \brief   Set speed command according to acoustic information
 * \detail  Should be declared static in private function declaration part
 *          in .c file but move hear to remove "defined but not used" warning
 *
 * \param   audio_data  Pointer to the audio structure
 * \param   rel_pos     Array of the relative position
 * \param   dist2wpSqr  Square of the distance to waypoint
 */
void acoustic_set_speed_command(audio_t* audio_data, float rel_pos[], float dist2wpSqr);

#endif