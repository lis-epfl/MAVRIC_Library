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
 * \file acoustic.c
 *
 * \author MAV'RIC Team
 * \author Meysam Basiri
 * \author Gregoire Heitz
 *
 * \brief Acoustic communication and processing functions
 *
 ******************************************************************************/


#define KP_YAW 0.2

#include "drivers/unsupported/acoustic.hpp"
#include "communication/mavlink_stream.hpp"

extern "C"
{
#include "uart_int.h"
#include "util/print_util.h"
#include "util/constants.hpp"
#include "util/quick_trig.h"
}

float az[STORE_SIZE] =          ///< Store pre-computed value for the azimuth
{
    0,
    PI / 2,
    -PI / 4,
    0
};
float el[STORE_SIZE] =          ///< Store pre-computed value for the elevation
{
    0,
    0,
    -PI / 4,
    0
};

float position_previous[3];     ///< store previous position in 3D
float position_current[3];      ///< store current position in 3D

quat_t attitude_previous;       ///< store previous attitude
quat_t attitude_current;        ///< store current attitude

float target_vect[3];           ///< store target vector in 3D

uint8_t first_run = 0;          ///< Flag to tell whether it is the first time you run the controller
uint16_t counter;               ///< counter of non-reliable data
float remote_switch_previous;   ///< store previous status of the remote switch, associated with the acoustic controller

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Send command to turn on the siren
 *
 * \param   out_stream  Pointer to the out stream
 */
static void turn_on_siren(byte_stream_t* out_stream);


/**
 * \brief   Check reliability of the audio data
 *
 * \param   audio_data  Pointer to the audio structure
 */
static void check_reliability(audio_t* audio_data);

/**
 * \brief   Process audio data
 *
 * \param   audio_data  Pointer to the audio structure
 */
static void acoustic_process(audio_t* audio_data);

/**
 * \brief   Set a new waypoint accoding to the acoustic information
 *
 * \param   audio_data  Pointer to the audio structure
 */

static void acoustic_set_waypoint_command(audio_t* audio_data);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void turn_on_siren(byte_stream_t* out_stream)
{
    out_stream->put(out_stream->data, 254);
    out_stream->put(out_stream->data, 131);
    out_stream->put(out_stream->data, 44);
}

void turn_off_siren(byte_stream_t* out_stream)
{
    out_stream->put(out_stream->data, 254);
    out_stream->put(out_stream->data, 131);
    out_stream->put(out_stream->data, 100);
}

static void check_reliability(audio_t* audio_data)
{
    float  cos_el, sin_el, cos_az, sin_az, angular_diff = 0;
    int8_t i;

    if (audio_data->new_data == 0)
    {
        //reset ReliableData flag
        audio_data->reliabe_data = 0;

        if (counter < WAIT_LIMIT)
        {
            counter ++;
        }
        else
        {
            //reset
            counter = 0;
            az[1]   = 0;
            az[2]   = PI / 2;
            az[3]   = -PI / 4;
            audio_data->azimuth     = -500;   // for plotting purposes
            audio_data->elevation   = -500;
        }
    }
    else //newData are available
    {
        counter = 0;
        attitude_current    = audio_data->ahrs->qe;             //store the current IMU attitude and position for later use
        position_current[0] = audio_data->position_estimation->local_position.pos[0];
        position_current[1] = audio_data->position_estimation->local_position.pos[1];
        position_current[2] = audio_data->position_estimation->local_position.pos[2];

        for (i = 1; i < STORE_SIZE; i++)
        {
            az[i - 1] = az[i];
            el[i - 1] = el[i];
        }
        az[STORE_SIZE - 1] = audio_data->azimuth    * PI / 180.0f;
        el[STORE_SIZE - 1] = audio_data->elevation * PI / 180.0f;

        cos_el = quick_trig_cos(el[STORE_SIZE - 1]);
        sin_el = quick_trig_sin(el[STORE_SIZE - 1]);

        for (i = 0; i < (STORE_SIZE - 1); i++)      //compute the total great arc circle between last bearing and the past 3 values
        {
            angular_diff += quick_trig_acos(cos_el * quick_trig_cos(el[i]) * quick_trig_cos(az[STORE_SIZE - 1] - az[i]) + sin_el * quick_trig_sin(el[i]));
        }

        if (angular_diff < RELIABILITY_ARC)
        {
            audio_data->reliabe_data    = 1;
            audio_data->reliabe_az      = az[STORE_SIZE - 1];
            audio_data->reliabe_el      = el[STORE_SIZE - 1];
            cos_az = quick_trig_cos(az[STORE_SIZE - 1]);
            sin_az = quick_trig_sin(az[STORE_SIZE - 1]);
            target_vect[0]  = cos_az * cos_el;
            target_vect[1]  = sin_az * cos_el;
            target_vect[2]  = sin_el;
        }
        else
        {
            audio_data->reliabe_data = 0;
        }

    }

}

static void acoustic_process(audio_t* audio_data)
{

    check_reliability(audio_data);                                          // check the reliability of the current measurement

    if (audio_data->new_data == 1)
    {
        if ((audio_data->reliabe_data == 1) && (audio_data->remote->channels[6] > 0.7f))
        {
            //calculate the approximate waypoint & follow azimuth
            acoustic_set_waypoint_command(audio_data);
        }

        attitude_previous = attitude_current;
        position_previous[0] = position_current[0];
        position_previous[1] = position_current[1];
        position_previous[2] = position_current[2];
    }
    if ((audio_data->remote->channels[6] < 0.7f) && (remote_switch_previous >= 0.7f))
    {
        //turn off controller, reset hover waypoint to current position
        waypoint_handler_hold_init(audio_data->waypoint_handler, audio_data->position_estimation->local_position);
    }

    //////////////for turning on the siren///////////////////////////
    if ((audio_data->remote->channels[6] > 0.9f) && (remote_switch_previous <= 0.9f))
    {
        //siren was off , turn on
        turn_on_siren(audio_data->telemetry_down_stream);
    }
    //if (centralData->acoustic_spin<0.9 && remote_switch_previous>=0.9)
    //{
    //siren was on , turn off
    //turn_off_siren(centralData->telemetry_down_stream);
    //}
    /////////////////////////////////////////////////////////////////

    //update switch previous status
    remote_switch_previous  = audio_data->remote->channels[6];
}

void acoustic_set_speed_command(audio_t* audio_data, float rel_pos[], float dist2wpSqr)
{
    float   norm_rel_dist, v_desiredxy, v_desiredz;
    float dir_desired_bf[3], new_velocity[3];

    norm_rel_dist =  rel_pos[2];

    dir_desired_bf[2] = rel_pos[2];

    v_desiredz = maths_f_min(audio_data->navigation->cruise_speed, (audio_data->navigation->dist2vel_gain * maths_soft_zone(norm_rel_dist, audio_data->stabilisation_copter->stabiliser_stack.velocity_stabiliser.thrust_controller.soft_zone_width)));

    if (audio_data->reliabe_el < 1.35)
    {
        v_desiredxy = maths_f_min(audio_data->navigation->cruise_speed, (maths_center_window_2(4.0f * audio_data->reliabe_az) * (-9.0f * audio_data->reliabe_el + (1.35 * 9))));
    }
    else
    {
        v_desiredxy = 0;
    }

    if (v_desiredz *  maths_f_abs(dir_desired_bf[Z]) > audio_data->navigation->max_climb_rate * norm_rel_dist)
    {
        v_desiredz = audio_data->navigation->max_climb_rate * norm_rel_dist / maths_f_abs(dir_desired_bf[Z]);
    }

    dir_desired_bf[X] = v_desiredxy * quick_trig_cos(audio_data->reliabe_az);
    dir_desired_bf[Y] = v_desiredxy * quick_trig_sin(audio_data->reliabe_az);
    dir_desired_bf[Z] = v_desiredz  * dir_desired_bf[Z] / norm_rel_dist;


    for (uint8_t i = 0; i < 3; i++)
    {
        new_velocity[i] = dir_desired_bf[i];
    }

    audio_data->controls_nav->tvel[X]   = new_velocity[X];
    audio_data->controls_nav->tvel[Y]   = new_velocity[Y];
    audio_data->controls_nav->tvel[Z]   = new_velocity[Z];
    audio_data->controls_nav->rpy[YAW]  = KP_YAW * audio_data->reliabe_az;
}



static void acoustic_set_waypoint_command(audio_t* audio_data)
{
    quat_t qtmp1, qtmp2;
    float target_vect_global[3];
    float speed_gain;

    qtmp1 = quaternions_create_from_vector(target_vect);
    qtmp2 = quaternions_local_to_global(attitude_previous, qtmp1);

    //unit vector to target
    target_vect_global[0] = qtmp2.v[0];
    target_vect_global[1] = qtmp2.v[1];
    target_vect_global[2] = qtmp2.v[2];

    //distance to target on ground (computed from the hight of robot)
    //speed_gain = f_abs(position_previous[2]/target_vect_global[2]);
    //speed_gain = f_abs(1.0-target_vect_global[2])*15.0

    speed_gain = maths_f_abs(quick_trig_acos(target_vect_global[2])) * 20.0f;

    if (speed_gain > 20.0f)
    {
        speed_gain = 20.0f;
    }
    else if (speed_gain < 3.0f)
    {
        speed_gain = 0.0f;
    }

    audio_data->waypoint_handler->waypoint_hold_coordinates.pos[0] = speed_gain * target_vect_global[0] + position_previous[0];
    audio_data->waypoint_handler->waypoint_hold_coordinates.pos[1] = speed_gain * target_vect_global[1] + position_previous[1];
    //centralData->controls_nav.theading=audio_data->azimuth*PI/180;

    if (speed_gain > 3.0f)
    {
        audio_data->waypoint_handler->waypoint_hold_coordinates.heading = atan2(target_vect_global[Y], target_vect_global[X]);
    }
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


void acoustic_init(audio_t*                     audio_data,
                   int32_t                      UID,
                   ahrs_t*                      ahrs,
                   Position_estimation*         position_estimation,
                   remote_t*                    remote,
                   navigation_t*                navigation,
                   stabilisation_copter_t*      stabilisation_copter,
                   control_command_t*           controls_nav,
                   Mavlink_waypoint_handler*    waypoint_handler,
                   byte_stream_t*               telemetry_down_stream)
{
    //init dependencies
    audio_data->ahrs                    =   ahrs;
    audio_data->position_estimation     =   position_estimation;
    audio_data->remote                  =   remote;
    audio_data->navigation              =   navigation;
    audio_data->stabilisation_copter    =   stabilisation_copter;
    audio_data->controls_nav            =   controls_nav;
    audio_data->waypoint_handler        =   waypoint_handler;
    audio_data->telemetry_down_stream   =   telemetry_down_stream;

    // uart setting
    usart_config_t usart_conf_audio             = {};
    usart_conf_audio.mode                       = UART_IN;
    usart_conf_audio.uart_device.uart           = (avr32_usart_t*)&AVR32_USART2;
    usart_conf_audio.uart_device.IRQ            = AVR32_USART2_IRQ;
    usart_conf_audio.uart_device.receive_stream = NULL;
    usart_conf_audio.options                    = {};
    usart_conf_audio.options.baudrate           = 57600;
    usart_conf_audio.options.charlength         = 8;
    usart_conf_audio.options.paritytype         = USART_NO_PARITY;
    usart_conf_audio.options.stopbits           = USART_1_STOPBIT;
    usart_conf_audio.options.channelmode        = USART_NORMAL_CHMODE;
    usart_conf_audio.rx_pin_map                 = {AVR32_USART2_RXD_0_1_PIN, AVR32_USART2_RXD_0_1_FUNCTION};
    usart_conf_audio.tx_pin_map                 = {AVR32_USART2_TXD_0_1_PIN, AVR32_USART2_TXD_0_1_FUNCTION};

    uart_int_set_usart_conf(UID, &usart_conf_audio);

    //uart configuration
    uart_int_init(UID);
    //uart_int_register_write_stream(uart_int_get_uart_handle(UID), &(centralData->audio_stream_out));
    // Registering streams
    buffer_make_buffered_stream_lossy(&(audio_data->audio_buffer), &(audio_data->audio_stream_in));
    uart_int_register_read_stream(uart_int_get_uart_handle(UID), &(audio_data->audio_stream_in));
}

bool acoustic_update(audio_t* audio_data)
{
    uint8_t buffer[6];

    //reset new_data flag
    audio_data->new_data = 0;

    if (first_run == 0)
    {
        first_run = 1;
        buffer_clear(&(audio_data->audio_buffer));
    }
    else if (buffer_bytes_available(&(audio_data->audio_buffer)) >= 6)
    {
        buffer[0] = buffer_get(&(audio_data->audio_buffer));
        if (buffer[0] == 254)
        {
            buffer[1] = buffer_get(&(audio_data->audio_buffer));
            buffer[2] = buffer_get(&(audio_data->audio_buffer));
            buffer[3] = buffer_get(&(audio_data->audio_buffer));
            buffer[4] = buffer_get(&(audio_data->audio_buffer));
            buffer[5] = buffer_get(&(audio_data->audio_buffer));

            //checksum check
            if ((-(buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF) == buffer[5])
            {
                audio_data->azimuth     = (float)((buffer[1] << 8) | buffer[2]);
                audio_data->elevation   = (float)((buffer[3] << 8) | buffer[4]);
                audio_data->new_data    = 1;
                buffer_clear(&(audio_data->audio_buffer));
            }
        }

    }

    acoustic_process(audio_data);

    return true;
}
