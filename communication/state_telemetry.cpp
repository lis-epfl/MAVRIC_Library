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
 * \file state_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the state
 *
 ******************************************************************************/


#include "communication/state_telemetry.hpp"
#include "communication/state.hpp"
#include "util/version.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.hpp"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief                       Sets the time at which the heartbeat was received and the number of message received
 *
 * \param   state               The pointer to the state structure
 * \param   sysid               The system ID
 * \param   msg                 The received MAVLink message structure
 */
void state_telemetry_heartbeat_received(State* state, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief                       Set the state and the mode of the vehicle
 *
 * \param   state               The pointer to the state structure
 * \param   sysid               The system ID
 * \param   msg                 The received MAVLink message structure
 */
static void state_telemetry_set_mav_mode(State_machine* state_machine, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Set the MAV mode from a command message
 *
 * \param   state               The pointer to the state structure
 * \param   packet              The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t state_telemetry_set_mode_from_cmd(State_machine* state_machine, mavlink_command_long_t* packet);

/**
 * \brief   Set the ARM mode from a command message
 *
 * \param   state               The pointer to the state structure
 * \param   packet              The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t state_telemetry_set_arm_from_cmd(State* state, mavlink_command_long_t* packet);

/**
 * \brief   Set state->mav_mode
 *
 * \details performs the following checks before changing
 *          - if arming is refused by state, reject new mode
 *          - if in calibration state, reject new mode
 *          the HIL flag is ignored
 *
 * \param   state               The pointer to the state structure
 * \param   mav_mode            Desired mav_mode
 *
 * \return  success             true if mode was accepted, false if refused
 */
bool state_telemetry_set_mode(State_machine* state_machine, Mav_mode mav_mode);

/**
 * \brief   Callback to the command 520 MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
 *
 * \param   state               The pointer to the state structure
 * \param   packet              The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
mav_result_t state_telemetry_send_autopilot_capabilities(State* state, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void state_telemetry_heartbeat_received(State* state,
                                        uint32_t __attribute__((unused)) sysid,
                                        mavlink_message_t __attribute__((unused)) * msg)
{
    state->first_connection_set = true;

    state->last_heartbeat_msg = time_keeper_get_s();
    state->msg_count++;
}

void state_telemetry_set_mav_mode(State_machine* state_machine, uint32_t sysid, mavlink_message_t* msg)
{
    // decode packet
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    // No component ID in mavlink_set_mode_t so no control
    if ((uint8_t)packet.target_system == (uint8_t)sysid)
    {
        // try to set the mode
        state_telemetry_set_mode(state_machine, packet.base_mode);
    }
}

static mav_result_t state_telemetry_set_mode_from_cmd(State_machine* state_machine, mavlink_command_long_t* packet)
{
    Mav_mode new_mode = packet->param1;
    return state_telemetry_set_mode(state_machine,new_mode) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
}

bool state_telemetry_set_mode(State_machine* state_machine, Mav_mode new_mode)
{
    // if calibrating, refuse new mode
    if (state_machine->state_.mav_state_ == MAV_STATE_CALIBRATING)
    {
        print_util_dbg_print("[STATE TELEMETRY] calibrating -> refusing new mode\r\n");
        return false;
    }

    // try to set arming state, if rejected, refuse new mode
    if(!state_machine->state_.set_armed(new_mode.is_armed()))
    {
        print_util_dbg_print("[STATE TELEMETRY] could not change arming state -> refusing new mode\r\n");
        return false;
    }

    // set mav_mode
    return state_machine->set_ctrl_mode(new_mode);
}

static mav_result_t state_telemetry_set_arm_from_cmd(State* state, mavlink_command_long_t* packet)
{
    float arm_cmd = packet->param1;
    return state->set_armed(arm_cmd == 1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
}

mav_result_t state_telemetry_send_autopilot_capabilities(State* state, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_ACCEPTED;

    // Autopilot version
    if (packet->param1 == 1)
    {
        mavlink_message_t msg;

        mavlink_msg_autopilot_version_pack( state->mavlink_stream_.sysid(),                         // uint8_t system_id,
                                            state->mavlink_stream_.sysid(),                         // uint8_t component_id,
                                            &msg,                                                   // mavlink_message_t* msg,
                                            0,                                                      // uint64_t capabilities,
                                            version::project,                                       // uint32_t flight_sw_version,
                                            version::mavric,                                        // uint32_t middleware_sw_version,
                                            0,                                                      // uint32_t os_sw_version,
                                            0,                                                      // uint32_t board_version,
                                            version::project_git_hash,                              // const uint8_t *flight_custom_version,
                                            version::mavric_git_hash,                               // const uint8_t *middleware_custom_version,
                                            version::project_name,                                  // const uint8_t *os_custom_version,
                                            0,                                                      // uint16_t vendor_id,
                                            0,                                                      // uint16_t product_id,
                                            0 );                                                    // uint64_t uid );
        state->mavlink_stream_.send(&msg);
    }

    // Add other capabilities here
    // if (packet->param2 == 1)
    // {
    //    // [...]
    // }
    // [...]
    // if (packet->param6 == 1)
    // {
    //    // [...]
    // }
    return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_telemetry_init(State_machine* state_machine, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Add callbacks for onboard parameters requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_HEARTBEAT; // 1
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &state_telemetry_heartbeat_received;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) &state_machine->state_;
    init_success &= message_handler->add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_SET_MODE; // 11
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &state_telemetry_set_mav_mode;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) state_machine;
    init_success &= message_handler->add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_DO_SET_MODE; // 176
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)         &state_telemetry_set_mode_from_cmd;
    callbackcmd.module_struct  = (Mavlink_message_handler::handling_module_struct_t) state_machine;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_COMPONENT_ARM_DISARM; // 400
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &state_telemetry_set_arm_from_cmd;
    callbackcmd.module_struct =                                 &state_machine->state_;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    callbackcmd.command_id    = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES; // 520
    callbackcmd.sysid_filter  = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &state_telemetry_send_autopilot_capabilities;
    callbackcmd.module_struct =                                 &state_machine->state_;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}

void state_telemetry_send_heartbeat(const State* state, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_heartbeat_pack(mavlink_stream->sysid(),
                               mavlink_stream->compid(),
                               msg,
                               state->autopilot_type,
                               state->autopilot_name,
                               state->mav_mode().bits(),
                               state->mav_mode_custom,
                               state->mav_state_);
}

void state_telemetry_send_status(const State* state, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_sys_status_pack(mavlink_stream->sysid(),
                                mavlink_stream->compid(),
                                msg,
                                state->sensor_present,                      // sensors present
                                state->sensor_enabled,                      // sensors enabled
                                state->sensor_health,                       // sensors health
                                0,                                          // load
                                (uint16_t)(1000.0f * state->battery_.voltage()), // bat voltage (mV)
                                0,                                          // current (mA)
                                (int8_t)state->battery_.level(),            // battery remaining
                                0, 0,                                       // comms drop, comms errors
                                0, 0, 0, 0);                                // autopilot specific errors

}
