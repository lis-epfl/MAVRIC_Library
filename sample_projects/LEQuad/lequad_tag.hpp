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
 * \file lequad_tag.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class for the lequad with the landing on tag extension
 *
 ******************************************************************************/


#ifndef LEQUAD_TAG_HPP_
#define LEQUAD_TAG_HPP_

#include "communication/mavlink_waypoint_handler_tag.hpp"
#include "sample_projects/LEQuad/lequad.hpp"
#include "sensing/offboard_tag_search.hpp"

/**
 * \brief Central data for indoor use
 */
class LEQuad_tag: public LEQuad
{
public:
    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline LEQuad::conf_t default_config(uint8_t sysid = 1);

    /**
     * \brief   Constructor
     */
    LEQuad_dronedome( Imu& imu,
                      Barometer& barometer,
                      Gps& gps,
                      Sonar& sonar,
                      Serial& serial_mavlink,
                      Serial& raspi_serial_mavlink,
                      Satellite& satellite,
                      State_display& state_display,
                      File& file_flash,
                      Battery& battery,
                      Servo& servo_0,
                      Servo& servo_1,
                      Servo& servo_2,
                      Servo& servo_3,
                      Servo& servo_4,
                      Servo& servo_5,
                      Servo& servo_6,
                      Servo& servo_7,
                      File& file1,
                      File& file2,
                      offboard_tag_search_conf_t& offboard_tag_search_conf,
                      LEQuad::conf_t config = LEQuad::default_config() ):
          LEQuad(imu, barometer, gps, sonar, serial_mavlink, satellite, state_display, file_flash, battery, 
                      servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
                      file1, file2, config),
          raspi_serial_mavlink(raspi_serial_mavlink),
          raspi_mavlink_communication(raspi_serial_mavlink, state, file_flash, config.mavlink_communication_config),
          offboard_tag_search(position_estimation, ahrs, waypoint_handler, raspi_mavlink_communication, offboard_tag_search_conf),
          waypoint_handler(cascade_controller_, cascade_controller_, cascade_controller_, ahrs, position_estimation, state, offboard_tag_search, raspi_mavlink_communication, mission_planner, communication.handler())
      {

      }

    /**
     * \brief   Inits the land on tag part of the autopilor
     *
     * \return  success
     */
    virtual bool init_tag(void)
    {
        bool ret = true;

        /* Offboard computer communication */
        ret &= scheduler.add_task(4000, (Scheduler_task::task_function_t)&Mavlink_communication_T::update, (Scheduler_task::task_argument_t)&raspi_mavlink_communication);

        /* Offboard tag search */
        // UP telemetry
        ret &= offboard_tag_search_telemetry_init(&offboard_tag_search,
                        &raspi_mavlink_communication.message_handler());

        // DOWN telemetry
        ret &= mavlink_communication_.add_msg_send( MAVLINK_MSG_ID_DEBUG_VECT,
                                                            250000, (Mavlink_communication_T::send_msg_function_t)&offboard_tag_search_goal_location_telemetry_send,
                                                            &offboard_tag_search);

        // Data logging
        ret &= data_logging_continuous.add_field(&offboard_tag_search.tag_location()[0], "tag_x", 3);
        ret &= data_logging_continuous.add_field(&offboard_tag_search.tag_location()[1], "tag_y", 3);
        ret &= data_logging_continuous.add_field(&offboard_tag_search.is_camera_running(), "camera_on");
        ret &= data_logging_continuous.add_field(&offboard_tag_search.picture_count(), "pic_count");

        /* Land on tag handler */
        ret &= mission_handler_registry.register_mission_handler(waypoint_handler);

        return ret;
    }

    /*
     * \brief   Initializes LEQuad
     * \details  Calls all init functions (init_*());
     *
     * \return  success
     */
    virtual bool init(void)
    {
        bool success = true;
        success &= LEQuad::init();
        success &= init_tag();
        return success;
    }

private:
    Serial& raspi_serial_mavlink;                                                                         ///< Reference to raspberry pi telemetry serial
    Mavlink_communication_T<10, 10, 10, 10> raspi_mavlink_communication;                                                    ///< The communication between the MAVRIC board and the offboard computer
    Offboard_Tag_Search offboard_tag_search;                                                              ///< The offboard camera control class
    Mission_handler_land_on_tag<Navigation_controller_I, Navigation_controller_I, XYposition_Zvel_controller_I> waypoint_handler;  ///< The land on tag mission handler
    
};

LEQuad::conf_t LEQuad_dronedome::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf = LEQuad::default_config(sysid);

    return conf;
}

#endif /* LEQUAD_TAG_HPP_ */
