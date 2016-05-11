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
 * \file mavlink_telemetry.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Definition of the messages sent by the autopilot to the ground station
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/mavlink_telemetry.hpp"
#include "sample_projects/LEQuad/central_data.hpp"

#include "communication/data_logging_telemetry.hpp"
#include "communication/hud_telemetry.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/onboard_parameters.hpp"
#include "communication/remote_telemetry.hpp"
#include "communication/state.hpp"
#include "communication/state_telemetry.hpp"

#include "control/stabilisation_telemetry.hpp"
#include "control/joystick_telemetry.hpp"
#include "control/manual_control_telemetry.hpp"

#include "sensing/ahrs_telemetry.hpp"
#include "sensing/imu_telemetry.hpp"
#include "sensing/position_estimation.hpp"
#include "sensing/position_estimation_telemetry.hpp"

#include "drivers/barometer_telemetry.hpp"
#include "drivers/gps_telemetry.hpp"

#include "drivers/servos_telemetry.hpp"
#include "drivers/sonar_i2cxl.hpp"
#include "drivers/sonar_telemetry.hpp"

#include "runtime/scheduler.hpp"
#include "runtime/scheduler_telemetry.hpp"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Add onboard logging parameters
 *
 * \param   data_logging            The pointer to the data logging structure
 *
 * \return  The initialization status of the module, succeed == true
 */
bool mavlink_telemetry_add_data_logging_parameters(Data_logging* data_logging, Central_data* central_data);

/**
 * \brief   Add onboard logging parameters
 *
 * \param   data_logging            The pointer to the data logging structure
 *
 * \return  The initialization status of the module, succeed == true
 */
bool mavlink_telemetry_add_data_logging_parameters_stat(Data_logging* data_logging, Central_data* central_data);

/**
 * \brief   Initialise the callback functions
 *
 * \param   p_central_data_            The pointer to the p_central_data structure
 *
 * \return  The initialization status of the module, succeed == true
 */
bool mavlink_telemetry_init_communication_module(Central_data* central_data);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool mavlink_telemetry_add_data_logging_parameters(Data_logging* data_logging, Central_data* central_data)
{
    bool init_success = true;

    // Add your logging parameters here, name length max < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
    // Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)

    //init_success &= data_logging->add_field(&central_data->imu.scaled_accelero.data[X], "acc_x", 4);
    //init_success &= data_logging->add_field(&central_data->imu.scaled_accelero.data[Y], "acc_y", 4);
    //init_success &= data_logging->add_field(&central_data->imu.scaled_accelero.data[Z], "acc_z", 4);

    // init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.latitude,   "origin_lat", 7);
    // init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.longitude, "origin_lon", 7);
    // init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.altitude,   "origin_alt", 3);

    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.pos[0], "local_x", 3);
    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.pos[1], "local_y", 3);
    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.pos[2], "local_z", 3);

    // init_success &= data_logging->add_field(&central_data->gps.latitude, "latitude", 7);
    // init_success &= data_logging->add_field(&central_data->gps.longitude, "longitude", 7);
    // init_success &= data_logging->add_field(&central_data->gps.altitude, "altitude", 3);

    init_success &= data_logging->add_field((uint32_t*)&central_data->state.mav_state_, "mav_state_");
    init_success &= data_logging->add_field(&central_data->state.mav_mode_, "mav_mode");

    return init_success;
};

bool mavlink_telemetry_add_data_logging_parameters_stat(Data_logging* data_logging, Central_data* central_data)
{
    // Add your logging parameters here, name length max < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
    // Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)

    bool init_success = true;

    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.latitude,  "origin_lat", 7);
    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.longitude, "origin_lon", 7);
    init_success &= data_logging->add_field(&central_data->position_estimation.local_position.origin.altitude,  "origin_alt", 3);

    return init_success;
}

bool mavlink_telemetry_init_communication_module(Central_data* central_data)
{
    bool init_success = true;
    Mavlink_message_handler* message_handler = &central_data->mavlink_communication.message_handler();
    init_success &= state_telemetry_init(&central_data->state,
                                         message_handler);

    init_success &= imu_telemetry_init(&central_data->imu,
                                       message_handler);

    init_success &= remote_telemetry_init(&central_data->manual_control.remote,
                                          message_handler);

    init_success &= joystick_telemetry_init(&central_data->manual_control.joystick,
                                            message_handler);

    init_success &= manual_control_telemetry_init(&central_data->manual_control,
                                                  message_handler);

    init_success &= position_estimation_telemetry_init(&central_data->position_estimation,
                                                       message_handler);

    init_success &= gps_telemetry_init(&central_data->gps,
                                       message_handler);

    init_success &= data_logging_telemetry_init(&central_data->data_logging,
                                                message_handler);

    init_success &= data_logging_telemetry_init(&central_data->data_logging2,
                                                message_handler);

    return init_success;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool mavlink_telemetry_add_onboard_parameters(Onboard_parameters* onboard_parameters, Central_data* central_data)
{
    bool init_success = true;

    stabiliser_t* rate_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;
    // stabiliser_t* attitude_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.attitude_stabiliser;
    stabiliser_t* velocity_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.velocity_stabiliser;
    // stabiliser_t* position_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.position_stabiliser;

    // System ID
    init_success &= onboard_parameters->add_parameter_uint32(&central_data->mavlink_communication.mavlink_stream().sysid_, "ID_SYSID");

    // Test attitude controller gains
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_angle[ROLL],    "GAIN_A_ROLL"        );
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_angle[PITCH],   "GAIN_A_PITCH"       );
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_angle[YAW],     "GAIN_A_YAW"         );
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_rate[ROLL],     "GAIN_R_ROLL"        );
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_rate[PITCH],    "GAIN_R_PITCH"       );
    //init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_controller.p_gain_rate[YAW],      "GAIN_R_YAW"         );

    // Roll rate
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain,              "ROLL_R_KP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.clip,         "ROLL_R_I_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.gain,         "ROLL_R_KI");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.clip,     "ROLL_R_D_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain,     "ROLL_R_KD");

    // Roll attitude
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain,                "ROLL_A_KP"       );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.clip,       "ROLL_A_I_CLIP"   );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.gain,       "ROLL_A_KI"       );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.clip,   "ROLL_A_D_CLIP"   );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain,   "ROLL_A_KD"       );

    // Pitch rate
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain,              "PITCH_R_KP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.clip,         "PITCH_R_I_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.gain,         "PITCH_R_KI");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.clip,     "PITCH_R_D_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain, "PITCH_R_KD");

    // Pitch attitude
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain,               "PITCH_A_KP"      );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.clip,      "PITCH_A_I_CLIP"  );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.gain,      "PITCH_A_KI"      );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.clip,  "PITCH_A_D_CLIP"  );
    //init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.gain,  "PITCH_A_KD"      );

    // Yaw rate
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].p_gain,                "YAW_R_KP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_max,          "YAW_R_P_CLMX");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_min,          "YAW_R_P_CLMN");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.clip,       "YAW_R_I_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.gain,       "YAW_R_KI");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.clip,   "YAW_R_D_CLIP");
    init_success &= onboard_parameters->add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.gain,   "YAW_R_KD");

    // Yaw attitude
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].p_gain,                 "YAW_A_KP"        );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_max,               "YAW_A_P_CLMX"    );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_min,               "YAW_A_P_CLMN"    );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.clip,        "YAW_A_I_CLIP"    );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.gain,        "YAW_A_KI"        );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.clip,    "YAW_A_D_CLIP"    );
    // init_success &= onboard_parameters->add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain,    "YAW_A_KD"        );


    // Roll velocity
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].p_gain,                "ROLL_V_KP"       );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.clip_pre,   "ROLL_V_I_CLPRE"  );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.gain,       "ROLL_V_KI"       );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.clip,       "ROLL_V_I_CLIP"   );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].differentiator.gain,   "ROLL_V_KD"       );


    // Pitch velocity
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].p_gain,               "PITCH_V_KP"      );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.clip_pre,  "PITCH_V_I_CLPRE" );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.gain,      "PITCH_V_KI"      );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.clip,      "PITCH_V_I_CLIP"  );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].differentiator.gain,  "PITCH_V_KD"      );

    // Thrust velocity
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->thrust_controller.p_gain,                  "THRV_KP"    );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.clip_pre,     "THRV_I_PREG");
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.gain,     "THRV_KD"    );
    init_success &= onboard_parameters->add_parameter_float(&velocity_stabiliser->thrust_controller.soft_zone_width,         "THRV_SOFT"  );

    // Roll position
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[ROLL].p_gain,                "ROLL_P_KP"   );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[ROLL].integrator.gain,       "ROLL_P_KI"   );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[ROLL].differentiator.gain,   "ROLL_P_KD"   );

    // Pitch position
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[PITCH].p_gain,               "PITCH_P_KP"  );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[PITCH].integrator.gain,      "PITCH_P_KI"  );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->rpy_controller[PITCH].differentiator.gain,  "PITCH_P_KD"  );

    // Thrust position
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->thrust_controller.p_gain,               "THR_P_KP"    );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->thrust_controller.integrator.gain,      "THR_P_KI"    );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->thrust_controller.differentiator.gain,  "THR_P_KD"    );
    //init_success &= onboard_parameters->add_parameter_float(&position_stabiliser->thrust_controller.soft_zone_width,      "THR_P_SOFT"  );

    // qfilter
    init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_filter.kp                                        , "QF_KP_ACC");
    init_success &= onboard_parameters->add_parameter_float(&central_data->attitude_filter.kp_mag                                    , "QF_KP_MAG");

    // Biases
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->gyroscope.bias[X]                                    , "BIAS_GYRO_X");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->gyroscope.bias[Y]                                    , "BIAS_GYRO_Y");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->gyroscope.bias[Z]                                    , "BIAS_GYRO_Z");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->accelerometer.bias[X]                                , "BIAS_ACC_X");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->accelerometer.bias[Y]                                , "BIAS_ACC_Y");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->accelerometer.bias[Z]                                , "BIAS_ACC_Z");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetometer.bias[X]                                 , "BIAS_MAG_X");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetometer.bias[Y]                                 , "BIAS_MAG_Y");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetometer.bias[Z]                                 , "BIAS_MAG_Z");

    // North vector
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetic_north[X], "NORTH_MAG_X");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetic_north[Y], "NORTH_MAG_Y");
    init_success &= onboard_parameters->add_parameter_float(&central_data->imu.get_config()->magnetic_north[Z], "NORTH_MAG_Z");


    // Position estimation
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_alt_baro,   "POS_KP_ALT_BARO" );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_vel_baro,   "POS_KP_VELB"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_pos_gps[0], "POS_KP_POS0"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_pos_gps[1], "POS_KP_POS1"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_pos_gps[2], "POS_KP_POS2"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_vel_gps[0], "POS_KP_VEL0"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_vel_gps[1], "POS_KP_VEL1"     );
    init_success &= onboard_parameters->add_parameter_float(&central_data->position_estimation.kp_vel_gps[2], "POS_KP_VEL2"     );

    // Navigation
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.dist2vel_gain,                           "VEL_DIST2VEL"    );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.cruise_speed,                            "VEL_CRUISESPEED" );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.max_climb_rate,                          "VEL_CLIMBRATE"   );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.soft_zone_size,                          "VEL_SOFTZONE"    );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.hovering_controller.p_gain,              "VEL_HOVER_PGAIN" );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.hovering_controller.differentiator.gain, "VEL_HOVER_DGAIN" );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.wpt_nav_controller.p_gain,               "VEL_WPT_PGAIN"   );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.wpt_nav_controller.differentiator.gain,  "VEL_WPT_DGAIN"   );
    init_success &= onboard_parameters->add_parameter_float(&central_data->navigation.kp_yaw,  "VEL_YAW_KPGAIN"   );
    /* WARNING the following 2 cast are necessary on stm32 architecture, otherwise it leads to execution error */
    init_success &= onboard_parameters->add_parameter_int32((int32_t*) &central_data->manual_control.control_source_, "CTRL_CTRL_SRC");
    init_success &= onboard_parameters->add_parameter_int32((int32_t*) &central_data->manual_control.mode_source_,     "COM_RC_IN_MODE");


    return init_success;
}


void altitude_estimation_telemetry_send(const Altitude_estimation* altitude_estimation, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    Mat<3,1> x = altitude_estimation->x();
    Mat<3,3> P = altitude_estimation->P();
    float cov[45];
    cov[0] = P(0,0);
    cov[1] = P(1,1);
    cov[2] = P(2,2);

    mavlink_msg_local_position_ned_cov_pack(mavlink_stream->sysid(),
                                            mavlink_stream->compid(),
                                            msg,
                                            time_keeper_get_ms(),
                                            0,
                                            0,
                                            0.0f,
                                            0.0f,
                                            x(0,0),
                                            0.0f,
                                            0.0f,
                                            x(1,0),
                                            0.0f,
                                            0.0f,
                                            x(2,0),
                                            cov);

    // mavlink_msg_local_position_ned_cov_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
    //                            uint32_t time_boot_ms, uint64_t time_utc, uint8_t estimator_type,
    //                            float x, float y, float z,
    //                            float vx, float vy, float vz,
    //                            float ax, float ay, float az,
    //                            const float *covariance)
}


bool mavlink_telemetry_init(Central_data* central_data)
{
    bool init_success = true;

    init_success &= central_data->data_logging.create_new_log_file("Log_file",
                    true,
                    central_data->mavlink_communication.sysid());


    init_success &= central_data->data_logging2.create_new_log_file("Log_Stat",
                    false,
                    central_data->mavlink_communication.sysid());

    init_success &= mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging, central_data);

    init_success &= mavlink_telemetry_add_data_logging_parameters_stat(&central_data->data_logging2, central_data);

    init_success &= mavlink_telemetry_init_communication_module(central_data);

    Mavlink_communication& mavlink_communication = central_data->mavlink_communication;

    stabiliser_t* stabiliser_show = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;

    init_success &= mavlink_communication.add_msg_send(1000000,  Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&state_telemetry_send_heartbeat,                                &central_data->state,                   MAVLINK_MSG_ID_HEARTBEAT);   // ID 0
    init_success &= mavlink_communication.add_msg_send(1000000,  Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&state_telemetry_send_status,                                   &central_data->state,                   MAVLINK_MSG_ID_SYS_STATUS);   // ID 1
    init_success &= mavlink_communication.add_msg_send(1000000,  Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&gps_telemetry_send_raw,                                        &central_data->gps,                     MAVLINK_MSG_ID_GPS_RAW_INT);   // ID 24
    init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&imu_telemetry_send_scaled,                                     &central_data->imu,                     MAVLINK_MSG_ID_SCALED_IMU);   // ID 26
    // init_success &= mavlink_communication.add_msg_send(100000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&imu_telemetry_send_raw,                                     &central_data->imu,                     MAVLINK_MSG_ID_RAW_IMU              );// ID 27
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&barometer_telemetry_send,                                      &central_data->barometer,               MAVLINK_MSG_ID_SCALED_PRESSURE);  // ID 29
    init_success &= mavlink_communication.add_msg_send(200000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&ahrs_telemetry_send_attitude,                                  &central_data->ahrs,                    MAVLINK_MSG_ID_ATTITUDE);    // ID 30
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&ahrs_telemetry_send_attitude_quaternion,                       &central_data->ahrs,                    MAVLINK_MSG_ID_ATTITUDE_QUATERNION); // ID 31
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&position_estimation_telemetry_send_position,                   &central_data->position_estimation,     MAVLINK_MSG_ID_LOCAL_POSITION_NED); // ID 32
    init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&position_estimation_telemetry_send_global_position,            &central_data->position_estimation,     MAVLINK_MSG_ID_GLOBAL_POSITION_INT); // ID 33
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&remote_telemetry_send_scaled,                                  &central_data->manual_control.remote,   MAVLINK_MSG_ID_RC_CHANNELS_SCALED); // ID 34
    init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_REGULAR,    Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&remote_telemetry_send_raw,                                   &central_data->manual_control.remote,   MAVLINK_MSG_ID_RC_CHANNELS_RAW);  // ID 35
    init_success &= mavlink_communication.add_msg_send(1000000,  Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&servos_telemetry_mavlink_send,                                 &central_data->servos_telemetry,        MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);  // ID 36

    init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_NEVER,    Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&joystick_telemetry_send_manual_ctrl_msg,                       &central_data->manual_control.joystick, MAVLINK_MSG_ID_MANUAL_CONTROL);  // ID 69
    //init_success &= mavlink_communication.add_msg_send(200000,   Scheduler_task::RUN_REGULAR,    Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&stabilisation_telemetry_send_control,                      &central_data->controls,                MAVLINK_MSG_ID_MANUAL_CONTROL       );// ID 69
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&hud_telemetry_send_message,                                    &central_data->hud_structure,           MAVLINK_MSG_ID_VFR_HUD);    // ID 74
    // init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&simulation_telemetry_send_state,                                &central_data->sim_model,               MAVLINK_MSG_ID_HIL_STATE            );// ID 90
    // init_success &= mavlink_communication.add_msg_send(500000,    Scheduler_task::RUN_NEVER,    Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&simulation_telemetry_send_quaternions,                         &central_data->sim_model,               MAVLINK_MSG_ID_HIL_STATE_QUATERNION );// ID 115
    init_success &= mavlink_communication.add_msg_send(500000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&sonar_telemetry_send,                                          &central_data->sonar,                   MAVLINK_MSG_ID_DISTANCE_SENSOR); // ID 119

    init_success &= mavlink_communication.add_msg_send(200000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&stabilisation_telemetry_send_rpy_speed_thrust_setpoint,        stabiliser_show,                        MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT); // ID 160

    init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&scheduler_telemetry_send_rt_stats,                             &central_data->scheduler,               MAVLINK_MSG_ID_NAMED_VALUE_FLOAT); // ID 251
    //init_success &= mavlink_communication.add_msg_send(100000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&sonar_telemetry_send,                            &central_data->sonar_i2cxl.data,            MAVLINK_MSG_ID_DISTANCE_SENSOR  );// ID 132
    //init_success &= mavlink_communication.add_msg_send(250000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&acoustic_telemetry_send,                                     &central_data->audio_data,              MAVLINK_MSG_ID_DEBUG_VECT           );// ID 250

    init_success &= mavlink_communication.add_msg_send(100000,   Scheduler_task::RUN_REGULAR,  Scheduler_task::PERIODIC_ABSOLUTE, Scheduler_task::PRIORITY_NORMAL, (Mavlink_communication::send_msg_function_t)&altitude_estimation_telemetry_send,                            &central_data->altitude_estimation_,    MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV           );// ID 64

    init_success &= central_data->mavlink_communication.scheduler().sort_tasks();

    print_util_dbg_init_msg("[TELEMETRY]", init_success);

    return init_success;
}
