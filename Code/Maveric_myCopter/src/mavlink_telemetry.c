/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 * 
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file mavlink_telemetry.c
 *
 * Definition of the messages sent by the autopilot to the ground station
 */ 


#include "mavlink_telemetry.h"
#include "central_data.h"
#include "onboard_parameters.h"
#include "mavlink_stream.h"
#include "scheduler.h"
#include "radar_module_driver.h"
#include "remote_controller.h"
#include "analog_monitor.h"
#include "tasks.h"
#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"
#include "analog_monitor.h"
#include "state.h"
#include "position_estimation.h"
#include "hud.h"
#include "ahrs.h"
#include "remote.h"

central_data_t *central_data;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Add all onboard parameters to the parameter list
 *
 * \param	The pointer to the onboard parameters structure
 */
void mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters);

//void mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters)
{

	stabiliser_t* rate_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;
	stabiliser_t* attitude_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.attitude_stabiliser;
	stabiliser_t* velocity_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.velocity_stabiliser;
	
	// System ID	
	onboard_parameters_add_parameter_int32    ( onboard_parameters , (int32_t*)&central_data->mavlink_communication.mavlink_stream.sysid              , "ID_SYSID"         );

	// Simulation mode
	onboard_parameters_add_parameter_int32    ( onboard_parameters , ( int32_t*)&central_data->state.simulation_mode              , "Sim_mode"         );
	
	// Test attitude controller gains
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[ROLL]  , "gainA_Roll"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[PITCH] , "gainA_Pitch"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[YAW]   , "gainA_Yaw"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[ROLL]   , "gainR_Roll"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[PITCH]  , "gainR_Pitch"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[YAW]    , "gainR_Yaw"     );

	// Roll rate PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].p_gain                         , "RollRPid_P_G"     );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.maths_clip          , "RollRPid_I_CLip"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.postgain            , "RollRPid_I_PstG"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.pregain             , "RollRPid_I_PreG"  );
	// onboard_parameters_add_parameter_float ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.maths_clip      , "RollRPid_D_Clip"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.gain            , "RollRPid_D_Gain"  );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.LPF             , "RollRPid_D_LPF"   );
	
	// Roll attitude PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].p_gain                     , "RollAPid_P_G"     );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.maths_clip      , "RollAPid_I_CLip"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.postgain        , "RollAPid_I_PstG"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.pregain         , "RollAPid_I_PreG"  );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.maths_clip  , "RollAPid_D_Clip"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.gain        , "RollAPid_D_Gain"  );
	// onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF         , "RollAPid_D_LPF"   );

	// Pitch rate PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].p_gain                        , "PitchRPid_P_G"    );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.maths_clip         , "PitchRPid_I_CLip" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.postgain           , "PitchRPid_I_PstG" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.pregain            , "PitchRPid_I_PreG" );
	// onboard_parameters_add_parameter_float ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.maths_clip     , "PitchRPid_D_Clip" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.gain           , "PitchRPid_D_Gain" );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.LPF            , "PitchRPid_D_LPF"  );
	
	// Pitch attitude PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].p_gain                    , "PitchAPid_P_G"    );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.maths_clip     , "PitchAPid_I_CLip" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.postgain       , "PitchAPid_I_PstG" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.pregain        , "PitchAPid_I_PreG" );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.maths_clip , "PitchAPid_D_Clip" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.gain       , "PitchAPid_D_Gain" );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.LPF        , "PitchAPid_D_LPF"  );

	// Yaw rate PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].p_gain                          , "YawRPid_P_G"      );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].clip_max                        , "YawRPid_P_CLmx"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].clip_min                        , "YawRPid_P_CLmn"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.maths_clip           , "YawRPid_I_CLip"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.postgain             , "YawRPid_I_PstG"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.pregain              , "YawRPid_I_PreG"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.maths_clip       , "YawRPid_D_Clip"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.gain             , "YawRPid_D_Gain"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.LPF              , "YawRPid_D_LPF"    );
	
	// Yaw attitude PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].p_gain                      , "YawAPid_P_G"      );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].clip_max                    , "YawAPid_P_CLmx"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].clip_min                    , "YawAPid_P_CLmn"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.maths_clip       , "YawAPid_I_CLip"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.postgain         , "YawAPid_I_PstG"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.pregain          , "YawAPid_I_PreG"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.maths_clip   , "YawAPid_D_Clip"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.LPF          , "YawAPid_D_LPF"    );


	// Roll velocity PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].p_gain                     , "RollVPid_P_G"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].integrator.postgain        , "RollVPid_I_PstG"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].integrator.pregain         , "RollVPid_I_PreG"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].differentiator.gain        , "RollVPid_D_Gain"  );

	// Pitch velocity PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].p_gain                    , "PitchVPid_P_G"    );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].integrator.postgain       , "PitchVPid_I_PstG" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].integrator.pregain        , "PitchVPid_I_PreG" );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].differentiator.gain       , "PitchVPid_D_Gain" );

	// Thrust velocity PID
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.p_gain                        , "ThrVPid_P_G"      );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.postgain           , "ThrVPid_I_PstG"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.pregain            , "ThrVPid_I_PreG"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.gain           , "ThrVPid_D_Gain"   );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.LPF            , "ThrVPid_D_LPF"    );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.soft_zone_width               , "ThrVPid_soft"     );



	// qfilter
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp                                        , "QF_kp_acc"        );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp_mag                                    , "QF_kp_mag"        );
	// onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	
	// Biaises
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[X]									  , "Bias_Gyro_X"      );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Y]									  , "Bias_Gyro_Y"      );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Z]									  , "Bias_Gyro_Z"      );
	
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[X]								  , "Bias_Acc_X"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Y]								  , "Bias_Acc_Y"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Z]								  , "Bias_Acc_Z"       );
	
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[X]								  , "Bias_Mag_X"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Y]								  , "Bias_Mag_Y"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Z]								  , "Bias_Mag_Z"       );
	
	// Scale factor
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[X]							  , "Scale_Gyro_X"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Y]							  , "Scale_Gyro_Y"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Z]							  , "Scale_Gyro_Z"     );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[X]                       , "Scale_Acc_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Y]                       , "Scale_Acc_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Z]                       , "Scale_Acc_Z"      );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[X]                        , "Scale_Mag_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Y]                        , "Scale_Mag_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Z]                        , "Scale_Mag_Z"      );

	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimator.kp_alt_baro                              , "Pos_kp_alt_baro"       );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimator.kp_vel_baro                              , "Pos_kp_velb"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimator.kp_pos_gps[0]                            , "Pos_kp_pos0"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimator.kp_pos_gps[1]                            , "Pos_kp_pos1"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimator.kp_pos_gps[2]                            , "Pos_kp_pos2"      );
	


	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.dist2vel_gain                            , "vel_dist2Vel"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.cruise_speed                            , "vel_cruiseSpeed"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.max_climb_rate                          , "vel_climbRate"    );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.soft_zone_size							  , "vel_softZone"     );


//	onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->data_logging.log_data, "Log_continue");

}

//void mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging)
//{
//	// if _USE_LFN == 0: Name: max 8 characters + 3 for extension; if _USE_LFN != 0: Name: max 255 characters + more flexible extension type
//	data_logging_create_new_log_file(data_logging, "NewFile");
//	
//	// Add your logging parameters here, name length max = MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
//	// Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)
//	
//	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[X], "acc_x");
//	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Y], "acc_y");
//	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Z], "acc_z");
//	
//	data_logging_add_parameter_double(data_logging, &central_data->gps.latitude, "latitude");
//	data_logging_add_parameter_double(data_logging, &central_data->gps.longitude, "longitude");
//	data_logging_add_parameter_float(data_logging, &central_data->gps.altitude, "altitude");
//	
//	// data_logging_add_parameter_int8(data_logging, &central_data->state_machine.rc_check, "rc_check");
//	//data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state_machine.rc_check, "rc_check");
//	
//	//data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state.mav_state, "mav_state");
//	data_logging_add_parameter_uint8(data_logging, &central_data->state.mav_mode.byte, "mav_mode");
//	
//};

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init(void)
{
	central_data = central_data_get_pointer_to_struct();
	
	mavlink_telemetry_add_onboard_parameters(&central_data->mavlink_communication.onboard_parameters);

//	mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging);

	scheduler_t* mavlink_scheduler = &central_data->mavlink_communication.scheduler; 

	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_heartbeat,								&central_data->state, 								MAVLINK_MSG_ID_HEARTBEAT	);							// ID 0
	scheduler_add_task(mavlink_scheduler,  1000000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_status,									&central_data->state,								MAVLINK_MSG_ID_SYS_STATUS	);							// ID 1
	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&gps_ublox_send_raw,								&central_data->gps,									MAVLINK_MSG_ID_GPS_RAW_INT	);							// ID 24
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_scaled,									&central_data->imu, 								MAVLINK_MSG_ID_SCALED_IMU	);							// ID 26
	scheduler_add_task(mavlink_scheduler,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_raw,										&central_data->imu, 								MAVLINK_MSG_ID_RAW_IMU	);								// ID 27
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&bmp085_send_pressure,								&central_data->pressure,							MAVLINK_MSG_ID_SCALED_PRESSURE	);						// ID 29
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&ahrs_send_attitude,								&central_data->ahrs,				 				MAVLINK_MSG_ID_ATTITUDE	);								// ID 30
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&ahrs_send_attitude_quaternion,						&central_data->ahrs,				 				MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);					// ID 31
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&position_estimation_send_position,					&central_data->position_estimator, 					MAVLINK_MSG_ID_LOCAL_POSITION_NED	);					// ID 32
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&position_estimation_send_global_position,			&central_data->position_estimator, 					MAVLINK_MSG_ID_GLOBAL_POSITION_INT	);					// ID 33
	
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_send_scaled,								&central_data->remote,								MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);					// ID 34
	// scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_controller_send_scaled_rc_channels,			&central_data->controls,								MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);					// ID 34

	scheduler_add_task(mavlink_scheduler,  250000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_send_raw,									&central_data->remote,								MAVLINK_MSG_ID_RC_CHANNELS_RAW	);						// ID 35
	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&servos_mavlink_send,								&central_data->servos, 								MAVLINK_MSG_ID_SERVO_OUTPUT_RAW	);						// ID 36
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&stabilisation_send_rpy_thrust_setpoint,			&central_data->controls, 							MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT	);		// ID 58
//	scheduler_add_task(mavlink_scheduler,  200000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&stabilisation_send_rpy_speed_thrust_setpoint,		&central_data->stabiliser_stack.rate_stabiliser,	MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);	// ID 59
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&hud_send_message,									&central_data->hud_structure, 						MAVLINK_MSG_ID_VFR_HUD	);								// ID 74
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&simulation_send_state,								&central_data->sim_model, 							MAVLINK_MSG_ID_HIL_STATE	);							// ID 90
	scheduler_add_task(mavlink_scheduler,  500000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&simulation_send_quaternions,						&central_data->sim_model,							MAVLINK_MSG_ID_HIL_STATE_QUATERNION	);					// ID 115
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&scheduler_send_rt_stats,							&central_data->scheduler, 							MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251
	// scheduler_add_task(mavlink_scheduler,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&mavlink_telemetry_send_sonar,						&central_data->i2cxl_sonar, 						MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251

	scheduler_sort_tasks(mavlink_scheduler);
	
	print_util_dbg_print("MAVlink telemetry initialiased\n");
}