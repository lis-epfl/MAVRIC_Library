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
 * \file mavlink_action.c
 *
 * Definition of the tasks executed on the autopilot
 */ 


#include "mavlink_actions.h"
#include "central_data.h"

#include "onboard_parameters.h"
#include "scheduler.h"
#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"

central_data_t *centralData;


void mavlink_actions_add_onboard_parameters(void) {
	onboard_parameters_t* onboard_parameters = &centralData->mavlink_communication.onboard_parameters;

	Stabiliser_t* rate_stabiliser = &centralData->stabiliser_stack.rate_stabiliser;
	Stabiliser_t* attitude_stabiliser = &centralData->stabiliser_stack.attitude_stabiliser;
	Stabiliser_t* velocity_stabiliser= &centralData->stabiliser_stack.velocity_stabiliser;
	
	// Simulation mode
	onboard_parameters_add_parameter_int32    ( onboard_parameters , ( int32_t*)&centralData->state_structure.simulation_mode                              , "Sim_mode"         );
	
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
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->attitude_filter.kp                                        , "QF_kp_acc"        );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->attitude_filter.kp_mag                                    , "QF_kp_mag"        );
	// onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	
	// Biaises
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_gyro.bias[X]                       , "Bias_Gyro_X"      );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_gyro.bias[Y]                       , "Bias_Gyro_Y"      );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_gyro.bias[Z]                       , "Bias_Gyro_Z"      );
	
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_accelero.bias[X]                        , "Bias_Acc_X"       );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_accelero.bias[Y]                        , "Bias_Acc_Y"       );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_accelero.bias[Z]                        , "Bias_Acc_Z"       );
	
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_compass.bias[X]                        , "Bias_Mag_X"       );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_compass.bias[Y]                        , "Bias_Mag_Y"       );
	 onboard_parameters_add_parameter_float ( onboard_parameters , &centralData->imu.calib_compass.bias[Z]                        , "Bias_Mag_Z"       );
	
	// Scale factor
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_gyro.scale_factor[X]                         , "Scale_Gyro_X"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_gyro.scale_factor[Y]                         , "Scale_Gyro_Y"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_gyro.scale_factor[Z]                         , "Scale_Gyro_Z"     );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_accelero.scale_factor[X]                          , "Scale_Acc_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_accelero.scale_factor[Y]                          , "Scale_Acc_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_accelero.scale_factor[Z]                          , "Scale_Acc_Z"      );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_compass.scale_factor[X]                          , "Scale_Mag_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_compass.scale_factor[Y]                          , "Scale_Mag_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->imu.calib_compass.scale_factor[Z]                          , "Scale_Mag_Z"      );

	//onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->position_estimator.kp_alt_baro                               , "Pos_kp_alt_baro"       );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->position_estimator.kp_vel_baro                          , "Pos_kp_velb"      );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->position_estimator.kp_pos_gps[0]                            , "Pos_kp_pos0"      );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->position_estimator.kp_pos_gps[1]                            , "Pos_kp_pos1"      );
	//onboard_parameters_add_parameter_float  ( onboard_parameters , &centralData->position_estimator.kp_pos_gps[2]                            , "Pos_kp_pos2"      );
	


	onboard_parameters_add_parameter_float    ( onboard_parameters , &centralData->navigationData.cruise_speed                                   , "vel_dist2Vel"     );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &centralData->navigationData.cruise_speed                                       , "vel_cruiseSpeed"  );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &centralData->navigationData.max_climb_rate                                          , "vel_climbRate"    );
	onboard_parameters_add_parameter_float    ( onboard_parameters , &centralData->navigationData.softZoneSize                                            , "vel_softZone"     );

}

void mavlink_actions_init(void) {
	
	centralData = central_data_get_pointer_to_struct();
	mavlink_actions_add_onboard_parameters();

	print_util_dbg_print("MAVlink actions initialiased\n");
}
