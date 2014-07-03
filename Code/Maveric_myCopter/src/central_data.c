/* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file central_data.c
 *
 *  Place where the central data is stored and initialized
 */


#include "central_data.h"
#include "remote_controller.h"
#include "simulation.h"
#include "conf_sim_model.h"


static volatile central_data_t centralData;

void initialise_central_data(){
		// Init servos
		for (int i = 0; i < NUMBER_OF_SERVO_OUTPUTS; ++i)
		{
			centralData.servos[i].value = -600;
			centralData.servos[i].min = -600;
			centralData.servos[i].max = 600;
			centralData.servos[i].failsafe_position = -600;
		}
		
		servos_failsafe(centralData.servos);
		set_servos(centralData.servos);

		// TODO change names! XXX_init()
		init_imu(&centralData.imu1);
		qfInit(&(centralData.imu1.attitude), (centralData.imu1.raw_scale), (centralData.imu1.raw_bias));
		
		init_pos_integration(&centralData.position_estimator, &centralData.pressure, &centralData.GPS_data);
	
		initQuat(&centralData.imu1.attitude);
		
		init_nav();
		init_waypoint_handler();
		//e_init();
		
		init_neighbors();
		init_orca();

		// init stabilisers
		init_stabilisation_copter(&centralData.stabiliser_stack);

		// init simulation (should be done after position_estimator)
		init_simulation(&(centralData.sim_model),&(centralData.imu1),centralData.position_estimator.localPosition);		

		// init controls
		// TODO: move to a module
		centralData.controls.rpy[ROLL]=0;
		centralData.controls.rpy[PITCH]=0;
		centralData.controls.rpy[YAW]=0;
		centralData.controls.tvel[X]=0;
		centralData.controls.tvel[Y]=0;
		centralData.controls.tvel[Z]=0;
		centralData.controls.thrust=-1.0;
		
		centralData.run_mode = MOTORS_OFF;
		

		centralData.simulation_mode=0;
		centralData.simulation_mode_previous = 0;

		// init waypoint navigation
		// TODO: move to 
		// waypoint_handler_init(&centralData.waypoint_handler);
		// centralData.waypoint_handler.number_of_waypoints= ...
		centralData.number_of_waypoints = 0;		
		centralData.waypoint_set = false;
		centralData.waypoint_sending = false;
		centralData.waypoint_receiving = false;
		
		centralData.critical_landing = false;
		
		centralData.collision_avoidance = false;
		centralData.automatic_take_off = false;
		centralData.automatic_landing = false;
		centralData.in_the_air = false;
		
		centralData.number_of_neighbors = 0;
		
		// default GPS home position
		// TODO: move to position_estimator_init();
		centralData.position_estimator.localPosition.origin.longitude=   HOME_LONGITUDE;
		centralData.position_estimator.localPosition.origin.latitude =   HOME_LATITUDE;
		centralData.position_estimator.localPosition.origin.altitude =   HOME_ALTITUDE;
		centralData.position_estimator.localPosition.pos[X]=0;
		centralData.position_estimator.localPosition.pos[Y]=0;
		centralData.position_estimator.localPosition.pos[Z]=0;


		//centralData.sim_model.localPosition = centralData.position_estimator.localPosition;

		// TODO: move to navigation_init()
		centralData.dist2vel_gain = 0.7;
		centralData.cruise_speed = 3.0;
		centralData.max_climb_rate = 1.0;
		centralData.softZoneSize = 0.0;

		// Init sonar
		// i2cxl_sonar_init(&centralData.i2cxl_sonar);

		// Init pitot tube
		// airspeed_analog_init(&centralData.pitot, &centralData.adc, ANALOG_RAIL_12);
}

central_data_t* get_central_data(void)
{
	return &centralData;
}