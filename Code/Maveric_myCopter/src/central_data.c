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
		imu_init(&centralData.imu1);
		qfilter_init(&(centralData.imu1.attitude), (centralData.imu1.raw_scale), (centralData.imu1.raw_bias));
		
		position_estimation_init(&centralData.position_estimator, &centralData.pressure, &centralData.GPS_data);
	
		qfilter_init_quaternion(&centralData.imu1.attitude);
		
		init_nav();
		init_waypoint_handler();
		
		neighbors_selection_init();
		init_orca();

		// init stabilisers
		init_stabilisation_copter(&centralData.stabiliser_stack);

		// init simulation (should be done after position_estimator)
		simulation_init(&(centralData.sim_model),&(centralData.imu1),centralData.position_estimator.localPosition);		
		

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


		//centralData.sim_model.localPosition = centralData.position_estimator.localPosition;

		// Init sonar
		// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* get_central_data(void)
{
	return &centralData;
}