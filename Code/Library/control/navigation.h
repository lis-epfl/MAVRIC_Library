/*
 * navigation.h
 *
 * Created: 13.08.2013 11:56:46
 *  Author: ndousse
 */ 


#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "qfilter.h"
#include "waypoint_navigation.h"

void init_nav();
void set_waypoint_from_frame(waypoint_struct current_wp);

void run_navigation();

void wp_hold_init();
void set_rel_pos_n_dist2wp(float rel_pos[], float *dist2wp_sqr);

void set_speed_command(float rel_pos[], float dist2wpSqr);

void low_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude, float rel_distance);
void high_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude);

#endif NAVIGATION_H_