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

void set_speed_command(float rel_pos[], float dist2wpSqr);

void low_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude, float rel_distance);
void high_speed_nav(float dir_desired_bf[], Quat_Attitude_t attitude);

void altitude_nav(float dir_desired_bf_z);
void heave_velocity_control(float dir_desired_bf_z, Quat_Attitude_t attitude);

float set_roll(float direction_bf_y, float vel_bf_y);
float set_pitch(float direction_bf_x, float vel_bf_x);
float set_yaw(float value_x, float value_y);

float min_max_bound(float value, float min, float max);

#endif NAVIGATION_H_