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

#define MAX_CLIMB_RATE 2.5

void init_nav();
void set_waypoint_from_frame(waypoint_struct current_wp);

void run_navigation();

void set_speed_command(double rel_pos[]);

void low_speed_nav(double dir_desired_bf[], Quat_Attitude_t attitude);
void high_speed_nav(double dir_desired_bf[], Quat_Attitude_t attitude);

void altitude_nav(double dir_desired_bf_z);
void heave_velocity_control(double dir_desired_bf_z);

float set_roll(double direction_bf_y, double vel_bf_y);
float set_pitch(double direction_bf_x, double vel_bf_x);
float set_yaw(double value_x, double value_y);

float min_max_bound(double value, double min, double max);

#endif NAVIGATION_H_