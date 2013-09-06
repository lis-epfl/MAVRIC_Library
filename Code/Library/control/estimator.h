/*
 * estimator.h
 *
 *  Created on: Mai 6, 2013
 *      Author: Philippe
 */

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#define POSITION	0
#define SPEED		1
#define BIAIS		2

bool filter_init_delta_t;
uint32_t prev_time;

typedef struct {
	double state[3][3]; // [z z_speed z_biais] in NED
	float delta_t_filter;
} Estimator_Data_t;

void e_init(void);
void init_pos_gps_estimator(void);
void e_kalman_init (int axis,float init_p);
void e_predict (UQuat_t *qe, float a[], float dt);
void e_kalman_predict (int axis,float accel_meas, float dt);
void e_kalman_predict_hf(int axis,float accel_meas, float dt);
void e_kalman_update_position (int axis, double position_meas);
void e_kalman_update_position_hf(int axis, double position_meas);
void e_kalman_update_speed(int axis,float speed_meas);
void e_kalman_update_speed_hf(int axis,float speed_meas);

void estimator_loop(void);
void quat_rot(UQuat_t *quat,float *vect);

#endif //ESTIMATOR_H_