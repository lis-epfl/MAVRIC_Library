/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file imu.c
 *
 * This file implements the code to read the IMU data
 */


#include "imu.h"

#include "delay.h"
#include "time_keeper.h"
#include "print_util.h"
#include "mavlink_stream.h"
#include "tasks.h"

int32_t ic;

void imu_raw2oriented(Imu_Data_t *imu);

/**
 * \brief	Computes the transition from raw values to scaled values
 *
 * \param	attitude	the pointer structure of the attitude
 * \param	rates		the array of angular rates (IMU), accelerations and magnetometer
 */
void imu_oriented2scale(Imu_Data_t *imu);

void imu_init (Imu_Data_t *imu, AHRS_t *attitude_estimation)
{	
	//imu_calibrate_Gyros(imu);
	
	//init gyro
	imu->calib_gyro.scale_factor[X] =  1.0f / RAW_GYRO_X_SCALE;
	imu->calib_gyro.scale_factor[Y] =  1.0f / RAW_GYRO_Y_SCALE;
	imu->calib_gyro.scale_factor[Z] =  1.0f / RAW_GYRO_Z_SCALE;
	imu->calib_gyro.bias[X] = 0.0f;
	imu->calib_gyro.bias[Y] = 0.0f;
	imu->calib_gyro.bias[Z] = 0.0f;
	imu->calib_gyro.orientation[X] = GYRO_AXIS_X;
	imu->calib_gyro.orientation[Y] = GYRO_AXIS_Y;
	imu->calib_gyro.orientation[Z] = GYRO_AXIS_Z;
	
	//init accelero
	imu->calib_accelero.scale_factor[X] =  1.0f / RAW_ACC_X_SCALE;
	imu->calib_accelero.scale_factor[Y] =  1.0f / RAW_ACC_Y_SCALE;
	imu->calib_accelero.scale_factor[Z] =  1.0f / RAW_ACC_Z_SCALE;
	imu->calib_accelero.bias[X] = ACC_BIAIS_X;
	imu->calib_accelero.bias[Y] = ACC_BIAIS_Y;
	imu->calib_accelero.bias[Z] = ACC_BIAIS_Z;
	imu->calib_accelero.orientation[X] = ACC_AXIS_X;
	imu->calib_accelero.orientation[Y] = ACC_AXIS_Y;
	imu->calib_accelero.orientation[Z] = ACC_AXIS_Z;
	
	//init compass
	imu->calib_compass.scale_factor[X] =  1.0f / RAW_MAG_X_SCALE;
	imu->calib_compass.scale_factor[Y] =  1.0f / RAW_MAG_Y_SCALE;
	imu->calib_compass.scale_factor[Z] =  1.0f / RAW_MAG_Z_SCALE;
	imu->calib_compass.bias[X] = MAG_BIAIS_X;
	imu->calib_compass.bias[Y] = MAG_BIAIS_Y;
	imu->calib_compass.bias[Z] = MAG_BIAIS_Z;
	imu->calib_compass.orientation[X] = MAG_AXIS_X;
	imu->calib_compass.orientation[Y] = MAG_AXIS_Y;
	imu->calib_compass.orientation[Z] = MAG_AXIS_Z;
		
	imu_last_update_init = false;
	
	//init AHRS_t attitude_estimation
	attitude_estimation->qe.s		= 1.0f;
	attitude_estimation->qe.v[X]	= 0.0f;
	attitude_estimation->qe.v[Y]	= 0.0f;
	attitude_estimation->qe.v[Z]	= 0.0f;
	
	attitude_estimation->last_update = 0.0f;
	attitude_estimation->dt = 0.0f;
	
	attitude_estimation->angular_speed[X] = 0.0f;
	attitude_estimation->angular_speed[Y] = 0.0f;
	attitude_estimation->angular_speed[Z] = 0.0f;
	attitude_estimation->linear_acc[X] = 0.0f;
	attitude_estimation->linear_acc[Y] = 0.0f;
	attitude_estimation->linear_acc[Z] = 0.0f;
}

void imu_calibrate_gyros(Imu_Data_t *imu)
{
	int32_t i,j;
	//imu_get_raw_data(imu);
	tasks_run_imu_update(0);
	
	for (j = 0; j < 3; j++)
	{
		imu->calib_gyro.bias[j] = (float)imu->oriented_gyro.data[j];
	}
	
	for (i = 0; i < 100; i++)
	{
		//imu_get_raw_data(imu);
		tasks_run_imu_update(0);

		//imu->imu->calib_sensor.bias[0 + ACC_OFFSET] = (0.9f * imu->imu->calib_accelero.bias[0] + 0.1f * (float)imu->oriented_accelero.data[0]);
		//imu->imu->calib_sensor.bias[1 + ACC_OFFSET] = (0.9f * imu->imu->calib_accelero.bias[1] + 0.1f * (float)imu->oriented_accelero.data[1]);
		//imu->imu->calib_sensor.bias[2 + ACC_OFFSET] = (0.9f * imu->imu->calib_accelero.bias[2] + 0.1f * ((float)imu->oriented_accelero.data[2] - imu->calib_accelero.scale_factor[2]));
		for (j = 0; j < 3; j++)
		{
			imu->calib_gyro.bias[j] = (0.9f * imu->calib_gyro.bias[j] + 0.1f * (float)imu->oriented_gyro.data[j]);
			//imu->attitude.raw_mag_mean[j] = (1.0 - fMAG_LPF) * imu->attitude.raw_mag_mean[j] + MAG_LPF * ((float)imu->oriented_compass.data[j]);
		}
	
		delay_ms(4);
	}
}

void imu_update(Imu_Data_t *imu)
{
	uint32_t t = time_keeper_get_time_ticks();
	
	if (!imu_last_update_init)
	{
		imu->last_update = t;
		imu_last_update_init = true;
	}
	else
	{
		imu->dt = time_keeper_ticks_to_seconds(t - imu->last_update);
		imu->last_update = t;
		imu_raw2oriented(imu);
		imu_oriented2scale(imu);
	}
}

void imu_raw2oriented(Imu_Data_t *imu)
{
	for (uint16_t i=0; i<3; i++)
	{
		imu->oriented_gyro.data[i]		= imu->raw_gyro.data[i] * imu->calib_gyro.orientation[i];
		imu->oriented_accelero.data[i] = imu->raw_accelero.data[i] * imu->calib_accelero.orientation[i];
		imu->oriented_compass.data[i]	= imu->raw_compass.data[i] * imu->calib_compass.orientation[i];
	}
}

void imu_oriented2scale(Imu_Data_t *imu)
{
	for (int16_t i = 0; i < 3; i++)
	{
		imu->scaled_gyro.data[i]  = (1.0f - GYRO_LPF) * imu->scaled_gyro.data[i] + GYRO_LPF * (((float)imu->oriented_gyro.data[i] - imu->calib_gyro.bias[i]) * imu->calib_gyro.scale_factor[i]);
		imu->scaled_accelero.data[i]   = (1.0f - ACC_LPF) * imu->scaled_accelero.data[i] + ACC_LPF * (((float)imu->oriented_accelero.data[i] - imu->calib_accelero.bias[i]) * imu->calib_accelero.scale_factor[i]);
		imu->scaled_compass.data[i] = (1.0f - MAG_LPF) * imu->scaled_compass.data[i] + MAG_LPF * (((float)imu->oriented_compass.data[i] - imu->calib_compass.bias[i]) * imu->calib_compass.scale_factor[i]);
	}
}

task_return_t mavlink_telemetry_send_scaled_imu(Imu_Data_t* imu)
{
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0,
								time_keeper_get_millis(),
								1000 * imu->scaled_accelero.data[IMU_X],
								1000 * imu->scaled_accelero.data[IMU_Y],
								1000 * imu->scaled_accelero.data[IMU_Z],
								1000 * imu->scaled_gyro.data[IMU_X],
								1000 * imu->scaled_gyro.data[IMU_Y],
								1000 * imu->scaled_gyro.data[IMU_Z],
								1000 * imu->scaled_compass.data[IMU_X],
								1000 * imu->scaled_compass.data[IMU_Y],
								1000 * imu->scaled_compass.data[IMU_Z]);
	
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_raw_imu(Imu_Data_t* imu)
{
	mavlink_msg_raw_imu_send(	MAVLINK_COMM_0,
								time_keeper_get_micros(),
								imu->oriented_accelero.data[IMU_X],
								imu->oriented_accelero.data[IMU_Y],
								imu->oriented_accelero.data[IMU_Z],
								imu->oriented_gyro.data[IMU_X],
								imu->oriented_gyro.data[IMU_Y],
								imu->oriented_gyro.data[IMU_Z],
								imu->oriented_compass.data[IMU_X],
								imu->oriented_compass.data[IMU_Y],
								imu->oriented_compass.data[IMU_Z]);
	
	return TASK_RUN_SUCCESS;
}