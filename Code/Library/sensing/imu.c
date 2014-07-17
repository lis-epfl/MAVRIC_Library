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
#include "lsm330dlc_driver.h"
#include "compass_hmc5883l.h"
#include "time_keeper.h"
#include "print_util.h"
#include "mavlink_stream.h"
#include "tasks.h"

int32_t ic;

void imu_raw2oriented(Imu_Data_t *imu1);

/**
 * \brief	Computes the transition from raw values to scaled values
 *
 * \param	attitude	the pointer structure of the attitude
 * \param	rates		the array of angular rates (IMU), accelerations and magnetometer
 */
void imu_oriented2scale(Imu_Data_t *imu1);

void imu_init (Imu_Data_t *imu1, AHRS_t *attitude_estimation)
{
	lsm330dlc_driver_init();
	print_util_dbg_print("LSM330 initialised \r");	
	
	compass_hmc58831l_init_slow();
	print_util_dbg_print("HMC5883 initialised \r");
	
	//imu_calibrate_Gyros(imu1);
	
	//init gyro
	imu1->calib_gyro.scale_factor[X] =  1.0f / RAW_GYRO_X_SCALE;
	imu1->calib_gyro.scale_factor[Y] =  1.0f / RAW_GYRO_Y_SCALE;
	imu1->calib_gyro.scale_factor[Z] =  1.0f / RAW_GYRO_Z_SCALE;
	imu1->calib_gyro.bias[X] = 0.0f;
	imu1->calib_gyro.bias[Y] = 0.0f;
	imu1->calib_gyro.bias[Z] = 0.0f;
	imu1->calib_gyro.orientation[X] = GYRO_AXIS_X;
	imu1->calib_gyro.orientation[Y] = GYRO_AXIS_Y;
	imu1->calib_gyro.orientation[Z] = GYRO_AXIS_Z;
	
	//init accelero
	imu1->calib_accelero.scale_factor[X] =  1.0f / RAW_ACC_X_SCALE;
	imu1->calib_accelero.scale_factor[Y] =  1.0f / RAW_ACC_Y_SCALE;
	imu1->calib_accelero.scale_factor[Z] =  1.0f / RAW_ACC_Z_SCALE;
	imu1->calib_accelero.bias[X] = ACC_BIAIS_X;
	imu1->calib_accelero.bias[Y] = ACC_BIAIS_Y;
	imu1->calib_accelero.bias[Z] = ACC_BIAIS_Z;
	imu1->calib_accelero.orientation[X] = ACC_AXIS_X;
	imu1->calib_accelero.orientation[Y] = ACC_AXIS_Y;
	imu1->calib_accelero.orientation[Z] = ACC_AXIS_Z;
	
	//init compass
	imu1->calib_compass.scale_factor[X] =  1.0f / RAW_MAG_X_SCALE;
	imu1->calib_compass.scale_factor[Y] =  1.0f / RAW_MAG_Y_SCALE;
	imu1->calib_compass.scale_factor[Z] =  1.0f / RAW_MAG_Z_SCALE;
	imu1->calib_compass.bias[X] = MAG_BIAIS_X;
	imu1->calib_compass.bias[Y] = MAG_BIAIS_Y;
	imu1->calib_compass.bias[Z] = MAG_BIAIS_Z;
	imu1->calib_compass.orientation[X] = MAG_AXIS_X;
	imu1->calib_compass.orientation[Y] = MAG_AXIS_Y;
	imu1->calib_compass.orientation[Z] = MAG_AXIS_Z;
		
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

void imu_calibrate_gyros(Imu_Data_t *imu1)
{
	int32_t i,j;
	//imu_get_raw_data(imu1);
	tasks_run_imu_update(0);
	
	for (j = 0; j < 3; j++)
	{
		imu1->calib_gyro.bias[j] = (float)imu1->oriented_gyro.data[j];
	}
	
	for (i = 0; i < 100; i++)
	{
		//imu_get_raw_data(imu1);
		tasks_run_imu_update(0);

		//imu1->imu1->calib_sensor.bias[0 + ACC_OFFSET] = (0.9f * imu1->imu1->calib_accelero.bias[0] + 0.1f * (float)imu1->oriented_accelero.data[0]);
		//imu1->imu1->calib_sensor.bias[1 + ACC_OFFSET] = (0.9f * imu1->imu1->calib_accelero.bias[1] + 0.1f * (float)imu1->oriented_accelero.data[1]);
		//imu1->imu1->calib_sensor.bias[2 + ACC_OFFSET] = (0.9f * imu1->imu1->calib_accelero.bias[2] + 0.1f * ((float)imu1->oriented_accelero.data[2] - imu1->calib_accelero.scale_factor[2]));
		for (j = 0; j < 3; j++)
		{
			imu1->calib_gyro.bias[j] = (0.9f * imu1->calib_gyro.bias[j] + 0.1f * (float)imu1->oriented_gyro.data[j]);
			//imu1->attitude.raw_mag_mean[j] = (1.0 - fMAG_LPF) * imu1->attitude.raw_mag_mean[j] + MAG_LPF * ((float)imu1->oriented_compass.data[j]);
		}
	
		delay_ms(4);
	}
}

void imu_update(Imu_Data_t *imu1)
{
	uint32_t t = time_keeper_get_time_ticks();
	
	if (!imu_last_update_init)
	{
		imu1->last_update = t;
		imu_last_update_init = true;
	}
	else
	{
		imu1->dt = time_keeper_ticks_to_seconds(t - imu1->last_update);
		imu1->last_update = t;
		imu_raw2oriented(imu1);
		imu_oriented2scale(imu1);
	}
}

void imu_raw2oriented(Imu_Data_t *imu1)
{
	for (uint16_t i=0; i<3; i++)
	{
		imu1->oriented_gyro.data[i]		= imu1->raw_gyro.data[i] * imu1->calib_gyro.orientation[i];
		imu1->oriented_accelero.data[i] = imu1->raw_accelero.data[i] * imu1->calib_accelero.orientation[i];
		imu1->oriented_compass.data[i]	= imu1->raw_compass.data[i] * imu1->calib_compass.orientation[i];
	}
}

void imu_oriented2scale(Imu_Data_t *imu1)
{
	for (int16_t i = 0; i < 3; i++)
	{
		imu1->scaled_gyro.data[i]  = (1.0f - GYRO_LPF) * imu1->scaled_gyro.data[i] + GYRO_LPF * (((float)imu1->oriented_gyro.data[i] - imu1->calib_gyro.bias[i]) * imu1->calib_gyro.scale_factor[i]);
		imu1->scaled_accelero.data[i]   = (1.0f - ACC_LPF) * imu1->scaled_accelero.data[i] + ACC_LPF * (((float)imu1->oriented_accelero.data[i] - imu1->calib_accelero.bias[i]) * imu1->calib_accelero.scale_factor[i]);
		imu1->scaled_compass.data[i] = (1.0f - MAG_LPF) * imu1->scaled_compass.data[i] + MAG_LPF * (((float)imu1->oriented_compass.data[i] - imu1->calib_compass.bias[i]) * imu1->calib_compass.scale_factor[i]);
	}
}