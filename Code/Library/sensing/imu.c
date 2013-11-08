/*
 * imu.c
 *
 *  Created on: Mar 7, 2010
 *      Author: felix
 */
#include "imu.h"

#include "qfilter.h"
#include "delay.h"
#include "itg3200_driver.h"
#include "adxl345_driver.h"
#include "lsm330dlc_driver.h"

#include "compass_hmc5883l.h"
#include "time_keeper.h"
#include "print_util.h"
#include "position_estimation.h"

int ic;
void init_imu (Imu_Data_t *imu1) {
	
	
	init_itg3200_slow();	
	init_adxl345_slow();
	//init_lsm330();
	
	init_hmc5883_slow();

	//calibrate_Gyros(imu1);
	imu1->raw_scale[0] =  RAW_GYRO_X_SCALE;
	imu1->raw_scale[1] =  RAW_GYRO_Y_SCALE;
	imu1->raw_scale[2] =  RAW_GYRO_Z_SCALE;
	imu1->raw_scale[0+ACC_OFFSET] =  RAW_ACC_X_SCALE;
	imu1->raw_scale[1+ACC_OFFSET] =  RAW_ACC_Y_SCALE;
	imu1->raw_scale[2+ACC_OFFSET] =  RAW_ACC_Z_SCALE;
	imu1->raw_scale[0+COMPASS_OFFSET] =  RAW_MAG_X_SCALE;
	imu1->raw_scale[1+COMPASS_OFFSET] =  RAW_MAG_Y_SCALE;
	imu1->raw_scale[2+COMPASS_OFFSET] =  RAW_MAG_Z_SCALE;
	
	imu1->raw_bias[0+GYRO_OFFSET]= 0.0;
	imu1->raw_bias[1+GYRO_OFFSET]= 0.0;
	imu1->raw_bias[2+GYRO_OFFSET]= 0.0;
	//myquad
	// acceleration biais
	imu1->raw_bias[0+ACC_OFFSET]= ACC_BIAIS_X;
	imu1->raw_bias[1+ACC_OFFSET]= ACC_BIAIS_Y;
	imu1->raw_bias[2+ACC_OFFSET]= ACC_BIAIS_Z;
	
	// magneto bias
	imu1->raw_bias[0+COMPASS_OFFSET]= MAG_BIAIS_X;
	imu1->raw_bias[1+COMPASS_OFFSET]= MAG_BIAIS_Y;
	imu1->raw_bias[2+COMPASS_OFFSET]= MAG_BIAIS_Z;
	
	
	imu_last_update_init = false;
	
	qfInit(&imu1->attitude, imu1->raw_scale, imu1->raw_bias);
	imu1->attitude.calibration_level=OFF;
}


void imu_get_raw_data(Imu_Data_t *imu1) {
	int i=0;
	
	gyro_data* gyros=get_gyro_data_slow();
	acc_data* accs=get_acc_data_slow();
	
//	lsm_gyro_data_t* gyros=lsm330_get_gyro_data();
//	lsm_acc_data_t* accs=lsm330_get_acc_data();
	compass_data* compass=get_compass_data_slow();


	imu1->raw_channels[GYRO_OFFSET+IMU_X]=(float)gyros->axes[RAW_GYRO_X];
	imu1->raw_channels[GYRO_OFFSET+IMU_Y]=(float)gyros->axes[RAW_GYRO_Y];
	imu1->raw_channels[GYRO_OFFSET+IMU_Z]=(float)gyros->axes[RAW_GYRO_Z];

	imu1->raw_channels[ACC_OFFSET+IMU_X]=(float)accs->axes[RAW_ACC_X];
	imu1->raw_channels[ACC_OFFSET+IMU_Y]=(float)accs->axes[RAW_ACC_Y];
	imu1->raw_channels[ACC_OFFSET+IMU_Z]=(float)accs->axes[RAW_ACC_Z];

	imu1->raw_channels[COMPASS_OFFSET+IMU_X]=(float)-compass->axes[RAW_COMPASS_X];
	imu1->raw_channels[COMPASS_OFFSET+IMU_Y]=(float)-compass->axes[RAW_COMPASS_Y];
	imu1->raw_channels[COMPASS_OFFSET+IMU_Z]=(float)compass->axes[RAW_COMPASS_Z];
	
}

void calibrate_Gyros(Imu_Data_t *imu1) {
	int i,j;
	imu_get_raw_data(imu1);
	for (j=0; j<3; j++) {
		imu1->raw_bias[j]=(float)imu1->raw_channels[j];
	}
	
	for (i=0; i<100; i++) {
		imu_get_raw_data(imu1);

		//imu1->raw_bias[0+ACC_OFFSET]  = (0.9*imu1->raw_bias[0+ACC_OFFSET]+0.1*(float)imu1->raw_channels[0+ACC_OFFSET]);
		//imu1->raw_bias[1+ACC_OFFSET]  = (0.9*imu1->raw_bias[1+ACC_OFFSET]+0.1*(float)imu1->raw_channels[1+ACC_OFFSET]);
		//imu1->raw_bias[2+ACC_OFFSET]  = (0.9*imu1->raw_bias[2+ACC_OFFSET]+0.1*((float)imu1->raw_channels[2+ACC_OFFSET]-imu1->raw_scale[2+ACC_OFFSET]));
		for (j=0; j<3; j++) {
			imu1->raw_bias[j]=(0.9*imu1->raw_bias[j]+0.1*(float)imu1->raw_channels[j]);
			imu1->attitude.raw_mag_mean[j] = (1.0-MAG_LPF)*imu1->attitude.raw_mag_mean[j]+MAG_LPF*((float)imu1->raw_channels[j+COMPASS_OFFSET]);
		}
		delay_ms(4);
	}


}

void imu_update(Imu_Data_t *imu1, position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps){
	uint32_t t=get_time_ticks();
	
	if (!imu_last_update_init)
	{
		imu1->last_update = t;
		imu_last_update_init = true;
	}else{
		imu1->dt=ticks_to_seconds(t - imu1->last_update);
		imu1->last_update=t;
		qfilter(&imu1->attitude, &imu1->raw_channels, imu1->dt, false);
		if (imu1->attitude.calibration_level==OFF) {
			position_integration(pos_est, &imu1->attitude, imu1->dt);
			position_correction(pos_est, barometer, gps, imu1->dt);
		}
	}
}




