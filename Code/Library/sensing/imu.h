/**
 * This file implement the code to read the IMU data
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */

#ifndef IMU_H_
#define IMU_H_

#include "compiler.h"
#include "qfilter.h"
#include "bmp085.h"
#include "position_estimation.h"
#include "gps_ublox.h"

#define IMU_AXES 6

#include "conf_platform.h"

typedef struct {
	Quat_Attitude_t attitude;			///< Attitude structure of the platform
	float raw_channels[9];				///< The array where the raw value of the IMU and compass are parsed
	float raw_bias[9];					///< The biaises of the IMU and compass
	float raw_scale[9];					///< The scales of the IMU and compass
	uint32_t last_update;				///< The time of the last IMU update in ms
	//uint8_t valid;						///< True if the message is valid (TODO: is it sill used?)
	float dt;							///< The time interval between two IMU updates
	//int8_t ready;							///< Is the IMU ready (TODO: is it still used?)
} Imu_Data_t;							///< the IMU structure

bool imu_last_update_init;				///< variable to initialize the IMU

/**
 * \brief	Initialize the IMU module
 *
 * \param	imu1		the pointer to the IMU structure
 *
 * \return	void
 */
void init_imu(Imu_Data_t *imu1);

/**
 * \brief	The function to be called to access the raw data of the IMU
 *
 * \param	imu1		the pointer to the IMU structure
 *
 * \return	void
 */
void imu_get_raw_data(Imu_Data_t *imu1);

/**
 * \brief	To calibrate the gyros at startup (not used know)
 *
 * \param	imu1		the pointer to the IMU structure
 *
 * \return	void
 */
void calibrate_Gyros(Imu_Data_t *imu1);

/**
 * \brief	Computes the attitude estimation, do the position estimation and the position correction
 *
 * \param	imu1		the pointer structure of the IMU
 * \param	pos_est		the pointer to the position estimation structure
 * \param	barometer	the pointer to the barometer structure
 * \param	gps			the pointer to the GPS structure
 *
 * \return	void
 */
void imu_update(Imu_Data_t *imu1, position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps);

#endif /* IMU_H_ */
