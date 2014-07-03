/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file compass_hmc58831l.h
*
* This file is the driver for the magnetometer HMC58831
*/


#ifndef COMPASS_HMC5883L_H_
#define COMPASS_HMC5883L_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "compiler.h"


#define ConfRegA 0x00					///< Configuration Register A
#define ConfRegB 0x01					///< Configuration Register B
#define ModeReg 0x02					///< Mode register
#define DataRegBegin 0x03				///< Data Register Begin Command

#define Measurement_Continuous 0x00		///< Continuous measurement Mode
#define Measurement_SingleShot 0x01		///< Single Shot measurement Mode
#define Measurement_Idle 0x03			///< Idle Mode

#define HMC5883_SLAVE_ADDRESS 0x1E		///< HMC5883L

/**
 * \brief Defines 4 states for the sampling average
*/
enum 
{
	HMC_SAMPLE_AVG_1, 
	HMC_SAMPLE_AVG_2,
	HMC_SAMPLE_AVG_4, 
	HMC_SAMPLE_AVG_8
}; 

/**
 * \brief Defines 7 states for the sampling rate
*/
enum 
{
	HMC_RATE_0_75,
	HMC_RATE_1_5,
	HMC_RATE_3_0, 
	HMC_RATE_7_5, 
	HMC_RATE_15, 
	HMC_RATE_30, 
	HMC_RATE_75
};

/**
 * \brief Defines 3 states for the sampling bias mode
*/
enum
{
	HMC_BIAS_MODE_NORMAL, 
	HMC_BIAS_MODE_POS_BIAS, 
	HMC_BIAS_MODE_NEG_BIAS
};

/**
 * \brief Defines 8 states for the sampling range
*/
enum
{
	HMC_RANGE_0_88_GA,
	HMC_RANGE_1_3_GA,
	HMC_RANGE_1_9_GA,
	HMC_RANGE_2_5_GA,
	HMC_RANGE_4_0_GA,
	HMC_RANGE_4_7_GA,
	HMC_RANGE_5_6_GA,
	HMC_RANGE_8_1_GA
};

/**
 * \brief Defines 3 states for the sampling mode
*/
enum 
{	
	HMC_MODE_CONTINUOUS, 
	HMC_MODE_SINGLE, 
	HMC_MODE_IDLE
};

#define HMC_SAMPLE_AVG	HMC_SAMPLE_AVG_4		///< Define the sampling average mode used
#define HMC_RATE		HMC_RATE_15				///< Define the sampling rate mode used
#define HMC_BIAS_MODE	HMC_BIAS_MODE_NORMAL	///< Define the sampling bias mode used
#define HMC_RANGE		HMC_RANGE_1_3_GA		///< Define the sampling range mode used
#define HMC_MODE		HMC_MODE_CONTINUOUS		///< Define the sampling mode used

/**
 * \brief structure for the magnetometer's data
*/
typedef struct
{
	uint8_t raw_data[6];
	int16_t axes[3];
} compass_data;

/**
 * \brief Initializes the magnetometer sensor
*/
void compass_hmc58831l_init(void);

/**
 * \brief Initializes the magnetometer sensor in slow mode
*/
void compass_hmc58831l_init_slow(void);

/**
 * \brief Returns the magnetometer's data in slow mode
 *
 * \return a pointer to the magnetometer's data in slow mode
*/
compass_data* compass_hmc58831l_get_data_slow(void);

#ifdef __cplusplus
	}
#endif

#endif /* COMPASS_HMC5883_H_ */