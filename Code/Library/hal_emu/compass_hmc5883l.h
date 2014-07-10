/*
 * compass_hmc5883.h
 *
 * Created: 12/03/2013 20:51:14
 *  Author: sfx
 */ 


#ifndef COMPASS_HMC5883L_H_
#define COMPASS_HMC5883L_H_

#include "compiler.h"


#define ConfRegA 0x00
#define ConfRegB 0x01
#define ModeReg 0x02
#define DataRegBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

#define HMC5883_SLAVE_ADDRESS 0x1E  ///HMC5883L

enum 
{HMC_SAMPLE_AVG_1, 
 HMC_SAMPLE_AVG_2,
 HMC_SAMPLE_AVG_4, 
 HMC_SAMPLE_AVG_8}; 

enum 
{HMC_RATE_0_75,
 HMC_RATE_1_5,
 HMC_RATE_3_0, 
 HMC_RATE_7_5, 
 HMC_RATE_15, 
 HMC_RATE_30, 
 HMC_RATE_75};

enum
{HMC_BIAS_MODE_NORMAL, 
 HMC_BIAS_MODE_POS_BIAS, 
 HMC_BIAS_MODE_NEG_BIAS};

enum
{HMC_RANGE_0_88_GA,
 HMC_RANGE_1_3_GA,
 HMC_RANGE_1_9_GA,
 HMC_RANGE_2_5_GA,
 HMC_RANGE_4_0_GA,
 HMC_RANGE_4_7_GA,
 HMC_RANGE_5_6_GA,
 HMC_RANGE_8_1_GA};

enum 
{HMC_MODE_CONTINUOUS, 
 HMC_MODE_SINGLE, 
 HMC_MODE_IDLE
};

#define HMC_SAMPLE_AVG	HMC_SAMPLE_AVG_4
#define HMC_RATE		HMC_RATE_15
#define HMC_BIAS_MODE	HMC_BIAS_MODE_NORMAL
#define HMC_RANGE		HMC_RANGE_1_3_GA
#define HMC_MODE		HMC_MODE_CONTINUOUS



typedef struct{
	uint8_t raw_data[6];
	int16_t axes[3];
} compass_data_t;



void compass_hmc58831l_init();

void compass_hmc58831l_init_slow();

compass_data_t* compass_hmc58831l_get_data_slow();



#endif /* COMPASS_HMC5883_H_ */