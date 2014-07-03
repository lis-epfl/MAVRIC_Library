/*
 * conf_imu_rev3.h
 *
 * Created: 20/11/2013 22:20:48
 *  Author: sfx
 */ 


#ifndef CONF_IMU_REV3_H_
#define CONF_IMU_REV3_H_


#define RAW_GYRO_X 1
#define RAW_GYRO_Y 0
#define RAW_GYRO_Z 2

#define RAW_GYRO_X_SCALE   12600.0
#define RAW_GYRO_Y_SCALE  -12600.0
#define RAW_GYRO_Z_SCALE   12600.0

#define RAW_ACC_X 0
#define RAW_ACC_Y 1
#define RAW_ACC_Z 2

//#define RAW_ACC_X_SCALE  261.5
//#define RAW_ACC_Y_SCALE  262.5
//#define RAW_ACC_Z_SCALE  255.0
//#define RAW_ACC_X_SCALE  259.67
//#define RAW_ACC_Y_SCALE  261.324
//#define RAW_ACC_Z_SCALE  256.724
// Felix outside
//#define RAW_ACC_X_SCALE  264.9173
#define RAW_ACC_X_SCALE  258.9173
#define RAW_ACC_Y_SCALE  258.9853
#define RAW_ACC_Z_SCALE  258.0829

#define RAW_MAG_X 2
#define RAW_MAG_Y 0
#define RAW_MAG_Z 1

// Inside values
//#define RAW_MAG_X_SCALE 579.41
//#define RAW_MAG_Y_SCALE 540.3
//#define RAW_MAG_Z_SCALE 525.59

// Outside values
//#define RAW_MAG_X_SCALE 534.90
//#define RAW_MAG_Y_SCALE 514.85
//#define RAW_MAG_Z_SCALE 478.57

// Felix Outside values
#define RAW_MAG_X_SCALE 530.2771
#define RAW_MAG_Y_SCALE 525.2934
#define RAW_MAG_Z_SCALE 498.4476

#define ACC_BIAIS_X 18.0;
#define ACC_BIAIS_Y 9.0;
#define ACC_BIAIS_Z -16.0;
//#define ACC_BIAIS_X 4.685
//#define ACC_BIAIS_Y 4.376
//#define ACC_BIAIS_Z -16.26

// Felix Outside values
// #define ACC_BIAIS_X  21.5871
// #define ACC_BIAIS_Y  10.0884
// #define ACC_BIAIS_Z  -14.9891


// Inside values
//#define MAG_BIAIS_X 34.20
//#define MAG_BIAIS_Y -47.07
//#define MAG_BIAIS_Z -76.93

// Outside values
//#define MAG_BIAIS_X 47.62
//#define MAG_BIAIS_Y -47.29
//#define MAG_BIAIS_Z -74.38

// Felix Outside values
#define MAG_BIAIS_X  131.7582
#define MAG_BIAIS_Y -26.1298
#define MAG_BIAIS_Z  61.1646





#endif /* CONF_IMU_REV3_H_ */