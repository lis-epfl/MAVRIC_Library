/*
 * conf_imu_rev4.h
 *
 * Created: 20/11/2013 22:21:49
 *  Author: sfx
 */ 


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_


#define RAW_GYRO_X 0
#define RAW_GYRO_Y 1
#define RAW_GYRO_Z 2

#define RAW_ACC_X 0
#define RAW_ACC_Y 1
#define RAW_ACC_Z 2

#define RAW_COMPASS_X 2
#define RAW_COMPASS_Y 0
#define RAW_COMPASS_Z 1

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111
#define RAW_GYRO_X_SCALE  818.5111
#define RAW_GYRO_Y_SCALE  818.5111
#define RAW_GYRO_Z_SCALE  818.5111

#define GYRO_AXIS_X  1.0
#define GYRO_AXIS_Y -1.0
#define GYRO_AXIS_Z -1.0

#define RAW_ACC_X_SCALE  4145.7462 //3924.0
#define RAW_ACC_Y_SCALE  4071.6653 //3844.8
#define RAW_ACC_Z_SCALE  4141.9266 //4119.6

#define ACC_BIAIS_X  -128.0 //-5.00//-97.20;
#define ACC_BIAIS_Y  130.0 //50.00//124.35;
#define ACC_BIAIS_Z  230.00 //240.00//-40.73;

#define ACC_AXIS_X  1.0
#define ACC_AXIS_Y -1.0
#define ACC_AXIS_Z -1.0

#define RAW_MAG_X_SCALE 671.1324 //601.3117//624.11
#define RAW_MAG_Y_SCALE 699.7155 //580.3974//590.77
#define RAW_MAG_Z_SCALE 600.8004 //513.8466//512.37

#define MAG_BIAIS_X 22.6809 //-339.00//-127.86
#define MAG_BIAIS_Y 53.48 //-251.5//-138.32
#define MAG_BIAIS_Z -357.72 //83.0//-79.38

#define MAG_AXIS_X -1.0
#define MAG_AXIS_Y -1.0
#define MAG_AXIS_Z -1.0

#endif /* CONF_IMU_REV4_H_ */