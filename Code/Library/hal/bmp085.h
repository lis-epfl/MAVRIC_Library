/*
 * bmp085.h
 *
 * Created: 12/03/2013 22:14:32
 *  Author: sfx
 */ 


#ifndef BMP085_H_
#define BMP085_H_


#include "compiler.h"


#define VARIO_LPF 0.05

#define BMP085_SLAVE_ADDRESS 0x77  //

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD 1
#define BMP085_HIGHRES 2
#define BMP085_ULTRAHIGHRES 3
#define BMP085_CAL_AC1 0xAA // R Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC // R Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE // R Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0 // R Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2 // R Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4 // R Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6 // R Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8 // R Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA // R Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC // R Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE // R Calibration data (16 bits)

#define BMP085_CONTROL 0xF4
#define BMP085_TEMPDATA 0xF6
#define BMP085_PRESSUREDATA 0xF6
#define BMP085_READTEMPCMD 0x2E
#define BMP085_READPRESSURECMD 0x34

#define oversampling BMP085_HIGHRES

typedef enum  pressure_sensor_state{IDLE, GET_TEMP, GET_PRESSURE} pressure_sensor_state;

typedef struct{
	uint8_t raw_pressure[3];
	uint8_t raw_temperature[2];
	float pressure;
	float temperature;
	float altitude;
	float altitude_offset;
	float vario_vz;
	uint32_t last_update;
	uint32_t last_state_update;
	pressure_sensor_state state;
} pressure_data;


void init_bmp085(void);

void init_bmp085_slow(void);

void start_pressure_measurement(void);

pressure_data* get_pressure_data_slow(float offset);

bool newValidBarometer(uint32_t *timePrevBarometer);

#endif /* BMP085_H_ */