/**
* This file is the driver for the barometer module: BMP085
*
* The MAV'RIC Framework
* Copyright © 2011-2014
*
* Laboratory of Intelligent Systems, EPFL
*
* This file is part of the MAV'RIC Framework.
*/

#ifndef BMP085_H_
#define BMP085_H_


#include "compiler.h"

#define BARO_ALT_LPF 0.95						///< low pass filter factor for altitude measured by the barometer
#define VARIO_LPF 0.95							///< low pass filter factor for the Vario altitude speed

#define BMP085_SLAVE_ADDRESS 0x77				///< Address of the barometer sensor on the i2c bus

#define BMP085_ULTRALOWPOWER 0					///< Ultra low power mode
#define BMP085_STANDARD 1						///< standard mode
#define BMP085_HIGHRES 2						///< high resolution mode
#define BMP085_ULTRAHIGHRES 3					///< ultra high resolution mode

#define BMP085_CAL_AC1 0xAA						///< R Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC						///< R Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE						///< R Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0						///< R Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2						///< R Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4						///< R Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6						///< R Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8						///< R Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA						///< R Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC						///< R Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE						///< R Calibration data (16 bits)

#define BMP085_CONTROL 0xF4						///< Control register of the barometer sensor 
#define BMP085_TEMPDATA 0xF6					///< Temperature register of the barometer sensor 
#define BMP085_PRESSUREDATA 0xF6				///< Pressure Data register of the barometer sensor 
#define BMP085_READTEMPCMD 0x2E					///< Read temperature Command register of the barometer sensor 
#define BMP085_READPRESSURECMD 0x34				///< Read Pressure Command register of the barometer sensor 

#define BMP085_OVERSAMPLING_MODE BMP085_HIGHRES	///< Set oversampling mode of the barometer sensor to high resolution mode

/**
 * \brief pressure_sensor_state can get three different state: Iddle, get Temperature or get Pressure
*/
typedef enum  pressure_sensor_state{IDLE, GET_TEMP, GET_PRESSURE} pressure_sensor_state;

/**
 * \brief structure containing all the barometer's data
*/
typedef struct{
	uint8_t raw_pressure[3];		///< raw pressure contained in 3 uint8_t
	uint8_t raw_temperature[2];		///< raw temperature contained in 2 uint8_t
	float pressure;					///< measured pressure as the concatenation of the 3 uint8_t raw_pressure
	float temperature;				///< measured temperature as the concatenation of the 2 uint8_t raw_temperature
	float last_altitudes[3];		///< array to store previous value of the altitude for low pass filtering the output
	float altitude;					///< measured altitude as the median filter of the 3 last_altitudes
	float altitude_offset;			///< offset of the barometer sensor for matching GPS altitude value
	float vario_vz;					///< vario altitude speed
	uint32_t last_update;			///< time of the last update of the barometer
	uint32_t last_state_update;		///< time of the last state update
	pressure_sensor_state state;	///< state of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
	float dt;						///< time step for the derivative
} pressure_data;

/**
 * \brief Initialize the barometer sensor
*/
void init_bmp085(void);

/**
 * \brief Initialize the barometer sensor in slow mode
*/
void init_bmp085_slow(void);

/**
 * \brief Start the pressure measurement
*/
void start_pressure_measurement(void);

/**
 * \brief Get the pressure data n slow mode
 *
 * \param offset Offset to add to the pressure measured by the barometer
 *
 * \return a pointer to the pressure data structure
*/
pressure_data* get_pressure_data_slow(float offset);

/**
 * \brief Returns whether a new valid measure is ready
 *
 * \return a boolean: true if a new valid measure is ready
*/
bool newValidBarometer(uint32_t *timePrevBarometer);

#endif /* BMP085_H_ */