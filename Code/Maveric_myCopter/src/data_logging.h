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
 * \file data_logging.h
 *
 *  Performs the data logging on the SD card
 */


#ifndef DATA_LOGGING_H__
#define DATA_LOGGING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FatFs/ff.h"
#include "tasks.h"
#include "imu.h"

typedef struct  
{	
	FRESULT fr;
	FATFS fs;
	FIL fil;

	bool file_init;
	
	bool continue_writing;
	
	const imu_t* imu;
	
}data_logging_t;


void data_logging_init(data_logging_t* data_logging, const imu_t* imu);

task_return_t data_logging_run(data_logging_t* data_logging);






#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGING_H__ */