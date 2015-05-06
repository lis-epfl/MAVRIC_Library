/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
 
/*******************************************************************************
 * \file sd_mounting.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the mounting/unmounting of the SD card
 *
 ******************************************************************************/


#ifndef SD_MOUNTING_H__
#define SD_MOUNTING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "libs/fat_fs/ff.h"
#include "state.h"


/**
 * \brief 	Configuration for the module data logging
 */
typedef struct
{
	uint32_t max_data_logging_count;			///< Maximum number of parameters
	uint16_t max_logs;							///< The max number of logged files with the same name on the SD card
	bool debug;									///< Indicates if debug messages should be printed for each param change
	uint32_t log_data;							///< The initial state of writing a file
} data_logging_conf_t;


typedef struct 
{
	const data_logging_conf_t* data_logging_conf;

	FRESULT fr;									///< The result of the fatfs functions
	FATFS fs;									///< The fatfs handler

	uint32_t loop_count;						///< Counter to try to mount the SD card many times
	uint32_t log_data;							///< A flag to stop/start writing to file

	bool sys_mounted;							///< A flag to tell whether the file system is mounted
	
	uint32_t num_file_opened;

	const state_t* state;
}sd_mounting_t;


bool sd_mounting_init(sd_mounting_t* sd_mounting, const data_logging_conf_t* data_logging_conf, const state_t* state);

void sd_mounting_mount(sd_mounting_t* sd_mounting, bool debug);
void sd_mounting_unmount(sd_mounting_t* sd_mounting, bool debug);



#ifdef __cplusplus
}
#endif

#endif /* SD_MOUNTING_H__ */