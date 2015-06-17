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
 * \file fat_fs_mounting.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the mounting/unmounting of the fat_fs file system
 *
 ******************************************************************************/


#ifndef FAT_FS_MOUNTING_H__
#define FAT_FS_MOUNTING_H__

#include "state.hpp"

extern "C"
{
	#include "libs/fat_fs/ff.h"
}


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

/**
 * \brief 	The fat_fs mounting structure
 */
typedef struct 
{
	data_logging_conf_t data_logging_conf;		///< The data logging configuration structre

	FRESULT fr;									///< The result of the fatfs functions
	FATFS fs;									///< The fatfs handler

	uint32_t loop_count;						///< Counter to try to mount the SD card many times
	uint32_t log_data;							///< A flag to stop/start writing to file

	bool sys_mounted;							///< A flag to tell whether the file system is mounted
	
	uint32_t num_file_opened;					///< Number of open files to now when the system can be unmounted

	const state_t* state;						///< The pointer to the state structure
}fat_fs_mounting_t;

/**
 * \brief	Initialise the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 * \param	data_logging_conf		The pointer to the configuration structure
 * \param	state					The pointer to the state structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool fat_fs_mounting_init(fat_fs_mounting_t* fat_fs_mounting, const data_logging_conf_t* data_logging_conf, const state_t* state);

/**
 * \brief	Mount the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 * \param	debug					A flag to tell if we print the result or not for debug purposes
 */
void fat_fs_mounting_mount(fat_fs_mounting_t* fat_fs_mounting, bool debug);

/**
 * \brief	Unmount the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 * \param	debug					A flag to tell if we print the result or not for debug purposes
 */
void fat_fs_mounting_unmount(fat_fs_mounting_t* fat_fs_mounting, bool debug);

/**
 * \brief	Prints on debug port the result's value of the fatfs operation
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 */
void fat_fs_mounting_print_error_signification(FRESULT fr);

#endif /* FAT_FS_MOUNTING_H__ */