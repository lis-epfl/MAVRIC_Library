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
 * \file fat_fs_mounting.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the mounting/unmounting of the fat_fs file system
 *
 ******************************************************************************/


#ifndef FAT_FS_MOUNTING_HPP_
#define FAT_FS_MOUNTING_HPP_

extern "C"
{
	#include <stdint.h>
	#include "libs/FatFs/src/ff.h"
}


/**
 * \brief 	The fat_fs mounting structure
 */
typedef struct 
{
	FATFS fs;									///< The fatfs handler

	uint32_t loop_count;						///< Counter to try to mount the SD card many times

	bool sys_mounted;							///< A flag to tell whether the file system is mounted
	
	uint32_t num_file_opened;					///< Number of open files to now when the system can be unmounted
}fat_fs_mounting_t;

/**
 * \brief	Initialise the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool fat_fs_mounting_init(fat_fs_mounting_t* fat_fs_mounting);

/**
 * \brief	Mount the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 * \param	debug					A flag to tell if we print the result or not for debug purposes
 *
 * \return	True if system was mounted, false otherwise
 */
bool fat_fs_mounting_mount(fat_fs_mounting_t* fat_fs_mounting, bool debug);

/**
 * \brief	Unmount the fat_fs system file
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 * \param	debug					A flag to tell if we print the result or not for debug purposes
 *
 * \return	True if system was unmounted, false otherwise
 */
bool fat_fs_mounting_unmount(fat_fs_mounting_t* fat_fs_mounting, bool debug);

/**
 * \brief	Prints on debug port the result's value of the fatfs operation
 *
 * \param	fat_fs_mounting			The pointer to the SD card mounting structure
 */
void fat_fs_mounting_print_error_signification(FRESULT fr);

#endif /* FAT_FS_MOUNTING_HPP_ */