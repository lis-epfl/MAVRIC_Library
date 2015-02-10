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
 * \file test_fat_fs.h
 *
 * This file can be used to test the low and mid layer of writing to SD card for fat fs use. 
 */


#ifndef TEST_FAT_FS_H__
#define TEST_FAT_FS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "integer.h"
#include "ff.h"

/**
 * \brief	A function to test the low layer (read/write bytes to the SD card)
 */
void test_fat_fs_low_layer(void);

/** 
 * \brief	Open the file path and seek the end to append
 *
 * \param	fp			[OUT] File object to create
 * \param	path		[IN]  File name to be opened
 * \param	opening_opt	The opening file options
 *
 * \result	The result of the process, a FRESULT enum (defined in ff.h)
 */
FRESULT test_fat_fs_open_append (FIL* fp, const char* path, BYTE opening_opt);

#ifdef __cplusplus
}
#endif

#endif // TEST_FAT_FS_H__