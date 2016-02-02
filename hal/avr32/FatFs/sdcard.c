/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file sdcard.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Glue file to implement FatFs SD card on avr32
 *
 ******************************************************************************/


#include "libs/FatFs/src/sdcard.h"
#include "hal/avr32/sd_spi.h"


bool MMC_disk_status(void)
{
	return sd_spi_status();
}


bool MMC_disk_initialize(void)
{
	return sd_spi_init();
}


bool MMC_disk_read(void* buff, uint32_t sector, uint8_t count)
{
	return sd_spi_read_given_sector_to_ram(buff,sector,count);
}


bool MMC_disk_write(const void *buff, uint32_t sector, uint8_t count)
{
	return sd_spi_write_given_sector_from_ram(buff, sector, count);
}


bool MMC_ctrl_sync(void)
{
	while( !sd_spi_wait_not_busy() )
	{
		;
	}
	return true;
}


bool MMC_get_sector_count(void* buff)
{
	sd_spi_get_sector_count(buff);
	return true;;
}


bool MMC_get_sector_size(void* buff)
{
	sd_spi_get_sector_size(buff);
	return true;
}


bool MMC_get_block_size(void* buff)
{
	sd_spi_get_sector_size(buff);
	return true;
}


bool MMC_ctrl_trim(void* buff)
{
	return false;
}


bool MMC_get_type(void* buff)
{
	return false;
}


bool MMC_get_csd(void* buff)
{
	return false;
}


bool MMC_get_cid(void* buff)
{
	return false;
}


bool MMC_get_ocr(void* buff)
{
	return false;
}


bool MMC_get_sdstat(void* buff)
{
	return false;
}
