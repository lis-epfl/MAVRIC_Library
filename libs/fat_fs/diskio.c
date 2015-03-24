/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/

/**
 * \file diskio.h
 *
 *  Link the FatFs level with the low level command of the disk
 *  If a working storage control module is available, it should be
 *  attached to the FatFs via a glue function rather than modifying it.
 *  This is an example of glue functions to attach various exsisting
 *  storage control module to the FatFs module with a defined API.
 */

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
//#include "sdcard.h"		/* Example: MMC/SDC contorl */

#include "sd_spi.h"
#include "print_util.h"

#include "time_keeper.h"
#include <maths.h>
// #include "delay.h"
#include "string.h"

/* Definitions of physical drive number for each media */
#define ATA		0
#define MMC		1
#define USB		2

DSTATUS actual_status;

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (BYTE pdrv)
{
	DSTATUS stat = STA_NOINIT;
	int result;

	actual_status = STA_NOINIT;

	uint8_t drive_num;
	if (pdrv==0)
	{
		drive_num = MMC;
	}
	else
	{
		return STA_NOINIT;
	}
	
	switch (drive_num)
	{
		case ATA :
			//result = ATA_disk_initialize();

			print_util_dbg_print("NO SUPPORTED! ATA init!\r\n");

			// translate the result code here
			result = RES_ERROR;
			stat = STA_NODISK;
			break;

		case MMC :
			//result = MMC_disk_initialize();
		
			result = sd_spi_init();
		
			if (result)
			{
				stat = 0;
				actual_status = 0;
			}
			else
			{
				stat = STA_NOINIT;
			}
			break;

		case USB :
			//result = USB_disk_initialize();

			print_util_dbg_print("NO SUPPORTED! USB init!\r\n");
		
			// translate the result code here
			result = RES_ERROR;
			stat = STA_NODISK;
			break;
	}
	return stat;
}

DSTATUS disk_status (BYTE pdrv)
{
	DSTATUS stat = STA_NOINIT;
	int result = RES_ERROR;

	if ((actual_status & STA_NOINIT)||(actual_status & STA_NODISK))
	{
		return STA_NOINIT;
	}

	// Only MMC supported
	if (pdrv!=0)
	{
		return RES_PARERR;
	}
	uint8_t drive_num = MMC;

	switch (drive_num)
	{
		case ATA :
			//result = ATA_disk_status();

			print_util_dbg_print("NO SUPPORTED! ATA status!\r\n");

			// translate the result code here
		
			result = RES_ERROR;
			stat = STA_NODISK;
			break;

		case MMC :
			//result = MMC_disk_status();

			// translate the result code here

			result = sd_spi_status();
		
			if (result)
			{
				stat = 0;
			}
			else
			{
				stat = STA_NODISK;
			}
			break;

		case USB :
			//result = USB_disk_status();

			print_util_dbg_print("NO SUPPORTED! USB status!\r\n");

			// translate the result code here
		
			result = RES_ERROR;
			stat = STA_NODISK;
			break;
	}
	return stat;
}

DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res = RES_PARERR;
	int result = RES_ERROR;

	if ((actual_status & STA_NOINIT)||(actual_status & STA_NODISK))
	{
		return RES_NOTRDY;
	}

	// Only MMC supported
	if (pdrv!=0)
	{
		return RES_PARERR;
	}
	uint8_t drive_num = MMC;

	switch (drive_num)
	{
		case ATA :
			// translate the arguments here

			print_util_dbg_print("NO SUPPORTED! ATA read!\r\n");

			//result = ATA_disk_read(buff, sector, count);

			// translate the result code here

			//if (result)
			//{
				//res = RES_OK;
			//}
			//else
			//{
				//res = RES_ERROR;
			//}
			result = RES_ERROR;
			res = RES_ERROR;
			break;

		case MMC :
			// translate the arguments here

			//result = MMC_disk_read(buff, sector, count);
			result = sd_spi_read_given_sector_to_ram(buff,sector,count);

			if (result)
			{
				res = RES_OK;
			}
			else
			{
				res = RES_ERROR;
			}
			break;

		case USB :
			// translate the arguments here

			//result = USB_disk_read(buff, sector, count);
		
			print_util_dbg_print("NO SUPPORTED! USB read!\r\n");
		
			result = RES_ERROR;
			res = RES_ERROR;
			break;
	}
	return res;
}

#if _USE_WRITE
DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res;
	int result = RES_ERROR;

	if ((actual_status & STA_NOINIT)||(actual_status & STA_NODISK))
	{
		return RES_NOTRDY;
	}

	if (actual_status & STA_PROTECT)
	{
		return RES_WRPRT;
	}

	// Only MMC supported
	if (pdrv!=0)
	{
		return RES_PARERR;
	}
	uint8_t drive_num = MMC;

	switch (drive_num)
	{
		case ATA :
			// translate the arguments here

			print_util_dbg_print("NO SUPPORTED! ATA write!\r\n");

			//result = ATA_disk_write(buff, sector, count);

			if (result)
			{
				res = RES_OK;
			}
			else
			{
				res = RES_ERROR;
			}

			// translate the result code here

			return res;

		case MMC :
			// translate the arguments here

			//result = MMC_disk_write(buff, sector, count);
			result = sd_spi_write_given_sector_from_ram(buff, sector, count);
		
			if (result)
			{
				res = RES_OK;
			}
			else
			{
				res = RES_ERROR;
			}
		
			// translate the result code here

			return res;

		case USB :
			// translate the arguments here

			print_util_dbg_print("NO SUPPORTED! USB init!\r\n");

			//result = USB_disk_write(buff, sector, count);

			if (result)
			{
				res = RES_OK;
			}
			else
			{
				res = RES_ERROR;
			}

			// translate the result code here

			return res;
	}
	return RES_PARERR;
}
#endif

#if _USE_IOCTL
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
	DRESULT res = RES_PARERR;
	int result = RES_ERROR;

	uint8_t drive_num;
	if (pdrv==0)
	{
		drive_num = MMC;
	}
	else
	{
		return STA_NODISK;
	}

	switch (drive_num)
	{
		case ATA :
			// pre-process here

			//result = ATA_disk_ioctl(cmd, buff);

			// post-process here
		
			result = RES_ERROR;
			res = RES_ERROR;
			break;

		case MMC :
			// pre-process here

			//result = MMC_disk_ioctl(cmd, buff);

			switch(cmd)
			{
				case CTRL_SYNC:
					while (!sd_spi_wait_not_busy());
					break;
			
				case GET_SECTOR_COUNT:
					sd_spi_get_sector_count(buff);
					break;
			
				case GET_SECTOR_SIZE:
					sd_spi_get_sector_size(buff);
					break;
			
				case GET_BLOCK_SIZE:
					sd_spi_get_block_size(buff);
					break;
			
				case CTRL_ERASE_SECTOR:
				break;
			}
			res = RES_OK;
			break;

		case USB :
			// pre-process here
			
			//result = USB_disk_ioctl(cmd, buff);
			
			// post-process here
			
			result = RES_ERROR;
			res = result;
			break;
	}
	return res;
}

DWORD get_fattime(void)
{
	uint32_t time = 0;
	
	double time_in_seconds;
	
	time_in_seconds = time_keeper_get_time();
	
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	
	minutes = (uint8_t) floor(((float)time_in_seconds) / 60.0f);
	
	seconds = (uint8_t) ((((uint32_t)time_in_seconds) % 60)/2);
	
	hours = (uint8_t) floor(((float)minutes) / 60.0f);
	
	minutes = (uint8_t) (minutes % 60);
	
	uint8_t day = 26;
	uint8_t month = 8;
	uint8_t year = 2014-1980;
	
	time += (seconds			& 0b00000000000000000000000000011111);
	time += ((minutes	<< 5)	& 0b00000000000000000000011111100000);
	time += ((hours		<< 11)	& 0b00000000000000001111100000000000);
	time += ((day		<< 16)	& 0b00000000000111110000000000000000);
	time += ((month		<< 21)	& 0b00000001111000000000000000000000);
	time += ((year		<< 25)	& 0b11111110000000000000000000000000);
	
	return time;
}
#endif