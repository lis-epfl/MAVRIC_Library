/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
//#include "sdcard.h"		/* Example: MMC/SDC contorl */

#include "sd_spi.h"
#include "print_util.h"

#include "time_keeper.h"
#include <maths.h>
/* Definitions of physical drive number for each media */
#define ATA		0
#define MMC		1
#define USB		2


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive number (0..) */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case ATA :
		//result = ATA_disk_initialize();

		// translate the result code here
		result = RES_ERROR;
		stat = STA_NODISK;

		return stat;

	case MMC :
		//result = MMC_disk_initialize();
		
		result = sd_spi_init();
		
		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NOINIT;
		}
		
		// translate the result code here

		return stat;

	case USB :
		//result = USB_disk_initialize();

		// translate the result code here
		result = RES_ERROR;
		stat = STA_NODISK;
		
		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case ATA :
		//result = ATA_disk_status();

		// translate the result code here
		
		result = RES_ERROR;
		stat = STA_NODISK;

		return stat;

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
		
		return stat;

	case USB :
		//result = USB_disk_status();

		// translate the result code here
		
		result = RES_ERROR;
		stat = STA_NODISK;

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	DRESULT res;
	int result = RES_ERROR;

	switch (pdrv) {
	case ATA :
		// translate the arguments here

		//result = ATA_disk_read(buff, sector, count);

		// translate the result code here

		if (result)
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;

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

		// translate the result code here

		return res;

	case USB :
		// translate the arguments here

		//result = USB_disk_read(buff, sector, count);
		
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



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	DRESULT res;
	int result = RES_ERROR;

	switch (pdrv) {
	case ATA :
		// translate the arguments here

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


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result = RES_ERROR;

	switch (pdrv) {
	case ATA :
		// pre-process here

		//result = ATA_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case MMC :
		// pre-process here

		//result = MMC_disk_ioctl(cmd, buff);

		print_util_dbg_print("use MMC\r");

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

		// post-process here

		return res;

	case USB :
		// pre-process here

		//result = USB_disk_ioctl(cmd, buff);

		// post-process here

		return res;
	}
	return RES_PARERR;
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
	
	uint8_t day = 13;
	uint8_t month = 8;
	uint8_t year = 34;
	
	time += seconds		& 0b00000000000000000000000000011111;
	time += minutes		& 0b00000000000000000000011111100000;
	time += hours		& 0b00000000000000001111100000000000;
	time += day			& 0b00000000000111110000000000000000;
	time += month		& 0b00000001111000000000000000000000;
	time += year		& 0b11111110000000000000000000000000;
	
	return time;
}


#endif
