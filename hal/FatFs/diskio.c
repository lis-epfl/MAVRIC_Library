/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "usbdisk.h"	/* Example: Header file of existing USB MSD control module */
#include "atadrive.h"	/* Example: Header file of existing ATA harddisk control module */
#include "sdcard.h"		/* Example: Header file of existing MMC/SDC contorl module */

/* Definitions of physical drive number for each drive */
#define ATA		0	/* Example: Map ATA harddisk to physical drive 0 */
#define MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case ATA :
		result = ATA_disk_status();

		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NODISK;
		}

		return stat;

	case MMC :
		result = MMC_disk_status();

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
		result = USB_disk_status();

		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NODISK;
		}

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case ATA :
		result = ATA_disk_initialize();

		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NOINIT;
		}

		return stat;

	case MMC :
		result = MMC_disk_initialize();

		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NOINIT;
		}

		return stat;

	case USB :
		result = USB_disk_initialize();

		if (result)
		{
			stat = 0;
		}
		else
		{
			stat = STA_NOINIT;
		}

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case ATA :
		result = ATA_disk_read(buff, sector, count);

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
		result = MMC_disk_read(buff, sector, count);

		if (result)
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;

	case USB :
		result = USB_disk_read(buff, sector, count);

		if (result)
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case ATA :
		result = ATA_disk_write(buff, sector, count);

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
		result = MMC_disk_write(buff, sector, count);

		if (result)
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;

	case USB :
		result = USB_disk_write(buff, sector, count);

		if (result)
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

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
	int result;

	switch (pdrv) {
	case ATA :

		switch(cmd)
		{
			case CTRL_SYNC:
				result = ATA_ctrl_sync();
			break;
		
			case GET_SECTOR_COUNT:
				result = ATA_get_sector_count(buff);
			break;
		
			case GET_SECTOR_SIZE:
				result = ATA_get_sector_size(buff);
			break;
		
			case GET_BLOCK_SIZE:
				result = ATA_get_block_size(buff);
			break;
		
			case CTRL_TRIM:
				result = ATA_ctrl_trim(buff);
			break;

			case ATA_GET_REV:
				result = ATA_get_rev(buff);
			break;

			case ATA_GET_MODEL:
				result = ATA_get_model(buff);
			break;

			case ATA_GET_SN:
				result = ATA_get_sn(buff);
			break;
		}

		if( result )
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;

	case MMC :

		switch(cmd)
		{
			case CTRL_SYNC:
				result = MMC_ctrl_sync();
			break;
		
			case GET_SECTOR_COUNT:
				result = MMC_get_sector_count(buff);
			break;
		
			case GET_SECTOR_SIZE:
				result = MMC_get_sector_size(buff);
			break;
		
			case GET_BLOCK_SIZE:
				result = MMC_get_block_size(buff);
			break;
		
			case CTRL_TRIM:
				result = MMC_ctrl_trim(buff);
			break;

			case MMC_GET_TYPE:	
				result = MMC_get_type(buff);
			break;

			case MMC_GET_CSD:		
				result = MMC_get_csd(buff);
			break;

			case MMC_GET_CID:		
				result = MMC_get_cid(buff);
			break;

			case MMC_GET_OCR:		
				result = MMC_get_ocr(buff);
			break;

			case MMC_GET_SDSTAT:
				result = MMC_get_sdstat(buff);
			break;
		}

		if( result )
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;

	case USB :

		switch(cmd)
		{
			case CTRL_SYNC:
				result = USB_ctrl_sync();
			break;
		
			case GET_SECTOR_COUNT:
				result = USB_get_sector_count(buff);
			break;
		
			case GET_SECTOR_SIZE:
				result = USB_get_sector_size(buff);
			break;
		
			case GET_BLOCK_SIZE:
				result = USB_get_block_size(buff);
			break;
		
			case CTRL_TRIM:
				result = USB_ctrl_trim(buff);
			break;
		}

		if( result )
		{
			res = RES_OK;
		}
		else
		{
			res = RES_ERROR;
		}

		return res;
	}



	return RES_PARERR;
}
#endif