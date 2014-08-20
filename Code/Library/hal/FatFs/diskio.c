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
#include "delay.h"
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

			print_util_dbg_print("NO SUPPORTED! ATA init!\r");

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

			print_util_dbg_print("NO SUPPORTED! USB init!\r");
		
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

			print_util_dbg_print("NO SUPPORTED! ATA status!\r");

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

			print_util_dbg_print("NO SUPPORTED! USB status!\r");

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

			print_util_dbg_print("NO SUPPORTED! ATA read!\r");

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
		
			print_util_dbg_print("NO SUPPORTED! USB read!\r");
		
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

			print_util_dbg_print("NO SUPPORTED! ATA write!\r");

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

			print_util_dbg_print("NO SUPPORTED! USB init!\r");

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

/**
 * \brief	A function giving the practical numbers
 *
 * \param	pns			The number of practical number
 *
 * \result	The pns^th pratical number
 */
static DWORD pn (DWORD pns);

static DWORD pn (DWORD pns)
{
	static DWORD lfsr;
	UINT n;


	if (pns)
	{
		lfsr = pns;
		for (n = 0; n < 32; n++)
		{
			pn(0);
		}
	}
	
	if (lfsr & 1)
	{
		lfsr >>= 1;
		lfsr ^= 0x80200003;
		} else {
		lfsr >>= 1;
	}
	return lfsr;
}

/**
 * \brief	To test the low layer of the disk
 *
 * \param	pdrv			Physical drive number to be checked (all data on the drive will be lost)
 * \param	ncyc			Number of test cycles
 * \param	buff			Pointer to the working buffer
 * \param	sz_buff			Size of the working buffer in unit of byte 
 */
int diskio_test (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff);

int diskio_test (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff)
{
	UINT n, cc, ns;
	DWORD sz_drv, lba, lba2, pns = 1;
	WORD sz_sect, sz_eblk;
	BYTE *pbuff = (BYTE*)buff;
	DSTATUS ds;
	DRESULT dr;



	print_util_dbg_print("test_diskio(");
	print_util_dbg_print_num(pdrv,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(ncyc,10);
	print_util_dbg_print(", 0x");
	print_util_dbg_print_num((UINT)buff,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sz_buff,10);

	if (sz_buff < _MAX_SS + 4)
	{
		print_util_dbg_print("Insufficient work area to test.\r");
		return 1;
	}

	for (cc = 1; cc <= ncyc; cc++)
	{
		print_util_dbg_print("**** Test cycle");
		print_util_dbg_print_num(cc,10);
		print_util_dbg_print(" of ");
		print_util_dbg_print_num(ncyc,10);
		print_util_dbg_print(" start ****\r");
		delay_ms(25);

		/* Initialization */
		print_util_dbg_print(" disk_initalize(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		ds = disk_initialize(pdrv);
		if (ds & STA_NOINIT)
		{
			print_util_dbg_print(" - failed.\r");
			return 2;
		}
		else
		{
			print_util_dbg_print(" - ok.\r");
		}
		delay_ms(25);

		/* Get drive size */
		print_util_dbg_print("**** Get drive size ****\r");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_SECTOR_COUNT, 0x");
		print_util_dbg_print_num((UINT)&sz_drv,10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		sz_drv = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 3;
		}
		delay_ms(25);
		
		if (sz_drv < 128)
		{
			print_util_dbg_print("Failed: Insufficient drive size to test.\r");
			return 4;
		}
		print_util_dbg_print(" Number of sectors on the drive ");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(" is ");
		print_util_dbg_print_num(sz_drv,10);
		print_util_dbg_print(".\r");
		delay_ms(25);
		
		#if _MAX_SS != _MIN_SS
		/* Get sector size */
		print_util_dbg_print("**** Get sector size ****\r");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_SECTOR_SIZE, 0x");
		print_util_dbg_print_num((UINT)&sz_sect,10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		sz_sect = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 5;
		}
		delay_ms(25);
		
		print_util_dbg_print(" Size of sector is ");
		print_util_dbg_print_num(sz_sect,10);
		print_util_dbg_print("bytes.\r");
		
		#else
		sz_sect = _MAX_SS;
		
		print_util_dbg_print(" Size of sector is ");
		print_util_dbg_print_num(sz_sect,10);
		print_util_dbg_print("bytes.\r");
		#endif

		/* Get erase block size */
		print_util_dbg_print("**** Get block size ****\r");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_BLOCK_SIZE, 0x");
		print_util_dbg_print_num((UINT)&sz_eblk,10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		sz_eblk = 0;
		dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
		}
		delay_ms(25);
		
		if (dr == RES_OK || sz_eblk >= 2)
		{
			print_util_dbg_print(" Size of the erase block is ");
			print_util_dbg_print_num(sz_eblk,10);
			print_util_dbg_print(" sectors.\r");
		}
		else
		{
			print_util_dbg_print(" Size of the erase block is unknown.\r");
		}
		delay_ms(25);

		/* Single sector write test */
		print_util_dbg_print("**** Single sector write test 1 ****\r");
		lba = 0;
		for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n] = (BYTE)pn(0);
		print_util_dbg_print(" disk_write(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)");
		delay_ms(25);
		
		dr = disk_write(pdrv, pbuff, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 6;
		}
		delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 7;
		}
		delay_ms(25);
		
		memset(pbuff, 0, sz_sect);
		print_util_dbg_print(" disk_read(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)");
		delay_ms(25);
		
		dr = disk_read(pdrv, pbuff, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 8;
		}
		delay_ms(25);
		
		for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE)pn(0); n++) ;
		if (n == sz_sect)
		{
			print_util_dbg_print(" Data matched.\r");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			return 10;
		}
		pns++;
		delay_ms(25);

		/* Multiple sector write test */
		print_util_dbg_print("**** Multiple sector write test ****\r");
		lba = 1; ns = sz_buff / sz_sect;
		
		if (ns > 4)
		{
			ns = 4;
		}
		
		for (n = 0, pn(pns); n < (UINT)(sz_sect * ns); n++)
		{
			pbuff[n] = (BYTE)pn(0);
		}
		print_util_dbg_print(" disk_write(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(ns, 10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		dr = disk_write(pdrv, pbuff, lba, ns);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 11;
		}
		delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 12;
		}
		
		memset(pbuff, 0, sz_sect * ns);
		print_util_dbg_print(" disk_read(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(ns, 10);
		print_util_dbg_print(")\r");
		delay_ms(25);
		
		dr = disk_read(pdrv, pbuff, lba, ns);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 13;
		}
		delay_ms(25);
		
		for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++) ;
		
		if (n == (UINT)(sz_sect * ns))
		{
			print_util_dbg_print(" Data matched.\r");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			return 14;
		}
		pns++;
		delay_ms(25);

		/* Single sector write test (misaligned memory address) */
		print_util_dbg_print("**** Single sector write test 2 ****\r");
		lba = 5;
		for (n = 0, pn(pns); n < sz_sect; n++)
		{
			pbuff[n+3] = (BYTE)pn(0);
		}
		print_util_dbg_print(" disk_write(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)(pbuff+3),10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)\r");
		delay_ms(25);
		
		dr = disk_write(pdrv, pbuff+3, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 15;
		}
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 16;
		}
		delay_ms(25);
		
		memset(pbuff+5, 0, sz_sect);
		print_util_dbg_print(" disk_read(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)(pbuff+5),10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)\r");
		delay_ms(25);
		
		dr = disk_read(pdrv, pbuff+5, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r");
		}
		else
		{
			print_util_dbg_print(" - failed.\r");
			return 17;
			
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (BYTE)pn(0); n++);
		
		if (n == sz_sect)
		{
			print_util_dbg_print(" Data matched.\r");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			return 18;
		}
		pns++;
		delay_ms(25);

		/* 4GB barrier test */
		print_util_dbg_print("**** 4GB barrier test ****\r");
		if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2))
		{
			lba = 6;
			lba2 = lba + 0x80000000 / (sz_sect / 2);
			
			for (n = 0, pn(pns); n < (UINT)(sz_sect * 2); n++)
			{
				pbuff[n] = (BYTE)pn(0);
			}
			
			
			print_util_dbg_print(" disk_write(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)pbuff,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba,10);
			print_util_dbg_print(",1 )\r");
			delay_ms(25);
			
			dr = disk_write(pdrv, pbuff, lba, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r");
			}
			else
			{
				print_util_dbg_print(" - failed.\r");
				return 19;
			}
			delay_ms(25);
			
			print_util_dbg_print(" disk_write(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)(pbuff+sz_sect),10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba2,10);
			print_util_dbg_print(", 1)\r");
			delay_ms(25);
			
			dr = disk_write(pdrv, pbuff+sz_sect, lba2, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r");
			}
			else
			{
				print_util_dbg_print(" - failed.\r");
				return 20;
			}
			delay_ms(25);
			
			print_util_dbg_print(" disk_ioctl(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", CTRL_SYNC, NULL)\r");
			delay_ms(25);
			
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r");
			}
			else
			{
				print_util_dbg_print(" - failed.\r");
				return 21;
			}
			delay_ms(25);
			
			memset(pbuff, 0, sz_sect * 2);
			
			print_util_dbg_print(" disk_read(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)pbuff,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba,10);
			print_util_dbg_print(", 1)\r");
			delay_ms(25);
			
			dr = disk_read(pdrv, pbuff, lba, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r");
			}
			else
			{
				print_util_dbg_print(" - failed.\r");
				return 22;
			}
			delay_ms(25);
			
			print_util_dbg_print(" disk_read(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)(pbuff+sz_sect),10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba2,10);
			print_util_dbg_print(", 1)\r");
			delay_ms(25);
			
			dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r");
			}
			else
			{
				print_util_dbg_print(" - failed.\r");
				return 23;
			}
			delay_ms(25);
			
			for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++) ;
			
			if (n == (UINT)(sz_sect * 2))
			{
				print_util_dbg_print(" Data matched.\r");
			}
			else
			{
				print_util_dbg_print("Failed: Read data differs from the data written.\r");
				return 24;
			}
		}
		else
		{
			print_util_dbg_print(" Test skipped.\r");
		}
		
		pns++;
		delay_ms(25);

		print_util_dbg_print("**** Test cycle");
		print_util_dbg_print_num(cc,10);
		print_util_dbg_print(" of ");
		print_util_dbg_print_num(ncyc,10);
		print_util_dbg_print(" completed ****\r");
		delay_ms(25);
	}

	return 0;
}

void diskio_test_low_layer(void)
{
	int rc;
	DWORD buff[512];  /* 2048 byte working buffer */
	
	/* Check function/compatibility of the physical drive #0 */
	rc = diskio_test(1, 1, buff, sizeof buff);
	if (rc)
	{
		print_util_dbg_print("Sorry the function/compatibility test failed.\rFatFs will not work on this disk driver.\r");
		print_util_dbg_print_num(rc,10);
	}
	else
	{
		print_util_dbg_print("Congratulations! The disk I/O layer works well.\r");
	}
}

FRESULT diskio_open_append (FIL* fp, const char* path)
{
	FRESULT fr;

	/* Opens an existing file. If not exist, creates a new file. */
	fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
	if (fr == FR_OK)
	{
		/* Seek to end of the file to append data */
		fr = f_lseek(fp, f_size(fp));
		if (fr != FR_OK)
		{
			f_close(fp);
		}
	}
	return fr;
}

void diskio_test_fatfs(void)
{
	FRESULT fr;
	FATFS fs;
	FIL fil;
	
	print_util_dbg_print("******** Mounting:\r");
	fr = f_mount(&fs, "", 1);
	print_util_dbg_print("Result mounting:");
	print_util_dbg_print_num(fr,10);
	print_util_dbg_print("\r");
	
	fr = diskio_open_append(&fil,"License.txt");
	print_util_dbg_print("Result opening:");
	print_util_dbg_print_num(fr,10);
	print_util_dbg_print("\r");
	
	int result = f_puts("Append data\t",&fil);
	print_util_dbg_print("Number of character written f_puts:");
	print_util_dbg_print_num(result,10);
	print_util_dbg_print("\r");
	
	result = f_printf(&fil,"f_printf append\n");
	print_util_dbg_print("Number of character written f_printf:");
	print_util_dbg_print_num(result,10);
	print_util_dbg_print("\r");
	
	fr = f_close(&fil);
	print_util_dbg_print("Result closing:");
	print_util_dbg_print_num(fr,10);
	print_util_dbg_print("\r");
	
	fr = f_mkdir("Nicolas");
	print_util_dbg_print("Result mkdir:");
	print_util_dbg_print_num(fr,10);
	print_util_dbg_print("\r");
	
	
	print_util_dbg_print("Yeah!");
}