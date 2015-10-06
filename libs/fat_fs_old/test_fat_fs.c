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
 * \file test_fat_fs.c
 *
 * This file can be used to test the low and mid layer of writing to SD card for fat fs use. 
 */


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
int test_fat_fs_test (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff);

int test_fat_fs_test(BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff)
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
		print_util_dbg_print("Insufficient work area to test.\r\n");
		return 1;
	}

	for (cc = 1; cc <= ncyc; cc++)
	{
		print_util_dbg_print("**** Test cycle");
		print_util_dbg_print_num(cc,10);
		print_util_dbg_print(" of ");
		print_util_dbg_print_num(ncyc,10);
		print_util_dbg_print(" start ****\r\n");
		time_keeper_delay_ms(25);

		/* Initialization */
		print_util_dbg_print(" disk_initalize(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		ds = disk_initialize(pdrv);
		if (ds & STA_NOINIT)
		{
			print_util_dbg_print(" - failed.\r\n");
			return 2;
		}
		else
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		time_keeper_delay_ms(25);

		/* Get drive size */
		print_util_dbg_print("**** Get drive size ****\r\n");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_SECTOR_COUNT, 0x");
		print_util_dbg_print_num((UINT)&sz_drv,10);
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		sz_drv = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 3;
		}
		time_keeper_delay_ms(25);
		
		if (sz_drv < 128)
		{
			print_util_dbg_print("Failed: Insufficient drive size to test.\r\n");
			return 4;
		}
		print_util_dbg_print(" Number of sectors on the drive ");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(" is ");
		print_util_dbg_print_num(sz_drv,10);
		print_util_dbg_print(".\r\n");
		time_keeper_delay_ms(25);
		
		#if _MAX_SS != _MIN_SS
		/* Get sector size */
		print_util_dbg_print("**** Get sector size ****\r\n");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_SECTOR_SIZE, 0x");
		print_util_dbg_print_num((UINT)&sz_sect,10);
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		sz_sect = 0;
		dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 5;
		}
		time_keeper_delay_ms(25);
		
		print_util_dbg_print(" Size of sector is ");
		print_util_dbg_print_num(sz_sect,10);
		print_util_dbg_print("bytes.\r\n");
		
		#else
		sz_sect = _MAX_SS;
		
		print_util_dbg_print(" Size of sector is ");
		print_util_dbg_print_num(sz_sect,10);
		print_util_dbg_print("bytes.\r\n");
		#endif

		/* Get erase block size */
		print_util_dbg_print("**** Get block size ****\r\n");
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", GET_BLOCK_SIZE, 0x");
		print_util_dbg_print_num((UINT)&sz_eblk,10);
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		sz_eblk = 0;
		dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
		}
		time_keeper_delay_ms(25);
		
		if (dr == RES_OK || sz_eblk >= 2)
		{
			print_util_dbg_print(" Size of the erase block is ");
			print_util_dbg_print_num(sz_eblk,10);
			print_util_dbg_print(" sectors.\r\n");
		}
		else
		{
			print_util_dbg_print(" Size of the erase block is unknown.\r\n");
		}
		time_keeper_delay_ms(25);

		/* Single sector write test */
		print_util_dbg_print("**** Single sector write test 1 ****\r\n");
		lba = 0;
		for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n] = (BYTE)pn(0);
		print_util_dbg_print(" disk_write(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)");
		time_keeper_delay_ms(25);
		
		dr = disk_write(pdrv, pbuff, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 6;
		}
		time_keeper_delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 7;
		}
		time_keeper_delay_ms(25);
		
		memset(pbuff, 0, sz_sect);
		print_util_dbg_print(" disk_read(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)pbuff,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)");
		time_keeper_delay_ms(25);
		
		dr = disk_read(pdrv, pbuff, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 8;
		}
		time_keeper_delay_ms(25);
		
		for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE)pn(0); n++) ;
		if (n == sz_sect)
		{
			print_util_dbg_print(" Data matched.\r\n");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r\n");
			return 10;
		}
		pns++;
		time_keeper_delay_ms(25);

		/* Multiple sector write test */
		print_util_dbg_print("**** Multiple sector write test ****\r\n");
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
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_write(pdrv, pbuff, lba, ns);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 11;
		}
		time_keeper_delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
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
		print_util_dbg_print(")\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_read(pdrv, pbuff, lba, ns);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 13;
		}
		time_keeper_delay_ms(25);
		
		for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++) ;
		
		if (n == (UINT)(sz_sect * ns))
		{
			print_util_dbg_print(" Data matched.\r\n");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r\n");
			return 14;
		}
		pns++;
		time_keeper_delay_ms(25);

		/* Single sector write test (misaligned memory address) */
		print_util_dbg_print("**** Single sector write test 2 ****\r\n");
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
		print_util_dbg_print(", 1)\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_write(pdrv, pbuff+3, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 15;
		}
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 16;
		}
		time_keeper_delay_ms(25);
		
		memset(pbuff+5, 0, sz_sect);
		print_util_dbg_print(" disk_read(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)(pbuff+5),10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)\r\n");
		time_keeper_delay_ms(25);
		
		dr = disk_read(pdrv, pbuff+5, lba, 1);
		if (dr == RES_OK)
		{
			print_util_dbg_print(" - ok.\r\n");
		}
		else
		{
			print_util_dbg_print(" - failed.\r\n");
			return 17;
			
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (BYTE)pn(0); n++);
		
		if (n == sz_sect)
		{
			print_util_dbg_print(" Data matched.\r\n");
		}
		else
		{
			print_util_dbg_print("Failed: Read data differs from the data written.\r\n");
			return 18;
		}
		pns++;
		time_keeper_delay_ms(25);

		/* 4GB barrier test */
		print_util_dbg_print("**** 4GB barrier test ****\r\n");
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
			print_util_dbg_print(",1 )\r\n");
			time_keeper_delay_ms(25);
			
			dr = disk_write(pdrv, pbuff, lba, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r\n");
			}
			else
			{
				print_util_dbg_print(" - failed.\r\n");
				return 19;
			}
			time_keeper_delay_ms(25);
			
			print_util_dbg_print(" disk_write(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)(pbuff+sz_sect),10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba2,10);
			print_util_dbg_print(", 1)\r\n");
			time_keeper_delay_ms(25);
			
			dr = disk_write(pdrv, pbuff+sz_sect, lba2, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r\n");
			}
			else
			{
				print_util_dbg_print(" - failed.\r\n");
				return 20;
			}
			time_keeper_delay_ms(25);
			
			print_util_dbg_print(" disk_ioctl(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", CTRL_SYNC, NULL)\r\n");
			time_keeper_delay_ms(25);
			
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r\n");
			}
			else
			{
				print_util_dbg_print(" - failed.\r\n");
				return 21;
			}
			time_keeper_delay_ms(25);
			
			memset(pbuff, 0, sz_sect * 2);
			
			print_util_dbg_print(" disk_read(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)pbuff,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba,10);
			print_util_dbg_print(", 1)\r\n");
			time_keeper_delay_ms(25);
			
			dr = disk_read(pdrv, pbuff, lba, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r\n");
			}
			else
			{
				print_util_dbg_print(" - failed.\r\n");
				return 22;
			}
			time_keeper_delay_ms(25);
			
			print_util_dbg_print(" disk_read(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)(pbuff+sz_sect),10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba2,10);
			print_util_dbg_print(", 1)\r\n");
			time_keeper_delay_ms(25);
			
			dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
			if (dr == RES_OK)
			{
				print_util_dbg_print(" - ok.\r\n");
			}
			else
			{
				print_util_dbg_print(" - failed.\r\n");
				return 23;
			}
			time_keeper_delay_ms(25);
			
			for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++) ;
			
			if (n == (UINT)(sz_sect * 2))
			{
				print_util_dbg_print(" Data matched.\r\n");
			}
			else
			{
				print_util_dbg_print("Failed: Read data differs from the data written.\r\n");
				return 24;
			}
		}
		else
		{
			print_util_dbg_print(" Test skipped.\r\n");
		}
		
		pns++;
		time_keeper_delay_ms(25);

		print_util_dbg_print("**** Test cycle");
		print_util_dbg_print_num(cc,10);
		print_util_dbg_print(" of ");
		print_util_dbg_print_num(ncyc,10);
		print_util_dbg_print(" completed ****\r\n");
		time_keeper_delay_ms(25);
	}

	return 0;
}

void test_fat_fs_low_layer(void)
{
	int rc;
	DWORD buff[512];  /* 2048 byte working buffer */
	
	/* Check function/compatibility of the physical drive #0 */
	rc = test_fat_fs_test(1, 1, buff, sizeof buff);
	if (rc)
	{
		print_util_dbg_print("Sorry the function/compatibility test failed.\rFatFs will not work on this disk driver.\r\n");
		print_util_dbg_print_num(rc,10);
	}
	else
	{
		print_util_dbg_print("Congratulations! The disk I/O layer works well.\r\n");
	}
}

FRESULT test_fat_fs_open_append (FIL* fp, const char* path, BYTE opening_opt)
{
	FRESULT fr;

	/* Opens an existing file. If not exist, creates a new file. */
	fr = f_open(fp, path, opening_opt);
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