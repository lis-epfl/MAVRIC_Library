/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */

extern "C" {
	#include "led.h"
	#include "delay.h"
	#include "print_util.h"
	#include "central_data.h"
	#include "boardsupport.h"
	#include "tasks.h"
	#include "mavlink_telemetry.h"
	#include "piezo_speaker.h"
	
	#include "gpio.h"
	#include "spi.h"
	#include "sd_spi.h"
	
	#include "FatFs/ff.h"
	#include "FatFs/diskio.h"
}
 
central_data_t *central_data;

void test_sd_spi()
{
	char write_data[] = "0123456789xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxedcba";
	 //"0123456789abcde";
	char buffer[1025];

	while (!sd_spi_mem_check())
	{
		print_util_dbg_print("No card detected yet!\r");
		delay_ms(15);
	}
	
	print_util_dbg_print("\rCard detected!");
	
	delay_ms(500);
	if(sd_spi_write_given_sector_from_ram((void*)&write_data,2,2))
	{
		print_util_dbg_print("Write in sector 1\r");
	}
	
	delay_ms(500);
	if(sd_spi_write_given_sector_from_ram((void*)&write_data,5,2))
	{
		print_util_dbg_print("Write in sector 5\r");
	}
	
	delay_ms(500);
	if (sd_spi_read_given_sector_to_ram((void*)&buffer,1,2))
	{
		print_util_dbg_print("Read in sector 1");
		print_util_dbg_print_num(sizeof(write_data),10);
		print_util_dbg_print("\r");
		for (uint16_t i = 0;i<sizeof(write_data); i++)
		{
			print_util_dbg_print_num((U8)*(buffer + i), 10);
			delay_ms(25);
		}
	}
	else
	{
		print_util_dbg_print("Read error in sector 1\r");
	}
	
	delay_ms(500);
	if (sd_spi_read_given_sector_to_ram((void*)&buffer,5,2))
	{
		print_util_dbg_print("Read in sector 5, size of ");
		print_util_dbg_print_num(sizeof(write_data),10);
		print_util_dbg_print("\r");
		for (uint16_t i = 0;i<sizeof(write_data); i++)
		{
			print_util_dbg_print_num((U8)*(buffer + i), 10);
			delay_ms(25);
		}
	}
	else
	{
		print_util_dbg_print("Read error in sector 5\r");
	}
	
	//delay_ms(500);
	//if (sd_spi_read_sector_to_ram_pcda((void*)&buffer,1,1))
	//{
		//print_util_dbg_print("Read pcda in sector 1");
		//print_util_dbg_print_num(sizeof(write_data),10);
		//print_util_dbg_print("\r");
		//for (uint16_t i = 0;i<sizeof(write_data); i++)
		//{
			//print_util_dbg_print_num((U8)*(buffer + i), 10);
			//delay_ms(25);
		//}
	//}
	//else
	//{
		//print_util_dbg_print("Read pcda error in sector 1\r");
	//}
	//
	//delay_ms(500);
	//if (sd_spi_read_sector_to_ram_pcda((void*)&buffer,5,1))
	//{
		//print_util_dbg_print("Read pcda in sector 5");
		//print_util_dbg_print_num(sizeof(write_data),10);
		//print_util_dbg_print("\r");
		//for (uint16_t i = 0;i<sizeof(write_data); i++)
		//{
			//print_util_dbg_print_num((U8)*(buffer + i), 10);
			//delay_ms(25);
		//}
	//}
	//else
	//{
		//print_util_dbg_print("Read pcda error in sector 5\r");
	//}
	
	print_util_dbg_print("End test\r");
	delay_ms(25);
}

FRESULT open_append (
FIL* fp,            /* [OUT] File object to create */
const char* path    /* [IN]  File name to be opened */
)
{
	FRESULT fr;

	/* Opens an existing file. If not exist, creates a new file. */
	fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
	if (fr == FR_OK) {
		/* Seek to end of the file to append data */
		fr = f_lseek(fp, f_size(fp));
		if (fr != FR_OK)
		f_close(fp);
	}
	return fr;
}

static
DWORD pn (
DWORD pns
)
{
	static DWORD lfsr;
	UINT n;


	if (pns) {
		lfsr = pns;
		for (n = 0; n < 32; n++) pn(0);
	}
	if (lfsr & 1) {
		lfsr >>= 1;
		lfsr ^= 0x80200003;
		} else {
		lfsr >>= 1;
	}
	return lfsr;
}


int test_diskio (
BYTE pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
UINT ncyc,      /* Number of test cycles */
DWORD* buff,    /* Pointer to the working buffer */
UINT sz_buff    /* Size of the working buffer in unit of byte */
)
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

	if (sz_buff < _MAX_SS + 4) {
		print_util_dbg_print("Insufficient work area to test.\r");
		return 1;
	}

	for (cc = 1; cc <= ncyc; cc++) {
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
		if (ds & STA_NOINIT) {
			print_util_dbg_print(" - failed.\r");
			return 2;
		} else {
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 3;
		}
		delay_ms(25);
		
		if (sz_drv < 128) {
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 5;
		}
		delay_ms(25);
		
		print_util_dbg_print(" Size of sector is ");
		print_util_dbg_print_num(sz_sect,10); 
		print_util_dbg_print("bytes.\r");
		
		#else
		sz_sect = _MAX_SS;
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
		}
		delay_ms(25);
		
		if (dr == RES_OK || sz_eblk >= 2) {
			print_util_dbg_print(" Size of the erase block is ");
			print_util_dbg_print_num(sz_eblk,10);
			print_util_dbg_print(" sectors.\r");
		} else {
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 6;
		}
		delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 8;
		}
		delay_ms(25);
		
		for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE)pn(0); n++) ;
		if (n == sz_sect) {
			print_util_dbg_print(" Data matched.\r");
		} else {
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			return 10;
		}
		pns++;
		delay_ms(25);

		/* Multiple sector write test */
		print_util_dbg_print("**** Multiple sector write test ****\r");
		lba = 1; ns = sz_buff / sz_sect;
		if (ns > 4) ns = 4;
		for (n = 0, pn(pns); n < (UINT)(sz_sect * ns); n++) pbuff[n] = (BYTE)pn(0);
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 11;
		}
		delay_ms(25);
		
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
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
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 13;
		}
		delay_ms(25);
		
		for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++) ;
		if (n == (UINT)(sz_sect * ns)) {
			print_util_dbg_print(" Data matched.\r");
		} else {
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			return 14;
		}
		pns++;
		delay_ms(25);

		/* Single sector write test (misaligned memory address) */
		print_util_dbg_print("**** Single sector write test 2 ****\r");
		lba = 5;
		for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n+3] = (BYTE)pn(0);
		print_util_dbg_print(" disk_write(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", 0x");
		print_util_dbg_print_num((UINT)(pbuff+3),10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(lba,10);
		print_util_dbg_print(", 1)\r");
		delay_ms(25);
		
		dr = disk_write(pdrv, pbuff+3, lba, 1);
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 15;
		}
		print_util_dbg_print(" disk_ioctl(");
		print_util_dbg_print_num(pdrv,10);
		print_util_dbg_print(", CTRL_SYNC, NULL)");
		delay_ms(25);
		
		dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
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
		
		dr = disk_write(pdrv, pbuff+3, lba, 1);
		dr = disk_read(pdrv, pbuff+5, lba, 1);
		if (dr == RES_OK) {
			print_util_dbg_print(" - ok.\r");
		} else {
			print_util_dbg_print(" - failed.\r");
			return 17;
		delay_ms(25);
		
		}
		for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (BYTE)pn(0); n++) ;
		if (n == sz_sect) {
			print_util_dbg_print(" Data matched.\r");
			print_util_dbg_print_num(n,10);
			print_util_dbg_print_num(sz_sect,10);
		} else {
			print_util_dbg_print("Failed: Read data differs from the data written.\r");
			print_util_dbg_print_num(n,10);
			print_util_dbg_print_num(sz_sect,10);
			return 18;
		}
		pns++;
		delay_ms(25);

		/* 4GB barrier test */
		print_util_dbg_print("**** 4GB barrier test ****\r");
		if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
			lba = 6; lba2 = lba + 0x80000000 / (sz_sect / 2);
			for (n = 0, pn(pns); n < (UINT)(sz_sect * 2); n++) pbuff[n] = (BYTE)pn(0);
			print_util_dbg_print(" disk_write(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", 0x");
			print_util_dbg_print_num((UINT)pbuff,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(lba,10);
			print_util_dbg_print(")\r");
			delay_ms(25);
			
			dr = disk_write(pdrv, pbuff, lba, 1);
			if (dr == RES_OK) {
				print_util_dbg_print(" - ok.\r");
			} else {
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
			if (dr == RES_OK) {
				print_util_dbg_print(" - ok.\r");
			} else {
				print_util_dbg_print(" - failed.\r");
				return 20;
			}
			delay_ms(25);
			
			print_util_dbg_print(" disk_ioctl(");
			print_util_dbg_print_num(pdrv,10);
			print_util_dbg_print(", CTRL_SYNC, NULL)");
			delay_ms(25);
			
			dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
			if (dr == RES_OK) {
				print_util_dbg_print(" - ok.\r");
			} else {
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
			if (dr == RES_OK) {
				print_util_dbg_print(" - ok.\r");
			} else {
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
			print_util_dbg_print(")\r");
			delay_ms(25);
			
			dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
			if (dr == RES_OK) {
				print_util_dbg_print(" - ok.\r");
			} else {
				print_util_dbg_print(" - failed.\r");
				return 23;
			}
			delay_ms(25);
			
			for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++) ;
			if (n == (UINT)(sz_sect * 2)) {
				print_util_dbg_print(" Data matched.\r");
			} else {
				print_util_dbg_print("Failed: Read data differs from the data written.\r");
				return 24;
			}
		} else {
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

void test_fatfs()
{
	int rc;
	DWORD buff[512];  /* 2048 byte working buffer */
	
	/* Check function/compatibility of the physical drive #0 */
	rc = test_diskio(1, 1, buff, sizeof buff);
	if (rc) {
		print_util_dbg_print("Sorry the function/compatibility test failed.\rFatFs will not work on this disk driver.\r");
		print_util_dbg_print_num(rc,10);
	} else {
		print_util_dbg_print("Congratulations! The disk I/O layer works well.\r");
	}
	
	
	FRESULT fr;
	FATFS fs;
	FIL fil;
	
	fr = f_mount(&fs, "", 0);
	print_util_dbg_print("Result mounting:");
	print_util_dbg_print_num(fr,10);
	print_util_dbg_print("\r");
	
	
	fr = open_append(&fil,"License.txt");
	if (fr == FR_OK)
	{
		print_util_dbg_print("Opening okay!\r");
	}
	else
	{
		print_util_dbg_print("Opening not okay... Error number:");
		print_util_dbg_print_num(fr,10);
		print_util_dbg_print("\r");
	}
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

void initialisation() 
{	
	central_data = central_data_get_pointer_to_struct();
	boardsupport_init(central_data);
	central_data_init();

	mavlink_telemetry_init();
	onboard_parameters_read_parameters_from_flashc(&central_data->mavlink_communication.onboard_parameters);

	//sd_spi_test(&central_data->sd_spi);


	LED_On(LED1);

	piezo_speaker_startup_melody();

	print_util_dbg_print("OK. Starting up.\r");

	//sd_spi_test();

	//test_sd_spi();
	
	test_fatfs();

	central_data->state.mav_state = MAV_STATE_STANDBY;

	central_data->imu.calibration_level = OFF;
}

int main (void)
{
	initialisation();
	tasks_create_tasks();
	
	while (1 == 1) 
	{
		scheduler_update(&central_data->scheduler);
	}

	return 0;
}