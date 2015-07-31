/*-----------------------------------------------------------------------/
/  Low level disk interface module include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

/**
 * \file diskio.h
 *
 *  Link the FatFs level with the low level command of the disk
 */


#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#define _USE_WRITE	1							///< 1: Enable disk_write function
#define _USE_IOCTL	1							///< 1: Enable disk_ioctl function

#include "integer.h"
#include "ff.h"


typedef BYTE	DSTATUS;						///< Status of Disk Functions

/** 
 * \brief	Results of Disk Functions 
 */
typedef enum 
{
	RES_OK = 0,									///< 0: Successful
	RES_ERROR,									///< 1: R/W Error
	RES_WRPRT,									///< 2: Write Protected
	RES_NOTRDY,									///< 3: Not Ready
	RES_PARERR									///< 4: Invalid Parameter
} DRESULT;

///< Disk Status Bits (DSTATUS)
#define STA_NOINIT		0x01					///< Drive not initialized
#define STA_NODISK		0x02					///< No medium in the drive
#define STA_PROTECT		0x04					///< Write protected


///< Command code for disk_ioctrl fucntion

///< Generic command (used by FatFs)
#define CTRL_SYNC			0					///< Flush disk cache (for write functions)
#define GET_SECTOR_COUNT	1					///< Get media size (for only f_mkfs())
#define GET_SECTOR_SIZE		2					///< Get sector size (for multiple sector size (_MAX_SS >= 1024))
#define GET_BLOCK_SIZE		3					///< Get erase block size (for only f_mkfs())
#define CTRL_ERASE_SECTOR	4					///< Force erased a block of sectors (for only _USE_ERASE)

///< Generic command (not used by FatFs)
#define CTRL_POWER			5					///< Get/Set power status
#define CTRL_LOCK			6					///< Lock/Unlock media removal
#define CTRL_EJECT			7					///< Eject media
#define CTRL_FORMAT			8					///< Create physical format on the media

///< MMC/SDC specific ioctl command
#define MMC_GET_TYPE		10					///< Get card type
#define MMC_GET_CSD			11					///< Get CSD
#define MMC_GET_CID			12					///< Get CID
#define MMC_GET_OCR			13					///< Get OCR
#define MMC_GET_SDSTAT		14					///< Get SD status

///< ATA/CF specific ioctl command
#define ATA_GET_REV			20					///< Get F/W revision
#define ATA_GET_MODEL		21					///< Get model name
#define ATA_GET_SN			22					///< Get serial number

/** 
 * \brief	Initialise the disk
 * 
 * \param	pdrv		The drive number (0..)
 *
 * \return	A DSTATUS value => STA_ #define
 */
DSTATUS disk_initialize (BYTE pdrv);

/** 
 * \brief	Get Disk Status
 * 
 * \param	pdrv		The drive number (0..)
 *
 * \return	A DSTATUS value => STA_ #define
 */
DSTATUS disk_status (BYTE pdrv);

/** 
 * \brief	Read a sector from the device and write to buff
 * 
 * \param	pdrv		The drive number (0..)
 * \param	buff		The data buffer to store read data
 * \param	sector		The start sector address to be read (LBA)
 * \param	count		The number of sectors to be read
 *
 * \result	The result of the read process, a DRESULT enum value 
 */
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);

/** 
 * \brief	Write the value of buff on the sector
 * 
 * \param	pdrv		The drive number (0..)
 * \param	buff		The pointer to the buffer to take the data to write from
 * \param	sector		The start sector on which it will be written (LBA)
 * \param	count		The number of sectors on which it will be written
 *
 * \result	The result of the write process, a DRESULT enum value 
 */
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);

/** 
 * \brief	Miscellaneous Functions, function to get information from the device
 * 
 * \param	pdrv		The drive number (0..)
 * \param	cmd			The command of the feature
 * \param	buff		The pointer to the buffer on which the result will be written
 *
 * \result	The result of the get information process 
 */
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

/** 
 * \brief	The time to write the metadata of the files
 *
 * \result	The time under a bit mask form
 */
DWORD get_fattime(void);

#ifdef __cplusplus
}
#endif

#endif
