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
 * \file sd_spi.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file is the sd card driver using SPI
 * 
 * \detail It has been implemented, using the SD/MMC using SPI Example for 
 * EVK1100 Documentation from ATMEL
 * (http://asf.atmel.com/docs/2.9.0/avr32.components.memory.sdmmc.spi.example.evk1100/html/index.html)
 * you can call sd_spi_test(&central_data->sd_spi); from main.cpp before 
 * creating task to try the functions
 * 
 ******************************************************************************/
 

#ifndef SD_SPI_H_
#define SD_SPI_H_

#include "compiler.h"
#include "user_board.h"

//Linked with SPI usage
#define SPI_LOW_SPEED					400000
#define SPI_HIGH_SPEED					6000000
#define SD_MMC_SPI_BITS					8

//linked with sd card
#define MMC_SECTOR_SIZE                   512   //default sector size is 512 bytes

// Card identification
#define MMC_CARD                          0
#define SD_CARD                           1
#define SD_CARD_2                         2
#define SD_CARD_2_SDHC                    3

/* status bits for card types */
#define SD_SPEC_1 0
#define SD_SPEC_2 1
#define SD_SPEC_SDHC 2


// Lock operations
#define OP_UNLOCK                         0x00
#define OP_LOCK                           0x04
#define OP_RESET_PWD                      0x02
#define OP_SET_PWD                        0x01
#define OP_FORCED_ERASE                   0x08

// MMC commands (taken from MMC reference)
#define MMC_GO_IDLE_STATE                 0     ///< initialize card to SPI-type access
#define MMC_SEND_OP_COND                  1     ///< set card operational mode
#define MMC_CMD2                          2     ///< illegal in SPI mode !
#define MMC_SEND_IF_COND                  8
#define MMC_SEND_CSD                      9     ///< get card's CSD
#define MMC_SEND_CID                      10    ///< get card's CID
#define MMC_SEND_STATUS                   13
#define MMC_SET_BLOCKLEN                  16    ///< Set number of bytes to transfer per block
#define MMC_READ_SINGLE_BLOCK             17    ///< read a block
#define MMC_WRITE_BLOCK                   24    ///< write a block
#define MMC_PROGRAM_CSD                   27
#define MMC_SET_WRITE_PROT                28
#define MMC_CLR_WRITE_PROT                29
#define MMC_SEND_WRITE_PROT               30
#define SD_TAG_WR_ERASE_GROUP_START       32
#define SD_TAG_WR_ERASE_GROUP_END         33
#define MMC_TAG_SECTOR_START              32
#define MMC_TAG_SECTOR_END                33
#define MMC_UNTAG_SECTOR                  34
#define MMC_TAG_ERASE_GROUP_START         35    ///< Sets beginning of erase group (mass erase)
#define MMC_TAG_ERASE_GROUP_END           36    ///< Sets end of erase group (mass erase)
#define MMC_UNTAG_ERASE_GROUP             37    ///< Untag (unset) erase group (mass erase)
#define MMC_ERASE                         38    ///< Perform block/mass erase
#define SD_SEND_OP_COND_ACMD              41              ///< Same as MMC_SEND_OP_COND but specific to SD (must be preceeded by CMD55)
#define MMC_LOCK_UNLOCK                   42              ///< To start a lock/unlock/pwd operation
#define SD_APP_CMD55                      55              ///< Use before any specific command (type ACMD)
#define SD_READ_OCR                       58
#define MMC_CRC_ON_OFF                    59    ///< Turns CRC check on/off
// R1 Response bit-defines
#define MMC_R1_BUSY                       0x80  ///< R1 response: bit indicates card is busy
#define MMC_R1_PARAMETER                  0x40
#define MMC_R1_ADDRESS                    0x20
#define MMC_R1_ERASE_SEQ                  0x10
#define MMC_R1_COM_CRC                    0x08
#define MMC_R1_ILLEGAL_COM                0x04
#define MMC_R1_ERASE_RESET                0x02
#define MMC_R1_IDLE_STATE                 0x01
// Data Start tokens
#define MMC_STARTBLOCK_READ               0xFE  ///< when received from card, indicates that a block of data will follow
#define MMC_STARTBLOCK_WRITE              0xFE  ///< when sent to card, indicates that a block of data will follow
#define MMC_STARTBLOCK_MWRITE             0xFC
// Data Stop tokens
#define MMC_STOPTRAN_WRITE                0xFD
// Data Error Token values
#define MMC_DE_MASK                       0x1F
#define MMC_DE_ERROR                      0x01
#define MMC_DE_CC_ERROR                   0x02
#define MMC_DE_ECC_FAIL                   0x04
#define MMC_DE_OUT_OF_RANGE               0x04
#define MMC_DE_CARD_LOCKED                0x04
// Data Response Token values
#define MMC_DR_MASK                       0x1F
#define MMC_DR_ACCEPT                     0x05
#define MMC_DR_REJECT_CRC                 0x0B
#define MMC_DR_REJECT_WRITE_ERROR         0x0D

#define SDHC_CARD                         1
#define SD_CARD_T                         0

#define SD_FAILURE                       -1
#define SD_MMC                            0

typedef struct  
{
	bool				init_done;
	uint8_t				csd[16];                    // stores the Card Specific Data
	uint8_t				card_type;                   // stores SD_CARD or MMC_CARD type card
	uint64_t			capacity;                   // stores the capacity in bytes
	uint16_t			capacity_mult;
	uint32_t			clock;
	uint32_t			last_block_address;
	uint16_t			erase_group_size;
}sd_spi_t;

//Functions definitions
bool sd_spi_mem_check(void);
void sd_spi_get_capacity(void);
bool sd_spi_reset_card(void);
void sd_spi_get_sector_count(void* res);
void sd_spi_get_sector_size(void* res);
void sd_spi_get_block_size(void* res);
bool is_sd_spi_write_pwd_locked(void);
bool sd_spi_init(void);
bool sd_spi_status(void);

bool sd_spi_wait_not_busy(void);

bool sd_spi_write_sector_from_ram(const void *ram);
bool sd_spi_read_sector_to_ram_pcda(void* ram, uint32_t beginning_sector, uint8_t count);
bool sd_mmc_spi_read_sector_to_ram(void *ram);
bool sd_spi_read_given_sector_to_ram(void* ram, uint32_t beginning_sector, uint8_t count);
bool sd_mmc_spi_read_open_PDCA (uint32_t pos);
bool sd_spi_write_given_sector_from_ram(const void *ram, uint32_t beginning_sector, uint8_t count);
void sd_spi_test(void);

#endif // SD_SPI_H_