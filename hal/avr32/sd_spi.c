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
 * \file sd_spi.c
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


#include "sd_spi.h"
#include "spi.h"
#include "gpio.h"
#include "pdca.h"
#include "time_keeper.h"

#include "led.h"
#include "print_util.h"

#include "user_board.h"

//to remove
static uint32_t  gl_ptr_mem;						///< Memory data pointer
sd_spi_t sd_spi;									///< Declare de SD_Spi object

#define PBA_HZ                16000000				///< frequency
#define BUFFERSIZE            64					///< size of the buffer

#define AVR32_PDCA_CHANNEL_USED_RX AVR32_PDCA_PID_SPI1_RX
#define AVR32_PDCA_CHANNEL_USED_TX AVR32_PDCA_PID_SPI1_TX

#define AVR32_PDCA_CHANNEL_SPI_RX 0 // In the example we will use the pdca channel 0.
#define AVR32_PDCA_CHANNEL_SPI_TX 1 // In the example we will use the pdca channel 1.


// PDCA Channel pointer
volatile avr32_pdca_channel_t* pdca_channelrx ;		///< PDCA Rx channel pointer
volatile avr32_pdca_channel_t* pdca_channeltx ;		///< PDCA Tx channel pointer

volatile bool end_of_transfer;						///< Used to indicate the end of PDCA transfer

// Local RAM buffer for the example to store data received from the SD/MMC card
volatile char ram_buffer[1000];						///< Local RAM buffer
volatile char ram_buffer2[1000];					///< Local RAM buffer2

// Dummy char table
const char dummy_data[] = "0123456789xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

// Prototype functions
/**
 * \brief Initialize PDCA
 */
static void local_pdca_init(void);
/**
 * \brief Check presence of the SD card
 *
 * \return	Returns true if SD card present
 */
static bool sd_spi_check_presence(void);
/**
 * \brief Check whether the card is high capacity
 *
 * \return	Returns true if SD card is high capactiy
 */
static int sd_spi_check_hc(void);
/**
 * \brief Returns SD card type
 *
 * \return	Returns SD card type
 */
static int sd_spi_get_card_type(void);

/**
 * \brief Returns SD card status
 *
 * \param	r2		Pointer to store the status
 *
 * \return	Returns SD card status
 */
static bool sd_spi_get_status(uint16_t *r2);
/**
 * \brief Send command
 *
 * \param	command		Command to send
 * \param	arg			argument
 *
 * \return	Returns true if succeed
 */
static uint8_t sd_spi_send_command(uint8_t command, uint32_t arg); 
/**
 * \brief Proccess command
 *
 * \param	command		Command to send
 * \param	arg			argument
 *
 * \return	Returns true if succeed
 */
static uint8_t sd_spi_command(uint8_t command, uint32_t arg);
/**
 * \brief Send and Read
 *
 * \param	data_to_send	Byte to send
 *
 * \return	Returns true if succeed
 */
static uint8_t sd_spi_send_and_read(uint8_t data_to_send);
/**
 * \brief SD card resources initialization
 *
 * \param spiOptions	Spi options for the resources initialization
 */
static void sd_spi_resources_init(spi_options_t spiOptions);
/**
 * \brief Get SD card CSD
 *
 * \param	buffer		Pointer to store de CSD 
 *
 * \return	Returns ture if succeed
 */
static bool sd_spi_get_csd(uint8_t *buffer);
#if (defined SD_MMC_READ_CID) && (SD_MMC_READ_CID == true)
	/**
	* \brief Get SD card CID
	*
	* \param	buffer		Pointer to store de CID
	*
	* \return	Returns true is succeed
	*/
	static bool sd_spi_get_cid(uint8_t *buffer);
#endif

/* interrupt handler to notify if the Data reception from flash is
 * over, in this case lunch the Memory(ram_buffer) to USART transfer and
 * disable interrupt*/
__attribute__((__interrupt__))
static void pdca_int_handler(void)
{
  // Disable all interrupts.
  Disable_global_interrupt();

  // Disable interrupt channel.
  pdca_disable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);

  //TODO we should have it, but i did not find a way to get sd_spi argument here
  //sd_mmc_spi_read_close_PDCA(); //unselects the SD/MMC memory.
  
  time_keeper_delay_ms(100);
  // Disable unnecessary channel
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_TX);
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_RX);

  // Enable all interrupts.
  Enable_global_interrupt();

  end_of_transfer = true;
}

static void local_pdca_init(void)
{
	// this PDCA channel is used for data reception from the SPI
	pdca_channel_options_t pdca_options_SPI_RX; // pdca channel options

	pdca_options_SPI_RX.addr = ram_buffer;
		// memory address. We take here the address of the string dummy_data. This string is located in the file dummy.h
	pdca_options_SPI_RX.size = 512;                              // transfer counter: here the size of the string
	pdca_options_SPI_RX.r_addr = NULL;                           // next memory address after 1st transfer complete
	pdca_options_SPI_RX.r_size = 0;                              // next transfer counter not used here
	pdca_options_SPI_RX.pid = AVR32_PDCA_CHANNEL_USED_RX;        // select peripheral ID - data are on reception from SPI1 RX line
	pdca_options_SPI_RX.transfer_size = PDCA_TRANSFER_SIZE_BYTE;  // select size of the transfer: 8,16,32 bits

	// this channel is used to activate the clock of the SPI by sending a dummy variables
	pdca_channel_options_t pdca_options_SPI_TX; // pdca channel options

	pdca_options_SPI_TX.addr = (void *)&dummy_data;             // memory address.
	// We take here the address of the string dummy_data.
	// This string is located in the file dummy.h
	pdca_options_SPI_TX.size = 512;                              // transfer counter: here the size of the string
	pdca_options_SPI_TX.r_addr = NULL;                           // next memory address after 1st transfer complete
	pdca_options_SPI_TX.r_size = 0;                              // next transfer counter not used here
	pdca_options_SPI_TX.pid = AVR32_PDCA_CHANNEL_USED_TX;        // select peripheral ID - data are on reception from SPI1 RX line
	pdca_options_SPI_TX.transfer_size = PDCA_TRANSFER_SIZE_BYTE;  // select size of the transfer: 8,16,32 bits

	// Init PDCA transmission channel
	pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_TX, &pdca_options_SPI_TX);

	// Init PDCA Reception channel
	pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_RX, &pdca_options_SPI_RX);

	INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_0, AVR32_INTC_INT1);  // pdca_channel_spi1_RX = 0

}

static bool sd_spi_check_presence(void)
{
	uint8_t r1; //card command response
	uint16_t retry = 0;
	
	if (sd_spi.init_done == false)
	{
		// If memory is not initialized, try to initialize it sending CMD0
		// If no valid response, there is no card
		while ((r1 = sd_spi_send_command(MMC_GO_IDLE_STATE, 0)) != 0x01)
		{
			spi_write(SD_MMC_SPI,0xFF);            // write dummy byte
			if (retry > 10)
			{
				return false; //card timed-out => no card
			}
			retry++; //increment retry counter
		}
		return true; //card respond => there is one
	}
	else //sd card has been initialized
	{
		// send a CRC command (CMD59) (supported only if card is initialized)
		if ((r1 = sd_spi_send_command(MMC_CRC_ON_OFF, 0)) == 0x00)
		{
			return true; //card respond => there is one
		}
		else
		{
			sd_spi.init_done = false;
			return false; //card timed-out => no card
		}
	}
}

bool sd_spi_mem_check(void)
{
	if (sd_spi_check_presence())
	{
		if (sd_spi.init_done == false)
		{
			return sd_spi_init();
		}
		else
		{
			return true;
		}
	}
	return false;
}

bool sd_spi_wait_not_busy(void)
{
	uint8_t r1; //card response
	uint32_t retry = 0;

	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
	
	//while card is busy, it cannot send anything back
	while((r1 = sd_spi_send_and_read(0xFF)) != 0xFF)
	{
		if (retry == 200000) //card timed-out
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
			print_util_dbg_print("card timed-out\r\n");
			return false; //card is still busy
		}
		retry++;
	}
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
	
	return true; //card is not busy anymore
}

static bool sd_spi_get_csd(uint8_t *buffer)
{
	uint8_t r1; //card command response
	uint8_t retry = 0; //retry counter
	unsigned short data_read;
	
	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false; //card is still busy
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI
	r1 = sd_spi_command(MMC_SEND_CSD, 0);
	if(r1 != 0x00)// if response is not valid
	{
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		sd_spi.init_done = false;
		return false; //card failed to send CSD
	}
	
	// wait for block start
	while((r1 = sd_spi_send_and_read(0xFF)) != MMC_STARTBLOCK_READ)
	{
		if (retry > 8) //card timed-out
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
			return false; //card did not send CSD data block
		}
		retry++; //increment retry counter
	}
	
	// store valid data block
	for (uint16_t i = 0;i < 16; i++)
	{
		spi_write(SD_MMC_SPI,0xFF);
		spi_read(SD_MMC_SPI,&data_read);
		buffer[i] = data_read;
	}
	spi_write(SD_MMC_SPI,0xFF);   // load CRC (not used)
	spi_write(SD_MMC_SPI,0xFF);
	spi_write(SD_MMC_SPI,0xFF);   // give clock again to end transaction
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	
	return true; //successfully received CSD data block
}

#if (defined SD_MMC_READ_CID) && (SD_MMC_READ_CID == true)
	static bool sd_spi_get_cid(uint8_t *buffer, void)
	{
		uint8_t r1; //card command response
		uint16_t r2; //card response
		uint8_t retry = 0; //retry counter
		unsigned short data_read;
	
		// wait for MMC not busy
		if (false == sd_spi_wait_not_busy())
		{
			return false;
		}
	
		spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // select SD_MMC_SPI
	
		r1 = sd_spi_command(MMC_SEND_CID, 0); //send command to get card CID
		if(r1 != 0x00) //if response is not valid
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
			sd_spi.init_done = false;
			return false; //card did not accept the command
		}
	
		// wait for data block start
		while((r2 = sd_spi_send_and_read(0xFF)) != MMC_STARTBLOCK_READ)
		{
			if (retry > 8) //card timed-out
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
				return false;
			}
			retry++; //increment retry counter
		}
	
		// store valid data block
		for (uint8_t i = 0; i <16; retry++)
		{
			spi_write(SD_MMC_SPI,0xFF); //write dummy Byte
			spi_read(SD_MMC_SPI,&data_read); //read Byte of the CID data block
			buffer[i] = data_read;
		}
		spi_write(SD_MMC_SPI,0xFF);   // load CRC (not used)
		spi_write(SD_MMC_SPI,0xFF);
		spi_write(SD_MMC_SPI,0xFF);   // give clock again to end transaction
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	
		return true; //successfully received the CID data block
	}
#endif

static int sd_spi_check_hc(void)
{
	uint8_t	r1; //card command response 
	unsigned char hc_bit; //high capacity bit
	
	// check if MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		print_util_dbg_print("check hc failed, SD card busy\r\n");
		return SD_FAILURE; //card is still busy
	}

	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI
	
	r1 = sd_spi_command(SD_READ_OCR, 0);
	if(r1 != 0) // if response not valid
	{
		print_util_dbg_print("check hc FAILED, response not valid:");
		print_util_dbg_print_num(r1,10);
		print_util_dbg_print("\r\n");
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		return SD_FAILURE; //card did not received the command properly
	}
	
	hc_bit = sd_spi_send_and_read(0xFF); // high capacity bit returned after the card received the SD_READ_OCR command
	
	r1 = sd_spi_send_and_read(0xFF); //write dummy Byte
	r1 = sd_spi_send_and_read(0xFF);
	r1 = sd_spi_send_and_read(0xFF);
	
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	
	if(hc_bit & 0x40) //check high capacity bit
	{
		print_util_dbg_print("High capacity sd card.\r\n");
		return SDHC_CARD; //is a high capacity card
	}
	return 0; //is not a high capacity card
}

static int sd_spi_get_card_type(void)
{
	uint8_t	r1; //card command response
	
	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return SD_FAILURE; //card is still busy
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI
	
	r1 = sd_spi_command(MMC_SEND_IF_COND, 0x000001AA);
	
	// check for valid response
	if((r1 & MMC_R1_ILLEGAL_COM) == 0)
	{
		r1 = sd_spi_send_and_read(0xFF);
		r1 = sd_spi_send_and_read(0xFF);
		r1 = sd_spi_send_and_read(0xFF);
		
		if((r1 & 0x01) == 0) 
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
			return SD_FAILURE;
		}
		else
		{
			r1 = sd_spi_send_and_read(0xFF);
			
			if(r1 != 0xaa) 
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
				return SD_FAILURE; /* wrong test pattern */
			}
			else
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
				print_util_dbg_print("Card type: SD_CARD_2\r\n");
				sd_spi.card_type = SD_CARD_2;
			}
		}
	}
	else
	{
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		sd_spi.card_type = MMC_CARD;
		
		// IDENTIFICATION OF THE CARD TYPE (SD or MMC)
		// Both cards will accept CMD55 command but only the SD card will respond to ACMD41
	 
		sd_spi_send_command(SD_APP_CMD55,0);
		spi_write(SD_MMC_SPI,0xFF);  // write dummy byte

		r1 = sd_spi_send_command(SD_SEND_OP_COND_ACMD, 0);
		spi_write(SD_MMC_SPI,0xFF);  // write dummy byte

		if ((r1&0xFE) == 0) 
		{   
			// ignore "in_idle_state" flag bit
			print_util_dbg_print("Card type: SD_CARD\r\n");
			sd_spi.card_type = SD_CARD;    // card has accepted the command, this is a SD card
		} 
		else 
		{
			print_util_dbg_print("Card type: MMC_CARD\r\n");
			sd_spi.card_type = MMC_CARD;   // card has not responded, this is a MMC card
		 
			// reset card again
			if(false == sd_spi_reset_card())
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
				return false; //did not manage to reset the card
			}
		 }
	 
		 // CHECK FOR SDHC
		/*if(sd_spi.card_type == SD_CARD_2) 
		{
			int is_high_capacity = sd_spi_check_hc();
			if (is_high_capacity == -1)
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
				return false;
			}
			else if (is_high_capacity == 1)
			{
				print_util_dbg_print("Card type: SDHC");
				sd_spi.card_type = SD_CARD_2_SDHC;
			}
		}*/
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	}
	
	return sd_spi.card_type;
}

void sd_spi_get_capacity(void)
{
	uint64_t c_size;
	uint8_t  c_size_mult;
	uint8_t  read_bl_len;
	uint8_t  erase_grp_size;
	uint8_t  erase_grp_mult;

	// extract variables from CSD array
	read_bl_len = sd_spi.csd[5] & 0x0F;
	
	if (sd_spi.card_type == SD_CARD_2_SDHC) 
	{
		c_size = ((sd_spi.csd[7] & 0x3F) << 16) | (sd_spi.csd[8] << 8) | sd_spi.csd[9];
		++c_size;

		sd_spi.capacity = c_size << 19;
		sd_spi.capacity_mult = (c_size >> 13) & 0x01FF;
		sd_spi.last_block_address = (sd_spi.capacity >> 9) + (sd_spi.capacity_mult << 23) - 1;
	} 
	else 
	{
		c_size      = ((sd_spi.csd[6] & 0x03) << 10) + (sd_spi.csd[7] << 2) + ((sd_spi.csd[8] & 0xC0) >> 6);
		c_size_mult = ((sd_spi.csd[9] & 0x03) << 1) + ((sd_spi.csd[10] & 0x80) >> 7);
		sd_spi.last_block_address = ((uint32_t)(c_size + 1) * (uint32_t)((1 << (c_size_mult + 2)))) - 1;
		sd_spi.capacity = (1 << read_bl_len) * (sd_spi.last_block_address + 1);
		sd_spi.capacity_mult = 0;
		if (read_bl_len > 9)   // 9 means 2^9 = 512b
		{	
			sd_spi.last_block_address <<= (read_bl_len - 9);
		}
	}
	
	if (sd_spi.card_type == MMC_CARD)
	{
		erase_grp_size = ((sd_spi.csd[10] & 0x7C) >> 2);
		erase_grp_mult = ((sd_spi.csd[10] & 0x03) << 3) | ((sd_spi.csd[11] & 0xE0) >> 5);
	}
	else
	{
		erase_grp_size = ((sd_spi.csd[10] & 0x3F) << 1) + ((sd_spi.csd[11] & 0x80) >> 7);
		erase_grp_mult = 0;
	}
	sd_spi.erase_group_size = (erase_grp_size + 1) * (erase_grp_mult + 1);
	
	print_util_dbg_print("Card capacity:");
	print_util_dbg_print_long(sd_spi.capacity,10);
	print_util_dbg_print(", erase group size:");
	print_util_dbg_print_num(sd_spi.erase_group_size,10);
	print_util_dbg_print(".\r\n");
	
}

void sd_spi_get_sector_count(void* res)
{
	uint32_t *_res = res;
	*_res = sd_spi.capacity / MMC_SECTOR_SIZE;
}

void sd_spi_get_sector_size(void* res)
{
	uint32_t *_res = res;
	*_res = (uint32_t)MMC_SECTOR_SIZE;
}

void sd_spi_get_block_size(void* res)
{
	uint16_t *_res = res;
	*_res = sd_spi.erase_group_size;
}

bool sd_spi_reset_card(void)
{
	uint16_t retry = 0;
	uint8_t r1; //card command response
	
	sd_spi.init_done = false;
	sd_spi.card_type = MMC_CARD; //by default
	
	do 
	{
		r1 = sd_spi_send_command(MMC_GO_IDLE_STATE, 0); //card will answer 0x01 if entered in Idle_State
		spi_write(SD_MMC_SPI,0xFF); // write dummy byte
		
		retry++; //increment retry counter
		if(retry > 1000)
		{
			print_util_dbg_print("Did not manage to set card in idle state:");
			print_util_dbg_print_num(r1,10);
			print_util_dbg_print("\r\n");
			return false; //did not manage to set card in idle_state
		}
		
	}while(r1 != 0x01); // check memory enters iddle_state
	
	return true; //successfully entered in idlle_state
}

bool is_sd_spi_write_pwd_locked(void)
{
	uint16_t r2;
	
	if (sd_spi.card_type == MMC_CARD)
	{
		if (((sd_spi.csd[0] >> 2) & 0x0F) < 2) // lock feature is not present on the card since the MMC is v1.x released !
		{
			return false; //card is not locked
		}
	}
	
	// get STATUS response
	if (false == sd_spi_get_status(&r2))
	{
		return false; //card is not locked
	}
	
	// check "card is locked" flag in R2 response
	if ((r2&0x0001) != 0)             
	{
		return true; //card is locked
	}

	return false; //card is not locked
}

static bool sd_spi_get_status(uint16_t *r2)
{
	uint8_t retry = 0;
	uint8_t spireg;

	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false;
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI

	// send command
	spi_write(SD_MMC_SPI,MMC_SEND_STATUS | 0x40);  // send command
	spi_write(SD_MMC_SPI,0);                       // send parameter
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0x95);            // correct CRC for first command in SPI (CMD0)
	// after, the CRC is ignored
	
	// end command
	// wait for response
	*r2 = 0xFFFF; //necessary ?
	spireg = 0xFF;
	while((spireg = sd_spi_send_and_read(0xFF)) == 0xFF)
	{
		if(retry > 10) //card timed-out
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
			return false; //card timed-out
		}
		retry++; //increment retry counter
	}
	*r2 = ((uint16_t)(spireg) << 8) + sd_spi_send_and_read(0xFF);    // first byte is MSb

	spi_write(SD_MMC_SPI,0xFF);   // give clock again to end transaction
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI

	return true;
}

//May be removed. 
static uint8_t sd_spi_send_command(uint8_t command, uint32_t arg)
{
	uint8_t	r1; //card command response
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI
	r1 = sd_spi_command(command, arg);
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	return r1;
}

static uint8_t sd_spi_command(uint8_t command, uint32_t arg)
{
	uint8_t retry = 0; //init retry counter
	uint8_t	r1; //card response
	
	spi_write(SD_MMC_SPI, 0xFF);            // write dummy byte
	spi_write(SD_MMC_SPI, command | 0x40);  // send command
	spi_write(SD_MMC_SPI, arg>>24);         // send parameter
	spi_write(SD_MMC_SPI, arg>>16);
	spi_write(SD_MMC_SPI, arg>>8 );
	spi_write(SD_MMC_SPI, arg    );
	
	switch(command)
	{
		case MMC_GO_IDLE_STATE:
			spi_write(SD_MMC_SPI, 0x95);
			break;
		case MMC_SEND_IF_COND:
			spi_write(SD_MMC_SPI, 0x87);
			break;
		default:
			spi_write(SD_MMC_SPI, 0xff);
			break;
	}// end command

	r1    = 0xFF; //necessary ??
	while((r1 = sd_spi_send_and_read(0xFF)) == 0xFF) // wait for response
	{
		if(retry > 10) // if more than 10 retries, card has timed-out and return the received 0xFF
		{
			return 0xFF; //card timed-out
		}
		retry++;	//increment retry counter
	}
	
	return r1; //return card response 
}

static uint8_t sd_spi_send_and_read(uint8_t data_to_send)
{
	unsigned short data_read;
	
	spi_write(SD_MMC_SPI, data_to_send);
	
	if( SPI_ERROR_TIMEOUT == spi_read(SD_MMC_SPI, &data_read) )
	{
		return 0xFF; //card timed-out
	}
	else
	{
		return data_read;
	}
}

static void sd_spi_resources_init(spi_options_t spiOptions)
{
	// GPIO pins used for SD/MMC interface
	static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
	{
		{SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
		{SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
		{SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
		{SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};
	
	//try to power down SD_SPI
	gpio_configure_pin(SD_MMC_SPI_SCK_PIN,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(SD_MMC_SPI_MISO_PIN,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(SD_MMC_SPI_MOSI_PIN,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(SD_MMC_SPI_NPCS_PIN,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	
	time_keeper_delay_ms(500);

	// Assign I/Os to SPI.
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
	sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster(SD_MMC_SPI, &spiOptions);

	// Set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

	// Enable SPI module.
	spi_enable(SD_MMC_SPI);
}

bool sd_spi_init(void)
{
	uint32_t retry = 0; //retry counter
	uint8_t r1 = 0; //card response
	
	//initialize sd_spi_t struct
	sd_spi.init_done		= false;
	for (uint8_t i = 0; i < 16; i++)
	{
		sd_spi.csd[i]		= 0; 
	}
	sd_spi.card_type			= MMC_CARD;
	sd_spi.capacity				= 0;
	sd_spi.capacity_mult		= 0;
	sd_spi.clock				= FOSC0;
	sd_spi.last_block_address	= 0;
	sd_spi.erase_group_size		= 0;
	
	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          = SD_MMC_SPI_NPCS,
		.baudrate     = SPI_LOW_SPEED,			// Start at low frequency
		.bits         = SD_MMC_SPI_BITS,        // Defined in sd_spi.h.
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};
	
	//init GPIO for SPI1 usage
	sd_spi_resources_init(spiOptions);
	
	//init SPI for sd card usage
	spi_setupChipReg(SD_MMC_SPI, &spiOptions, sd_spi.clock);
	
	/* card needs 74 cycles minimum to start up */
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI
	for(uint8_t i = 0; i < 10; ++i) 
	{
		spi_write(SD_MMC_SPI,0xFF);
	}
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI

	// RESET THE MEMORY CARD
	if(false == sd_spi_reset_card())
	{
		print_util_dbg_print("sd_card FAILED reset \r\n");
		return false; //did not manage to reset the card
	}
	
	sd_spi_get_card_type();
	
	// CONTINUE INTERNAL INITIALIZATION OF THE CARD
	// Continue sending CMD1 while memory card is in idle state
	do {
		switch(sd_spi.card_type) 
		{
			case MMC_CARD:
				r1 = sd_spi_send_command(MMC_SEND_OP_COND, 0);
				spi_write(SD_MMC_SPI,0xFF);            // write dummy byte
				break;
			
			case SD_CARD:
				sd_spi_send_command(SD_APP_CMD55,0);
				r1 = sd_spi_send_command(SD_SEND_OP_COND_ACMD, 0);
				spi_write(SD_MMC_SPI,0xFF);            // write dummy byte
				break;
			
			case SD_CARD_2:
				// set high capacity bit mask
				sd_spi_send_command(SD_APP_CMD55,0);
				r1 = sd_spi_send_command(SD_SEND_OP_COND_ACMD, 0x40000000);
				spi_write(SD_MMC_SPI,0xFF);            // write dummy byte
				break;
		}
		
		if(retry == 50000) // measured approx. 500 on several cards
		{
			print_util_dbg_print("sd_card FAILED timed-out \r\n");
			return false; //card timed-out
		}
		retry++; //increment retry counter
	} while (r1);
	
	// CHECK FOR SDHC
	if(sd_spi.card_type == SD_CARD_2) 
	{
		int is_high_capacity = sd_spi_check_hc();
		if (is_high_capacity == -1)
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
			print_util_dbg_print("sd_card FAILED check high capacity \r\n");
			return false;
		}
		else if (is_high_capacity == 1)
		{
			print_util_dbg_print("Init check Card type: SDHC\r\n");
			sd_spi.card_type = SD_CARD_2_SDHC;
		}
	}
	
	// DISABLE CRC TO SIMPLIFY AND SPEED UP COMMUNICATIONS
	r1 = sd_spi_send_command(MMC_CRC_ON_OFF, 0);  // disable CRC (should be already initialized on SPI init)
	spi_write(SD_MMC_SPI,0xFF);            // write dummy byte

	// SET BLOCK LENGTH TO 512 BYTES
	r1 = sd_spi_send_command(MMC_SET_BLOCKLEN, 512);
	spi_write(SD_MMC_SPI,0xFF);            // write dummy byte
	if (r1 != 0x00) //if card did not accept command SET_BLOCKLEN
	{
		print_util_dbg_print("sd_card FAILED set block length \r\n");
		return false;    // card unsupported block length
	}
	
	// GET CARD SPECIFIC DATA
	if (false ==  sd_spi_get_csd(sd_spi.csd))
	{
		print_util_dbg_print("sd_card FAILED get CSD \r\n");
		return false;
	}

	// GET CARD CAPACITY and NUMBER OF SECTORS
	sd_spi_get_capacity();

	// GET CARD IDENTIFICATION DATA IF REQUIRED
	#if (defined SD_MMC_READ_CID) && (SD_MMC_READ_CID == true)
		if (false ==  sd_spi_get_cid(cid))
		{
			print_util_dbg_print("sd_card FAILED get CID \r\n");
			return false;
		}
	#endif

	sd_spi.init_done = true;

	// Set SPI Speed to MAX
	spiOptions.baudrate = SPI_HIGH_SPEED;
	spi_setupChipReg(SD_MMC_SPI, &spiOptions, FOSC0);
	
	// Initialize PDCA controller for the sd_spi driver
	local_pdca_init();
	
	print_util_dbg_print("Sd_card initialized \r\n");
	return true; //successfully initialize the sd card
}

bool sd_spi_status(void)
{
	uint8_t retry = 0;
	uint8_t spireg;

	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false;
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI

	// send command
	spi_write(SD_MMC_SPI,MMC_SEND_STATUS | 0x40);  // send command
	spi_write(SD_MMC_SPI,0);                       // send parameter
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0);
	spi_write(SD_MMC_SPI,0x95);            // correct CRC for first command in SPI (CMD0)
	// after, the CRC is ignored
	
	// end command
	// wait for response
	uint16_t r2 = 0xFFFF; //necessary ?
	spireg = 0xFF;
	while((spireg = sd_spi_send_and_read(0xFF)) == 0xFF)
	{
		if(retry > 10) //card timed-out
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
			return false; //card timed-out
		}
		retry++; //increment retry counter
	}
	r2 = ((uint16_t)(spireg) << 8) + sd_spi_send_and_read(0xFF);    // first byte is MSb

	spi_write(SD_MMC_SPI,0xFF);   // give clock again to end transaction
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI

	return true;
}

bool sd_mmc_spi_read_open_PDCA (uint32_t pos)
{
	uint8_t r1; //card response

	// Set the global memory ptr at a Byte address.
	uint32_t address = pos << 9; // address = pos * 512

	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		print_util_dbg_print("card busy");
		return false;
	}

	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // select SD_MMC_SPI

	// issue command
	if(sd_spi.card_type == SD_CARD_2_SDHC) 
	{
		r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, address>>9);
	} 
	else 
	{
		r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, address);
	}

	if (r1 != 0x00) //if response not valid
	{
		print_util_dbg_print("not received valid response");
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		return false;
	}

	// wait for token (may be a datablock start token OR a data error token !)
	uint16_t read_time_out = 0;
	while((r1 = sd_spi_send_and_read(0xFF)) == 0xFF)
	{
		if (read_time_out == 30000)   // TIME-OUT
		{
			print_util_dbg_print("timed-out");
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI
			return false;
		}
		read_time_out++;
	}

	// check token
	if (r1 != MMC_STARTBLOCK_READ)
	{
		print_util_dbg_print("did not receive startblock_read");
		spi_write(SD_MMC_SPI,0xFF);
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		return false;
	}
	return true;   // Read done.
}


bool sd_mmc_spi_read_sector_to_ram(void *ram)
{
	uint8_t r1; //card response
	uint8_t *_ram = ram;
	unsigned short data_read;
	
	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false;
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI

	// issue command
	if(sd_spi.card_type == SD_CARD_2_SDHC) 
	{
		r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, gl_ptr_mem>>9);
	} 
	else 
	{
		r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, gl_ptr_mem);
	}

	if (r1 != 0x00) //if response not valid
	{
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
		return false;
	}

	// wait for token (may be a datablock start token OR a data error token !)
	uint16_t  read_time_out = 0;
	while((r1 = sd_spi_send_and_read(0xFF)) == 0xFF)
	{
		if (read_time_out == 30000)   // TIME-OUT
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI
			return false; //card timed-out
		}
		read_time_out++;
	}

	// check token
	if (r1 != MMC_STARTBLOCK_READ)
	{
		spi_write(SD_MMC_SPI,0xFF);
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI
		return false;
	}

	// store datablock
	for(uint16_t  i = 0; i < MMC_SECTOR_SIZE; i++)
	{
		spi_write(SD_MMC_SPI,0xFF);
		spi_read(SD_MMC_SPI,&data_read);
		*_ram++ = data_read;
	}
	gl_ptr_mem += 512; // Update the memory pointer.

	// load 16-bit CRC (ignored)
	spi_write(SD_MMC_SPI,0xFF);
	spi_write(SD_MMC_SPI,0xFF);

	// continue delivering some clock cycles
	spi_write(SD_MMC_SPI,0xFF);
	spi_write(SD_MMC_SPI,0xFF);

	// release chip select
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI

	return true; // Read done.
}

bool sd_spi_read_given_sector_to_ram(void* ram, uint32_t beginning_sector, uint8_t count)
{
	uint8_t r1; //card response
	uint8_t *_ram = ram;
	unsigned short data_read;
	
	uint32_t sector = beginning_sector;
	
	uint16_t read_time_out;
	uint16_t  i,j;
	
	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false;
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);    // select SD_MMC_SPI

	for (j = 0;j < count; j++)
	{
		// issue command
		if(sd_spi.card_type == SD_CARD_2_SDHC)
		{
			r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, sector);
		}
		else
		{
			r1 = sd_spi_command(MMC_READ_SINGLE_BLOCK, sector << 9); // address = pos * 512
		}

		if (r1 != 0x00) //if response not valid
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
			return false;
		}

		// wait for token (may be a datablock start token OR a data error token !)
		read_time_out = 0;
		while((r1 = sd_spi_send_and_read(0xFF)) == 0xFF)
		{
			if (read_time_out == 30000)   // TIME-OUT
			{
				spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI
				return false; //card timed-out
			}
			read_time_out++;
		}

		// check token
		if (r1 != MMC_STARTBLOCK_READ)
		{
			spi_write(SD_MMC_SPI,0xFF);
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI
			return false;
		}

		// store datablock
		for(i = 0; i < MMC_SECTOR_SIZE; i++)
		{
			spi_write(SD_MMC_SPI,0xFF);
			spi_read(SD_MMC_SPI,&data_read);
			*_ram++ = data_read;
		}
		sector += 1; // Update the memory pointer.

		// load 16-bit CRC (ignored)
		spi_write(SD_MMC_SPI,0xFF);
		spi_write(SD_MMC_SPI,0xFF);

		// continue delivering some clock cycles
		spi_write(SD_MMC_SPI,0xFF);
		spi_write(SD_MMC_SPI,0xFF);
	}
	
	// release chip select
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // unselect SD_MMC_SPI

	return true; // Read done.
}

bool sd_spi_read_sector_to_ram_pcda(void* ram, uint32_t beginning_sector, uint8_t count)
{
	
	uint16_t j;
	
	char buffer[1000];
	
	// Enable all interrupts.
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	if (!global_interrupt_enabled)
	{
		cpu_irq_enable ();
	}
	
	for (j=beginning_sector;j<count;j++)
	{
		// Configure the PDCA channel: the address of memory ram_buffer to receive the data at sector address j
		pdca_load_channel(  AVR32_PDCA_CHANNEL_SPI_RX,
							&buffer,
							512);

		pdca_load_channel(  AVR32_PDCA_CHANNEL_SPI_TX,
							(void *)&dummy_data,
							512); //send dummy to activate the clock

		end_of_transfer = false;
		// open sector number j
		if(sd_mmc_spi_read_open_PDCA (j))
		{
			print_util_dbg_print("\r\rFirst 512 Bytes of Transfer number ");
			print_util_dbg_print_num(j,10);
			print_util_dbg_print(" :\r\r\n");

			spi_write(SD_MMC_SPI,0xFF); // Write a first dummy data to synchronise transfer
			pdca_enable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);
			pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
			pdca_channeltx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
			pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
			pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer

			while(!end_of_transfer);

			// Display the first 2O bytes of the ram_buffer content
			for( uint16_t i = 0; i < 20; i++)
			{
				// Print the ASCII value of the buffer
				print_util_dbg_print_num((U8)*(buffer + i), 10);
			}
			time_keeper_delay_ms(250);
		}
		else
		{
			print_util_dbg_print("\r\r! Unable to open memory \r\r\n");
			if (!global_interrupt_enabled)
			{
				cpu_irq_disable ();
			}
			return false;
		}
	}
	
	if (!global_interrupt_enabled)
	{
		cpu_irq_disable ();
	}
	
	return true;
}

bool sd_spi_write_sector_from_ram(const void *ram)
{
	uint8_t r1; //card response
	const uint8_t *_ram = ram;
	uint16_t i;

	// wait for MMC not busy
	if (false == sd_spi_wait_not_busy())
	{
		return false;
	}
	
	spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // select SD_MMC_SPI

	// send write command
	if(sd_spi.card_type == SD_CARD_2_SDHC) 
	{
		r1 = sd_spi_command(MMC_WRITE_BLOCK, gl_ptr_mem>>9);
	} 
	else 
	{
		r1 = sd_spi_command(MMC_WRITE_BLOCK, gl_ptr_mem);
	}

	if(r1 != 0x00) //if response not valid
	{
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
		return false;
	}
	
	spi_write(SD_MMC_SPI,0xFF); // send dummy to give clock again to end transaction

	// send data start token
	spi_write(SD_MMC_SPI,MMC_STARTBLOCK_WRITE);
	// write data
	for(i = 0; i < MMC_SECTOR_SIZE; i++)
	{
		spi_write(SD_MMC_SPI,*_ram++);
	}

	spi_write(SD_MMC_SPI,0xFF); // send CRC (field required but value ignored)
	spi_write(SD_MMC_SPI,0xFF);

	// read data response token
	r1 = sd_spi_send_and_read(0xFF);
	if( (r1&MMC_DR_MASK) != MMC_DR_ACCEPT)
	{
		spi_write(SD_MMC_SPI,0xFF); // send dummy bytes
		spi_write(SD_MMC_SPI,0xFF);
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
		return false; // return ERROR byte
	}

	spi_write(SD_MMC_SPI,0xFF);    // send dummy bytes
	spi_write(SD_MMC_SPI,0xFF);

	// release chip select
	spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	gl_ptr_mem += 512;        // Update the memory pointer.

	// wait card not busy after last programming operation
	uint8_t retry = 0;
	while (false == sd_spi_wait_not_busy())
	{
		if (retry == 10) //timed-out
		{
			return false; //card still busy
		}
		retry++; //increment retry counter
	}

	return true; // Write done
}

bool sd_spi_write_given_sector_from_ram(const void *ram, uint32_t beginning_sector, uint8_t count)
{
	uint8_t r1; //card response
	const uint8_t *_ram = ram;
	uint16_t i,j;

	uint32_t sector = beginning_sector;

	for (j = 0;j < count; j++)
	{
		// wait for MMC not busy
		if (false == sd_spi_wait_not_busy())
		{
			print_util_dbg_print("SD SPI busy.\r\n");
			return false;
		}
		
		spi_selectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS); // select SD_MMC_SPI

	
		// send write command
		if(sd_spi.card_type == SD_CARD_2_SDHC)
		{
			r1 = sd_spi_command(MMC_WRITE_BLOCK, sector);
		}
		else
		{
			r1 = sd_spi_command(MMC_WRITE_BLOCK, sector << 9); // address = pos * 512
		}

		if(r1 != 0x00) //if response not valid
		{
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
			print_util_dbg_print("Response not valid\r\n");
			return false;
		}
	
		spi_write(SD_MMC_SPI,0xFF); // send dummy to give clock again to end transaction

		// send data start token
		spi_write(SD_MMC_SPI,MMC_STARTBLOCK_WRITE);
		// write data
		for(i = 0; i < MMC_SECTOR_SIZE; i++)
		{
			if (_ram!=NULL)
			{
				spi_write(SD_MMC_SPI,*_ram);
				_ram++;
			}
			else
			{
				spi_write(SD_MMC_SPI,0);
			}
		}

		spi_write(SD_MMC_SPI,0xFF); // send CRC (field required but value ignored)
		spi_write(SD_MMC_SPI,0xFF);

		// read data response token
		r1 = sd_spi_send_and_read(0xFF);
		if( (r1&MMC_DR_MASK) != MMC_DR_ACCEPT)
		{
			spi_write(SD_MMC_SPI,0xFF); // send dummy bytes
			spi_write(SD_MMC_SPI,0xFF);
			spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);
			
			print_util_dbg_print("wrong response token");
			return false; // return ERROR byte
		}

		spi_write(SD_MMC_SPI,0xFF);    // send dummy bytes
		spi_write(SD_MMC_SPI,0xFF);

		sector += 1;        // Update the memory pointer.
		
		// release chip select
		spi_unselectChip(SD_MMC_SPI, SD_MMC_SPI_NPCS);  // unselect SD_MMC_SPI
	}


	// wait card not busy after last programming operation
	uint8_t retry = 0;
	while (false == sd_spi_wait_not_busy())
	{
		if (retry == 10) //timed-out
		{
			return false; //card still busy
		}
		retry++; //increment retry counter
	}

	return true; // Write done
}

void sd_spi_test(void)
{
	// Wait for a card to be inserted
	while (!sd_spi_mem_check());
	
	print_util_dbg_print("\r\rCard detected!");

	// Read Card capacity
	sd_spi_get_capacity();
	print_util_dbg_print("Capacity = ");
	print_util_dbg_print_num(sd_spi.capacity >> 20,10); //>>20 to round capacity to MegaBytes
	print_util_dbg_print(" MBytes \r\r\n");
	
	//try to write something on the sd_card
	print_util_dbg_print("write succeed ? ");
	print_util_dbg_print_num(sd_spi_write_sector_from_ram((void *)&dummy_data),10);
	print_util_dbg_print("write succeed ? ");
	print_util_dbg_print_num(sd_spi_write_sector_from_ram((void *)&dummy_data),10);
	print_util_dbg_print("write succeed ? ");
	print_util_dbg_print_num(sd_spi_write_sector_from_ram((void *)&dummy_data),10);

	//bool as = sd_mmc_spi_read_sector_to_ram((void*)ram_buffer2);
	if (sd_mmc_spi_read_sector_to_ram((void*)ram_buffer2))
	{
		print_util_dbg_print("Read successful\r\n");
	}
	


	// Enable all interrupts.
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	if (!global_interrupt_enabled)
	{
		cpu_irq_enable ();
	}
	

	// Read the first sectors number 1, 2, 3 of the card
	for(uint16_t j = 1; j <= 3; j++)
	{
		// Configure the PDCA channel: the address of memory ram_buffer to receive the data at sector address j
		pdca_load_channel(  AVR32_PDCA_CHANNEL_SPI_RX,
							&ram_buffer,
							512);

		pdca_load_channel(  AVR32_PDCA_CHANNEL_SPI_TX,
							(void *)&dummy_data,
							512); //send dummy to activate the clock

		end_of_transfer = false;
		// open sector number j
		if(sd_mmc_spi_read_open_PDCA (j))
		{
			print_util_dbg_print("\r\rFirst 512 Bytes of Transfer number ");
			print_util_dbg_print_num(j,10);
			print_util_dbg_print(" :\r\r\n");

			spi_write(SD_MMC_SPI,0xFF); // Write a first dummy data to synchronise transfer
			pdca_enable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);
			pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
			pdca_channeltx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
			pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
			pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer

			while(!end_of_transfer);

			// Display the first 2O bytes of the ram_buffer content
			for( uint16_t i = 0; i < 20; i++)
			{
				
				//print_util_dbg_print_num( atoi(&(*(ram_buffer + i))), 10); // prints the decimal value of the ascii
				// Print the ASCII value of the ram_buffer
				print_util_dbg_print_num((U8)*(ram_buffer + i), 10);
				//print_util_dbg_print_num( (U8)atoi(&(*(ram_buffer2 + i))), 10);
			}
			time_keeper_delay_ms(250);
		}
		else
		{
			print_util_dbg_print("\r\r! Unable to open memory \r\r\n");
		}
	}
	if (!global_interrupt_enabled)
	{
		cpu_irq_disable ();
	}
	print_util_dbg_print("\r\rEnd of the example.\r\r\n");
}