/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief SPI Master driver for AVR UC3.
 *
 * This file defines a useful set of functions for the SPI interface on AVR UC3
 * devices.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR UC3
 * - Supported devices:  All AVR UC3 devices with an SPI module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

#ifndef _SPI_MASTER_H_
#define _SPI_MASTER_H_

#include "compiler.h"
#include "sysclk.h"
#include "status_codes.h"
#include "spi.h"

/*! \name Spi Master Management Configuration
 */
//! @{
#include "conf_spi_master.h"

//! Default Config Spi Master Delay BCS
#ifndef CONFIG_SPI_MASTER_DELAY_BCS
#define CONFIG_SPI_MASTER_DELAY_BCS          0
#endif

//! Default Config Spi Master Bits per Transfer Definition
#ifndef CONFIG_SPI_MASTER_BITS_PER_TRANSFER
#define CONFIG_SPI_MASTER_BITS_PER_TRANSFER  8
#endif

//! Default Config Spi Master Delay BCT
#ifndef CONFIG_SPI_MASTER_DELAY_BCT
#define CONFIG_SPI_MASTER_DELAY_BCT          0
#endif

//! Default Config Spi Master Delay BS
#ifndef CONFIG_SPI_MASTER_DELAY_BS
#define CONFIG_SPI_MASTER_DELAY_BS           0
#endif

//! Default Config Spi Master Dummy Field
#ifndef CONFIG_SPI_MASTER_DUMMY
#define CONFIG_SPI_MASTER_DUMMY              0xFF
#endif
//! @}

//! Spi Flags definition
typedef uint8_t spi_flags_t;

//! Board Spi Select Id Definition
typedef uint8_t board_spi_select_id_t;

//! \brief Polled SPI device defintion
struct spi_device {
	//! Board specific select id
	board_spi_select_id_t	id;
};

/*! \brief Initializes the SPI in master mode.
 *
 * \param spi       Base address of the SPI instance.
 *
 */
static inline void spi_master_init(volatile avr32_spi_t *spi)
{
#ifdef AVR32_SPI
	sysclk_enable_pba_module(SYSCLK_SPI);	
#else
	if ((uint32_t)spi == AVR32_SPI0_ADDRESS)
	{
	sysclk_enable_pba_module(SYSCLK_SPI0);	
	}
#ifdef AVR32_SPI1
	else if ((uint32_t)spi == AVR32_SPI1_ADDRESS)
	{
	sysclk_enable_pba_module(SYSCLK_SPI1);	
	}
#endif
#endif
  
  spi_reset(spi);
  spi_set_master_mode(spi);
  spi_disable_modfault(spi);
  spi_disable_loopback(spi);
  spi_set_chipselect(spi,(1 << AVR32_SPI_MR_PCS_SIZE) - 1);
  spi_disable_variable_chipselect(spi);
  spi_disable_chipselect_decoding(spi);
  spi_set_delay(spi,CONFIG_SPI_MASTER_DELAY_BCS);
}

/**
 * \brief Setup a SPI device.
 *
 * The returned device descriptor structure must be passed to the driver
 * whenever that device should be used as current slave device.
 *
 * \param spi       Base address of the SPI instance.
 * \param device    Pointer to SPI device struct that should be initialized.
 * \param flags     SPI configuration flags. Common flags for all
 *                  implementations are the SPI modes SPI_MODE_0 ...
 *                  SPI_MODE_3.
 * \param baud_rate Baud rate for communication with slave device in Hz.
 * \param sel_id    Board specific seclet id
 */
extern void spi_master_setup_device(volatile avr32_spi_t *spi, struct spi_device *device, 
     spi_flags_t flags, uint32_t baud_rate,
     board_spi_select_id_t sel_id);

/*! \brief Enables the SPI.
 *
 * \param spi Base address of the SPI instance.
 */
extern void spi_enable(volatile avr32_spi_t *spi);

/*! \brief Disables the SPI.
 *
 * Ensures that nothing is transferred while setting up buffers.
 *
 * \param spi Base address of the SPI instance.
 *
 * \warning This may cause data loss if used on a slave SPI.
 */
extern void spi_disable(volatile avr32_spi_t *spi);

/*! \brief Tests if the SPI is enabled.
 *
 * \param spi Base address of the SPI instance.
 *
 * \return \c 1 if the SPI is enabled, otherwise \c 0.
 */
extern int spi_is_enabled(volatile avr32_spi_t *spi);

/**
 * \brief Select given device on the SPI bus
 *
 * Set device specific setting and calls board chip select.
 *
 * \param spi Base address of the SPI instance.
 * \param device SPI device
 *
 */
static inline void spi_select_device(volatile avr32_spi_t *spi, struct spi_device *device)
{
	spi_selectChip(spi,device->id);
}

/**
 * \brief Deselect given device on the SPI bus
 *
 * Calls board chip deselect.
 *
 * \param spi Base address of the SPI instance.
 * \param device SPI device
 *
 * \pre SPI device must be selected with spi_select_device() first
 */
static inline void spi_deselect_device(volatile avr32_spi_t *spi, struct spi_device *device)
{
	spi_unselectChip(spi,device->id);
}

/*! \brief Write one byte to a SPI device.
 *
 * \param spi    Base address of the SPI instance.
 * \param data   Data to write
 *
 */
static inline void spi_write_single(volatile avr32_spi_t *spi, uint8_t data)
{
	spi_put(spi,(uint16_t)data);
}

/**
 * \brief Send a sequence of bytes to a SPI device
 *
 * Received bytes on the SPI bus are discarded.
 *
 * \param spi    Base address of the SPI instance.
 * \param data   Data buffer to write
 * \param len    Length of data
 *
 * \pre SPI device must be selected with spi_select_device() first
 */
extern status_code_t spi_write_packet(volatile avr32_spi_t *spi,
    const uint8_t *data, size_t len);

/*! \brief Receive one byte from a SPI device.
 *
 * \param spi    Base address of the SPI instance.
 * \param data   Data to read
 *
 */
static inline void spi_read_single(volatile avr32_spi_t *spi, uint8_t *data)
{
	*data = (uint8_t)spi_get(spi);
}

/**
 * \brief Receive a sequence of bytes from a SPI device
 *
 * All bytes sent out on SPI bus are sent as value 0.
 *
 * \param spi    Base address of the SPI instance.
 * \param data   Data buffer to read
 * \param len    Length of data
 *
 * \pre SPI device must be selected with spi_select_device() first
 */
extern status_code_t spi_read_packet(volatile avr32_spi_t *spi,
   uint8_t *data, size_t len);

/*! \brief Checks if all transmissions are complete.
 *
 * \param spi Base address of the SPI instance.
 *
 * \return Status.
 *   \retval 1  All transmissions complete.
 *   \retval 0  Transmissions not complete.
 */
extern bool spi_is_tx_empty(volatile avr32_spi_t *spi);

/*! \brief Checks if all transmissions is ready.
 *
 * \param spi Base address of the SPI instance.
 *
 * \return Status.
 *   \retval 1  All transmissions complete.
 *   \retval 0  Transmissions not complete.
 */
extern bool spi_is_tx_ready(volatile avr32_spi_t *spi);

/*! \brief Check if the SPI contains a received character.
 *
 * \param spi Base address of the SPI instance.
 *
 * \return \c 1 if the SPI Receive Holding Register is full, otherwise \c 0.
 */
extern bool spi_is_rx_full(volatile avr32_spi_t *spi);

/*! \brief Checks if all reception is ready.
 *
 * \param spi Base address of the SPI instance.
 *
 * \return \c 1 if the SPI Receiver is ready, otherwise \c 0.
 */
extern bool spi_is_rx_ready(volatile avr32_spi_t *spi);

#endif  // _SPI_MASTER_H_
