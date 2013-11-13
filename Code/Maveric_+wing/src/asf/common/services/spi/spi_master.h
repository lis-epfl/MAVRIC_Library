/**
 * \file
 *
 * \brief SPI Master Mode management
 *
 * Copyright (C) 2010 Atmel Corporation. All rights reserved.
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
#ifndef SPI_MASTER_H_INCLUDED
#define SPI_MASTER_H_INCLUDED

#include <parts.h>

#if XMEGA
# include "xmega_spi/spi_master.h"
#elif (defined(__GNUC__) && defined(__AVR32__)) || (defined(__ICCAVR32__) || defined(__AAVR32__))
# include "uc3_spi/spi_master.h"
#else
# error Unsupported chip type
#endif

/**
 *
 * \defgroup spi_group Serial Peripheral Interface (SPI)
 *
 * This is the common API for SPIs on AVRs. Additional features are available
 * in the documentation of the specific modules.
 *
 * \section spi_group_platform Platform Dependencies
 *
 * The spi API is partially chip- or platform-specific. While all
 * platforms provide mostly the same functionality, there are some
 * variations around how different bus types and clock tree structures
 * are handled.
 *
 * The following functions are available on all platforms, but there may
 * be variations in the function signature (i.e. parameters) and
 * behaviour. These functions are typically called by platform-specific
 * parts of drivers, and applications that aren't intended to be
 * portable:
 *   - spi_master_init()
 *   - spi_master_setup_device()
 *   - spi_select_device()
 *   - spi_deselect_device()
 *   - spi_write_single()
 *   - spi_write_packet()
 *   - spi_read_single()
 *   - spi_read_packet()
 *   - spi_is_tx_empty()
 *   - spi_is_tx_ready()
 *   - spi_is_rx_full()
 *   - spi_is_rx_ready()
 *   - spi_enable()
 *   - spi_disable()
 *   - spi_is_enabled()
 *
 *
 * @{
 */

//! @}

#endif /* SPI_MASTER_H_INCLUDED */
