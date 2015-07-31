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
 * \file spi_buffered.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief SPI functions for the ATMega / AVR microcontrollers
 * This library provides buffered SPI send and receive
 * 
 ******************************************************************************/


#ifndef SPI_BUFFERED_H
#define SPI_BUFFERED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <avr32/io.h>
#include "preprocessor.h"
#include <stdint.h>
#include "user_board.h"
#include "spi_master.h"
#include "dma_channel_config.h"
typedef void (function_pointer_t)(void);

#define SPI_BUFFER_SIZE 32								///< The SPI buffer size, this has to be a power of 2

#define SPI_BUFFER_MASK (SPI_BUFFER_SIZE - 1)			///< The SPI buffer mask

#define SPI_NUMBER 2									///< The number of SPI


/**
 * \brief The SPI buffer structure definition
 */
typedef struct 
{
	volatile avr32_spi_t *spi;								///< The pointer to the avr32 spi structure
	volatile uint8_t spi_out_buffer[SPI_BUFFER_SIZE];		///< The SPI outgoing buffer of size SPI_BUFFER_SIZE
	volatile uint8_t spi_in_buffer[SPI_BUFFER_SIZE];		///< The SPI ingoing buffer of size SPI_BUFFER_SIZE
	volatile uint8_t spi_in_buffer_head;					///< The head of the SPI ingoing buffer
	volatile uint8_t spi_in_buffer_tail;					///< The tail of the SPI ingoing buffer
	volatile uint8_t spi_out_buffer_head;					///< The head of the SPI outgoing buffer
	volatile uint8_t spi_out_buffer_tail;					///< The tail of the SPI outgoing buffer
	volatile uint8_t spi_receiver_on;						///< Flag to activate or not the SPI reception
	volatile uint8_t traffic;								///< Read incoming data from SPI port
	volatile uint8_t transmission_in_progress;				///< Flag to know if there is a transmission going on
	volatile uint8_t automatic;								///< Flag to send automatically over SPI or pause/stop transmission
	volatile function_pointer_t* callback_function;			///< The callback function that gets called when the buffer is empty
	struct spi_device adc_spi;								///< The SPI device
} spi_buffer_t;


/** 
 * \brief Initialize SPI interface: this interface uses internal buffers and interrupts. 
 * Bytes in the outgoing buffer are sent automatically via interrupt
 * 
 * \param	spi					The SPI chip address
 * \param	spi_index			The SPI index
 */
void spi_buffered_init(volatile avr32_spi_t *spi, int32_t spi_index);

/**
 * \brief	To get the SPI ingoing buffer
 *
 * \param	spi_index			The index of the SPI buffer wanted
 *
 * \return	The pointer to the incoming SPI buffer
 */
uint8_t* spi_buffered_get_spi_in_buffer(int32_t spi_index);

/** 
 * \brief	InitialiZe DMA-based SPI transfer mode
 *
 * \param	spi_index			The SPI index
 * \param	block_size			The block size
 */
void spi_buffered_init_DMA(int32_t spi_index, int32_t block_size);

/**
 * \brief specify a callback function, that gets called when a DMA transfer completes. 
 * start a DMA-based transfer of given size. This will reset the output and input buffers.
 * At completion, the bytes read from SPI will be in the SPI read buffer, and the callback function will be called (if specified).
 *
 * \param	spi_index			The index of the SPI
 * \param	block_size			The size of the block
 */
void spi_buffered_trigger_DMA(int32_t spi_index, int32_t block_size);

/** 
 * \brief	Specify a callback function, that gets called when the SPI buffer is empty
 *
 * \param	spi_index			The index of the SPI
 * \param	function_pointer		The pointer to the callback function
 */
void spi_buffered_set_callback(int32_t spi_index, function_pointer_t* function_pointer);

/**
 * \brief	Enables SPI communication
 *
 * \param	spi_index			The index of the SPI to enable
 */
void spi_buffered_enable(int32_t spi_index);

/**
 * \brief	Disables SPI communication
 *
 * \param	spi_index			The index of the SPI to disable
 */
void spi_buffered_disable(int32_t spi_index);

/**
 * \brief	Pauses the sending on SPI
 *
 * \param	spi_index			The index of the SPI to pause
 */
void spi_buffered_pause(int32_t spi_index);

/**
 * \brief	Resumes the SPI automatic sending
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_resume(int32_t spi_index);

/**
 * \brief	Initiates pending transmissions by sending first byte, if in paused or non-automatic mode
 *
 * \param	spi_index			The index of the SPI
 */
void  spi_buffered_start(int32_t spi_index);

/**
 * \brief	Activates reception of SPI
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_activate_receive(int32_t spi_index);

/**
 * \brief	Deactivates reception of SPI
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_deactivate_receive(int32_t spi_index);

/**
 * \brief	Clear SPI buffer
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_clear_read_buffer(int32_t spi_index);

/**
 * \brief	Get the traffic of SPI line index
 *
 * \param	spi_index			The index of the SPI
 *
 * \return	The traffic of SPI index
 */
uint8_t spi_buffered_get_traffic(int32_t spi_index);

/** 
 * \brief	Reads one byte from the incoming SPI buffer, if the buffer is empty, waits for data
 *
 * \param	spi_index			The index of the SPI
 *
 * \return	The byte read
 */
uint8_t spi_buffered_read(int32_t spi_index);

/**
 * \brief	Writes one byte to the outgoing SPI buffer
 * if buffer is full, this method blocks
 *
 * \warning if sending is paused and buffer runs full, sending is automatically resumed!!
 *
 * \param	spi_index			The index of the SPI
 * \param	value				The byte to be written on the SPI
 */
void spi_buffered_write(int32_t spi_index, uint8_t value);

/**
 * \brief	Read data from buffer index and copy it to SPI unit
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_transmit(int32_t spi_index);

/**
 * \brief	Check if the transmission is finished
 *
 * \param	spi_index			The index of the SPI
 *
 * \return	1 if the transmission is finished, 0 otherwise
 */
int8_t spi_buffered_is_transfered_finished(int32_t spi_index);

/** 
 * \brief	Waits until whole buffer is written to SPI bus
 *  automatically resumes sending if SPI interface was paused 
 *
 * \param	spi_index			The index of the SPI
 */
void spi_buffered_flush_buffer(int32_t spi_index);

/** 
 * \brief	Returns the number of bytes in the incoming buffer
 *
 * \param	spi_index			The index of the SPI
 *
 * \return	Returns the number of bytes in the incoming buffer
 */
uint8_t spi_buffered_bytes_available(int32_t spi_index);

#ifdef __cplusplus
}
#endif

#endif // SPI_BUFFERED_H