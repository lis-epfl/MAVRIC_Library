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
 * \file spi_buffered.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief SPI functions for the ATMega / AVR microcontrollers
 * This library provides buffered SPI send and receive
 * 
 ******************************************************************************/


#include "spi_buffered.h"
#include "spi_master.h"
#include "gpio.h"
#include "pdca.h"
#include "led.h"

static volatile spi_buffer_t spi_buffers[SPI_NUMBER];				///< Allocated memory for SPI buffers

__attribute__((__interrupt__)) void spi0_int_handler(void);
__attribute__((__interrupt__)) void spi1_int_handler(void);
void spi_handler(int32_t spi_index);

/** interrupt handler
  * manages sending and receiving data
*/
__attribute__((__interrupt__))
void spi0_int_handler(void)
{
	//LED_On(LED1);
	spi_handler(0);
}

__attribute__((__interrupt__))
void spi1_int_handler(void)
{
	spi_handler(1);
}

/*! \brief The PDCA interrupt handler.
 */
__attribute__((__interrupt__))
static void pdca_int_handler_spi0(void)
{
	AVR32_PDCA.channel[SPI0_DMA_CH_RECEIVE].isr;
	AVR32_PDCA.channel[SPI0_DMA_CH_TRANSMIT].isr;
	pdca_disable(SPI0_DMA_CH_RECEIVE);
	pdca_disable(SPI0_DMA_CH_TRANSMIT);
	pdca_disable_interrupt_transfer_complete(SPI0_DMA_CH_RECEIVE);
	spi_deselect_device(spi_buffers[0].spi, (struct spi_device *)&spi_buffers[0].adc_spi);
	// call callback function to process data, at end of transfer
	// to process data, and maybe add some more data
	spi_buffers[0].spi_in_buffer_tail = spi_buffers[0].transmission_in_progress;
	spi_buffers[0].transmission_in_progress = 0;
	//spi_buffers[0].traffic++;
   
	if ((spi_buffers[0].callback_function)) spi_buffers[0].callback_function();
}

void spi_buffered_init(volatile avr32_spi_t *spi, int32_t spi_index)
{
	// init SPI
	spi_buffers[spi_index].spi= spi;
	
	spi_buffers[spi_index].adc_spi.id = 0;
	
	spi_master_init(spi_buffers[spi_index].spi);
	spi_master_setup_device(spi_buffers[spi_index].spi, (struct spi_device *)&spi_buffers[spi_index].adc_spi, SPI_MODE_0, 20000000, 0);
	
	//spi_buffers[spi_index].spi->cr = AVR32_SPI_SWRST_MASK;
	
	//spi_buffers[spi_index].spi->mr = AVR32_SPI_MSTR_MASK;

	
	gpio_enable_module_pin(AVR32_SPI0_MOSI_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION);
	//gpio_configure_pin(AVR32_SPI0_MOSI_0_0_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_enable_module_pin(AVR32_SPI0_MISO_0_0_PIN, AVR32_SPI0_MISO_0_0_FUNCTION);
	gpio_enable_module_pin(AVR32_SPI0_SCK_0_0_PIN, AVR32_SPI0_SCK_0_0_FUNCTION);

	spi_buffers[spi_index].spi_in_buffer_head = 0;
	spi_buffers[spi_index].spi_in_buffer_tail = 0;
	spi_buffers[spi_index].spi_out_buffer_head = 0;
	spi_buffers[spi_index].spi_out_buffer_tail = 0;
	spi_buffers[spi_index].spi_receiver_on = 1;
	spi_buffers[spi_index].traffic = 0;
	spi_buffers[spi_index].automatic = 1;
	spi_buffers[spi_index].callback_function = 0;
	spi_buffers[spi_index].transmission_in_progress = 0;
	
	//set up interrupt
		// Initialize interrupt vectors.
	//INTC_init_interrupts();
	//INTC_register_interrupt(&spi0_int_handler, AVR32_SPI0_IRQ, AVR32_INTC_INT0);
	
	//INTC_register_interrupt(&spi1_int_handler, AVR32_SPI1_IRQ, AVR32_INTC_INT0);
	//spi_buffers[spi_index].spi->imr = AVR32_SPI_RDRF;

	spi_buffered_enable(spi_index);
	//spi_buffers[spi_index].spi->cr = AVR32_SPI_SPIEN_MASK;	
}

uint8_t* spi_buffered_get_spi_in_buffer(int32_t spi_index)
{
	return (uint8_t*)spi_buffers[spi_index].spi_in_buffer;
}

void spi_buffered_init_DMA(int32_t spi_index, int32_t block_size)
{
	static  pdca_channel_options_t PDCA_TX_OPTIONS =
	{
		.addr = 0,                                // memory address
		.pid = AVR32_PDCA_PID_SPI0_TX,            // select peripheral - transmit to SPI0
		.size = 12,	                              // transfer counter
		.r_addr = NULL,                           // next memory address
		.r_size = 0,                              // next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer
	};
	static  pdca_channel_options_t PDCA_RX_OPTIONS =
	{
		.addr = 0,                                // memory address
		.pid = AVR32_PDCA_PID_SPI0_RX,            // select peripheral - receiving from SPI0
		.size = 12,                               // transfer counter
		.r_addr = NULL,                           // next memory address
		.r_size = 0,                              // next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer
	};
	
	PDCA_TX_OPTIONS.addr = (void *) spi_buffers[spi_index].spi_out_buffer;
	PDCA_RX_OPTIONS.addr = (void *) spi_buffers[spi_index].spi_in_buffer;
	
	// Init PDCA channel with the pdca_options.
	pdca_init_channel(SPI0_DMA_CH_TRANSMIT, &PDCA_TX_OPTIONS); // init PDCA channel with options.
	pdca_init_channel(SPI0_DMA_CH_RECEIVE, &PDCA_RX_OPTIONS); // init PDCA channel with options.

	// Register PDCA IRQ interrupt.
	INTC_register_interrupt( (__int_handler) &pdca_int_handler_spi0, SPI0_DMA_IRQ, AVR32_INTC_INT0);
}

void spi_buffered_trigger_DMA(int32_t spi_index, int32_t block_size)
{
	//spi_buffers[spi_index].spi_in_buffer_head = 0;
	//spi_buffers[spi_index].spi_in_buffer_tail = 0;
	//spi_buffers[spi_index].spi_out_buffer_head = 0;
	//spi_buffers[spi_index].spi_out_buffer_tail = 0;
	//spi_buffers[spi_index].transmission_in_progress = block_size;
	//spi_buffers[spi_index].spi_in_buffer[0] = 42;
	//spi_buffers[spi_index].spi_in_buffer[3] = 42;
	//spi_buffers[spi_index].spi_in_buffer[6] = 42;
	//spi_buffers[spi_index].spi_in_buffer[9] = 42;
	pdca_load_channel(SPI0_DMA_CH_TRANSMIT, (void *)spi_buffers[spi_index].spi_out_buffer, block_size);
	pdca_load_channel(SPI0_DMA_CH_RECEIVE,  (void *)(spi_buffers[spi_index].spi_in_buffer), block_size);

	
	spi_select_device(spi_buffers[spi_index].spi, (struct spi_device *)&spi_buffers[spi_index].adc_spi);
	// Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
	// the whole block was received
	pdca_enable_interrupt_transfer_complete(SPI0_DMA_CH_RECEIVE);
	
	pdca_enable(SPI0_DMA_CH_RECEIVE);
	pdca_enable(SPI0_DMA_CH_TRANSMIT);
}

void spi_buffered_set_callback(int32_t spi_index, function_pointer_t* function_pointer)
{
	spi_buffers[spi_index].callback_function = (volatile function_pointer_t*)function_pointer;
}

void spi_buffered_enable(int32_t spi_index)
{
	spi_enable(spi_buffers[spi_index].spi);
}

void spi_buffered_disable(int32_t spi_index)
{
	spi_disable(spi_buffers[spi_index].spi);
}

void spi_buffered_pause(int32_t spi_index)
{
	spi_buffers[spi_index].automatic = 0;
}

void spi_buffered_resume(int32_t spi_index)
{
	spi_buffers[spi_index].automatic = 1;
	spi_buffered_start(spi_index);
}

void spi_buffered_start(int32_t spi_index)
{
	// check flag if transmission is in progress
	if ((spi_buffers[spi_index].transmission_in_progress == 0)
	&&(spi_buffers[spi_index].spi_out_buffer_head != spi_buffers[spi_index].spi_out_buffer_tail))
	{
		// if not, initiate transmission by sending first byte
		//!!!!PORTB &= ~_BV(SPI_CS);	// pull chip select low to start transmission
		spi_select_device(spi_buffers[spi_index].spi, (struct spi_device *)&spi_buffers[spi_index].adc_spi);

		spi_buffers[spi_index].transmission_in_progress = 1;
		// activate interrupt to initiate transmission
		//!!!!SPCR|=_BV(SPIE);
		spi_buffers[spi_index].spi->ier = AVR32_SPI_IER_RDRF_MASK | AVR32_SPI_IER_TDRE_MASK;

		spi_buffered_transmit(spi_index);
	}
}

void spi_buffered_activate_receive(int32_t spi_index)
{
	spi_buffers[spi_index].spi_receiver_on = 1;
}

void spi_buffered_deactivate_receive(int32_t spi_index)
{
	spi_buffers[spi_index].spi_receiver_on = 0;
}

void spi_buffered_clear_read_buffer(int32_t spi_index)
{
	spi_buffers[spi_index].spi_in_buffer_tail = spi_buffers[spi_index].spi_in_buffer_head;
}

uint8_t spi_buffered_get_traffic(int32_t spi_index)
{
	return spi_buffers[spi_index].traffic;
}

uint8_t spi_buffered_read(int32_t spi_index)
{
	uint8_t byte;
	// if buffer empty, wait for incoming data
	while (spi_buffers[spi_index].spi_in_buffer_head == spi_buffers[spi_index].spi_in_buffer_tail);
	byte=spi_buffers[spi_index].spi_in_buffer[spi_buffers[spi_index].spi_in_buffer_tail];
	spi_buffers[spi_index].spi_in_buffer_tail=  (spi_buffers[spi_index].spi_in_buffer_tail + 1)&SPI_BUFFER_MASK;
	return byte;
}

void spi_buffered_write(int32_t spi_index, uint8_t value)
{
	uint8_t new_index;

	new_index = (spi_buffers[spi_index].spi_out_buffer_head + 1)&SPI_BUFFER_MASK;
	// check if buffer is already full and wait
	//while (new_index == spi_buffers[spi_index].spi_out_buffer_tail) 
	//{
	//if (spi_buffers[spi_index].automatic == 0) spi_buffered_resume(spi_index);
	//}
	spi_buffers[spi_index].spi_out_buffer[(spi_buffers[spi_index].spi_out_buffer_head)] = value;
	spi_buffers[spi_index].spi_out_buffer_head = new_index;


	if (spi_buffers[spi_index].automatic == 1) spi_buffered_start(spi_index);
}

void spi_buffered_transmit(int32_t spi_index)
{
	if (spi_buffers[spi_index].spi_out_buffer_head != spi_buffers[spi_index].spi_out_buffer_tail) 
	{
		// read data from buffer and copy it to SPI unit
		spi_buffers[spi_index].spi->tdr = spi_buffers[spi_index].spi_out_buffer[spi_buffers[spi_index].spi_out_buffer_tail];
		spi_buffers[spi_index].transmission_in_progress = 1;    
		// update buffer index
		spi_buffers[spi_index].spi_out_buffer_tail=  (spi_buffers[spi_index].spi_out_buffer_tail + 1)&SPI_BUFFER_MASK;
		//spi_enable(spi_buffers[spi_index].spi);
	} else {
		spi_buffers[spi_index].spi_out_buffer_tail=spi_buffers[spi_index].spi_out_buffer_head;
		//PORTB |= _BV(SPI_CS);	// pull chip select high to end transmission
		spi_deselect_device(spi_buffers[spi_index].spi, (struct spi_device *)&spi_buffers[spi_index].adc_spi);
		spi_buffers[spi_index].transmission_in_progress=0;
		spi_buffers[spi_index].spi->idr =AVR32_SPI_IER_RDRF_MASK | AVR32_SPI_IER_TDRE_MASK;
		//SPCR&=~_BV(SPIE);
	}
}

int8_t spi_buffered_is_transfered_finished(int32_t spi_index) 
{
	return (spi_buffers[spi_index].spi_out_buffer_head==spi_buffers[spi_index].spi_out_buffer_tail);
}

void spi_buffered_flush_buffer(int32_t spi_index)
{
	spi_buffered_resume(spi_index);
	while (spi_buffers[spi_index].spi_out_buffer_head!=spi_buffers[spi_index].spi_out_buffer_tail);
}

uint8_t spi_buffered_bytes_available(int32_t spi_index)
{
  return (SPI_BUFFER_SIZE + spi_buffers[spi_index].spi_in_buffer_head - spi_buffers[spi_index].spi_in_buffer_tail)&SPI_BUFFER_MASK;
}

void spi_handler(int32_t spi_index)
{
	uint8_t in_data;
	uint8_t tmp;
	in_data=spi_buffers[spi_index].spi->rdr;

	if ((spi_buffers[spi_index].spi->sr & AVR32_SPI_SR_TDRE_MASK)!=0) {
	// initiate transfer if necessary
	spi_buffered_transmit(spi_index);
	}	
	// only process received data when receiver is activated
	if ((spi_buffers[spi_index].spi_receiver_on==1)&& ((spi_buffers[spi_index].spi->sr & AVR32_SPI_SR_RDRF_MASK)!=0)) {
		// read incoming data from SPI port
	spi_buffers[spi_index].traffic++;

	tmp=(spi_buffers[spi_index].spi_in_buffer_head+1)&SPI_BUFFER_MASK;
    
	if (tmp==spi_buffers[spi_index].spi_in_buffer_tail) {
		//error: receive buffer overflow!!
		// lose old incoming data at the end of the buffer
		spi_buffers[spi_index].spi_in_buffer_tail=(spi_buffers[spi_index].spi_in_buffer_tail+1)&SPI_BUFFER_MASK;
	} 
	// store incoming data in buffer
	spi_buffers[spi_index].spi_in_buffer[spi_buffers[spi_index].spi_in_buffer_head] = in_data;
	// move head pointer forward
	spi_buffers[spi_index].spi_in_buffer_head=tmp;
	}
}
