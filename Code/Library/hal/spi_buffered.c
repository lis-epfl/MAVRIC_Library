 /* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file spi_buffered.c
 *
 *  SPI functions for the ATMega / AVR microcontrollers
 *  This library provides buffered SPI send and receive
 */


#include "spi_buffered.h"
#include "spi_master.h"
#include "gpio.h"
#include "pdca.h"
#include "led.h"


static volatile spi_buffer_t spi_buffers[SPI_NUMBER];				///< Allocated memory for SPI buffers

void spi_handler(int spi_index);

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
	spi_deselect_device(spi_buffers[0].spi, &spi_buffers[0].adc_spi);
	// call callback function to process data, at end of transfer
	// to process data, and maybe add some more data
	spi_buffers[0].SPIinBufferTail=spi_buffers[0].transmission_in_progress;
	spi_buffers[0].transmission_in_progress=0;
	//spi_buffers[0].traffic++;
   
	if ((spi_buffers[0].callbackFunction)) spi_buffers[0].callbackFunction();
}

void initSPI(volatile avr32_spi_t *spi, int spi_index)
{
	// init SPI
	spi_buffers[spi_index].spi= spi;
	
	spi_buffers[spi_index].adc_spi.id=0;
	
	spi_master_init(spi_buffers[spi_index].spi);
	spi_master_setup_device(spi_buffers[spi_index].spi, &spi_buffers[spi_index].adc_spi, SPI_MODE_0, 20000000, 0);
	
	//spi_buffers[spi_index].spi->cr=AVR32_SPI_SWRST_MASK;
	
	//spi_buffers[spi_index].spi->mr=AVR32_SPI_MSTR_MASK;

	
	gpio_enable_module_pin(AVR32_SPI0_MOSI_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION);
	//gpio_configure_pin(AVR32_SPI0_MOSI_0_0_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_enable_module_pin(AVR32_SPI0_MISO_0_0_PIN, AVR32_SPI0_MISO_0_0_FUNCTION);
	gpio_enable_module_pin(AVR32_SPI0_SCK_0_0_PIN, AVR32_SPI0_SCK_0_0_FUNCTION);

	spi_buffers[spi_index].SPIinBufferHead=0;
	spi_buffers[spi_index].SPIinBufferTail=0;
	spi_buffers[spi_index].SPIoutBufferHead=0;
	spi_buffers[spi_index].SPIoutBufferTail=0;
	spi_buffers[spi_index].spiReceiverOn=1;
	spi_buffers[spi_index].traffic=0;
	spi_buffers[spi_index].automatic=1;
	spi_buffers[spi_index].callbackFunction=0;
	spi_buffers[spi_index].transmission_in_progress=0;
	
	//set up interrupt
		// Initialize interrupt vectors.
	//INTC_init_interrupts();
	//INTC_register_interrupt(&spi0_int_handler, AVR32_SPI0_IRQ, AVR32_INTC_INT0);
	
	//INTC_register_interrupt(&spi1_int_handler, AVR32_SPI1_IRQ, AVR32_INTC_INT0);
	//spi_buffers[spi_index].spi->imr = AVR32_SPI_RDRF;

	enableSPI(spi_index);
	//spi_buffers[spi_index].spi->cr=AVR32_SPI_SPIEN_MASK;	
}

uint8_t* get_spi_in_buffer(int spi_index)
{
	return spi_buffers[spi_index].SPIInBuffer;
}

void spiInitDMA(int spi_index, int block_size)
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
	
	PDCA_TX_OPTIONS.addr=(void *) spi_buffers[spi_index].SPIOutBuffer;
	PDCA_RX_OPTIONS.addr=(void *) spi_buffers[spi_index].SPIInBuffer;
	
	// Init PDCA channel with the pdca_options.
	pdca_init_channel(SPI0_DMA_CH_TRANSMIT, &PDCA_TX_OPTIONS); // init PDCA channel with options.
	pdca_init_channel(SPI0_DMA_CH_RECEIVE, &PDCA_RX_OPTIONS); // init PDCA channel with options.

	// Register PDCA IRQ interrupt.
	INTC_register_interrupt( (__int_handler) &pdca_int_handler_spi0, SPI0_DMA_IRQ, AVR32_INTC_INT0);
}

void spiTriggerDMA(int spi_index, int block_size)
{
	//spi_buffers[spi_index].SPIinBufferHead=0;
	//spi_buffers[spi_index].SPIinBufferTail=0;
	//spi_buffers[spi_index].SPIoutBufferHead=0;
	//spi_buffers[spi_index].SPIoutBufferTail=0;
	//spi_buffers[spi_index].transmission_in_progress=block_size;
	//spi_buffers[spi_index].SPIInBuffer[0]=42;
	//spi_buffers[spi_index].SPIInBuffer[3]=42;
	//spi_buffers[spi_index].SPIInBuffer[6]=42;
	//spi_buffers[spi_index].SPIInBuffer[9]=42;
	pdca_load_channel(SPI0_DMA_CH_TRANSMIT, (void *)spi_buffers[spi_index].SPIOutBuffer, block_size);
	pdca_load_channel(SPI0_DMA_CH_RECEIVE,  (void *)(spi_buffers[spi_index].SPIInBuffer), block_size);

	
	spi_select_device(spi_buffers[spi_index].spi, &spi_buffers[spi_index].adc_spi);
	// Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
	// the whole block was received
	pdca_enable_interrupt_transfer_complete(SPI0_DMA_CH_RECEIVE);
	
	pdca_enable(SPI0_DMA_CH_RECEIVE);
	pdca_enable(SPI0_DMA_CH_TRANSMIT);
}

void setSPIcallBack(int spi_index, functionpointer* functionPointer)
{
	spi_buffers[spi_index].callbackFunction=functionPointer;
}

void enableSPI(int spi_index)
{
	spi_enable(spi_buffers[spi_index].spi);
}

void disableSPI(int spi_index)
{
	spi_disable(spi_buffers[spi_index].spi);
}

void pauseSPI(int spi_index)
{
	spi_buffers[spi_index].automatic=0;
}

void resumeSPI(int spi_index)
{
	spi_buffers[spi_index].automatic=1;
	startSPI(spi_index);
}

void startSPI(int spi_index)
{
	// check flag if transmission is in progress
	if ((spi_buffers[spi_index].transmission_in_progress==0)
	&&(spi_buffers[spi_index].SPIoutBufferHead!=spi_buffers[spi_index].SPIoutBufferTail))
	{
		// if not, initiate transmission by sending first byte
		//!!!!PORTB &= ~_BV(SPI_CS);	// pull chip select low to start transmission
		spi_select_device(spi_buffers[spi_index].spi, &spi_buffers[spi_index].adc_spi);

		spi_buffers[spi_index].transmission_in_progress=1;
		// activate interrupt to initiate transmission
		//!!!!SPCR|=_BV(SPIE);
		spi_buffers[spi_index].spi->ier = AVR32_SPI_IER_RDRF_MASK | AVR32_SPI_IER_TDRE_MASK;

		SPItransmit(spi_index);
	}
}

void activateReceiveSPI(int spi_index)
{
	spi_buffers[spi_index].spiReceiverOn=1;
}

void deactivateReceiveSPI(int spi_index)
{
	spi_buffers[spi_index].spiReceiverOn=0;
}

void clearSPIReadBuffer(int spi_index)
{
	spi_buffers[spi_index].SPIinBufferTail=spi_buffers[spi_index].SPIinBufferHead;
}

uint8_t getTraffic(int spi_index)
{
	return spi_buffers[spi_index].traffic;
}

uint8_t readSPI(int spi_index)
{
	uint8_t byte;
	// if buffer empty, wait for incoming data
	while (spi_buffers[spi_index].SPIinBufferHead==spi_buffers[spi_index].SPIinBufferTail);
	byte=spi_buffers[spi_index].SPIInBuffer[spi_buffers[spi_index].SPIinBufferTail];
	spi_buffers[spi_index].SPIinBufferTail=  (spi_buffers[spi_index].SPIinBufferTail+1)&SPI_BUFFER_MASK;
	return byte;
}

void writeSPI(int spi_index, uint8_t value)
{
	uint8_t newIndex;

	newIndex=(spi_buffers[spi_index].SPIoutBufferHead+1)&SPI_BUFFER_MASK;
	// check if buffer is already full and wait
	//while (newIndex==spi_buffers[spi_index].SPIoutBufferTail) 
	//{
	//if (spi_buffers[spi_index].automatic==0) resumeSPI(spi_index);
	//}
	spi_buffers[spi_index].SPIOutBuffer[(spi_buffers[spi_index].SPIoutBufferHead)]=value;
	spi_buffers[spi_index].SPIoutBufferHead = newIndex;


	if (spi_buffers[spi_index].automatic==1) startSPI(spi_index);
}

void SPItransmit(int spi_index)
{
	if (spi_buffers[spi_index].SPIoutBufferHead!=spi_buffers[spi_index].SPIoutBufferTail) 
	{
		// read data from buffer and copy it to SPI unit
		spi_buffers[spi_index].spi->tdr = spi_buffers[spi_index].SPIOutBuffer[spi_buffers[spi_index].SPIoutBufferTail];
		spi_buffers[spi_index].transmission_in_progress=1;    
		// update buffer index
		spi_buffers[spi_index].SPIoutBufferTail=  (spi_buffers[spi_index].SPIoutBufferTail+1)&SPI_BUFFER_MASK;
		//spi_enable(spi_buffers[spi_index].spi);
	} else {
		spi_buffers[spi_index].SPIoutBufferTail=spi_buffers[spi_index].SPIoutBufferHead;
		//PORTB |= _BV(SPI_CS);	// pull chip select high to end transmission
		spi_deselect_device(spi_buffers[spi_index].spi, &spi_buffers[spi_index].adc_spi);
		spi_buffers[spi_index].transmission_in_progress=0;
		spi_buffers[spi_index].spi->idr =AVR32_SPI_IER_RDRF_MASK | AVR32_SPI_IER_TDRE_MASK;
		//SPCR&=~_BV(SPIE);
	}
}

int8_t SPITransferFinished(int spi_index) 
{
	return (spi_buffers[spi_index].SPIoutBufferHead==spi_buffers[spi_index].SPIoutBufferTail);
}

void SPIFlushBuffer(int spi_index)
{
	resumeSPI(spi_index);
	while (spi_buffers[spi_index].SPIoutBufferHead!=spi_buffers[spi_index].SPIoutBufferTail);
}

uint8_t SPIBytesAvailable(int spi_index)
{
  return (SPI_BUFFER_SIZE + spi_buffers[spi_index].SPIinBufferHead - spi_buffers[spi_index].SPIinBufferTail)&SPI_BUFFER_MASK;
}

void spi_handler(int spi_index)
{
	uint8_t inData;
	uint8_t tmp;
	inData=spi_buffers[spi_index].spi->rdr;

	if ((spi_buffers[spi_index].spi->sr & AVR32_SPI_SR_TDRE_MASK)!=0) {
	// initiate transfer if necessary
	SPItransmit(spi_index);
	}	
	// only process received data when receiver is activated
	if ((spi_buffers[spi_index].spiReceiverOn==1)&& ((spi_buffers[spi_index].spi->sr & AVR32_SPI_SR_RDRF_MASK)!=0)) {
		// read incoming data from SPI port
	spi_buffers[spi_index].traffic++;

	tmp=(spi_buffers[spi_index].SPIinBufferHead+1)&SPI_BUFFER_MASK;
    
	if (tmp==spi_buffers[spi_index].SPIinBufferTail) {
		//error: receive buffer overflow!!
		// lose old incoming data at the end of the buffer
		spi_buffers[spi_index].SPIinBufferTail=(spi_buffers[spi_index].SPIinBufferTail+1)&SPI_BUFFER_MASK;
	} 
	// store incoming data in buffer
	spi_buffers[spi_index].SPIInBuffer[spi_buffers[spi_index].SPIinBufferHead] = inData;
	// move head pointer forward
	spi_buffers[spi_index].SPIinBufferHead=tmp;
	}
}
