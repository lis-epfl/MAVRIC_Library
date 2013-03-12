/** Driver for DMA-based SPI 
Author: Felix Schill  

All rights reserved (2011).
*/



#ifndef SPI_BUFFERED_H
#define SPI_BUFFERED_H
#include <avr32/io.h>
#include "preprocessor.h"
#include "compiler.h"
#include "user_board.h"
#include "spi_master.h"
#include "dma_channel_config.h"
typedef void (functionpointer)(void);

#define SPI_BUFFER_SIZE 32      // this has to be a power of 2

#define SPI_BUFFER_MASK (SPI_BUFFER_SIZE-1)

#define SPI_NUMBER 2


typedef struct {

  volatile avr32_spi_t *spi;
  volatile uint8_t SPIOutBuffer[SPI_BUFFER_SIZE];
  volatile uint8_t SPIInBuffer[SPI_BUFFER_SIZE];
  volatile uint8_t SPIinBufferHead, SPIinBufferTail, SPIoutBufferHead, SPIoutBufferTail,spiReceiverOn, traffic, transmission_in_progress, automatic;
  volatile functionpointer* callbackFunction; 
  struct spi_device adc_spi;
 
} spi_buffer_t;


/** initialises SPI interface
 *  this interface uses internal buffers and interrupts. 
 *  bytes in the outgoing buffer are sent automatically via interrupt
 */
void initSPI(volatile avr32_spi_t *spi, int spi_index);

uint8_t* get_spi_in_buffer(int spi_index);

// initialise DMA-based SPI transfer mode
void spiInitDMA(int spi_index, int block_size);
/** specify a callback function, that gets called when a DMA transfer completes
 */

// start a DMA-based transfer of given size. This will reset the output and input buffers. 
// At completion, the bytes read from SPI will be in the SPI read buffer, and the 
// callback function will be called (if specified).
void spiTriggerDMA(int spi_index, int block_size);
void setSPIcallBack(int spi_index, functionpointer* functionPointer);

void enableSPI(int spi_index);
void disableSPI(int spi_index);

/** pauses sending */
void pauseSPI(int spi_index);

/** resumes automatic sending
 */
 void resumeSPI(int spi_index);

/** initiates pending transmissions, if in paused or non-automatic mode */
void  startSPI(int spi_index);


void activateReceiveSPI(int spi_index);

void deactivateReceiveSPI(int spi_index);

void clearSPIReadBuffer(int spi_index);

uint8_t getTraffic(int spi_index);
/** reads one byte from the incoming SPI buffer
 */
uint8_t readSPI(int spi_index);

/** writes one byte to the outgoing SPI buffer
 */
void writeSPI(int spi_index, uint8_t value);


int8_t SPITransferFinished(int spi_index);

/** waits until all SPI bytes are sent
*/
void SPIFlushBuffer(int spi_index);

/** returns the number of bytes in the incoming buffer
 */
uint8_t SPIBytesAvailable(int spi_index);




#endif