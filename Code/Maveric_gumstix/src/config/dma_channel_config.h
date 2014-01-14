/*
 * dma_channel_config.h
 *
 * Created: 14/05/2012 15:22:08
 *  Author: sfx
 */ 


#ifndef DMA_CHANNEL_CONFIG_H_
#define DMA_CHANNEL_CONFIG_H_

#define SPI0_DMA_CH_TRANSMIT 0
#define SPI0_DMA_CH_RECEIVE 1
#define SPI0_DMA_IRQ AVR32_PDCA_IRQ_1

#define TWI0_DMA_CH 2
#define TWI1_DMA_CH 3
#define TWI0_DMA_IRQ AVR32_PDCA_IRQ_2
#define TWI1_DMA_IRQ AVR32_PDCA_IRQ_3

#endif /* DMA_CHANNEL_CONFIG_H_ */