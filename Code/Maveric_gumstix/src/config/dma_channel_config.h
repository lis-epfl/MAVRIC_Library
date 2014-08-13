/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file dma_channel_config.h
 *
 * This file configures the imu for the rev 4 of the maveric autopilot
 */ 


#ifndef DMA_CHANNEL_CONFIG_H_
#define DMA_CHANNEL_CONFIG_H_

#ifdef __cplusplus
extern "C" {
	#endif

#define SPI0_DMA_CH_TRANSMIT 0				///< Define the DMA channel to use with SPI transmission
#define SPI0_DMA_CH_RECEIVE 1				///< Define the DMA channel to use with SPI transmission
#define SPI0_DMA_IRQ AVR32_PDCA_IRQ_1		///< Define the DMA interruption pin

///< DMA definitions
#define TWI0_DMA_CH 2
#define TWI1_DMA_CH 3
#define TWI0_DMA_IRQ AVR32_PDCA_IRQ_2
#define TWI1_DMA_IRQ AVR32_PDCA_IRQ_3

#ifdef __cplusplus
}
#endif

#endif /* DMA_CHANNEL_CONFIG_H_ */