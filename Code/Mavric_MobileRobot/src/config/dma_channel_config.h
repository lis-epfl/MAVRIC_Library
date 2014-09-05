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
 * \file dma_channel_config.h
 * 
 * \author MAV'RIC Team
 * 
 ******************************************************************************/


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