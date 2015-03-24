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
 * \file i2c_driver.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief The i2c driver
 *
 ******************************************************************************/

 
#include "i2c_driver.h"
#include "gpio.h"
#include "pdca.h"
#include "sysclk.h"
#include "print_util.h"

static volatile i2c_schedule_event_t schedule[I2C_DEVICES][I2C_SCHEDULE_SLOTS];

static volatile int8_t current_schedule_slot[I2C_DEVICES];


/*!  The PDCA interrupt handler.
 */
__attribute__((__interrupt__))
static void pdca_int_handler_i2c0(void)
{
	AVR32_TWIM0.cr = AVR32_TWIM_CR_MDIS_MASK;
	pdca_disable(TWI0_DMA_CH);

	pdca_disable_interrupt_transfer_complete(TWI0_DMA_CH);

	// call callback function to process data, at end of transfer
	// to process data, and maybe add some more data
	schedule[0][current_schedule_slot[0]].transfer_in_progress = 0;

	if (schedule[0][current_schedule_slot[0]].callback) 
	{
		schedule[0][current_schedule_slot[0]].callback;
	}
   print_util_dbg_print( "!");
}


int32_t i2c_driver_init(uint8_t  i2c_device) 
{
	int32_t i;
	volatile avr32_twim_t *twim;
	
	switch (i2c_device) 
	{
		case 0: 
			twim = &AVR32_TWIM0;
			// Register PDCA IRQ interrupt.
			INTC_register_interrupt( (__int_handler) &pdca_int_handler_i2c0, TWI0_DMA_IRQ, AVR32_INTC_INT0);
			gpio_enable_module_pin(AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION);
			gpio_enable_module_pin(AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION);
		break;
		
		case 1:
			twim = &AVR32_TWIM1;// Register PDCA IRQ interrupt.
			INTC_register_interrupt( (__int_handler) &pdca_int_handler_i2c0, TWI1_DMA_IRQ, AVR32_INTC_INT0);
			gpio_enable_module_pin(AVR32_TWIMS1_TWCK_0_0_PIN, AVR32_TWIMS1_TWCK_0_0_FUNCTION);
			gpio_enable_module_pin(AVR32_TWIMS1_TWD_0_0_PIN, AVR32_TWIMS1_TWD_0_0_FUNCTION);
			gpio_enable_pin_pull_up(AVR32_TWIMS1_TWCK_0_0_PIN);
			gpio_enable_pin_pull_up(AVR32_TWIMS1_TWD_0_0_PIN);
		break;
		
		default: // invalid device ID
			return -1;
	}		
	for (i = 0; i < I2C_SCHEDULE_SLOTS; i++) 
	{
		schedule[i2c_device][i].active = -1;
	}
				
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	// Disable TWI interrupts
	if (global_interrupt_enabled) 
	{
		cpu_irq_disable ();
	}
	twim->idr = ~0UL;
	// Enable master transfer
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	// Reset TWI
	twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	
	if (global_interrupt_enabled) 
	{
		cpu_irq_enable ();
	}
	// Clear SR
	twim->scr = ~0UL;
	
	// register Register twim_master_interrupt_handler interrupt on level CONF_TWIM_IRQ_LEVEL
//	irqflags_t flags = cpu_irq_save();
//	irq_register_handler(twim_master_interrupt_handler,
//			CONF_TWIM_IRQ_LINE, CONF_TWIM_IRQ_LEVEL);
//	cpu_irq_restore(flags);
	
	// Select the speed
	if (twim_set_speed(twim, 100000, sysclk_get_pba_hz()) == 
			ERR_INVALID_ARG) 
	{	
		return ERR_INVALID_ARG;
	}
	return STATUS_OK;
}



int8_t  i2c_driver_reset(uint8_t  i2c_device) 
{
	volatile avr32_twim_t *twim;
	
	switch (i2c_device) 
	{
		case 0: 
			twim = &AVR32_TWIM0;
		break;
		case 1:
			twim = &AVR32_TWIM1;
		break;
		default: // invalid device ID
			return -1;
	}		
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	// Disable TWI interrupts
	if (global_interrupt_enabled) 
	{
		cpu_irq_disable ();
	}
	twim->idr = ~0UL;
	// Enable master transfer
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	// Reset TWI
	twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	if (global_interrupt_enabled) 
	{
		cpu_irq_enable ();
	}
	// Clear SR
	twim->scr = ~0UL;
}
int8_t  i2c_driver_add_request(uint8_t  i2c_device, i2c_schedule_event_t* new_event)
{
	// find free schedule slot
	int32_t i = 0;
	
	for (i = 0; i < I2C_SCHEDULE_SLOTS; i++) 
	{
		if(schedule[i2c_device][i].active < 0)
		{
			break;
		}
	}
	// add request to schedule
	if (i < I2C_SCHEDULE_SLOTS) 
	{
		new_event->schedule_slot = i;
		new_event->transfer_in_progress = 0;
		new_event->active=1;
		schedule[i2c_device][i] = *new_event;
	} 
	else 
	{
		i = -1;
	}
	// return assigned schedule slot
	return i;
}
int8_t  i2c_driver_change_request(uint8_t  i2c_device, i2c_schedule_event_t* new_event)
{
	int32_t i = new_event->schedule_slot;

	if ((i>=0) && (i < I2C_SCHEDULE_SLOTS)) 
	{
		new_event->transfer_in_progress = 0;
		new_event->active=1;
		schedule[i2c_device][i] = *new_event;
	}
}


int8_t  i2c_driver_trigger_request(uint8_t  i2c_device, uint8_t  schedule_slot) 
{
	// initiate transfer of given request
	// set up DMA channel
	volatile avr32_twim_t *twim;
	
	i2c_packet_conf_t* conf = &schedule[i2c_device][schedule_slot].config;
	static  pdca_channel_options_t PDCA_OPTIONS =
			{
				.addr = 0,                                // memory address
				.pid = AVR32_TWIM0_PDCA_ID_TX,            // select peripheral 
				.size = 4,	                              // transfer counter
				.r_addr = NULL,                           // next memory address
				.r_size = 0,                              // next transfer counter
				.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer
			};
	switch (i2c_device) 
	{
	case 0: 
		twim = &AVR32_TWIM0;
		twim->cr = AVR32_TWIM_CR_MEN_MASK;
		twim->cr = AVR32_TWIM_CR_SWRST_MASK;
		twim->cr = AVR32_TWIM_CR_MDIS_MASK;
		
		switch (conf->direction)  
		{
		case I2C_WRITE1_THEN_READ:
		case I2C_READ:
			PDCA_OPTIONS.pid = AVR32_TWIM0_PDCA_ID_RX;
			PDCA_OPTIONS.addr = (void *)conf->read_data;
			PDCA_OPTIONS.size=conf->read_count;
			// Init PDCA channel with the pdca_options.
			pdca_init_channel(TWI0_DMA_CH, &PDCA_OPTIONS); // init PDCA channel with options.
			break;
			
		case I2C_WRITE:
			PDCA_OPTIONS.pid = AVR32_TWIM0_PDCA_ID_TX;
			PDCA_OPTIONS.addr = (void *)conf->write_data;
			PDCA_OPTIONS.size=conf->write_count;
			
			// Init PDCA channel with the pdca_options.
			pdca_init_channel(TWI0_DMA_CH, &PDCA_OPTIONS); // init PDCA channel with options.
			pdca_load_channel(TWI0_DMA_CH, (void *)conf->write_data, conf->write_count);
			break;
		}
		
		//pdca_load_channel(TWI0_DMA_CH, (void *)schedule[i2c_device][schedule_slot].config.write_data, schedule[i2c_device][schedule_slot].config.write_count);
		// Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
		// the whole block was received
		pdca_enable_interrupt_transfer_complete(TWI0_DMA_CH);
		pdca_enable_interrupt_transfer_error(TWI0_DMA_CH);
	break;
		
	case 1:
		twim = &AVR32_TWIM1;
	break;
	
	default: // invalid device ID
		return -1;
	}		

	// set up I2C speed and mode
	//twim_set_speed(twim, 100000, sysclk_get_pba_hz());
    
	switch (conf->direction)  
	{
		case I2C_READ:
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| (conf->read_count << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						| (0 << AVR32_TWIM_CMDR_READ_OFFSET);
		break;
		
		case I2C_WRITE1_THEN_READ:
			print_util_dbg_print( "wr");
			
			// set up next command register for the burst read transfer
			// set up command register to initiate the write transfer. The DMA will take care of the reading once this is done.
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| (1 << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						;
			
			twim->ncmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->read_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						| (0 << AVR32_TWIM_CMDR_READ_OFFSET);
			// set up writing of one byte (usually a slave register index)
			//twim->cr = AVR32_TWIM_CR_MEN_MASK;
			twim->thr = conf->write_then_read_preamble;
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
		break;	
			
		case I2C_WRITE:
			print_util_dbg_print( "w");
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->write_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						;
			twim->ncmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->write_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						//| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						;	
			
		break;	
	}		
	// start transfer

	current_schedule_slot[i2c_device] = schedule_slot;
	schedule[i2c_device][schedule_slot].transfer_in_progress = 1;
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	pdca_enable(TWI0_DMA_CH);
	return 0;
}

int8_t  i2c_driver_pause_request(uint8_t  i2c_device, uint8_t  schedule_slot)
{
	// pause scheduler
	// if this request currently active, wait for current transfer to finish
	// deactivate request
	// resume scheduler
}

int8_t  i2c_driver_enable_request(uint8_t  i2c_device, uint8_t  schedule_slot){
	return 0;
}

int8_t  i2c_driver_remove_request(uint8_t  i2c_device, uint8_t  schedule_slot){
	return 0;
}


