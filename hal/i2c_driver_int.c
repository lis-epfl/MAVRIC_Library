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
 * \file i2c_driver_int.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for i2c with interruptions
 *
 ******************************************************************************/


#include "i2c_driver_int.h"
#include "gpio.h"
#include "sysclk.h"
#include "print_util.h"

i2c_packet_t transfer_queue[I2C_SCHEDULE_SLOTS];		///< buffer containing the transfer queue for the i2c
int32_t current_slot, last_slot;							///< current and last slot for the scheduling of the transfer using i2c
i2c_packet_t *current_transfer;							///< pointer to the i2c structure

///< Function prototype definition
/**
 * \brief Reset the i2c driver
 *
 * \param i2c_device i2c device number
 *
 * \return error status
 */
int8_t  i2c_driver_reset(uint8_t  i2c_device);
/**
 * \brief Trigger a request on the i2c driver
 *
 * \param	i2c_device	i2c device number
 * \param	transfer	Pointer to an object containing the i2c packet structure
 *
 * \return error status
 */
int8_t  i2c_driver_trigger_request(uint8_t  i2c_device, i2c_packet_t *transfer);

/**  
 * \brief The I2C interrupt handler.
*/
ISR(i2c_int_handler_i2c0,CONF_TWIM_IRQ_GROUP,CONF_TWIM_IRQ_LEVEL)
{
	volatile avr32_twim_t *twim = &AVR32_TWIM0;
	///< get masked status register value
	uint32_t status = twim->sr &(AVR32_TWIM_SR_STD_MASK |AVR32_TWIM_SR_TXRDY_MASK |AVR32_TWIM_SR_RXRDY_MASK) ;
	
	///< this is a NACK
	if (status & AVR32_TWIM_SR_STD_MASK) 
	{
		///< if we get a nak, clear the valid bit in cmdr, 
		///< otherwise the command will be resent.
		current_transfer->transfer_in_progress = false;
		twim->CMDR.valid = 0;
		twim->scr = ~0UL;
		twim->idr = ~0UL;
	}
	///< this is a RXRDY
	else if (status & AVR32_TWIM_SR_RXRDY_MASK) 
	{	
		///< get data from Receive Holding Register
		if (current_transfer->data_index < current_transfer->data_size) 
		{
			current_transfer->data[current_transfer->data_index]= twim->rhr;
			current_transfer->data_index++;
		} 
		else 
		{			
			///< finish the receive operation
			twim->idr = AVR32_TWIM_IDR_RXRDY_MASK;
			twim->cr = AVR32_TWIM_CR_MDIS_MASK;
			///< set busy to false
			current_transfer->transfer_in_progress = false;
		}
	}
	///< this is a TXRDY
	else if (status & AVR32_TWIM_SR_TXRDY_MASK) 
	{	
		///< get data from transmit data block
		if (current_transfer->data_index < current_transfer->data_size) 
		{	
			///< put the byte in the Transmit Holding Register
			twim->thr = current_transfer->data[current_transfer->data_index];
			current_transfer->data_index++;
		} else 
		{ 
			///<nothing more to write
			twim->idr = AVR32_TWIM_IDR_TXRDY_MASK;
			if (current_transfer->direction == I2C_WRITE1_THEN_READ) 
			{
				///< reading should already be set up in next command register...	
			}	
			else  
			{ 
				///< all done
				twim->cr = AVR32_TWIM_CR_MDIS_MASK;
				///< set busy to false
				current_transfer->transfer_in_progress = false;				
			}
		}
	}
	//return;
	
	///< call callback function to process data, at end of transfer
	///< to process data, and maybe add some more data
	//schedule[0][current_schedule_slot[0]].transfer_in_progress = 0;
   
	//if (schedule[0][current_schedule_slot[0]].callback) schedule[0][current_schedule_slot[0]].callback;
	//print_util_putstring(&AVR32_USART0, "!");
}


void i2c_driver_init(uint8_t  i2c_device, twim_options_t twi_opt) 
{
	volatile avr32_twim_t *twim;
	switch (i2c_device) 
	{
	case I2C0: 
		twim = &AVR32_TWIM0;
		///< Register PDCA IRQ interrupt.
		INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c0, AVR32_TWIM0_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin(AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION);

	break;
	case I2C1:
		twim = &AVR32_TWIM1;///< Register PDCA IRQ interrupt.
		//INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c1, AVR32_TWIM1_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin(AVR32_TWIMS1_TWCK_0_0_PIN, AVR32_TWIMS1_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS1_TWD_0_0_PIN, AVR32_TWIMS1_TWD_0_0_FUNCTION);
	break;
	default: ///< invalid device ID
		return;
	}
	
	twi_opt.pba_hz = sysclk_get_pba_hz();
	
	status_code_t ret_twim_init = twim_master_init(twim, &twi_opt);
	
	print_util_dbg_print("\r\n");
	
	switch(ret_twim_init)
	{
		case ERR_IO_ERROR :
			print_util_dbg_print("NO Twim probe here \r\n");
		case STATUS_OK :
			print_util_dbg_print("I2C initialised \r\n");
			break;
		default :
			print_util_dbg_print("Error initialising I2C \r\n");
			break;
	}
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
		default: ///< invalid device ID
			return -1;
	}		
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	///< Disable TWI interrupts
	if (global_interrupt_enabled) 
	{
		cpu_irq_disable ();
	}
	twim->idr = ~0UL;
	///< Enable master transfer
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	///< Reset TWI
	twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	if (global_interrupt_enabled) 
	{
		cpu_irq_enable ();
	}
	///< Clear SR
	twim->scr = ~0UL;
	
	return STATUS_OK; //No error
}

int8_t  i2c_driver_trigger_request(uint8_t  i2c_device, i2c_packet_t *transfer) 
{
	///< initiate transfer of given request
	///< set up DMA channel
	volatile avr32_twim_t *twim;
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	
	if (global_interrupt_enabled) 
	{
		cpu_irq_disable ();
	}
	
	switch (i2c_device) 
	{
		case 0: 
			twim = &AVR32_TWIM0;
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
			twim->cr = AVR32_TWIM_CR_SWRST_MASK;
			twim->cr = AVR32_TWIM_CR_MDIS_MASK;
			twim->scr = ~0UL;
			// Clear the interrupt flags
			twim->idr = ~0UL;
			if (twim_set_speed(twim, transfer->i2c_speed, sysclk_get_pba_hz()) == ERR_INVALID_ARG) 
			{
				return ERR_INVALID_ARG;
			}
		
			//pdca_load_channel(TWI0_DMA_CH, (void *)schedule[i2c_device][schedule_slot].config.write_data, schedule[i2c_device][schedule_slot].config.write_count);
			///< Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
			///< the whole block was received
		
			//pdca_enable_interrupt_transfer_error(TWI0_DMA_CH);		
			break;
		case 1:
			twim = &AVR32_TWIM1;
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
			twim->cr = AVR32_TWIM_CR_SWRST_MASK;
			twim->cr = AVR32_TWIM_CR_MDIS_MASK;
			twim->scr = ~0UL;
			///< Clear the interrupt flags
			twim->idr = ~0UL;
			if (twim_set_speed(twim, transfer->i2c_speed, sysclk_get_pba_hz()) == ERR_INVALID_ARG) 
			{
				return ERR_INVALID_ARG;
			}
			break;
		default: ///< invalid device ID
			return -1;
	}		

	///< set up I2C speed and mode
	//twim_set_speed(twim, 100000, sysclk_get_pba_hz());
    
	switch (1/*conf->direction*/)  
	{
		case I2C_READ:
 			twim->cmdr = (transfer->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
 						| (transfer->data_size << AVR32_TWIM_CMDR_NBYTES_OFFSET)
 						| (AVR32_TWIM_CMDR_VALID_MASK)
 						| (AVR32_TWIM_CMDR_START_MASK)
 						| (AVR32_TWIM_CMDR_STOP_MASK)
 						| (AVR32_TWIM_CMDR_READ_MASK);
			twim->ncmdr = 0;					
			twim->ier = AVR32_TWIM_IER_STD_MASK |  AVR32_TWIM_IER_RXRDY_MASK;
			break;	
		case I2C_WRITE1_THEN_READ:
			///< set up next command register for the burst read transfer
			///< set up command register to initiate the write transfer. The DMA will take care of the reading once this is done.
 			twim->cmdr = (transfer->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
 						| ((1)<< AVR32_TWIM_CMDR_NBYTES_OFFSET)
 						| (AVR32_TWIM_CMDR_VALID_MASK)
 						| (AVR32_TWIM_CMDR_START_MASK)
 						//| (AVR32_TWIM_CMDR_STOP_MASK)
 						| (0 << AVR32_TWIM_CMDR_READ_OFFSET);
 						;	 
 			twim->ncmdr = (transfer->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
 						| ((transfer->data_size) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
 						| (AVR32_TWIM_CMDR_VALID_MASK)
 						| (AVR32_TWIM_CMDR_START_MASK)
 						| (AVR32_TWIM_CMDR_STOP_MASK)
 						| (AVR32_TWIM_CMDR_READ_MASK);
			twim->ier = AVR32_TWIM_IER_STD_MASK | AVR32_TWIM_IER_RXRDY_MASK | AVR32_TWIM_IER_TXRDY_MASK;
			///< set up writing of one byte (usually a slave register index)
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
			twim->thr = transfer->write_then_read_preamble;
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
			break;	
		case I2C_WRITE:
 			twim->cmdr = (transfer->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
 						| ((transfer->data_size) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
 						| (AVR32_TWIM_CMDR_VALID_MASK)
 						| (AVR32_TWIM_CMDR_START_MASK)
 						| (AVR32_TWIM_CMDR_STOP_MASK);
			twim->ncmdr = 0;						;	
			twim->ier = AVR32_TWIM_IER_NAK_MASK |  AVR32_TWIM_IER_TXRDY_MASK;
		break;	
	}
			
	///< start transfer
	current_transfer = transfer;
	current_transfer->transfer_in_progress = 1;
	current_transfer->data_index = 0;
	
	if (global_interrupt_enabled) 
	{
		cpu_irq_enable ();
	}	
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	
	return STATUS_OK;
}

//NOT IMPLEMENTED YET
//int32_t i2c_append_transfer(i2c_packet_t *request) {};
//
//int32_t i2c_append_read_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler) {};
	//
//int32_t i2c_append_write_transfer(uint8_t slave_address, uint8_t *data, uint16_t size, task_handle_t *event_handler) {};
	//
//int32_t i2c_append_register_read_transfer(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint16_t size, task_handle_t *event_handler) {};
//
//int32_t i2c_clear_queue() {};
//
//bool i2c_is_ready() {};
//
//int32_t i2c_get_queued_requests() {};
