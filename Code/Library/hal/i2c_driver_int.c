/*
 * i2c_driver.c
 *
 * Created: 16/05/2012 17:31:58
 *  Author: sfx
 */ 
#include "i2c_driver_int.h"
#include "gpio.h"
#include "sysclk.h"
#include "print_util.h"

static volatile i2c_schedule_event schedule[I2C_DEVICES][I2C_SCHEDULE_SLOTS];

static volatile char current_schedule_slot[I2C_DEVICES];


/*!  The I2C interrupt handler.
 */

ISR(i2c_int_handler_i2c0,CONF_TWIM_IRQ_GROUP,CONF_TWIM_IRQ_LEVEL)
//__attribute__((__interrupt__))
//static void i2c_int_handler_i2c0(void)
{
	 volatile avr32_twim_t *twim = &AVR32_TWIM0;
	 i2c_schedule_event* current_event=&schedule[I2C_DEVICES][current_schedule_slot[0]];
		// get masked status register value
	uint32_t status = twim->sr &(AVR32_TWIM_SR_STD_MASK |AVR32_TWIM_SR_TXRDY_MASK |AVR32_TWIM_SR_RXRDY_MASK) ;
	// this is a NACK
	if (status & AVR32_TWIM_SR_STD_MASK) {
		//if we get a nak, clear the valid bit in cmdr, 
		//otherwise the command will be resent.
		current_event->transfer_in_progress =(status & AVR32_TWIM_IER_NAK_MASK) ? 
							TWI_RECEIVE_NACK : TWI_ARBITRATION_LOST;
		twim->CMDR.valid = 0;
		twim->scr = ~0UL;
		twim->idr = ~0UL;
		putstring(&AVR32_USART0, "E");
	}
	// this is a RXRDY
	else if (status & AVR32_TWIM_SR_RXRDY_MASK) {
		putstring(&AVR32_USART0, "R");
		
		
		// get data from Receive Holding Register
		if (current_event->config.read_data_index < current_event->config.read_count) {
			current_event->config.read_data[current_event->config.read_data_index]= twim->rhr;
			current_event->config.read_data_index++;
		} else {			
			// receive complete
			putstring(&AVR32_USART0, "Fr");
			// finish the receive operation
			twim->idr = AVR32_TWIM_IDR_RXRDY_MASK;
			twim->cr = AVR32_TWIM_CR_MDIS_MASK;
			// set busy to false
			if (current_event->repetition_rate_ms==0) {
				current_event->active=0;
			}			
			current_event->transfer_in_progress=0;
		}
	}
	// this is a TXRDY
	else if (status & AVR32_TWIM_SR_TXRDY_MASK) {
		
		putstring(&AVR32_USART0, "W");
		// get data from transmit data block
		if (current_event->config.write_data_index < current_event->config.write_count) {
			
			// put the byte in the Transmit Holding Register
			twim->thr = current_event->config.write_data[current_event->config.write_data_index];
			current_event->config.write_data_index++;
			
		} else { //nothing more to write
			twim->idr = AVR32_TWIM_IDR_TXRDY_MASK;
			
			putstring(&AVR32_USART0, "Fw");
			if (current_event->config.direction==I2C_WRITE1_THEN_READ) {
				
				// reading should already be set up in next command register...
				
			}	else  { // all done
				twim->cr = AVR32_TWIM_CR_MDIS_MASK;
				// set busy to false
				if (current_event->repetition_rate_ms==0) {
					current_event->active=0;
				}			
				current_event->transfer_in_progress=0;				
			}		
			
		
		}
		
	}
	//return;
	
   // call callback function to process data, at end of transfer
   // to process data, and maybe add some more data
//   schedule[0][current_schedule_slot[0]].transfer_in_progress=0;
   
//   if (schedule[0][current_schedule_slot[0]].callback) schedule[0][current_schedule_slot[0]].callback;
   //putstring(&AVR32_USART0, "!");
}

/*!  The I2C interrupt handler.
 */
__attribute__((__interrupt__))
static void i2c_int_handler_i2c1(void)
{

}



int init_i2c(unsigned char i2c_device) {
	int i;
	volatile avr32_twim_t *twim;
	switch (i2c_device) {
	case 0: 
		twim=&AVR32_TWIM0;
		// Register PDCA IRQ interrupt.
		INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c0, AVR32_TWIM0_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin(AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION);

	break;
	case 1:
		twim=&AVR32_TWIM1;// Register PDCA IRQ interrupt.
		INTC_register_interrupt( (__int_handler) &i2c_int_handler_i2c1, AVR32_TWIM1_IRQ, AVR32_INTC_INT1);
		gpio_enable_module_pin(AVR32_TWIMS1_TWCK_0_0_PIN, AVR32_TWIMS1_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS1_TWD_0_0_PIN, AVR32_TWIMS1_TWD_0_0_FUNCTION);
		//gpio_enable_pin_pull_up(AVR32_TWIMS1_TWCK_0_0_PIN);
		//gpio_enable_pin_pull_up(AVR32_TWIMS1_TWD_0_0_PIN);
	break;
	default: // invalid device ID
		return -1;
	}		
	for (i=0; i<I2C_SCHEDULE_SLOTS; i++) {
		schedule[i2c_device][i].active=-1;
	}
				
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	// Disable TWI interrupts
	if (global_interrupt_enabled) {
		cpu_irq_disable ();
	}
	twim->idr = ~0UL;
	// Enable master transfer
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	// Reset TWI
	twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	twim->cr = AVR32_TWIM_CR_MDIS_MASK;
	
	
	if (global_interrupt_enabled) {
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
	if (twim_set_speed(twim, 200000, sysclk_get_pba_hz()) == 
			ERR_INVALID_ARG) {
		
		return ERR_INVALID_ARG;
	}
	return STATUS_OK;				

}



char i2c_reset(unsigned char i2c_device) {
	volatile avr32_twim_t *twim;
	switch (i2c_device) {
	case 0: 
		twim=&AVR32_TWIM0;
	break;
	case 1:
		twim=&AVR32_TWIM1;
	break;
	default: // invalid device ID
		return -1;
	}		
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	// Disable TWI interrupts
	if (global_interrupt_enabled) {
		cpu_irq_disable ();
	}
	twim->idr = ~0UL;
	// Enable master transfer
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	// Reset TWI
	twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	if (global_interrupt_enabled) {
		cpu_irq_enable ();
	}
	// Clear SR
	twim->scr = ~0UL;
}
char i2c_add_request(unsigned char i2c_device, i2c_schedule_event* new_event){
	// find free schedule slot
	int i=0;
	for (i=0; (i<I2C_SCHEDULE_SLOTS)&& (schedule[i2c_device][i].active>=0); i++) {
		putstring(&AVR32_USART0, ".");
	}
	// add request to schedule
	if (i<I2C_SCHEDULE_SLOTS) {
		new_event->schedule_slot=i;
		new_event->transfer_in_progress=0;
		new_event->active=1;
		schedule[i2c_device][i]=*new_event;
		schedule[i2c_device][i].config=new_event->config;
		putstring(&AVR32_USART0, "slave address:");
		putnum(&AVR32_USART0, schedule[i2c_device][i].config.slave_address, 2);
	} else i=-1;
	// return assigned schedule slot
	return i;
}
char i2c_change_request(unsigned char i2c_device, i2c_schedule_event* new_event){
	int i=new_event->schedule_slot;
	if ((i>=0) && (i<I2C_SCHEDULE_SLOTS)) {
		new_event->transfer_in_progress=0;
		new_event->active=1;
		schedule[i2c_device][i]=*new_event;
	};
}


char i2c_trigger_request(unsigned char i2c_device, unsigned char schedule_slot) {
	// initiate transfer of given request
	// set up DMA channel
	volatile avr32_twim_t *twim;
	i2c_packet_conf* conf=&schedule[i2c_device][schedule_slot].config;
	
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	if (global_interrupt_enabled) {
		cpu_irq_disable ();
	}
	
	switch (i2c_device) {
	case 0: 
		twim=&AVR32_TWIM0;
		twim->cr = AVR32_TWIM_CR_MEN_MASK;
		twim->cr = AVR32_TWIM_CR_SWRST_MASK;
		twim->cr = AVR32_TWIM_CR_MDIS_MASK;
		twim->scr = ~0UL;
		// Clear the interrupt flags
		twim->idr = ~0UL;
		if (twim_set_speed(twim, 200000, sysclk_get_pba_hz()) == 
			ERR_INVALID_ARG) {
			return ERR_INVALID_ARG;
	    }
		switch (conf->direction)  {
		case I2C_WRITE1_THEN_READ:
		case I2C_READ:
			
			break;
		case I2C_WRITE:
			
			break;
		}
		
//		pdca_load_channel(TWI0_DMA_CH, (void *)schedule[i2c_device][schedule_slot].config.write_data, schedule[i2c_device][schedule_slot].config.write_count);
		// Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
		// the whole block was received
		
		//pdca_enable_interrupt_transfer_error(TWI0_DMA_CH);
		
		
		break;
	case 1:
		twim=&AVR32_TWIM1;
	break;
	default: // invalid device ID
		return -1;
	}		

	// set up I2C speed and mode
//	twim_set_speed(twim, 100000, sysclk_get_pba_hz());
    
	switch (conf->direction)  {
		case I2C_READ:
			putnum(&AVR32_USART0, conf->slave_address, 2);
			putstring(&AVR32_USART0, "r");
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| (conf->read_count << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (AVR32_TWIM_CMDR_STOP_MASK)
						| (AVR32_TWIM_CMDR_READ_MASK);
			twim->ncmdr=0;					

			twim->ier = AVR32_TWIM_IER_STD_MASK |  AVR32_TWIM_IER_RXRDY_MASK;
			break;	
		case I2C_WRITE1_THEN_READ:
			putnum(&AVR32_USART0, conf->slave_address, 2);
			putstring(&AVR32_USART0, "wr");
			putnum(&AVR32_USART0, conf->write_count, 10);
			// set up next command register for the burst read transfer
			// set up command register to initiate the write transfer. The DMA will take care of the reading once this is done.
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->write_count)<< AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						//| (AVR32_TWIM_CMDR_STOP_MASK)
						| (0 << AVR32_TWIM_CMDR_READ_OFFSET);
						;
			

			twim->ncmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->read_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (AVR32_TWIM_CMDR_STOP_MASK)
						| (AVR32_TWIM_CMDR_READ_MASK);
						
			twim->ier = AVR32_TWIM_IER_STD_MASK | AVR32_TWIM_IER_RXRDY_MASK | AVR32_TWIM_IER_TXRDY_MASK;
			// set up writing of one byte (usually a slave register index)
			//twim->cr = AVR32_TWIM_CR_MEN_MASK;
			//twim->thr=conf->write_then_read_preamble;
			//twim->cr = AVR32_TWIM_CR_MEN_MASK;
			
			break;	
		case I2C_WRITE:
			putnum(&AVR32_USART0, conf->slave_address, 2);
			putstring(&AVR32_USART0, "w");
			putnum(&AVR32_USART0, conf->write_count, 10);
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->write_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (AVR32_TWIM_CMDR_STOP_MASK)					
						;
			//twim->ncmdr=0;						;	
			twim->ier = AVR32_TWIM_IER_NAK_MASK |  AVR32_TWIM_IER_TXRDY_MASK;
			
		break;	
	}		
	// start transfer
	

	current_schedule_slot[i2c_device]=schedule_slot;
	schedule[i2c_device][schedule_slot].transfer_in_progress=1;
	schedule[i2c_device][schedule_slot].config.read_data_index=0;
	schedule[i2c_device][schedule_slot].config.write_data_index=0;
	if (global_interrupt_enabled) {
			cpu_irq_enable ();
		}	
	twim->cr = AVR32_TWIM_CR_MEN_MASK;
	
	
	
}

char i2c_pause_request(unsigned char i2c_device, unsigned char schedule_slot){
	// pause scheduler
	// if this request currently active, wait for current transfer to finish
	// deactivate request
	// resume scheduler
}

char i2c_enable_request(unsigned char i2c_device, unsigned char schedule_slot){
	
}

char i2c_remove_request(unsigned char i2c_device, unsigned char schedule_slot){
	
}


