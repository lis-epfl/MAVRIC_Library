/*
 * i2c_driver.c
 *
 * Created: 16/05/2012 17:31:58
 *  Author: sfx
 */ 
#include "i2c_driver_dma.h"
#include "gpio.h"
#include "pdca.h"
#include "sysclk.h"
#include "print_util.h"

static volatile i2c_schedule_event schedule[I2C_DEVICES][I2C_SCHEDULE_SLOTS];

static volatile char current_schedule_slot[I2C_DEVICES];


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
   
   if (schedule[0][current_schedule_slot[0]].callback) schedule[0][current_schedule_slot[0]].callback;
   print_util_putstring(&AVR32_USART0, "!");
}




int i2c_driver_init(unsigned char i2c_device) {
	int i;
	volatile avr32_twim_t *twim;
	switch (i2c_device) {
	case 0: 
		twim = &AVR32_TWIM0;
		// Register PDCA IRQ interrupt.
//		INTC_register_interrupt( (__int_handler) &pdca_int_handler_i2c0, TWI0_DMA_IRQ, AVR32_INTC_INT0);
		gpio_enable_module_pin(AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION);
		gpio_enable_pin_pull_up(AVR32_TWIMS1_TWCK_0_0_PIN);
		gpio_enable_pin_pull_up(AVR32_TWIMS1_TWD_0_0_PIN);

	break;
	case 1:
		twim = &AVR32_TWIM1;// Register PDCA IRQ interrupt.
//		INTC_register_interrupt( (__int_handler) &pdca_int_handler_i2c0, TWI1_DMA_IRQ, AVR32_INTC_INT0);
		gpio_enable_module_pin(AVR32_TWIMS1_TWCK_0_0_PIN, AVR32_TWIMS1_TWCK_0_0_FUNCTION);
		gpio_enable_module_pin(AVR32_TWIMS1_TWD_0_0_PIN, AVR32_TWIMS1_TWD_0_0_FUNCTION);
		gpio_enable_pin_pull_up(AVR32_TWIMS1_TWCK_0_0_PIN);
		gpio_enable_pin_pull_up(AVR32_TWIMS1_TWD_0_0_PIN);
	break;
	default: // invalid device ID
		return -1;
	}		
	for (i = 0; i < I2C_SCHEDULE_SLOTS; i++) {
		schedule[i2c_device][i].active = -1;
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
	
	
	
	// register Register twim_master_interrupt_handler interrupt on level CONF_TWIM_IRQ_LEVEL
//	irqflags_t flags = cpu_irq_save();
//	irq_register_handler(twim_master_interrupt_handler,
//			CONF_TWIM_IRQ_LINE, CONF_TWIM_IRQ_LEVEL);
//	cpu_irq_restore(flags);
	
	// Select the speed
	if (twim_set_speed(twim, 100000, sysclk_get_pba_hz()) == 
			ERR_INVALID_ARG) {
		
		return ERR_INVALID_ARG;
	}
	return STATUS_OK;				

}



char i2c_driver_reset(unsigned char i2c_device) {
	volatile avr32_twim_t *twim;
	switch (i2c_device) {
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
char i2c_driver_add_request(unsigned char i2c_device, i2c_schedule_event* new_event){
	// find free schedule slot
	int i = 0;
	for (i = 0; (i < I2C_SCHEDULE_SLOTS)&& (schedule[i2c_device][i].active >= 0); i++) {
		print_util_putstring(&AVR32_USART0, ".");
	}
	// add request to schedule
	if (i < I2C_SCHEDULE_SLOTS) {
		new_event->schedule_slot = i;
		new_event->transfer_in_progress = 0;
		new_event->active = 1;
		schedule[i2c_device][i] = *new_event;
		schedule[i2c_device][i].config = new_event->config;
		print_util_putstring(&AVR32_USART0, "slave address:");
		print_util_putnum(&AVR32_USART0, schedule[i2c_device][i].config.slave_address, 2);
	} else i = -1;
	// return assigned schedule slot
	return i;
}
char i2c_driver_change_request(unsigned char i2c_device, i2c_schedule_event* new_event){
	int i = new_event->schedule_slot;
	if ((i >= 0) && (i < I2C_SCHEDULE_SLOTS)) {
		new_event->transfer_in_progress = 0;
		new_event->active=1;
		schedule[i2c_device][i] = *new_event;
	};
}


char i2c_driver_trigger_request(unsigned char i2c_device, unsigned char schedule_slot) {
	// initiate transfer of given request
	// set up DMA channel
	volatile avr32_twim_t *twim;
	i2c_packet_conf* conf = &schedule[i2c_device][schedule_slot].config;
	static  pdca_channel_options_t PDCA_OPTIONS =
			{
				.addr = 0,                                // memory address
				.pid = AVR32_TWIM0_PDCA_ID_TX,            // select peripheral 
				.size = 4,	                              // transfer counter
				.r_addr = NULL,                           // next memory address
				.r_size = 0,                              // next transfer counter
				.transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer
			};
	bool global_interrupt_enabled = cpu_irq_is_enabled ();
	if (global_interrupt_enabled) {
		cpu_irq_disable ();
	}
	switch (i2c_device) {
	case 0: 
		twim = &AVR32_TWIM0;
		twim->cr = AVR32_TWIM_CR_MEN_MASK;
		twim->cr = AVR32_TWIM_CR_SWRST_MASK;
		twim->cr = AVR32_TWIM_CR_MDIS_MASK;
		twim->scr = ~0UL;
		if (twim_set_speed(twim, 100000, sysclk_get_pba_hz()) == 
			ERR_INVALID_ARG) {
			return ERR_INVALID_ARG;
	    }
		switch (conf->direction)  {
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
			PDCA_OPTIONS.r_addr = (void *)conf->write_data;
			PDCA_OPTIONS.r_size=conf->write_count;
			// Init PDCA channel with the pdca_options.
			pdca_init_channel(TWI0_DMA_CH, &PDCA_OPTIONS); // init PDCA channel with options.
			pdca_load_channel(TWI0_DMA_CH, (void *)conf->write_data, conf->write_count);
			break;
		}
		if (global_interrupt_enabled) {
			cpu_irq_enable ();
		}	
//		pdca_load_channel(TWI0_DMA_CH, (void *)schedule[i2c_device][schedule_slot].config.write_data, schedule[i2c_device][schedule_slot].config.write_count);
		// Enable pdca interrupt each time the reload counter reaches zero, i.e. each time
		// the whole block was received
		pdca_enable_interrupt_transfer_complete(TWI0_DMA_CH);
		//pdca_enable_interrupt_transfer_error(TWI0_DMA_CH);
		
		
		break;
	case 1:
		twim = &AVR32_TWIM1;
	break;
	default: // invalid device ID
		return -1;
	}		

	// set up I2C speed and mode
//	twim_set_speed(twim, 100000, sysclk_get_pba_hz());
    
	switch (conf->direction)  {
		case I2C_READ:
			print_util_putnum(&AVR32_USART0, conf->slave_address, 2);
			print_util_putstring(&AVR32_USART0, "r");
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| (conf->read_count << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (0 << AVR32_TWIM_CMDR_STOP_OFFSET)
						| (0 << AVR32_TWIM_CMDR_READ_OFFSET);
			break;	
		case I2C_WRITE1_THEN_READ:
			print_util_putnum(&AVR32_USART0, conf->slave_address, 2);
			print_util_putstring(&AVR32_USART0, "wr");
			
			// set up next command register for the burst read transfer
			// set up command register to initiate the write transfer. The DMA will take care of the reading once this is done.
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| (1 << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						//| (AVR32_TWIM_CMDR_STOP_MASK)
						;
			

			twim->ncmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->read_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (AVR32_TWIM_CMDR_STOP_MASK);
						
			// set up writing of one byte (usually a slave register index)
			//twim->cr = AVR32_TWIM_CR_MEN_MASK;
			twim->thr = conf->write_then_read_preamble;
			twim->cr = AVR32_TWIM_CR_MEN_MASK;
			
			break;	
		case I2C_WRITE:
			print_util_putnum(&AVR32_USART0, conf->slave_address, 16);
			print_util_putstring(&AVR32_USART0, "w");
			print_util_putnum(&AVR32_USART0, conf->write_count, 10);
			twim->cmdr = (conf->slave_address << AVR32_TWIM_CMDR_SADR_OFFSET)
						| ((conf->write_count) << AVR32_TWIM_CMDR_NBYTES_OFFSET)
						| (AVR32_TWIM_CMDR_VALID_MASK)
						| (AVR32_TWIM_CMDR_START_MASK)
						| (AVR32_TWIM_CMDR_STOP_MASK)
						
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
	
	
}

char i2c_driver_pause_request(unsigned char i2c_device, unsigned char schedule_slot){
	// pause scheduler
	// if this request currently active, wait for current transfer to finish
	// deactivate request
	// resume scheduler
}

char i2c_driver_enable_request(unsigned char i2c_device, unsigned char schedule_slot){
	
}

char i2c_driver_remove_request(unsigned char i2c_device, unsigned char schedule_slot){
	
}


