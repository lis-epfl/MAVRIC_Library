
#include "adns_firmware.h"
#include "print_util.h"

void spi_init_module(void)
{
	//Todo: Move this to board init?
	static const gpio_map_t SPI_GPIO_MAP =
	{
		//{AVR32_SPI0_NPCS_0_0_PIN , AVR32_SPI0_NPCS_0_0_FUNCTION},
		{AVR32_SPI0_SCK_0_0_PIN  , AVR32_SPI0_SCK_0_0_FUNCTION },
		{AVR32_SPI0_MISO_0_0_PIN , AVR32_SPI0_MISO_0_0_FUNCTION},
		{AVR32_SPI0_MOSI_0_0_PIN , AVR32_SPI0_MOSI_0_0_FUNCTION}
	};

	gpio_enable_module(SPI_GPIO_MAP , sizeof(SPI_GPIO_MAP)/sizeof(SPI_GPIO_MAP[0]));
	
	// No need to do this. Board init does it!
	//sysclk_init();
	//sysclk_enable_peripheral_clock(&AVR32_SPI0);
	//sysclk_enable_pba_module(SYSCLK_SPI0);
	
	//Init SPI module as master
	spi_initMaster(SPI_0_PORT,&adns_spi_options);
	spi_selectionMode( &AVR32_SPI0, 0, 0, 0);
	
	
	//Setup configuration for chip connected to CS1
	spi_setupChipReg(SPI_0_PORT,&adns_spi_options,sysclk_get_pba_hz());
	//Allow the module to transfer data
	spi_enable(SPI_0_PORT);	
	cpu_clkhz = sysclk_get_cpu_hz();
}

//Todo: make this inline?
void adns_ss_assert(){
	gpio_clr_gpio_pin(ADNS_PIN);
}

void adns_ss_deassert(){
	gpio_set_gpio_pin(ADNS_PIN);
}

void adns_write(uint8_t regaddr, uint8_t txdata) {
	
	// todo: use spi_write_packet() !!
	uint8_t txreg;
	txreg = 0b10000000 | regaddr;  //1 in MSB + 7 bit reg address
	adns_ss_assert();
	spi_selectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	//spi_write_packet(SPI_0_PORT, 0xaa , 1);
	while (!spi_is_tx_ready(SPI_0_PORT));
	spi_write_single(SPI_0_PORT,txreg);
	while (!spi_is_tx_ready(SPI_0_PORT))
	spi_write_single(SPI_0_PORT,txdata);
	while(!spi_is_tx_empty(SPI_0_PORT));
	spi_unselectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	usdelay(20);
	adns_ss_deassert();
	
}

uint8_t adns_read(uint8_t regaddr){
	uint8_t rxdata;
	adns_ss_assert();
	spi_selectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	while (!spi_is_tx_ready(SPI_0_PORT));
	spi_write_single(SPI_0_PORT,regaddr);
	while(!spi_is_tx_empty(SPI_0_PORT));
	spi_unselectChip(SPI_0_PORT,SPI_SLAVECHIP_NUMBER);
	
	//eh, isn't this dirty? can you make it neater?
	//t_sRAD delay for adns
	usdelay(120);
	
	spi_selectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	while (!spi_is_tx_ready(SPI_0_PORT));
	spi_write_single(SPI_0_PORT,regaddr);
	while(!spi_is_tx_empty(SPI_0_PORT));
	
	while(!spi_is_rx_ready(SPI_0_PORT));
	spi_read_single(SPI_0_PORT, &rxdata);
	
	spi_unselectChip(SPI_0_PORT,SPI_SLAVECHIP_NUMBER);

	adns_ss_deassert();
	//print
// 	dbg_print("ADNS: Received data: ");
// 	dbg_print_num(rxdata, 16);
// 	dbg_print(" ### From register: ");
// 	dbg_print_num(regaddr, 16);
// 	dbg_print(" \n");
	return rxdata;
}

motion_burst_t adns_burstread(){
	
	static int8_t motion_burst[14];
	static int16_t x,y,squal;
	
	for(int i=0; i<14; i++){
		motion_burst[i]=0;
	}
	
	uint8_t rxdata;
	uint8_t regaddr = ADNS_MOTION_BURST;
	adns_ss_assert();
	spi_selectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	while (!spi_is_tx_ready(SPI_0_PORT));
	spi_write_single(SPI_0_PORT,regaddr);
	while(!spi_is_tx_empty(SPI_0_PORT));
	spi_unselectChip(SPI_0_PORT,SPI_SLAVECHIP_NUMBER);
	
	//eh, isn't this dirty? todo: make it neater
	//t_sRAD delay for adns
	usdelay(120);
	
	spi_selectChip(SPI_0_PORT, SPI_SLAVECHIP_NUMBER);
	for(int i=0; i<14; i++){
		while (!spi_is_tx_ready(SPI_0_PORT));
		spi_write_single(SPI_0_PORT,0xff);
		while(!spi_is_tx_empty(SPI_0_PORT));
		while(!spi_is_rx_ready(SPI_0_PORT));
		spi_read_single(SPI_0_PORT, &rxdata);
		motion_burst[i]=rxdata;
		rxdata = 0;
	}			
	spi_unselectChip(SPI_0_PORT,SPI_SLAVECHIP_NUMBER);
	adns_ss_deassert();
	usdelay(2);
 	x = ((int16_t)motion_burst[3]<<8) | (int16_t)motion_burst[2];
 	y = ((int16_t)motion_burst[5]<<8) | (int16_t)motion_burst[4];
 	squal = (int16_t)motion_burst[6];
	motion_burst_t motion_data;
	motion_data.flowx = x;
	motion_data.flowy = y;
	motion_data.squal = squal;
	
	//clear residual motion.
	//adns_write(ADNS_MOTION,0x00);
	return motion_data;
}

void adns_reset()
{
	//Drive NCS high and then low
	adns_ss_deassert();
	usdelay(10);
	adns_ss_assert();
	usdelay(10);
	adns_ss_deassert();
	dbg_print("ADNS: reset done \n");
}

void adns_readmotion(){
	adns_read(ADNS_MOTION);
	usdelay(100);
	adns_read(ADNS_DELTAXL);
	usdelay(100);
	adns_read(ADNS_DELTAXH);
	usdelay(100);
	adns_read(ADNS_DELTAYL);
	usdelay(100);
	adns_read(ADNS_DELTAYH);
	usdelay(100);
}

void adns_init(){
	// ---- ADNS Powerup sequence
	// The delays correspond to the timing requirements mentioned in the datasheet.
	
	//0. SPI Init
	spi_init_module();
	
	//1. Reset
	adns_reset();
	
	//2. Write to powerup reg
	adns_write(ADNS_POWER_UP, ADNS_POWER_UP_CMD);
	dbg_print("ADNS: power on reg written. waiting 50ms now. \n");
	
	//3. wait at least 50ms
	delay_ms(80);
	
	//4. Read motion regs
	adns_burstread();
	
	//5. SROM download. Doesn't seem to be necessary.
	//usdelay(100);
	//adns_upload_firmware();
	//delay_ms(15);
	
	//6.  Enable laser by writing 0x00 to 0x20
	dbg_print("ADNS: Enabling laser.. reading laser_ctrl \n");
	uint8_t dat = adns_read(ADNS_LASER_CTRL0);
	usdelay(100);
	uint16_t dat2 = dat & 0b11110000;
	dbg_print("ADNS: Writing to laser register:  ");
	dbg_print_num(dat2, 16);
	dbg_print("  \n");
	adns_write(ADNS_LASER_CTRL0, dat2);
	usdelay(100);
	// ---- End ADNS Powerup sequence

	// Write to 0x0f config reg and then read it. Should be 45(dec)
	adns_write(ADNS_CONF1, 0x20);
	usdelay(100);
	adns_read(ADNS_CONF1);
	usdelay(100);
	
	dat = adns_read(ADNS_PRODUCT_ID);
	if (dat == 0x33){
		dbg_print("ADNS: Initialized. \n");	
	}
	else
	{
		dbg_print("ADNS: Initialization error. \n");	
	}
	
	
	//adns_write(0x20,0x80);
	//adns_write(0x21,0x42);
	//adns_read(0x10);
	
	adns_read(ADNS_LASER_CTRL0);
	adns_read(0x21);
	//adns_read(0x22);
	//adns_read(0x23);
}


