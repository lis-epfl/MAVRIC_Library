/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */

extern "C" {
	#include "led.h"
	#include "delay.h"
	#include "print_util.h"
	#include "central_data.h"
	#include "boardsupport.h"
	#include "navigation.h"
	#include "tasks.h"
	#include "orca.h"
	#include "piezo_speaker.h"
	#include "airspeed_analog.h"
}

//	GLE: CURVACE SPI communication
extern "C" {
	//#include <asf.h>
	//#include "generator.h"
	//#include "time_keeper.h"
	//#include "streams.h"
	//#include "bmp085.h"
	//#include "scheduler.h"
	//#include "mavlink_waypoint_handler.h"
	//#include "neighbor_selection.h"
	#include "gpio.h"
	#include "spi.h"
	#include "sysclk.h"
	//#include "pwm4.h"
	//#include "board.h"
	//#include "compiler.h"
	//#include <avr32/uc3c1512c.h>
}
//Apparently mode 3 has no definition. What we need for ADNS is mode 3, curvace needs 0
#define SPI_SLAVECHIP_NUMBER    (0)
#define slaveSelectTop AVR32_PIN_PC11
#define slaveSelectBot AVR32_PIN_PC12
//#define slaveSelectBot AVR32_SPI0_NPCS_0_0_PIN
//*///	GLE: end

 
//#include <asf.h>
//#include "stdio_serial.h"
//#include "mavlink_waypoint_handler.h"
//#include "neighbor_selection.h"
//#include "flashvault.h"
//#include "generator.h"
//#include "time_keeper.h"
//#include "streams.h"
//#include "bmp085.h"
//#include "scheduler.h"

central_data_t *centralData;

//		GLE: CURVACE SPI communication
static void spi_resources_init(void)
{
	//both slave select to 1 to avoid interference
	int pinTest=slaveSelectBot;

	gpio_enable_gpio_pin(pinTest); 
	gpio_set_gpio_pin(pinTest);
	gpio_enable_gpio_pin(slaveSelectTop);
	gpio_set_gpio_pin(slaveSelectTop);
	
	//setup spi
	/*
  static const gpio_map_t SPI_GPIO_MAP =
  {
    {15,0 }, //GPIO15, Function A  // SPI Clock.
    {25,0 }, //GPIO25, Function A  // MISO.
    {14,0 }, //GPIO14, Function A  // MOSI.
    {16,0 }, //GPIO16, Function A  // CS0
  };
  */
  static const gpio_map_t SPI_GPIO_MAP =
  {
    {AVR32_SPI0_SCK_0_0_PIN, AVR32_SPI0_SCK_0_0_FUNCTION }, //SCK
    {AVR32_SPI0_MISO_0_0_PIN, AVR32_SPI0_MISO_0_0_FUNCTION}, //MISO
    {AVR32_SPI0_MOSI_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION} //MOSI
    //{AVR32_SPI0_NPCS_0_0_PIN , AVR32_SPI0_NPCS_0_0_FUNCTION  }  //CS
    //{AVR32_SPI0_NPCS_0_1_PIN , AVR32_SPI0_NPCS_0_1_FUNCTION  }  //CS
  };

  // SPI options.
  // GLE: C++ compatible
  spi_options_t spiOptions;
  
  spiOptions.reg          = 0;			//CS0
  spiOptions.baudrate     = 500000;		//500khz
  spiOptions.bits         = 16;			// Bits!
  spiOptions.spck_delay   = 0;			// # clocks to delay.
  spiOptions.trans_delay  = 0;			// ?
  spiOptions.stay_act     = 0;			// auto-unselect...?
  spiOptions.spi_mode     = SPI_MODE_1;	// active high, low level idle -> mode 0
  spiOptions.modfdis      = 1;			// Mode fault detection bit -> disable at 1
  /*spi_options_t spiOptions =
  {
    .reg          = 0,         //CS0
    //.baudrate     = 5000000,   //5Mhz
	.baudrate     = 500000,   //500khz
    .bits         = 16,         // Bits!
    .spck_delay   = 0,         // # clocks to delay.
    .trans_delay  = 0,         // ?
    .stay_act     = 0,         // auto-unselect...?
    .spi_mode     = SPI_MODE_1,         // active high, low level idle -> mode 0
    .modfdis      = 1         // ...?
  };*/
  // GLE: end C++ compatible
  
  // Assign I/Os to SPI.
  gpio_enable_module(SPI_GPIO_MAP, sizeof(SPI_GPIO_MAP) / sizeof(SPI_GPIO_MAP[0]));

  // Initialize as master.
  spi_initMaster((&AVR32_SPI0), &spiOptions);

  // Set selection mode: variable_ps, pcs_decode, delay.
  spi_selectionMode((&AVR32_SPI0), 0, 0, 0);

  //Set how we're talking to the chip. (Bits!  et al)
  spi_setupChipReg((&AVR32_SPI0), &spiOptions, sysclk_get_pba_hz()); //very important!

  // Enable SPI.
  spi_enable((&AVR32_SPI0));

}

static void curvace_init(){
	spi_resources_init();
	
	int i;
	
	gpio_clr_gpio_pin(slaveSelectTop);

	spi_selectChip(&AVR32_SPI0, SPI_SLAVECHIP_NUMBER);
	
	uint16_t data=7;
	//centralData->framesCurvace=data;
	
	uint16_t winSize=7;
	centralData->curvace_roiNum=1;
	
	//number of regions
	delay_ms(50);
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, centralData->curvace_roiNum );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[0]=data+2;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(30);
	
	//region boundaries
	for( i=0;i<centralData->curvace_roiNum;i++ ){
		
		while (!spi_is_tx_ready(&AVR32_SPI0));
		spi_put(&AVR32_SPI0,0x0101 );
		while(!spi_is_tx_empty(&AVR32_SPI0));
		data=spi_get(&AVR32_SPI0 );
		centralData->curvace_ini[1]=data;
		//while(!spi_is_tx_empty(&AVR32_SPI0));
		delay_ms(5);
		
		while (!spi_is_tx_ready(&AVR32_SPI0));
		spi_put(&AVR32_SPI0,(winSize<<8)+winSize );
		while(!spi_is_tx_empty(&AVR32_SPI0));
		data=spi_get(&AVR32_SPI0 );
		centralData->curvace_ini[2]=data;
		//while(!spi_is_tx_empty(&AVR32_SPI0));
		delay_ms(5);
		
	}
	//FR
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,2 );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[3]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(5);
	
	//not used
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,10 );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[4]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(10);
	
	//Hbits nframes
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,0 );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[5]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(5);
	
	//Lbits nframes
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,3000 );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[6]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(5);
	
	//dummies
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,0xD );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[7]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(5);
	
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,0xD );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->curvace_ini[8]=data;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(5);
	
	//start
	/*
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0,0xAAAA );
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0 );
	centralData->framesCurvace=data+2;
	//while(!spi_is_tx_empty(&AVR32_SPI0));
	delay_ms(120);
	*///////
	
	spi_unselectChip(&AVR32_SPI0, SPI_SLAVECHIP_NUMBER);
	delay_ms(20);
	gpio_set_gpio_pin(slaveSelectTop);	
	
}

static void spi_test(){
	
	uint16_t data=7;

	spi_selectChip(&AVR32_SPI0, SPI_SLAVECHIP_NUMBER);
	
	gpio_set_gpio_pin(slaveSelectBot);
	gpio_clr_gpio_pin(slaveSelectTop);
	
	delay_ms(10);
	
	//number of regions
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 1);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[0]=data;
	delay_ms(1);
	
	//region boundaries
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0x0101);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[1]=data;
	delay_ms(1);
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0x0F2A);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[2]=data;
	delay_ms(1);
	
	//FR
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 25);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[3]=data;
	delay_ms(1);

	//not used
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[4]=data;
	delay_ms(1);
	
	//Hbits nframes
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[5]=data;
	delay_ms(1);
	
	//Lbits nframes
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 1000);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[6]=data;
	delay_ms(1);
	
	//dummies
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0xD);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[7]=data;
	delay_ms(1);
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0xD);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	centralData->curvace_ini[8]=data;
	delay_ms(10);
	
	//start
	while (!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0xAAAA);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	delay_ms(10);
	
	//reset
	while(!spi_is_tx_ready(&AVR32_SPI0));
	spi_put(&AVR32_SPI0, 0x5555);
	while(!spi_is_tx_empty(&AVR32_SPI0));
	data=spi_get(&AVR32_SPI0);
	delay_ms(10);
	//*/
	gpio_set_gpio_pin(slaveSelectTop);
	
	spi_unselectChip(&AVR32_SPI0, SPI_SLAVECHIP_NUMBER);
	delay_ms(10);	
	
}
//*///	GLE: end

void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;
	
	centralData = get_central_data();
	initialise_board(centralData);
	initialise_central_data();
	qfInit(&(centralData->imu1.attitude), (centralData->imu1.raw_scale), (centralData->imu1.raw_bias));
	
	relevel_imu();

	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//init_gps_ubx(engine_nav_settings);
	
	servos_failsafe(centralData->servos);
	set_servos(centralData->servos);
	
	init_onboard_parameters();
	init_mavlink_actions();
	init_pos_integration(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	
	initQuat(&centralData->imu1.attitude);
	
	init_nav();
	init_waypoint_handler();
	//e_init();
	
	init_neighbors();
	init_orca();
	
	airspeed_analog_calibrate(&centralData->pitot);

	LED_On(LED1);
	init_piezo_speaker_binary();

}

int main (void)
{
	int i;
	// turn on simulation mode: 1: simulation mode, 0: reality
	centralData->simulation_mode = 0;
	centralData->simulation_mode_previous = centralData->simulation_mode;
	initialisation();
		
	create_tasks();

	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
		centralData->position_estimator.localPosition.pos[i]=0.0;
	}
	
	//dbg_print("Initialise HIL Simulator...\n");
	init_simulation(&(centralData->sim_model),&(centralData->imu1),centralData->position_estimator.localPosition);

	// main loop
	delay_ms(10);
	dbg_print("Reset home position...\n");
	position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data, &centralData->sim_model.localPosition);
	dbg_print("OK. Starting up.\n");

	for (i=1; i<8; i++) {
		beep(100, 500*i);
		delay_ms(2);
	}
	
	//		GLE: CURVACE SPI communication
	//delay since the power takes some time to get to the curvace
	spi_resources_init();
	delay_ms(3000);
	//curvace_init();
	spi_test();
	//*///	GLE: end
	
	while (1==1) {
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
	}
	return 0;
}