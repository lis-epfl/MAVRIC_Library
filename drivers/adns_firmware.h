
#ifndef ADNS_FIRMWARE_H_
#define ADNS_FIRMWARE_H_


#include <asf.h>
#include "delay.h"
#include "sysclk.h"


//----- SPI Definitions
#define SPI_0_PORT              (&AVR32_SPI0)
#define SPI_SLAVECHIP_NUMBER    (0)
#define ADNS_PIN				AVR32_SPI0_NPCS_0_0_PIN

// Apparently mode 3 has no definition. What we need for ADNS is mode 3
#define SPI_MODE_3				3

//----- ADNS register address
#define ADNS_PRODUCT_ID			0x00
#define ADNS_REVISION_ID		0x01
#define ADNS_MOTION				0x02
#define ADNS_DELTAXL			0x03
#define ADNS_DELTAXH			0x04
#define ADNS_DELTAYL			0x05
#define ADNS_DELTAYH			0x06
#define	ADNS_SQUAL_REG			0x07
#define ADNS_INVERSE_PRODUCT_ID	0x3f
#define ADNS_CONF1				0x0f
#define ADNS_CONF2				0x10
#define ADNS_FRAME_CAPTURE		0x12
#define ADNS_LASER_CTRL0		0x20
#define ADNS_CONF5				0x2f
#define ADNS_POWER_UP			0x3a
#define ADNS_MOTION_BURST		0x50
#define ADNS_PIXEL_BURST 		0x64
#define REG_Configuration_V     0x2f
#define REG_Configuration_IV    0x39
#define REG_SROM_Enable         0x13
#define REG_SROM_ID             0x2a
#define REG_SROM_Load_Burst     0x62

//----- ADNS Commands
#define ADNS_POWER_UP_CMD		0x5A

static uint32_t cpu_clkhz;


typedef struct
{
	int16_t flowx;
	int16_t flowy;
	int16_t squal;
}motion_burst_t;

static spi_options_t adns_spi_options={
	// The SPI channel to set up : Memory is connected to CS0
	SPI_SLAVECHIP_NUMBER,
	// Preferred baudrate for the SPI.
	1000000,
	// Number of bits in each character (8 to 16).
	8,
	// Delay before first clock pulse after selecting slave (in PBA clock periods).
	0,
	// Delay between each transfer/character (in PBA clock periods).
	0,
	// Sets this chip to stay active after last transfer to it. :tag
	0,
	// Which SPI mode to use when transmitting.
	SPI_MODE_3,
	// Disables the mode fault detection.
	// With this bit cleared, the SPI master mode will disable itself if another
	// master tries to address it.
	1
};


static inline void usdelay(unsigned long delay){
	cpu_delay_us(delay, cpu_clkhz);
}


void spi_init_module(void);
void adns_ss_assert(void);			// make these inline?
void adns_ss_deassert(void);
void adns_write(uint8_t regaddr, uint8_t txdata); 
uint8_t adns_read(uint8_t regaddr);
motion_burst_t adns_burstread(void);
void adns_reset(void);
void adns_readmotion(void);
void adns_init(void);


#endif /* ADNS_FIRMWARE_H_ */