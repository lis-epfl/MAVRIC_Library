/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <board.h>
#include "compiler.h"
#include "conf_board.h"
#include "gpio.h"

#if defined (CONF_BOARD_AT45DBX)
#define AT45DBX_MEM_CNT             1
#endif

void board_init(void)
{

#if defined (CONF_BOARD_AT45DBX)
	static const gpio_map_t AT45DBX_SPI_GPIO_MAP =
	{
		{AT45DBX_SPI_SCK_PIN,          AT45DBX_SPI_SCK_FUNCTION         },  // SPI Clock.
		{AT45DBX_SPI_MISO_PIN,         AT45DBX_SPI_MISO_FUNCTION        },  // MISO.
		{AT45DBX_SPI_MOSI_PIN,         AT45DBX_SPI_MOSI_FUNCTION        },  // MOSI.
#define AT45DBX_ENABLE_NPCS_PIN(NPCS, unused) \
		{AT45DBX_SPI_NPCS##NPCS##_PIN, AT45DBX_SPI_NPCS##NPCS##_FUNCTION},  // Chip Select NPCS.
		MREPEAT(AT45DBX_MEM_CNT, AT45DBX_ENABLE_NPCS_PIN, ~)
#undef AT45DBX_ENABLE_NPCS_PIN
	};

	// Assign I/Os to SPI.
	gpio_enable_module(AT45DBX_SPI_GPIO_MAP,
		sizeof(AT45DBX_SPI_GPIO_MAP) / sizeof(AT45DBX_SPI_GPIO_MAP[0]));
#endif

	// Configure the pins connected to LEDs as output and set their default
	// initial state to high (LEDs off).        
	gpio_configure_pin(LED0_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(LED1_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	// Configure the pin connected to the button 0 and 1 button as input.
	//gpio_configure_pin(GPIO_PUSH_BUTTON_0,GPIO_DIR_INPUT);
	//gpio_configure_pin(GPIO_PUSH_BUTTON_1,GPIO_DIR_INPUT);

#if defined (CONF_BOARD_COM_PORT)
	// USART GPIO pin configuration.
	static const gpio_map_t COMPORT_GPIO_MAP =
	{
		{USART_RXD_PIN, USART_RXD_FUNCTION },
		{USART_TXD_PIN, USART_TXD_FUNCTION }
	};
	gpio_enable_module(COMPORT_GPIO_MAP,
			sizeof(COMPORT_GPIO_MAP) / sizeof(COMPORT_GPIO_MAP[0]));
#endif

#if defined (CONF_BOARD_TWI)	
  static const gpio_map_t TWI_GPIO_MAP =
  {
    {AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION},
    {AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION}
  };

  // TWI gpio pins cofiguration
  gpio_enable_module(TWI_GPIO_MAP, sizeof(TWI_GPIO_MAP) / sizeof(TWI_GPIO_MAP[0]));
#endif
}
