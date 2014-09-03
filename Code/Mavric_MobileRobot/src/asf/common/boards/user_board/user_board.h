/**
 * \file
 *
 * \brief User board definition template
 *
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

/* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */

#ifdef AVR32_SCIF_101_H_INCLUDED
#define AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  0x00000003
#define AVR32_SCIF_OSCCTRL0_STARTUP_16384_RCOSC 0x00000006
#define AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC 0x00000002
#endif


/*! \name Oscillator Definitions
 */
//! @{

#define FOSC32          AVR32_SCIF_OSC32_FREQUENCY              //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

// Osc0 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc0 crystal is mounted on your STK600
#define FOSC0           16000000                                //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
// #define FOSC1           12000000                              //!< Osc1 frequency: Hz.
// #define OSC1_STARTUP    AVR32_SCIF_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

//! @}

#define BOARD_OSC0_HZ           16000000
#define BOARD_OSC0_STARTUP_US   2000
#define BOARD_OSC0_IS_XTAL      true
#define BOARD_OSC32_HZ          32768
#define BOARD_OSC32_STARTUP_US  71000
#define BOARD_OSC32_IS_XTAL     true

/*! \name SDRAM Definitions
 */
//! @{

//! Part header file of used SDRAM(s).
//#define SDRAM_PART_HDR  "mt48lc16m16a2tg7e/mt48lc16m16a2tg7e.h"

//! Data bus width to use the SDRAM(s) with (16 or 32 bits; always 16 bits on
//! UC3).
//#define SDRAM_DBW       16
//! @}


/*! \name USB Definitions
 */
//! @{
//! Multiplexed pin used for USB_ID: AVR32_USBB_USB_ID_x_x.
//! To be selected according to the AVR32_USBB_USB_ID_x_x_PIN and
//! AVR32_USBB_USB_ID_x_x_FUNCTION definitions from <avr32/uc3cxxxx.h>.
#if (defined AVR32_USBB)
#  define USB_ID                             AVR32_USBB_ID_0_0
#else
#  define USB_ID                             AVR32_USBC_ID
#endif

//! Multiplexed pin used for USB_VBOF: AVR32_USBB_USB_VBOF_x_x.
//! To be selected according to the AVR32_USBB_USB_VBOF_x_x_PIN and
//! AVR32_USBB_USB_VBOF_x_x_FUNCTION definitions from <avr32/uc3cxxxx.h>.
#if (defined AVR32_USBB)
#  define USB_VBOF                           AVR32_USBB_VBOF_0_0
#else
#  define USB_VBOF                           AVR32_USBC_VBOF
#endif

//! Active level of the USB_VBOF output pin.
#  define USB_VBOF_ACTIVE_LEVEL       LOW

//! USB overcurrent detection pin.
#  define USB_OVERCURRENT_DETECT_PIN  AVR32_PIN_PB7

//! @}


//! Number of LEDs.
#define LED_COUNT   3

/*! \name GPIO Connections of LEDs
 */
//! @{
// #  define LED0_GPIO 32 //  AVR32_PIN_PB00
// #  define LED1_GPIO 33 //  AVR32_PIN_PB01
#  define LED1_GPIO AVR32_PIN_PB02
#  define LED2_GPIO AVR32_PIN_PB01
#  define LED0_GPIO AVR32_PIN_PB03		// LED0 does not exist but is added to let LED1 and LET2 match the labels on the board
//! @}

/*! \name PWM Channels of LEDs
 */
//! @{
#define LED0_PWM      (-1)
#define LED1_PWM      (-1)
#define LED2_PWM      (-1)
#define LED3_PWM      (-1)
//! @}

/*! \name PWM Functions of LEDs
 */
//! @{
/* TODO: Implement PWM functionality */
#define LED0_PWM_FUNCTION   (-1)//AVR32_PWM_0_FUNCTION
#define LED1_PWM_FUNCTION   (-1)//AVR32_PWM_1_FUNCTION
#define LED2_PWM_FUNCTION   (-1)
#define LED3_PWM_FUNCTION   (-1)
//! @}

/*! \name Color Identifiers of LEDs to Use with LED Functions
 */
//! @{
#define LED_MONO0_GREEN   LED0
#define LED_MONO1_RED     LED1
//! @}

/*! \name GPIO Connections of Push Buttons
 */
//! @{
#define GPIO_PUSH_BUTTON_0            AVR32_PIN_PD14
#define GPIO_PUSH_BUTTON_0_PRESSED    0
#define GPIO_PUSH_BUTTON_1            AVR32_PIN_PD24
#define GPIO_PUSH_BUTTON_1_PRESSED    0
//! @}

/*! \name GPIO Connections of J33
 */
//! @{
#  define UH   AVR32_PIN_PB08
#  define UL   AVR32_PIN_PB09
#  define VH   AVR32_PIN_PB10
#  define VL   AVR32_PIN_PB11
#  define WH   AVR32_PIN_PB12
#  define WL   AVR32_PIN_PB13
#  define XH   AVR32_PIN_PB14
#  define XL   AVR32_PIN_PB15
#  define ShV  AVR32_PIN_PA21
//! @}

/*! \name SPI Connections of the AT45DBX Data Flash Memory
 */
//! @{
#define AT45DBX_SPI                 (&AVR32_SPI1)
#define AT45DBX_SPI_NPCS            1
#define AT45DBX_SPI_SCK_PIN         AVR32_SPI1_SCK_0_1_PIN
#define AT45DBX_SPI_SCK_FUNCTION    AVR32_SPI1_SCK_0_1_FUNCTION
#define AT45DBX_SPI_MISO_PIN        AVR32_SPI1_MISO_0_1_PIN
#define AT45DBX_SPI_MISO_FUNCTION   AVR32_SPI1_MISO_0_1_FUNCTION
#define AT45DBX_SPI_MOSI_PIN        AVR32_SPI1_MOSI_0_1_PIN
#define AT45DBX_SPI_MOSI_FUNCTION   AVR32_SPI1_MOSI_0_1_FUNCTION
#define AT45DBX_SPI_NPCS0_PIN       AVR32_SPI1_NPCS_1_2_PIN
#define AT45DBX_SPI_NPCS0_FUNCTION  AVR32_SPI1_NPCS_1_2_FUNCTION
//! @}

/*! \name GPIO and SPI Connections of the SD/MMC Connector
 */
//! @{
#define SD_MMC_CARD_DETECT_PIN      AVR32_PIN_PB22
#define SD_MMC_WRITE_PROTECT_PIN    AVR32_PIN_PD30
#define SD_MMC_SPI                  (&AVR32_SPI1)
#define SD_MMC_SPI_NPCS             3
#define SD_MMC_SPI_SCK_PIN          AVR32_SPI1_SCK_0_0_PIN
#define SD_MMC_SPI_SCK_FUNCTION     AVR32_SPI1_SCK_0_0_FUNCTION
#define SD_MMC_SPI_MISO_PIN         AVR32_SPI1_MISO_0_0_PIN
#define SD_MMC_SPI_MISO_FUNCTION    AVR32_SPI1_MISO_0_0_FUNCTION
#define SD_MMC_SPI_MOSI_PIN         AVR32_SPI1_MOSI_0_0_PIN
#define SD_MMC_SPI_MOSI_FUNCTION    AVR32_SPI1_MOSI_0_0_FUNCTION
#define SD_MMC_SPI_NPCS_PIN         AVR32_SPI1_NPCS_3_2_PIN
#define SD_MMC_SPI_NPCS_FUNCTION    AVR32_SPI1_NPCS_3_2_FUNCTION
//! @}


//! @}




/*! \name USART connection to the UC3B board controller
 */
//! @{
#define USART                        (&AVR32_USART2)
#define USART_RXD_PIN                AVR32_USART2_RXD_0_1_PIN
#define USART_RXD_FUNCTION           AVR32_USART2_RXD_0_1_FUNCTION
#define USART_TXD_PIN                AVR32_USART2_TXD_0_1_PIN
#define USART_TXD_FUNCTION           AVR32_USART2_TXD_0_1_FUNCTION
#define USART_IRQ                    AVR32_USART2_IRQ
#define USART_IRQ_GROUP              AVR32_USART2_IRQ_GROUP
#define USART_SYSCLK                 SYSCLK_USART2
//! @}

#define ADC_VEXT_PIN         AVR32_PKGANA_ADCIN5_0_0_PIN
#define ADC_VEXT_FUNCTION    AVR32_PKGANA_ADCIN5_0_0_FUNCTION


#endif // USER_BOARD_H
