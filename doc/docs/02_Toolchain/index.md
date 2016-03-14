We currently support three hardware architectures: avr32, stm32 and emulation. 

Each architecture requires a specific toolchain to compile and flash your code.

## AVR32
For boards using avr32 microcontrollers from Atmel. List of supported boards:
- Megafly

## STM32
For boards using stm32f4 microcontrollers from ST. List of supported boards:
- Mavrimini

## Emulation
To run a simulated MAV on the host computer. In this mode, the autopilot is compiled into a binary that can be executed directly on the machine (Linux, Mac OS or Windows).
- Sensor values are not read from physical sensors, but simulated according to a dynamic model of the MAV.
- Debug messages are printed on the console instead of through usb 
- Telemetry messages are sent via udp instead of via an antenna