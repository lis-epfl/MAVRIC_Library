dfu-programmer-win-0.6.2.zip contains the pre-compiled Windows executable and the USB drivers for use with Atmel chips in DFU bootloader mode.
The Windows executable does not need any installation or setup. Just extract the executable and run it.
The Windows driver can be installed when prompted by Windows when a DFU device is attached. Do not let Windows search for a driver; specify the path to search for a driver and point it to the .inf file.
dfu-programming.bat is a script to flash your last compiled .hex to your maveric board through DFU. This file must be placed in your project directory (ex. Maveric_myCopter) and will flash (if not modified) the file Debug/megafly.hex.
