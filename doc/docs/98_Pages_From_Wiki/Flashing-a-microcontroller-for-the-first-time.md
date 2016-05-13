If you flash a new microcontroller for the very first time, follow these steps:
N.B.: If at any time, a popup shows asking you whether you want to continue because the voltage is too low, answer yes.
* Plug in programmer to drone and computer
* Open Atmel Studio
* Go to "Tools"->"Device Programming"
* Select
 * JTAGICE3 as Tool
 * AT32UC3C1512C as Device
 * JTAG as interface
* Click on "Apply"
* If it worked, you should get some tabs below
* Go to "Memories"
 * Select "Erase Chip" and click on "Erase now"
 * Select "Erase UserPage" and click on "Erase now"
 * Under Flash, select MAVRIC/Utilities/Bootloader/BOOTLOADER_AT32UC3C1512.hex
 * Under Flash, press "Program"
  * If you get an error, disconnect the battery wires from the MAVRIC board and try again
 * Under User Page, select MAVRIC/Utilities/Bootloader/USERPAGE_BOOT_SDA1.hex
 * Under User Page, press "Program"
* Go to "Fuses"
 * Insert as value for FGPFRLO and USERPAGE_WORD_0: 0xFFFFFFFF
 * Set BOOTPROT to BOOTAREA_8KB
 * Click on "Program"
