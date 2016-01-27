If you flash a new microcontroller for the very first time, follow these steps:
N.B.: If at any time, a popup shows asking you whether you want to continue because the voltage is too low, answer yes. 
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
* Go to "Fuses"
 * Insert as value for FGPFRLO and USERPAGE_WORD_0: 0xFFFFFFFF
 * Click on "Program"
* Go back to "Memories"
 * Follow the instructions on how to flash your microcontroller