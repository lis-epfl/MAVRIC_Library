The hardware of the maveric autopilot board must be ready at this point. For the maveric board 4.1.1 see <...>\MAVRIC\Electronics\Maveric32_autopilot\Rev4.1.1\Board_Assembly411.pdf on github
## WINDOWS only

Installing the DFU bootloader on Atmel AVR32 devices  

This document describes the necessary steps to flash a DFU bootloader onto the AVR32 (AT32UC3C1512C) used on the maveric autopilot. Note that all Atmel devices with USB support have the DFU bootloader already installed by default, however, a chip erase carried out with the JTAG will erase the bootloader, and it needs to be re-flashed if needed. To avoid removing the bootloader when using JTAG in Atmel Studio, go to Project -> Properties, and change "Programming Settings" to "Erase only program area".


### 1) Flashing the bootloader image

 
- connect your maveric board to the JTAG cable which is in turn plugged to the JTAGICE3 programmer and debugger (in case of issues with its driver see [here](http://atmel.force.com/support/articles/en_US/FAQ/How-to-downgrade-firmware-version-of-the-JTAGICE3-debbuger-so-that-it-can-work-with-IAR-for-AVR)) which is in turn plugged to the computer, and check if it has power.
   - Open Atmel Studio, go to Tools -> Device Programming.
   - select JTAGICE3 tool (should be listed if connected properly), Device AT32UC3C1512C, Interface JTAG, and click Apply. 
   - to verify if the board is powered, click "Target Voltage". It should be above 3.0 V and not give and error. 
   - to verify if the chip is responding, click "Device information". If any of the above steps fail, check all cables and connections and repeat.
- Go to Tab "Memories"
   - optional: Do a full chip erase
   - under "Flash", select the bootloader image file <...>/MAVRIC/Utilities/Bootloader/BOOTLOADER_AT32UC3C1512.hex
   - Program the flash
   - under "Userpage", select the bootloader userpage file <...>/MAVRIC/Utilities/Bootloader/USERPAGE_BOOT_SDA1.hex
   - Program the user page
- Go to Tab "Fuses"
   - verify that "Auto Read" is enabled, or "Read" fuses from the AVR
   - for BOOTPROT, select BOOTAREA_8KB
   - Program
- You can unplug everything