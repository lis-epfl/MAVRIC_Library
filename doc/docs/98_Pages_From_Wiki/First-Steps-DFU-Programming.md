Installing and using DFU-programmer with the MAV'RIC board
# 1) Instructions for dfu-programmer

## LINUX instructions

### a) Install dfu-programmer

- install libusb-1.0-0-dev as proposed in the README from dfu-programmer:  
`sudo apt-get install libusb-1.0-0-dev`  
- download dfu-programmer (tar-ball file) from [[ Sourceforge | http://sourceforge.net/projects/dfu-programmer/?source=typ_redirect ]]  
- extract it somewhere:  
`unzip <downloaded file>.zip` or `tar xvfz <downloaded file>.tar.gz`  
- enter directory and follow build instructions: 
```
./configure
make
sudo cp src/dfu-programmer /usr/local/bin
```

### b) Allow standard users to access the board via usb:

- Create the file `/etc/udev/rules.d/99-dfu_programmer.rules` with the following content:  
WARNING: GROUP="your user name or a group that you belong to"  
`ATTR{idVendor}=="03eb", ATTR{idProduct}=="*", GROUP="users", MODE="0660"`  
- Reboot


### c) A typical flashing sequence:
```
dfu-programmer at32uc3c1512 erase
dfu-programmer at32uc3c1512 get 
dfu-programmer at32uc3c1512 flash <flash-image.hex> --suppress-bootloader-mem 
dfu-programmer at32uc3c1512 reset 
```
The script `dfu_program.sh` with this command sequence is useful to add to an IDE "Run" command, so that flashing can be done by clicking a button or automatically after make. 

## WINDOWS instructions

### a) Install driver


- connect a USB cable to a MAV'RIC board WITH the bootloader FLASHED (If it's not the case, read the documentation related to the bootloader)  
WARNING: Some cellphone cables are only for charging through USB, DO NOT use one of them
- install the driver: (DO NOT let windows search for the driver), go to "Control panel", "Device manager", select the DFU programmer, Specify the path to the .inf file found in the folder `MAVRIC\Utilities\DFU-programmer\DFU\dfu-prog-usb-1.2.2` 


### b) A typical flashing sequence:
```
dfu-programmer at32uc3c1512 erase
dfu-programmer at32uc3c1512 get 
dfu-programmer at32uc3c1512 flash <flash-image.hex> --suppress-bootloader-mem 
dfu-programmer at32uc3c1512 reset 
```
The script `dfu-programming.bat` with this command sequence has to be in your project folder. This script flashes your last compiled .hex to your MAV'RIC board (by default the file `<YourProject>/Release/megafly.hex`)

- copy `mklink_egDFU.bat` from `MAVRIC\Utilities\DFU-programmer\` in your project directory and rename it to mklink_DFU.bat
- edit this file and replace <relative path to MAVRIC> by the relative path from project to MAVRIC (e.g: `../../MAVRIC`)
- copy `dfu-programming.bat` from `MAVRIC\Utilities\DFU-programmer\` in your project directory
- execute this script whenever you want to flash your code (the .hex file by default is `Release/megafly.hex`)

# 2) Instructions for flashing code in the MAV'RIC board

In order to flash the code on the MAV'RIC board, a bootloader is running at startup. During the boot, the pin SDA1 (corresponding to PC04) is tested to determine wheter the DFU software or the application will start.
During a normal startup, the application will start as the pin SDA1 is pulled up by a resistor.
For launching the DFU software, the pin SDA1 must be set to 0, for example, by connecting the pin n°1 (SDA1) and n°5 (GND) of an I2C 1 connector. An extra push button might also be used if soldered between SDA1 and GND.
After a flash of the bootloader, the DFU software is forced to start allowing a first application flashing.   


### a) Flashing code right after flashing the bootloader

- connect the usb cable to a MAV'RIC board WITH the bootloader FLASHED (If it's not the case, read the documentation related to the bootloader)
- run the script `dfu-programming.bat` in windows, `dfu_program.sh` in linux or execute manually a flashing sequence in a terminal


### b) Flashing code with an existing application

- connect the usb cable to a MAV'RIC board WITH the bootloader FLASHED (If it's not the case, read the documentation related to the bootloader)
- Start in bootloader mode:
 - Board v4.1.1: Press the push-button on the corner of the board while pressing the microcontroller's reset push-button
 - Board v4.1: Ground SDA1 pin using a wire between pin 1 and 5 of an I2C connector and press the microcontroller's reset push-button
- run the script `dfu-programming.bat`, `dfu_program.sh` in linux or execute manually a flashing sequence in a terminal  
- remove the grounding of SDA1 pin
