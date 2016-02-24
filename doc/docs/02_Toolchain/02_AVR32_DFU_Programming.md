# Linux

### a) Install dfu-programmer

- install libusb-1.0-0-dev as proposed in the README from dfu-programmer:  
`sudo apt-get install libusb-1.0-0-dev`  
- download dfu-programmer (tar-ball file) from [[ Sourceforge | http://sourceforge.net/projects/dfu-programmer/?source=typ_redirect ]]  
- extract it somewhere:  
`unzip <downloaded file>.zip` or `tar xvfz <downloaded file>.tar.gz`  
- enter directory and follow build instructions: 

        ./configure
        make
        sudo cp src/dfu-programmer /usr/local/bin

### b) Allow standard users to access the board via usb:

- Create the file `/etc/udev/rules.d/99-dfu_programmer.rules` with the following content:  
        
        WARNING: GROUP="your user name or a group that you belong to"  
        `ATTR{idVendor}=="03eb", ATTR{idProduct}=="*", GROUP="users", MODE="0660"`  

- Reboot




___

# Windows

- connect a USB cable to a MAV'RIC board with the bootloader *flashed* (If it's not the case, read the documentation related to the bootloader)  
WARNING: Some cellphone cables are only for charging through USB, DO NOT use one of them
- install the driver: (DO NOT let windows search for the driver), go to "Control panel", "Device manager", select the DFU programmer, Specify the path to the .inf file found in the folder `MAVRIC\Utilities\DFU-programmer\DFU\dfu-prog-usb-1.2.2` 

___


# Typical flashing sequence:
To flash an avr32 autopilot:
- Connect the board with a micro USB cable. The board should be flashed with  the bootloader. If it's not the case, read the documentation related to the bootloader.
- Put the board in bootloader mode (press and release the reset button while the bootloader button is pressed) 
- then type
        
        make flash