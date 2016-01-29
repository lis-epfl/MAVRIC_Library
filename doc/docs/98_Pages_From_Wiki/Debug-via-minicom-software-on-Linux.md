To use the debug console on Linux, the software minicom can be used. 

### To install minicom
Type in a terminal `sudo apt-get install minicom`
### To configure minicom to read the stream
Make sure your user is part of the dialgroup, for that type in a terminal `groups YOUR_LINUX_USERNAME`, if it is not, `sudo adduser YOUR_LINUX_USERNAME dialout`
- Change the permission on the stream: type in a terminal `sudo chmod a+rw /dev/ttyACM0`, this allow to launch minicom without typing sudo every time. 
- Set up minicom
Type in a terminal `sudo minicom -s`
 - Go to "Serial port setup" and press enter
 - Press "A" and write the port `/dev/ttyACM0`, press "enter"
 - Press "E" and select the correct baudrate (speed), by default, you should select 57600, i.e. press once "B", then press "enter" twice
 - Go to "Save setup as dfl", press "enter"
 - Go to "Exit from Minicom"

Now your minicom software is configured and can be run directly by typing `minicom` in a terminal

### On the embedded system
- Go to src/boardsupport.c file,
- Comment line `console_init(CONSOLE_UART4, usart_default_config_console, usb_default_config_console);`
- Uncomment line `//console_init(CONSOLE_USB, usart_default_config_console, usb_default_config_console);`
- Compile and load the program on your microcontroller