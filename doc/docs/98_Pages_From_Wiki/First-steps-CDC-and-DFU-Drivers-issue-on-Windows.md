This part explains how to setup the driver issue on windows machine in case you experiment any issues.

When the mavering board is running, it uses the CDC USB driver.
It is used to debug the autopilot, so that you can print messages in X-CTU.

When you put it in DFU mode, for programming through USB, it uses the DFU driver.

You have to solve windows driver issue in both cases, in CDC mode and DFU mode.

The first time you will plug a USB cable from the autopilot to your computer, it will try to search for this driver. Unfortunately, Windows does not automatically find the correct driver.
So you will have to find yourself the driver.  
For this,
* Go in your Device manager
* Right-click on the corresponding USB device (in `other devices` field)
* Click on Update driver software
* Browse my computer
* Guide it to <...>\maveric\Code\Library\USB-console
* Select look in sub-folders
* it will find and install the driver
* Check in the device manager that the device is now in `Atmel USB section` at the top of the device manager window.

You should then be able to use X-CTU to monitor the initialization proccess of the autopilot.  
For this, 
* Connect the USB cable from the autopilot to your computer
* Start X-CTU
* Select the corresponding COM port
* Go in "Terminal" Tab
* Press "Close Com Port"
* Press the autopilot reset button
* Quickly press "Open COM port"
* The initialization messages will be printed

If windows complains about libusb0.dll missing, in DFU mode,
* go in mavric\utilities\USB-console
* copy the libusb0.dll file
* paste it in `c:\Windows\system32\drivers\`
Done