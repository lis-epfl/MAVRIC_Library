We are using [QGroundControl software](http://www.qgroundcontrol.org), an Open Source Micro Air Vehicle Ground Control Station designed by ETHZ. 

We use it to visualize onboard variables, plot them and program the flight plan, in real time.

## Connect to QGroundControl
* Plug your Xbee ground Station in a USB port
* Open QGroundControl
* Select the corresponding COM port (Eg. COM78)
* Select the correct baudrate : "56200"
* Press the connect button


If your flying platform is power-on and the xbee modules paired together
* in **"Mission" Tab**, your MAV should appear
* Check the sys_id