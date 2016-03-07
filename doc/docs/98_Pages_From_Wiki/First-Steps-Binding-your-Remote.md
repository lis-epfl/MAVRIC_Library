This part explain how to pair your remote with the satellite on the autopilot side, which is called the binding process.

## Create or import a widget on QGroundControl
* The start of the binding sequence is sent to the autopilot via a mavlink_command_long message,
* Import on your QGroundControl the widget located in the QGroundControl folder of Utilities,
 * In QGroundControl, click on "Tool Widgets" -> "Load Custom Widget File",
 * Go to your local copy of the "Utilities/QGroundControl" folder, select the widget "remote_binding_widget 10bits.qgw",
 * Click open,
 * Re-click on "Tool Widgets" -> "Remote Controller Binding",
 * To start pairing, click on "Start bind mode", or,
* Create your own widget,
 * In QGroundControl, click on "Tool Widgets" -> "New Custom Widget",
 * Right click on the created widget,
 * Click on "New MAV Command Button"
 * Choose as command ID, the MAV_CMD_RX_PAIR command
 * Set the parameter number 2 to 1
 * [Optional] Rename the button
 * Click on "Done"
 * Repeat the process for a second button, with parameter 3 to 1 and parameter 2 to 0

## Bind the remote
* Power-off your remote
* Press the bind button at the back of the remote
* Power-on your remote, keeping the bind button pressed. 
* Wait for a steady led on the satellite receiver. 
* Release the bind button

## Final Step
* To stop pairing, click on "Stop bind mode"
or 
* Press reset button
* Now, arm the drone by moving the left joystick in the bottom left position and the right joystick in the bottom right position. If you move the joystick the propeller will turn. Do disarm, the left joystick must be moved in the bottom right position and the right joystick in the bottom right position.  

As you can see, the propeller doesn't start turning at the same time, for this:
* Open Atmel Studio and your project in MAVRIC\LEQuad
* Open the file /src/central_data.c
* write 'pwm_servos_calibrate_esc( &central_data.servos);' just before 'pwm_servos_write_to_hardware( &central_data.servos );'
* Build and flash your drone with that code. 
* Unplug the USB cable used for flashing or the AVR programmer, and plug the battery to the drone, then press the reset button (next to the USB port, on the other face).
* Arm the quad and check with the remote controller that every motors are turning at the same speed.
* In Atmel, remove 'pwm_servos_calibrate_esc( &central_data.servos);'
* Build and flash your drone with that code.

You are done !

## Use more than 7 channels
Be default, the remote controller and the receiver are binded in 10bits. The transmission protocol is DSM2 (others are DSMX, PPM). It allows you to send up to 7 channels. To send more channels. You will have to:
*Turn on your remote controller (Turnigy 9XR for the example) and go in the 'setup' page, then scroll down to 'Proto', check that 'PPM' is written and put the number of channels you want.
*Open Qgroundcontrol and do what is written above in 'Create or import a widget on QGroundControl' but with te file "remote_binding_widget 11bits.qgw".
*You have to set the emitter to 11bits as well, for this, at the back of your remote controller press 3 times with 0.5sec interval on the bind button. Then everything should work.
