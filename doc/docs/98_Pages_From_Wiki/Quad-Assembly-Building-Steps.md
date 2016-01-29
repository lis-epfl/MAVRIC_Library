For the maveric board electronics, see <...>\MAVRIC\Electronics\Maveric32_autopilot\Rev4.1.1\Board_Assembly411.pdf which is a clone from this github.

* Solder the 2mm short connector 
(Male on the motor phases - Female on the Speed controller phases)
* Solder the Male 3.5mm gold connectors on the speed controller power-lines
* Cut one Female connector of the Y connector
* Solder the JST red connector

* Cut the carbon rods and strips respectively to 49cm and 21.5cm long.
* Sand the extremity of each carbon parts (to ease the slicing of 3D parts in)

* Slice the central connection part along the rods
* Center this part compare to the rod extremity
* Slice the speed controller holders
* Slice the motor holders so that the lower rod motor holder is aligned with the upper rod motor holder
* Slice the carbon strips into the motor holders
* Adjust the same distance between each rod extremity and motor holder (around 7.5cm)

* Gently screw in and out one screw in all battery rack holders, to ease the next steps
* Put the first layer of foam on the battery rack holder
* Place the carbon rods on top, to have a X-shape Quadcopter.
* Put the second layer of foam
* Place the central part (with the autopilot not screwed in)
* Screws them together, gently. Otherwise you will brake the battery rack holders !!!

* Screw the motors on
* Fix the speed controller using a cir-clip
* Connect the motor phases with the speed controller phases
* Connect the speed controller power-lines with the Y connector (red-red & black-black)
* Place the Y connector on the bottom right part of the central part (not to disturb magnetometers which are on the left part of the autopilot)
* Make the speed controller signal wiring going through the hole at each corner of the central part.
* Place the satellite receiver in its slot on the central part.
* Before going further, you should have calibrated your autopilot. If not first check the calibration part
* Screw the autopilot on the central part, leaving the speed controller wiring going out from each corner.
* Place the GPS

We are almost done

Plug the motors in the autopilot’s motor-plug (1: back-left, 2:front-left, 3:front-right and 4 back-right) 
Plug the satellite receiver in the autopilot.
Plug the GPS signal in the autopilot.
Plug the JST connector in the autopilot
Last minute check
Power on the remote
Plug a battery in the Y connector
Arm the motors, they should turn:
Front-left clock-wise
Front-right counter-clock-wise
Back-right clock-wise
Back-left counter- clock-wise
If the motors don’t turn in the correct direction:
Unplug the battery
Unplug the two phases of that motor
Swap them together
Check again

# Electronic plugs
The back of the drone is where the receiver(satellite) is placed
* The back left ESC must be plugged in the place 1 (triplet of pin)
* The front left ESC in 2
* The front right ESC in 3
* The back right ESC in 4
* The GPS in GPS plug
* The emitter in DC1
* The ultrasonic sensor in I2C4