This page will explain you how to calibrate the MAVRIC autopilot.
It is based on a paper by xxxx called : yyy

This page explains how to get the scale_factor of the IMU:
resume: 
You will attach the board to the calibration setup (not the quad yet) and roll it smoothly, while logging the raw values of the accelerometer and magnetometer. It will give you points for all possible orientation of the MAV'RIC board. The crucial part, is not to add any external force while rolling the board, not to decrease the calibration performance.

# Steps:
## Preliminar:
* Open your project code and check that in src/mavlink_telemetry.c file, in `mavlink_telemetry_init(void)` function, RUN_REGULAR is set for imu_telemetry_send_raw, else set it.
* Compile and flash the code (don't close Atmel as you will reuse it later on)
* Attach the MAV'RIC board to the calibration setup
* Plug a xbee module in the computer
* Pair the xbee module with the one on the quad if not already done, see [[ Pairing XBee modules | First steps pairing XBee modules ]].
* Power-up the MAV'RIC board with a 2s battery, that you can slice between the board and the calibration setup.
* Open QGroundControl.
* Connect through QGroundControl (top right of the window) and check that the connection succeed. (you should see the attitude moving for example)
* Set the MAV'RIC board to a given orientation (e.g. parallel to the calibration setup with the graduation teeth in front)
* Click Pro->plot in QGroundControl and select the six values RAW_IMU.xacc, .yacc, .zacc, .xmag, .ymag and .zmag .
* Be ready to roll the setup smoothly, then you can press "Start Logging" (NOT "LOG") at the bottom part of QGoundControl window.
* Set the storage place of the log file to MAVRIC/Utilities/Calib_IMU, and call it day_month_year_MAVLINK_SYS_ID (e.g. 32_08_2014_MAV033 for a log made the 32nd of August 2014 with MAV'RIC board n°033). The ID is normally 201 by default, ask Grégoire Heitz for a new ID number.

## Calibration:
* Start rolling the board smoothly, 1 revolution/turn per graduation, then smoothly shift the MAV'RIC board orientation by one graduation, and follow until to reach the starting orientation graduation.
* Press on "Start logging" again. It will end logging.

## Get scale factors:
* Make sure you are in your git branch 
* Go in MAVRIC/LEQuad/src/config/MAVcalib
* Copy and paste MAV007_imu_config.h file,
* rename the copy into MAVxxx_conf_imu_rev4.h, replacing xxx by your MAVLINK_SYS_ID.
* Open Matlab and set the current folder to maveric/Utilities/Calib_IMU
* open calibration_scripts.m
* Change the file name to point to your logging file ending by _compressed.txt
* Run the script.
* Some figures will show up

* Read in Matlab console the output scale factor for the accelerometer.
* Paste them in MAVRIC/LEQuad/src/config/MAVcalib/MAVxxx_imu_config.h file (RAW_ACC_X_SCALE, RAW_ACC_Y_SCALE and RAW_ACC_Z_SCALE)
* Then, repeat the same for the scale factor of the magnetometer.

* Edit conf_platform.h file in MAVRIC/LEQuad/src/config, to change MAVLINK_SYS_ID with yours.
* Drag and drop the file MAVRIC\LEQuad\src\config\MAVcalib\MAVxxx_imu_config.h in src/config/MAVcalib in Atmel Studio.

* In conf_imu.h: before,
```
#else
#include "conf_imu_rev4.h"
#endif
```
add:
```
#elif MAVLINK_SYS_ID == xxx
#include "MAVcalib/MAVxxx_imu_config.h"
```
changing xxx by your own MAVLINK_SYS_ID.
* Edit main.cpp file and add `onboard_parameters_write_parameters_to_flashc(&central_data->mavlink_communication.onboard_parameters);` in the `initialization()` function. You can copy this just above `onboard_parameters_write_parameters_to_flashc(...)`.
* Compile and flash the code
* Edit main.cpp file to remove the `onboard_parameters_write_parameters_to_flashc(...)` line in the `initialization()` function.
* You can now check if the scale factors have been correctly updated : 
* Start the autopilot and connect in QGroundControl
* In QGroundControl, in the Analyse tab, press Refresh
* Check the scale factor values (you should see 1/your_scale_factor)

### You are DONE
Now you can have a look on how to [[tune the bias for the accelerometer and magnetometer|Calibration Bias Factors]].