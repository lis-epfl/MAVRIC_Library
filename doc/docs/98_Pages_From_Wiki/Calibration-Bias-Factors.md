For this part, you need to screw the MAV'RIC board to your quad.

### To calibrate the accelerometer bias factors, your goal is:
For SCALED_IMU.xacc and .yacc, you should have their mean value at zero.
For SCALED_IMU.zacc, you should have its mean value at -1000 (= -1 gravity unit * 1000)

#### If you just did the calibration of the scaled factor, you can skip the following points:
* Edit mavlink_telemetry.c->mavlink_telemetry_init(void) function
* Check that RUN_REGULAR is set for imu_telemetry_send_raw and imu_telemetry_send_scaled, else set it.
* Compile and flash the code

## But in every case, do the following
### Preliminar:
* Open QGroundControl
* Connect to your quad and go into Analyse.
* Level your quad on a nice flat surface !!!

### Calibration:
* Change in this tab the bias of the IMU to fit those mean values.  
=> set Bias_Acc_X as current mean of RAW_IMU.xacc,  
=> set Bias_Acc_Y as current mean of RAW_IMU.yacc.  
=> Adjust Bias_Acc_Z manually in order to have the mean of SCALED_IMU.zacc equal to -1000 (Press Save after each adjustement). 
* Check in QGroundControl->Flight that the attitude estimation is leveled correctly.
* Report this values in MAVRIC\LEQuad\src\config\MAVcalib\MAVxxx_imu_config.h with xxx your MAV ID.

### Your are done for the accelerometer

# To calibrate the magnetometer Bias factors:

### WARNING:
Any magnetic interference would mess up your calibration of the magnetometer !!! Go outside!

Before the first flight, or when arriving to a new location, you can (and should) recompute the biais of the magnetometer. In the Utilities/QGroundControl folder, you can find a widget allowing a on-the-go biais computation. 

* Open QGroundControl and go in 'Instruments' tab
* Connect your communication port
* Click on "Tools Widgets"->"Load Custom Widget File" and import the calibration_widget located in the Utilities/QGroundControl folder
* Re-click on "Tools Widgets"->"Sensor Calibration"
* Estimate the magnetic North direction
* To start the magnetometer calibration, click on the "Magneto calib" button, it starts the calibration process  
* The quadcopter can be seen as a cube with 6 faces and their normals. Once the calibration started, align each face to the north and move/shake the drone so that the normal of the face remain in a small angle related to the initial one, the purpose being that during this motion the normal points once really in the north. Do it for the 6 faces.
* Re-click on the "Magneto calib" button, this ends the calibration process
* Go in pro->plot, To have your new biais, refresh the value of your on-board parameters, (click on the "Get" button of the on-board parameter widget)
# Calibrate the Gyro initial bias
Here we want to use the gyro bias to converge quicker at boot.

### Steps:
* In QGroundControl
* Connect to your quad 
* Go in Pro->plot.
* Level your quad properly
* wait until RAW_IMU.xgyro/ygyro/zgyro values stabilize (1min)
* Press Get button to refresh parameters.
* Read the Parameters->Component->Bias->BiasGyro_x/y/z
* Press Write to write them in the flash.
* Edit <...>\maveric\Your_Project\src\config\MAVsettings\MAVxxx_imu_config.h
* Write the gyro bias there.

Before leaving:
* Go back to Atmel
* undo previous editing of the code that are not relevant for future uses, as in maveric_telemetry.c and in main.cpp 
* compile and flash again

### You are DONE
Now, in case you did not calibrate scale factors of your board, have a look on the [[scale factors page|Calibration Scale Factors]].