There are 2 differents ways to log data from MAVRIC autopilot:
- Logging on onboard SD card
- Logging on ground computer (via QGroundControl)

---

# Logging on onboard SD card

### Preparation
Hardware:
- format a micro SD card in fat32. With the power disconnected board 
- insert the SD card in the slot which is on the bottom face of the autopilot, below the Xbee module.

Software: 
- make sure the data_logging module is created and initialised in central_data. 
- chose which variables to log, this is done in mavlink_telemetry.c, in the function: 
`bool mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging)`.

You should then compile and flash the new code on the autopilot.

### Usage:
* open QGroundControl Software, 
* connect your platform.
* go in one of the menu which is not too overcrowded such as `Plan` or `Fly`. 
* then, go in the top menu `Advanced->Tool Widgets->Custom Commands`. 

A new panel should appear on the window. You shall be able to `select a QML file`.
* browth to `Your_PATH_TO\MAVRIC\Utilities\QGroundControl\` 
* select the qml file called `Start_Stop_Log`

This will add two buttons on the custom command. One for starting the log, the other to stop it.

---

# Logging on ground computer (via QGroundControl)

### Preparation

For this you need to have the autopilot sending the corresponding variable through [mavlink](https://github.com/lis-epfl/mavlink)

Thus, go in the file `mavlink_telemetry.c`, and edit the function `mavric_telemetry_init()`

Here is an example of one line of this function :
```
init_success &= mavlink_communication_add_msg_send( mavlink_communication, 
                                                    500000, 
                                                    RUN_REGULAR,  
                                                    PERIODIC_ABSOLUTE, 
                                                    PRIORITY_NORMAL,
                                                    (mavlink_send_msg_function_t)&bmp085_telemetry_send_pressure,
                                                    &central_data->pressure, 
                                                    MAVLINK_MSG_ID_SCALED_PRESSURE	);// ID 29
```

Important points:
- make sure that the variable you want to log is activated with the parameter `RUN_REGULAR`. (with `RUN_NEVER`, the variable will not be sent to the ground computer). 
- adjust the period in micro-seconds at which you want to update that variable.
- make sure the structure given as 7th argument (`&central_data->pressure` in the example above) corresponds to what is expected by the function given as 6th argument (`&bmp085_telemetry_send_pressure` in the example above)

### Usage
* start the autopilot
* open QGroundControl 
* connect to your platform. 
* Go in `Analyze` menu
* Select the variables you want to log
* And then press on `Start Logging` at the very bottom of the `Analyze` menu page.
* Chose the location and name of the file you want to log.
You are Done !!

At the end of the log, QGroundControl will ask you whether empty values should be filled with previous one or with zeros. It is because variables are not updated at the same frequency. I suggest to select `fill with previous value` not to introduce discontinuity within the log.
