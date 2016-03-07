We are using [QGroundControl software](http://www.qgroundcontrol.org), an Open Source Micro Air Vehicle Ground Control Station designed by ETHZ. 

We use it to visualize onboard variables, plot them and program the flight plan, in real time.

### For windows
* Download the [software](http://www.qgroundcontrol.org/downloads)

### For Linux
* Follow the steps on [build from source](https://github.com/mavlink/qgroundcontrol/blob/master/README.md)

### For MacOSX

* Follow the steps on [build from source](https://github.com/mavlink/qgroundcontrol/blob/master/README.md)

__NB__ If you encounter an error like `Project ERROR: Could not resolve SDK path for 'macosx10.9'` : 
* Go to `qgroundcontrol.pro`, `QGCCommon.pri`, or `user_config.pri` and find the `MacBuild` section
* Change the variables like follows : 

```
MacBuild {
    # Stuff
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.11  # Xcode 7 -> 10.11 // Xcode 6 -> 10.10
    QMAKE_MAC_SDK = macosx10.11             # Xcode 7 -> 10.11 // Xcode 6 -> 10.10
    # More stuff
}
```

### For all platforms

* Open QGroundControl

It opens a window and ask which platform you gonna flight with. 

* Select the plane icon, and press Enter. 

If you don’t want to be asked this each time you run the software: 
* Check the check-box at the bottom left asking whether you want QGroundControl to remember your choice