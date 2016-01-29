This part explains how to flash a new code on the autopilot.

## 1. Compile (on windows)
* Open Atmel Studio 6.
* Make sure you are in your personal git branch (previously called 'mycode')
* Open your project <..>\MAVRIC\LEQuad\Maveric_LEQuad.cproj
* Check that the project loaded correctly
* Select at the top Release (or Debug if you need to do some)
* In "Build" tab, press rebuild (to clean and build) or simply build
* Check the output. You should not get any error, warning nor message.

In case of troubles in the following part read [[ Windows driver fix | First steps CDC and DFU Drivers issue on Windows ]]

## 2. Flash
* Close Atmel
* Make sure you are in your personal git branch (previously called 'mycode')
* Go in <..>MAVRIC/LEQuad
* plug the maveric board on the mini USB port to the computer
* Maintain the DFU button down (the one close to the ESC connectors) while pressing once on the other (reset button, next to the USB plug) 
* Double-click on dfu-programming script (.bat on windows or .sh on Linux)

It will open a console.
* Check the output message

It should be:

`<...>\maveric\Code\Your_Project>Library\DFU-programmer\DFU\dfu-programmer at32uc3c1512` (Followed by)
* `erase`
* `get (responding: `"dfu-programmer: no device present."`)`
* `flash Release\MegaFly2.hex --suppress-bootloader-mem`

Responding: 
1. `Validating...`
2. `xxx bytes used <yy%> (Eg. 215220 bytes used <41.70%>)`

* `reset`
* `pause`
* `Press any key to continue`

### if anything is different try to see what is going on.
You can simply edit the script to change the file path for eg.

Now you can calibrate the maveric board, see [[ Scale Factors | Calibration Scale Factors ]]