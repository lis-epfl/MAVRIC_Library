This document describes the necessary steps to pare two XBees.
# 1. On Windows
## 1.1 Getting X-CTU

For doing this, you will need a software, called X-CTU from Digi.
* [**Download** it](http://www.digi.com/support/productdetail?pid=3352&type=utilities)
* **Install** it.
* Do **not open** it yet. Due to some bugs X-CTU does not refresh USB port enumeration. So anything open after the software is running will not be listed.


## 1.2. Configuring your Xbee module
Plug your Xbee ground station in a USB port.
Check that the related driver is loaded properly.


Open X-CTU.
* In **"PC Settings" tab** select the corresponding COM Port (E.g: "USB Serial Port (COM78)").
* In **"Modem Configuration" tab** click **"Read"** to check if the reading works.
* In **"Modem Configuration" tab** click **"Load"** and load the file "<...>/maveric/Utilities/Xbee/Xbee-* Configuration.pro".
* Under **"Networking & Security -> PAN ID"** change the ID to whatever free number of 4 digits (E.g: "1234", but different from your neighbors! Check the bottom of this page to see the used PAN ID).
* Click **"Save"**.
* Click **"Write"**.
* In **"PC Setings" tab**, change the baud to "57600".
* In **"Modem Configuration" tab** click "Read" to check the settings.
* Close X-CTU

Unplug the Xbee ground station.
Change  Xbee module and plug it in.


Restart X-CTU.
* Repeat the same steps for the second module

N.B.: If you cannot read the settings of the XBee from the **"Modem Configuration" tab**, you can follow the same procedure than for Linux to set the settings. 

# 2. On linux

Instructions taken from [here](http://www.thekanes.org/2012/02/21/how-to-configure-an-xbee-on-linux-mac-or-any-other-operating-system/) 

## 2.1. Install and open a serial port terminal (e.g. minicom)
Install minicom from the package manager or from [here](https://alioth.debian.org/projects/minicom/)

## 2.2. Configure the XBee 
Open minicom with super user rights, with the option -D and the path to the XBee USB port (e.g. /dev/ttyUSB1) 
* **"sudo minicom -D /dev/ttyUSB1"**

### 2.2.1. Set the correct serial port setup at which the XBee is currently set:
Default factory values: baud rate 9600, parity Null, data 8, stopbits 1
* Type **"CTRL-A Z"** to enter the menu
* Type **"O"** to enter the "cOnfigure Minicom" menu
* Type the up or the down arrow on the keyboard until "Serial port setup" is highlighted
* Type **"enter"**
* Type **"E"** to enter the menu to change the settings
* Type **"C"** to set the baudrate speed to 9600
* Type **"L"** to set the Parity to None
* Type **"V"** to set the Data to 8
* Type **"W"** to set the Stopbits to 1
* Type **"enter"** to exit this menu
* Type again **"enter"** to exit the setting menu
* Type the up or the down arrow on the keyboard until "Exit" is highlighted
* Type **"enter"** to go back to exit the setting menu

### 2.2.2. [Optional but recommended] Activate the local Echo of commands
* Type **"CTRL-A Z"** to enter the menu
* Type **"E"** to activate the local Echo

### 2.2.3. Change the baudrate
Put the XBee in command mode
* Type quickly **"+++"** (without pressing **"enter"**), the XBee should answer "OK"
* Type **"AT"** and **"enter"**, the XBee should answer "OK"

Change the baudrate
* Type **"ATBD 6"** and **"enter"** to put the XBee in baudrate 57600, if you want to use another baudrate, select the correct number from this list
  * 0 = 1200
  * 1 = 2400
  * 2 = 4800
  * 3 = 9600
  * 4 = 19200
  * 5 = 38400
  * 6 = 57600
  * 7 = 115200
* Type **"ATWR"** and **"enter"** to write the settings to the flash memory of the XBee, the XBee will then send with the new baudrate

Change the minicom settings to listen to the new baudrate (Not needed if you stay on the same baudrate)
* Type **"CTRL-A Z"** to enter the menu
* Type **"O"** to enter the "cOnfigure Minicom" menu
* Type the up or the down arrow on the keyboard until "Serial port setup" is highlighted
* Type **"enter"**
* Type **"E"** to enter the menu to change the settings
* The usual baudrate for MAV'RIC is 57600 but is not a default option
 * Type on **"D"** to select the 38400 baudrate, then type **"A"** to select the next baudrate, i.e. 57600
* Type **"enter"** to exit this menu
* Type again **"enter"** to exit the setting menu
* Type the up or the down arrow on the keyboard until "Exit" is highlighted
* Type **"enter"** to go back to exit the setting menu

### 2.2.4. Change the Pan ID
Put the XBee in command mode
* Type quickly **"+++"** (without pressing **"enter"**), the XBee should answer "OK" (Note: If the XBee is not answering at this point, it means your settings are not properly set, redo previous steps)
* Type **"AT"** and **"enter"**, the XBee should answer "OK"

Change the Pan ID
* Type **"ATID PANID"** and **"enter"** where PANID is the new Pan ID that you want to give to your device
* Type **"ATWR"** and **"enter"** to write the settings to the flash memory of the XBee

### 2.2.5 Used Pan ID
* 1234: Grégoire
* 1986: Nicolas
* 4321: Julien
* 1122: Alex
* 1111: Basil
* 4567: Matteo
* 3333: Matt