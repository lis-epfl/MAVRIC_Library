# Using Windows as development platform

It is possible to build AVR32 project on Windows. There are two ways of doing it.
- 1) Use [Atmel Studio](http://www.atmel.com/tools/ATMELSTUDIO.aspx)
- 2) Use [Sublime](http://www.sublimetext.com/) with [Cygwin](https://www.cygwin.com/) in background

The method 1) uses a .cproj file as the 'Makefile' file. The .cproj file works only with Atmel Studio. The method 2) uses the Makefile (same as in Linux) which is maintained and modified by all linux users (a lot at LIS!). The second possibility is better suited for development with collaborators.
But in any case, to install the [bootloader](https://github.com/lis-epfl/MAVRIC/wiki/First-Steps-Installing-Bootloader) into the MAVRIC board, Atmel Studio must be used.

## Install Atmel Studio
You will need to download [Atmel Studio](http://www.atmel.com/tools/ATMELSTUDIO.aspx).
The installation of Atmel Studio will install the toolchain required to compile the code for the AVR32 microcontroller (mainly here: C:\Program Files (x86)\Atmel\Atmel Toolchain\).

# Using Atmel Studio
The code edition and the compiling can be both made from Atmel Studio. For this, open the .cproj file in Atmel Studio. This file is the 'Makefile' for Atmel Studio. This file contains the choice of the compilator and all the dependences. You can download is from github as well as all the code. If it doesn't exist you need to create through Atmel Studio wizard. For that, Click 'File'->'New'->'Project' and follow the instructions.

# Using Eclipse and Cygwin
The possibility allows you to compile using the Makefile used in linux. Follow the instructions.
- Download [Eclipse](http://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/marsr)
- Download [Cygwin](https://www.cygwin.com/). Take care to download the packages that can runs the 'make' commands ("Devel" section, find the make package and take it). Some recommended packages are found on this [page](http://www.dogsbodynet.com/openr/install_cygwin.html) - part 9.

The Makefile will call 'avr-g++' or 'avr-gcc' to compile. These functions must be made global in windows to be called.
- Add the link 'C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.435\avr32-gnu-toolchain\bin' into your Environment Variables. This link depends on your installation folder.

## Compile with Cygwin
Now you can compile using Cygwin.
- Open Cygwin
- Write 'cd /cygdrive/C' to go to your hard drive 'C:' if your Makefile is there.
- Go to your Makefile folder using 'cd'
- type 'make'. It should produce your .hex file.

## Code with Eclipse
- Click 'file'->'New -> 'Makefile Project with existing code' and select the folder where the code is in.

You can now edit the code.

## Compile with Eclipse
You can compile with Eclipse. Eclipse will run the commands you indicated.
 - Add the link 'C:/Program Files/cygwin64/bin' into your Environment Variables (path). This link depends on your installation folder of Cygwin.
 - Go to 'Project'->'Properties'->'C/C++ build' -> Builder Settings
 - Uncheck "Use default build command"
 - In "Build Command" write: 'make -j8 -C path_to_the_Makefile'
 - Go to 'Project'->'Properties'->'C/C++ build' -> Behavior
 - In "Clean" write : 'clean -C path_to_the_Makefile'

'path_to_the_Makefile' must be the link to your Makefile. 

To add a shortcut to flash the board, do the following. But you should have your .bat file which run the flashing automatically.
 - Go to 'Run' -> 'External tools' -> 'External Tools Configurations'
 - In the "Main" -> "Location" panel paste the link to the .bat file including it
 - In the "Main" -> "Working Directory" base the link to the .bat file without including it
 - In the "Build" uncheck build before launch.

# Using Sublime and Cygwin
The possibility allows you to compile using the Makefile used in linux. Follow the instructions.
- Download [Sublime](http://www.sublimetext.com/)
- Download [Cygwin](https://www.cygwin.com/). Take care to download the packages that can runs the 'make' commands ("Devel" section, find the make package and take it). Some recommended packages are found on this [page](http://www.dogsbodynet.com/openr/install_cygwin.html) - part 9.

The Makefile will call 'avr-g++' or 'avr-gcc' to compile. These functions must be made global in windows to be called.
- Add the link 'C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.435\avr32-gnu-toolchain\bin' into your Environment Variables. This link depends on your installation folder.

## Compile with Cygwin
Now you can compile using Cygwin.
- Open Cygwin
- Write 'cd /cygdrive/C' to go to your hard drive 'C:' if your Makefile is there.
- Go to your Makefile folder using 'cd'
- type 'make'. It should produce your .hex file.

## Code with Sublime
- Click 'file'->'open folder' and select the folder where the code is in.

You can now edit the code.

## Compile with Sublime
You can compile with Sublime. Sublime will run the commands you indicated.
 - Add the link 'C:/Program Files/cygwin64/bin' into your Environment Variables (path). This link depends on your installation folder of Cygwin.
 - Go to "Tools"->"Build Systems"->"New Build System..."
 - Paste this text in the file
 ```
{
"working_dir": "${project_path:${folder:${file_path}}}",
"cmd": ["make","-j8","-C","D:/_windows/_github_doc/symbiotic/leproto1"],

"variants":
	[
	  {
	    "name": "Clean",
	    "cmd": ["make","clean","-C","D:/_windows/_github_doc/symbiotic/leproto1"]
	  },
	]
}
 ```
'D:/_windows/_github_doc/symbiotic/leproto1' must be the link to your Makefile. 
- Save the file where Sublime propose automatically (the name is not important, just remember it to further modify it, in my case: 'C:\Users\lis\AppData\Roaming\Sublime Text 3\Packages\User')
 - Go to "Tools"->"Build Systems" and select "YOUR_SYSTEM_BUILD"

- To setup a shortcut for "clean" and "rebuild" options
 - Go to "Preferences"->"Key Bindings - User"
 - Paste this text in the opened file
 ```
[
    { "keys": ["ctrl+shift+b"], "command": "show_overlay", "args": {"overlay": "command_palette", "text": "Build: "} }
]
```
- To add a shortcut to flash the board add the following code after the command "clean". It will launch the .bat file which must be here in your current folder
```
{
  "name": "Flash",
  "cmd": ["dfu-programming.bat"]
}
```
 - Save and close the file

- To compile the code: click on ctrl+b
- To clean the code: click on ctrl+shift+b and select "Build: Clean"
