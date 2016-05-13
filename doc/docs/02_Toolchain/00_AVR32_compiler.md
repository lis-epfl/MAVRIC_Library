# Linux

The avr32-gcc toolchain can be compiled from source. Alternatively, a pre-compiled version is available on Atmel's servers, however it does not work on all linux versions:
- ubuntu 14.04: OK
- archlinux : does not work as of 23/02/2016


## From source
Follow the instructions in the repository [lis-epfl/avr32-toolchain](https://github.com/lis-epfl/avr32-toolchain) to compile the toolchain from source.

## Pre-compiled toolchain

- get the Atmel headers
- go to the Utilities folder of the MAV'RIC folder. Enter the Atmel headers folder or
- download  `avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz` and `atmel-headers-6.1.3.1475.zip` from Atmel's website (or the more current version if available). We assume the files are in `~/Downloads/`  
- unpack the toolchain:

        cd ~/Downloads
        tar xvfz avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
        sudo mv avr32-gnu-toolchain-linux_x86 /usr/local

- install headers: the zip file is located in the repository in Code/Library/Atmel_headers folder. 

        unzip atmel-headers-6.1.3.1475.zip
        sudo mv atmel-headers-6.1.3.1475/* /usr/local/avr32-gnu-toolchain-linux_x86/avr32/include
        rmdir atmel-headers-6.1.3.1475
        
- create symbolic links in /usr/local/bin:  
        
        sudo ln -s /usr/local/avr32-gnu-toolchain-linux_x86/bin/avr32* /usr/local/bin

Now avr32-gcc should be available. Try `avr32-gcc -v` to verify.

#### Note: alternatively, the toolchain can be compiled from sources

<!-- ###################################################################### -->
___

# Mac OS


Follow the instructions in the repository [lis-epfl/avr32-toolchain](https://github.com/denravonska/avr32-toolchain) to compile the toolchain from source.


<!-- ###################################################################### -->

___

# Windows

It is possible to build AVR32 project on Windows. There are two ways of doing it.
1. [Cygwin](https://www.cygwin.com/) (alone or with a IDE of your choice)
2. [Atmel Studio](http://www.atmel.com/tools/ATMELSTUDIO.aspx)

It is highly recommended to use the first method. The method 1 uses the same Makefile as linux and MAC OS users, thus is better suited for development with collaborators.
The method 2) uses a .cproj file, which is independent from our Makefiles, and works only with Atmel Studio. 

But in any case, to install the bootloader into the Megafly board, Atmel Studio must be used.

## Cygwin

- Download [Cygwin](https://www.cygwin.com/).

Be sure to have the following packages included:
- Devel / "binutils"
- Devel / "gcc-g++" & "gcc-core"
- Devel / "make"

The Makefile will call 'avr32-g++' or 'avr32-gcc' to compile. These functions must be made global in windows to be called.
- Add the link 'C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.435\avr32-gnu-toolchain\bin' into your Environment Variables. This link depends on your installation folder.

### Compile using command line in Cygwin

Now you can compile using Cygwin.
- Open Cygwin
- Write 'cd /cygdrive/C' to go to your hard drive 'C:' if your Makefile is there.
- Go to your Makefile folder using 'cd'
- type 'make'. It should produce your .hex file.


### Compile using Eclipse
- Download [Eclipse](http://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/marsr)



- In Eclipse, click 'file'->'New -> 'Makefile Project with existing code' and select the folder where the code is in.
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

### Compile using Sublime Text

- Download [Sublime](http://www.sublimetext.com/)
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

        [
            { "keys": ["ctrl+shift+b"], "command": "show_overlay", "args": {"overlay": "command_palette", "text": "Build: "} }
        ]

- To add a shortcut to flash the board add the following code after the command "clean". It will launch the .bat file which must be here in your current folder

        {
          "name": "Flash",
          "cmd": ["dfu-programming.bat"]
        }

 - Save and close the file

- To compile the code: click on ctrl+b
- To clean the code: click on ctrl+shift+b and select "Build: Clean"

## Atmel Studio
You will need to download [Atmel Studio](http://www.atmel.com/tools/ATMELSTUDIO.aspx).
The installation of Atmel Studio will install the toolchain required to compile the code for the AVR32 microcontroller (mainly here: C:\Program Files (x86)\Atmel\Atmel Toolchain\).

The code edition and the compiling can be both made from Atmel Studio. For this, open the .cproj file in Atmel Studio. This file is the 'Makefile' for Atmel Studio. This file contains the choice of the compilator and all the dependences. You can download is from github as well as all the code. If it doesn't exist you need to create through Atmel Studio wizard. For that, Click 'File'->'New'->'Project' and follow the instructions.
