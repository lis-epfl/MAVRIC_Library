# Using linux as development platform

It is possible to build an AVR32 project in Linux. Setting up new projects is probably easier in Atmel Studio, as the Atmel Software Framework is easiest to set up with the built-in wizard. However, once created, it is easily possible to create additional Makefiles or workspaces/projects in Linux, even in the same directory. It is recommended to use a different build directory (e.g. "Debug_linux" instead of "Debug").

To build an AVR32 project in Linux, we will need:
- CodeLite or Sublime
- avr32-gcc (from Atmel) 
- atmel-headers (also from Atmel, has to go into the avr32/include directory of the avr32-gcc install). 

## 1 - Install the toolchain
- get the Atmel headers
 - go to the Utilities folder of the MAV'RIC folder. Enter the Atmel headers folder or
 - download  `avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz` and `atmel-headers-6.1.3.1475.zip` from Atmel's website (or the more current version if available). We assume the files are in `~/Downloads/`  
- unpack the toolchain:
```
cd ~/Downloads
tar xvfz avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
sudo mv avr32-gnu-toolchain-linux_x86 /usr/local
```
- install headers: the zip file is located in the repository in Code/Library/Atmel_headers folder. 
```
unzip atmel-headers-6.1.3.1475.zip
sudo mv atmel-headers-6.1.3.1475/* /usr/local/avr32-gnu-toolchain-linux_x86/avr32/include
rmdir atmel-headers-6.1.3.1475
```
- create symbolic links in /usr/local/bin:  
`sudo ln -s /usr/local/avr32-gnu-toolchain-linux_x86/bin/avr32* /usr/local/bin`

Now avr32-gcc should be available. Try `avr32-gcc -v` to verify.

#### Note: alternatively, the toolchain can be compiled from sources
Follow the instructions in this repository [[https://github.com/denravonska/avr32-toolchain]]

## 2 - Set-up project in CodeLite

A [[codelite|http://codelite.org]] project is provided (see [[ here | http://codelite.org/LiteEditor/Repositories#toc1 ]] for installation on ubuntu), but it is also possible to use a another IDE like eclipse, or simply a makefile.

### 2-0 Using the existing CodeLite workspace

- Open workspace "Maveric_???.workspace" in codelite. 

- Set up the avr32 compiler:
    * go to Settings -> build settings, create a 'New...' compiler profile (e.g. `avr32gcc`) 
    * under `Tools`, enter `avr32-gcc` for C compiler, assembler name, and enter `avr32-g++` for C++ and linker. 
    * For the share object linker, enter `avr32-g++ -shared -fPIC`, 
    * For the Archive, enter 'avr32-ar rcu' and for the resource compiler, enter 'windres'
    * To speed up build time, change 'make' to 'make -j 4'
    * Under the tab "Files types", double click on the extension "s" and replace the comilation line by 
```
$(AS) $(ASFLAGS) "$(FileFullPath)" $(ObjectSwitch)$(IntermediateDirectory)/$(ObjectName)$(ObjectSuffix) -I$(IncludePath)
```

- Go to Menu Workspace -> Open Active project settings under Common Settings -> General, select the avr32gcc compiler profile.

The project should now build successfully.

### 2-1 Creating a new workspace/project:
If you want to create a new project from scratch, follow the explanations on [[this page|Create-new-project-in-Codelite]].

## 3 - Set-up on Sublime
- Install sublime text editor, version 2 or 3 ([link](http://www.sublimetext.com/)). 
- Open sublime and open your project folder
- To set the build command
 - Go to "Tools"->"Build Systems"->"New Build System..."
 - Paste this text in the file
 ```
{
   "shell": true,
   "cmd": ["make -j8 -C avr32_proj"],
   "file_regex": "^(..[^:]*):([0-9]+):?([0-9]+)?:? (.*)$",
   "working_dir": "${project_path:${folder:${file_path}}}",
   "selector": "source.makefile",
   "path": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games",
   "variants":
    [
      {
        "name": "Clean",
        "cmd": ["make -C avr32_proj clean"]
      },
      {
	"name": "Rebuild",
	"cmd": ["make -C avr32_proj clean && make -j8 -C avr32_proj"]
      }
    ]
}
 ```
 - Save the file where Sublime propose automatically (the name is not important, just remember it)
 - Go to "Tools"->"Build Systems" and select "YOUR_SYSTEM_BUILD"

- To setup a shortcut for "clean" and "rebuild" options
 - Go to "Preferences"->"Key Bindings - User"
 - Paste this text in the opened file
 ```
[
    { "keys": ["ctrl+alt+shift+b"], "command": "show_overlay", "args": {"overlay": "command_palette", "text": "Build: "} }
]
```
 - Save and close the file

- To compile the code: click on ctrl+b
- To clean the code: click on ctrl+alt+shift+b and select "Build: Clean"
- To rebuild the code: click on ctrl+alt+shift+b and select "Build: Rebuild"

## 4 - Flashing the autopilot
Flashing the binary image to the AVR32 can be done via DFU after flashing a bootloader (check separate instructions [[ Installing Bootloader | First Steps Installing Bootloader ]] and [[ DFU Programming | First Steps DFU Programming ]]). Unfortunately, the JTAGICE3 is not supported in Linux. 