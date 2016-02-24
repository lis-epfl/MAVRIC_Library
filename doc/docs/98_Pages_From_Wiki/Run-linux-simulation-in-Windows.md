This part explains how to run the linux simulation of the mavric in windows using cygwin.

## 1. Install Cygwin
Download [cygwin](https://www.cygwin.com/) for your operating system (32 or 64 bits). NB: The following was working on win7 64bits.
Install cygwin and be sure to have the following packages included:
- Devel / "binutils"
- Devel / "gcc-g++" & "gcc-core"
- Devel / "make"

## 2. Compile and run the simulation
Open Cygwin and go to the folder of the Makefile (type 'cd /cygdrive/C' to go in C:/) and type 'make' to compile it.
- It compiles the code and produces and .elf file as output. 
- Then type './xyz.elf' to run the file (here named xyz).