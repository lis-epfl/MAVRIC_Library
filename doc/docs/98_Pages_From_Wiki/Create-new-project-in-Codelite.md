If you use an existing codelite project, you can skip this section.

- Select "New workspace...", create a new workspace in the desired directory
- Select "New Project", create a new gcc executable project in the desired directory containing the files. 
- Choose avr32gcc as the compiler.
- Right-click on the new project, select "Import files from directory" and select all the required directories to import and the extension of all files (.c .cpp .h .hpp .S ...)

- Go to project Settings, and set up compiler, assembler and linker options, as well as Include paths (all Library directories that should be included). It's probably easiest to copy this from either an existing codelite project, or open the makefile that Atmel Studio created and search for the compiler/linker/assembler options there. Here are the current compiler options as starter:
    * C++ Compiler Options: ```-O3 -std=gnu++0x -mhard-float -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c  -Wpointer-arith -mrelax -MD -MP -MF ```
    * C Compiler Options: ``` -O3 -mhard-float -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF ```
    * Assemble Options: ```  -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax ```
    * Preprocessors: ``` BOARD=USER_BOARD;DSP_OPTIMIZATION=DSP_OPTI_SPEED;DSP_ADPCM;DSP_RESAMPLING;DSP_FILTERS;DSP_OPERATORS;DSP_SIGNAL_GENERATION;DSP_TRANSFORMS;DSP_VECTORS;DSP_WINDOWING;UDD_ENABLE ```
    * Linker Options ``` -nostartfiles -Wl,-Map="mavric.map" -Wl,--start-group -lm  -Wl,--end-group -L"src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc"  -Wl,--gc-sections -mpart=uc3c1512c -Wl,--relax -Wl,-e,_trampoline ```

- Set up "post-build" commands to convert the .elf file to a .hex file, and print the size:
```
avr32-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature  $(IntermediateDirectory)/../$(ProjectName).elf $(IntermediateDirectory)/../$(ProjectName).hex
avr32-size $(IntermediateDirectory)/../$(ProjectName).elf
```

- Try building the project. There are most likely some files that were imported which are redundant or otherwise in conflict. If so, remove those files from the project ("Remove"). When asked, only remove them from the project, but do not delete from disk - they might be needed by other projects.