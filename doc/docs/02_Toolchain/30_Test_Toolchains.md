In order to make sure the toolchains are installed successfully, sample projects are available in the library. They can be found in the folder `MAVRIC_Library/sample_projects`. These projects are automatically tested by travis CI before every push to master, so if you download your code from the master branch, the 3 sample projects should compile.

___

# AVR32 Sample Project
```bash
cd /path-to-library/MAVRIC_Library/sample_projects/LEQuad/proj_avr32
make
```

It compiles the code and produces the files `LEQuad_megafly.elf`, `LEQuad_megafly.bin` and `LEQuad_megafly.hex` as output. To flash the program on a megafly board, type:
```bash
make flash
```
___

# STM32 Sample Project
```bash
cd /path-to-library/MAVRIC_Library/sample_projects/LEQuad/proj_stm32
make
```

It compiles the code and produces the files `LEQuad_Mavrimini.bin` and `LEQuad_Mavrimini.hex` as output. To flash the program on a mavrimini board, type:
```bash
make flash
```
___

# Emulation Sample Project

```bash
cd /path-to-library/MAVRIC_Library/sample_projects/LEQuad/proj_linux
make
```

It compiles the code and produces the program LEQuad_Mavrinux.elf as output. To start the program, type:
```bash
./LEQuad_Mavrinux.elf
```