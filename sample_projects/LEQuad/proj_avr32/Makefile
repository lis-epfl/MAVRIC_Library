################################################################################
# MAVRIC MAKEFILE
#
# Configure only the first part of this makefile
################################################################################

# Binaries will be generated with this name
PROJ_NAME=LEQuad_megafly

# ------------------------------------------------------------------------------
# PROJECT FOLDER
# ------------------------------------------------------------------------------
# Project source folder
MAVRIC_SRC=../

# Sources files for project folder (*.c and *.cpp)
SRCS += main_avr32.cpp
SRCS += central_data.cpp
SRCS += mavlink_telemetry.cpp
SRCS += tasks.cpp

# ------------------------------------------------------------------------------
# MAVRIC LIBRARY
# ------------------------------------------------------------------------------
# MAVRIC_Library code directory
MAVRIC_LIB=../../../

# Include folders for Library
LIB_INC += -I$(MAVRIC_LIB)

# add library source files
include ${MAVRIC_LIB}rules_common.mk
include ${MAVRIC_LIB}rules_libs.mk
include ${MAVRIC_LIB}rules_avr32.mk

# add headers for libs/asf and for boards/mega_fly library (from includes)
LIB_INC += $(addprefix -I${MAVRIC_LIB}, ${LIB_ASF_INC})
LIB_INC += $(addprefix -I${MAVRIC_LIB}, ${LIB_MEGAFLY_INC})

# ------------------------------------------------------------------------------
# PREPROCESSOR OPTIONS
# ------------------------------------------------------------------------------
PREPROC_OPT  = -DBOARD=USER_BOARD 
PREPROC_OPT += -DDSP_OPTIMIZATION=DSP_OPTI_SPEED 
PREPROC_OPT += -DDSP_ADPCM
PREPROC_OPT += -DDSP_RESAMPLING 
PREPROC_OPT += -DDSP_FILTERS 
PREPROC_OPT += -DDSP_OPERATORS 
PREPROC_OPT += -DDSP_SIGNAL_GENERATION
PREPROC_OPT += -DDSP_TRANSFORMS 
PREPROC_OPT += -DDSP_VECTORS 
PREPROC_OPT += -DDSP_WINDOWING 
PREPROC_OPT += -DUDD_ENABLE

# ------------------------------------------------------------------------------
# ASSEMBLY COMPILER OPTIONS
# ------------------------------------------------------------------------------
AS = avr32-gcc
ASFLAGS = -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax -MMD -MP

# Include files from MAVRIC library and source folder
ASFLAGS += -I.
ASFLAGS += ${LIB_INC}
ASFLAGS += ${SRCS_INC}

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
CC = avr32-gcc
CFLAGS  = -O3 -mhard-float -fdata-sections -muse-rodata-section -g2 -Wall 
CFLAGS += -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes 
CFLAGS += -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MMD -MP
# CFLAGS += -ffunction-sections
CFLAGS += ${PREPROC_OPT}

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}
CFLAGS += ${SRCS_INC}


# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
CXX = avr32-g++
# CXXFLAGS  = -O3 -std=gnu++0x -mhard-float -fdata-sections -muse-rodata-section 
CXXFLAGS  = -O3 -std=c++0x -mhard-float -fdata-sections -muse-rodata-section 
CXXFLAGS += -g2 -Wall -mpart=uc3c1512c -c -Wpointer-arith -mrelax -MMD -MP
# CXXFLAGS += -D_DEFAULT_SOURCE 
# CXXFLAGS += -ffunction-sections
CXXFLAGS += ${PREPROC_OPT}

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}
CXXFLAGS += ${SRCS_INC}

# ------------------------------------------------------------------------------
# LINKER OPTIONS
# ------------------------------------------------------------------------------
LINKER = avr32-g++
OBJCOPY = avr32-objcopy
SIZER = avr32-size

OBJCOPYFLAGS = -R .eeprom -R .fuse -R .lock -R .signature

LDFLAGS  = -nostartfiles -Wl,-Map="mavric.map" -lm
LDGLAGS += -L"src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc"
LDFLAGS += -Wl,--gc-sections -mpart=uc3c1512c -Wl,--relax -Wl,-e,_trampoline

# Include files from MAVRIC library and source folder
LDFLAGS += -I.
LDFLAGS += ${LIB_INC}
LDFLAGS += ${SRCS_INC}


################################################################################
# Normally you shouldn't need to change anything below this line!
################################################################################

# ------------------------------------------------------------------------------
# OBJECT FILES
# ------------------------------------------------------------------------------
BUILD_DIR = build

# Get the names of the .o files from .c and .cpp files
OBJS += $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(LIB_SRCS))))
OBJS += $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(SRCS))))

# ------------------------------------------------------------------------------
# DEPENDENCY FILES (*.d)
# ------------------------------------------------------------------------------
DEPS += $(addsuffix .d, $(basename $(OBJS)))	# create list of dependency files
-include $(DEPS)								# include existing dependency files


# ------------------------------------------------------------------------------
# COMMANDS FOR FANCY OUTPUT
# ------------------------------------------------------------------------------
NO_COLOR=\033[0m
OK_COLOR=\033[32;01m
ERROR_COLOR=\033[31;01m
WARN_COLOR=\033[33;01m
 
OK_STRING=$(OK_COLOR)[OK]$(NO_COLOR)
ERROR_STRING=$(ERROR_COLOR)[ERRORS]$(NO_COLOR)
WARN_STRING=$(WARN_COLOR)[WARNINGS]$(NO_COLOR)
 
AWK_CMD = awk '{ printf "%-60s %-10s\n",$$1, $$2; }'
PRINT_ERROR = printf "$@ $(ERROR_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n" && false
PRINT_WARNING = printf "$@ $(WARN_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n"
PRINT_OK = printf "$@ $(OK_STRING)\n" | $(AWK_CMD)
BUILD_CMD = LOG=$$($(CMD) 2>&1) ; if [ $$? -eq 1 ]; then $(PRINT_ERROR); elif [ "$$LOG" != "" ] ; then $(PRINT_WARNING); else $(PRINT_OK); fi;


# ------------------------------------------------------------------------------
# MAKEFILE RULES
# ------------------------------------------------------------------------------

# Main rule
all: proj
proj: $(PROJ_NAME).elf
	@$(SIZER) $^

# Linking
${PROJ_NAME}.elf: ${OBJS}
	@echo Linking...
	@$(CXX) $(LDFLAGS) $^ -o $@  
	@$(OBJCOPY) -O ihex ${OBJCOPYFLAGS} $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@$(BUILD_CMD)

# Assembly files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.S
	@mkdir -p $(dir $@)
	@$(AS) $(ASFLAGS) -c $< -o $@
	@$(BUILD_CMD)
	
# C files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.c
	@mkdir -p $(dir $@)
	@$(CC) -c $< -o $@ $(CFLAGS)
	@$(BUILD_CMD)

# CPP files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.cpp
	@mkdir -p $(dir $@)
	@$(CXX) -c $< -o $@ $(CXXFLAGS)
	@$(BUILD_CMD)

# Assembly files in source folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.S
	@mkdir -p $(dir $@)
	@$(AS) $(ASFLAGS) -c $< -o $@
	@$(BUILD_CMD)

# C files in source folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.c
	@mkdir -p $(dir $@)
	@$(CC) -c $< -o $@ $(CFLAGS)
	@$(BUILD_CMD)

# CPP files in SRC folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.cpp
	@mkdir -p $(dir $@)
	@$(CXX) -c $< -o $@ $(CXXFLAGS)
	@$(BUILD_CMD)

# Flash
flash: ${PROJ_NAME}.elf
	@dfu-programmer at32uc3c1512 erase
	@dfu-programmer at32uc3c1512 get 
	@dfu-programmer at32uc3c1512 flash ${PROJ_NAME}.hex --suppress-bootloader-mem 
	@dfu-programmer at32uc3c1512 reset 

.PHONY: clean rebuild
clean:
	@rm -f $(OBJS) $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin $(DEPS)
	@rm -rf build/
	@$(PRINT_OK)

rebuild: clean proj

.DEFAULT_GOAL := all