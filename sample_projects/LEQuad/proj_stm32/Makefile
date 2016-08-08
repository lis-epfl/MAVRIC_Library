################################################################################
# MAVRIC MAKEFILE
#
# Configure only the first part of this makefile
################################################################################

# Binaries will be generated with this name
PROJ_NAME=LEQuadST

# ------------------------------------------------------------------------------
# PROJECT FOLDER
# ------------------------------------------------------------------------------
# Project source files (*.c and *.cpp)
# LIB_SRCS += sample_projects/LEQuad/main_stm32.cpp
LIB_SRCS += sample_projects/LEQuad/main_sparky_v2.cpp
LIB_SRCS += sample_projects/LEQuad/lequad.cpp

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
include ${MAVRIC_LIB}rules_dummy.mk
include ${MAVRIC_LIB}rules_stm32.mk

# ------------------------------------------------------------------------------
# LIBOPENCM3
# ------------------------------------------------------------------------------
OPENCM3_DIR 	= $(MAVRIC_LIB)/libs/libopencm3
CM3_INCLUDE_DIR	= $(OPENCM3_DIR)/include
CM3_LIB_DIR		= $(OPENCM3_DIR)/lib
CM3_SCRIPT_DIR	= $(OPENCM3_DIR)/scripts
CM3_LIBNAME 	= opencm3_stm32f4
CM3_DEFS		= -DSTM32F4
# CM3_FP_FLAGS	?= -mfloat-abi=hard -mfpu=fpv4-sp-d16
CM3_FP_FLAGS	= -mfloat-abi=hard -mfpu=fpv4-sp-d16
CM3_ARCH_FLAGS	= -mthumb -mcpu=cortex-m4 $(CM3_FP_FLAGS)
# CM3_LDSCRIPT 	= $(OPENCM3_DIR)/lib/libopencm3_stm32f4.ld
CM3_LDSCRIPT 	= mavrimini.ld
# CM3_LDSCRIPT 	= stm32f4-discovery.ld
# CM3_LDSCRIPT 	= arm-gcc-link.ld

# ------------------------------------------------------------------------------
# COMPILER
# ------------------------------------------------------------------------------
CC		:= arm-none-eabi-gcc
CXX		:= arm-none-eabi-g++
LD		:= arm-none-eabi-g++
AR		:= arm-none-eabi-ar
AS		:= arm-none-eabi-as
OBJCOPY	:= arm-none-eabi-objcopy
OBJDUMP	:= arm-none-eabi-objdump
GDB		:= arm-none-eabi-gdb
SIZER 	:= arm-none-eabi-size

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
CFLAGS	+= -O3 -g -std=gnu99 -MMD -MP
CFLAGS	+= -Wall
# CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
# CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS	+= -fno-common -ffunction-sections -fdata-sections

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}

# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
CXXFLAGS	+= -O3 -g -std=c++11 -MMD -MP
CXXFLAGS	+= -Wall
# CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}


# ------------------------------------------------------------------------------
# C & C++ preprocessor common flags
# ------------------------------------------------------------------------------
CPPFLAGS	+= -MMD -MP
CPPFLAGS	+= -Wall -Wundef
CPPFLAGS	+= -I$(CM3_INCLUDE_DIR) $(CM3_DEFS)


# ------------------------------------------------------------------------------
# Linker flags
# ------------------------------------------------------------------------------
LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(CM3_LIB_DIR)
LDFLAGS		+= -T$(CM3_LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map
LDFLAGS		+= -Wl,--gc-sections
LDFLAGS		+= --specs=nosys.specs -z muldefs

LDLIBS		+= -l$(CM3_LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group


# Include files from MAVRIC library and source folder
# LDFLAGS += -I.
# LDFLAGS += ${LIB_INC}
# LDFLAGS += ${SRCS_INC}


################################################################################
# Normally you shouldn't need to change anything below this line!
################################################################################

# ------------------------------------------------------------------------------
# OBJECT FILES
# ------------------------------------------------------------------------------
BUILD_DIR = build

# Get the names of the .o files from .c and .cpp files
OBJS += $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(LIB_SRCS))))

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
all: $(PROJ_NAME)

$(PROJ_NAME): $(PROJ_NAME).bin $(PROJ_NAME).elf
	@$(BUILD_CMD)

# libopencm3
$(CM3_LIB_DIR)/lib$(CM3_LIBNAME).a:
	@echo Building libopencm3...
	@cd $(OPENCM3_DIR) && make --silent
	@$(BUILD_CMD)

# Linking
${PROJ_NAME}.elf ${PROJ_NAME}.bin: $(CM3_LIB_DIR)/lib$(CM3_LIBNAME).a ${OBJS}
	@echo Linking...
	@$(LD) $(LDFLAGS) $(CM3_ARCH_FLAGS) $(OBJS) $(LDLIBS) ${OBJS} -o ${PROJ_NAME}.elf
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@$(SIZER) $(PROJ_NAME).elf
	@$(BUILD_CMD)

# C files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.c
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) $(CPPFLAGS) $(CM3_ARCH_FLAGS) -c $< -o $@ $(CFLAGS)
	@$(BUILD_CMD)

# CPP files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.cpp
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(CM3_ARCH_FLAGS) -c $< -o $@ $(CXXFLAGS)
	@$(BUILD_CMD)

flash: $(PROJ_NAME).bin
	@st-flash write $(PROJ_NAME).bin 0x8000000

dfu: $(PROJ_NAME).bin
	@dfu-util --device 0x483:0xdf11 -a 0 -s 0x8000000:leave -D $(PROJ_NAME).bin

.PHONY: clean rebuild
clean:
	@rm -f $(OBJS) $(DEPS)
	@rm -rf build/
	@$(PRINT_OK)

rebuild: clean all

.DEFAULT_GOAL := all
