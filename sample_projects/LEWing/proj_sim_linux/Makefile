################################################################################
# MAVRIC MAKEFILE
#
# Configure only the first part of this makefile
################################################################################

# Binaries will be generated with this name
PROJ_NAME=LEWingSimLI

# ------------------------------------------------------------------------------
# PROJECT FOLDER
# ------------------------------------------------------------------------------
# Project source files (*.c and *.cpp)
LIB_SRCS += sample_projects/LEWing/main_sim_linux.cpp
LIB_SRCS += sample_projects/LEWing/central_data.cpp
LIB_SRCS += sample_projects/LEWing/tasks.cpp
LIB_SRCS += sample_projects/LEWing/mavlink_telemetry.cpp

# ------------------------------------------------------------------------------
# MAVRIC LIBRARY
# ------------------------------------------------------------------------------
# MAVRIC_Library code directory
MAVRIC_LIB=../../../

# Include folders for Library
LIB_INC += -I$(MAVRIC_LIB)

# add library source files
include ${MAVRIC_LIB}rules_common.mk
#include ${MAVRIC_LIB}rules_libs.mk
include ${MAVRIC_LIB}rules_dummy.mk
include ${MAVRIC_LIB}rules_linux.mk

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
# CC = gcc
OBJCOPY = objcopy

CFLAGS += -g -O2 -Wall -std=gnu99 -MMD -MP

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}

# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
# CXX = g++
OBJCOPY = objcopy

CXXFLAGS += -g -O2 -Wall -std=c++11 -MMD -MP

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}

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

# Main rule
lib: ${OBJS}

proj: $(PROJ_NAME).elf

# Linking
${PROJ_NAME}.elf: ${OBJS}
	@echo Linking...
	@$(CXX) $^ -o $@ $(LDFLAGS)
	@$(BUILD_CMD)

# versions: ${BUILD_DIR}/%.o

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

.PHONY: clean rebuild
clean:
	@rm -f $(OBJS) $(DEPS)
	@rm -rf build/
	@$(PRINT_OK)

rebuild: clean proj

.DEFAULT_GOAL := all
