################################################################################
# MAVRIC MAKEFILE
#
# Configure only the first part of this makefile
################################################################################

# ------------------------------------------------------------------------------
# MAVRIC LIBRARY
# ------------------------------------------------------------------------------
# MAVRIC_Library code directory
MAVRIC_LIB=../

# Include folders for Library
LIB_INC += -I$(MAVRIC_LIB)
LIB_INC += -I$(MAVRIC_LIB)/boards
LIB_INC += -I$(MAVRIC_LIB)/communication
LIB_INC += -I$(MAVRIC_LIB)/control
LIB_INC += -I$(MAVRIC_LIB)/drivers
LIB_INC += -I$(MAVRIC_LIB)/hal/common
LIB_INC += -I$(MAVRIC_LIB)/hal/linux
LIB_INC += -I$(MAVRIC_LIB)/hal/dummy
LIB_INC += -I$(MAVRIC_LIB)/hal
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/common
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/mavric
LIB_INC += -I$(MAVRIC_LIB)/libs/fats
LIB_INC += -I$(MAVRIC_LIB)/libs
LIB_INC += -I$(MAVRIC_LIB)/sensing
LIB_INC += -I$(MAVRIC_LIB)/runtime
LIB_INC += -I$(MAVRIC_LIB)/util

# Library source files (*.c and *.cpp)
LIB_SRCS += util/buffer.c
LIB_SRCS += util/coord_conventions.c
LIB_SRCS += util/kalman.c
LIB_SRCS += util/linear_algebra.c
LIB_SRCS += util/print_util.c
LIB_SRCS += util/quick_trig.c

# LIB_SRCS += boards/megafly_rev4.cpp

# LIB_SRCS += drivers/ads1274.c                   
# LIB_SRCS += drivers/adxl345_driver.c
LIB_SRCS += drivers/airspeed_analog.c           
LIB_SRCS += drivers/battery.c                    
LIB_SRCS += drivers/bmp085.c                    
LIB_SRCS += drivers/bmp085_telemetry.c          
# LIB_SRCS += drivers/curvace.c                   
# LIB_SRCS += drivers/gps_ublox.c                 
LIB_SRCS += drivers/gps_ublox_telemetry.c       
LIB_SRCS += drivers/hmc5883l.c             
# LIB_SRCS += drivers/itg3200.c              
# LIB_SRCS += drivers/lsm330dlc.c            
# LIB_SRCS += drivers/radar_driver.c         
# LIB_SRCS += drivers/radar_module_driver.c
LIB_SRCS += drivers/servos.c                              
LIB_SRCS += drivers/servos_telemetry.c
LIB_SRCS += drivers/sonar_i2cxl.c
LIB_SRCS += drivers/sonar_telemetry.c
LIB_SRCS += drivers/spektrum_satellite.c
# LIB_SRCS += drivers/xbee.c

LIB_SRCS += control/adaptive_parameter.c          
LIB_SRCS += control/attitude_controller.c         
LIB_SRCS += control/attitude_controller_p2.c      
LIB_SRCS += control/attitude_error_estimator.c    
LIB_SRCS += control/joystick_parsing.c            
LIB_SRCS += control/joystick_parsing_telemetry.c  
# LIB_SRCS += control/navigation.c                   
LIB_SRCS += control/pid_controller.c               
LIB_SRCS += control/servos_mix_quadcopter_cross.c  
LIB_SRCS += control/servos_mix_quadcopter_diag.c   
LIB_SRCS += control/stabilisation.c
# LIB_SRCS += control/stabilisation_copter.c
# LIB_SRCS += control/stabilisation_telemetry.c
# LIB_SRCS += control/vector_field_waypoint.c
# LIB_SRCS += control/velocity_controller_copter.c

LIB_SRCS += sensing/ahrs.c            
LIB_SRCS += sensing/ahrs_telemetry.c  
LIB_SRCS += sensing/imu.c             
LIB_SRCS += sensing/imu_telemetry.c                  
# LIB_SRCS += sensing/position_estimation.c            
# LIB_SRCS += sensing/position_estimation_telemetry.c
LIB_SRCS += sensing/qfilter.c
# LIB_SRCS += sensing/simulation.c
# LIB_SRCS += sensing/simulation_telemetry.c   

LIB_SRCS += runtime/scheduler.cpp  
LIB_SRCS += runtime/scheduler_telemetry.c

# LIB_SRCS += communication/acoustic.c                      
# LIB_SRCS += communication/acoustic_telemetry.c      
# LIB_SRCS += communication/console.c                 
# LIB_SRCS += communication/data_logging.c            
# LIB_SRCS += communication/data_logging_telemetry.c  
# LIB_SRCS += communication/hud_telemetry.c           
LIB_SRCS += communication/mavlink_communication.c     
LIB_SRCS += communication/mavlink_message_handler.c   
LIB_SRCS += communication/mavlink_stream.cpp            
# LIB_SRCS += communication/mavlink_waypoint_handler.c  
# LIB_SRCS += communication/onboard_parameters.c
LIB_SRCS += communication/remote.c
LIB_SRCS += communication/remote_telemetry.c
LIB_SRCS += communication/state.c
# LIB_SRCS += communication/state_machine.c
LIB_SRCS += communication/state_telemetry.c

LIB_SRCS += libs/fat_fs/diskio.c  
LIB_SRCS += libs/fat_fs/ff.c  
LIB_SRCS += libs/fat_fs/option/ccsbcs.c
LIB_SRCS += libs/fat_fs/option/syscall.c
LIB_SRCS += libs/fat_fs/option/unicode.c

LIB_SRCS += hal/dummy/serial_dummy.cpp
LIB_SRCS += hal/dummy/i2c_dummy.cpp
LIB_SRCS += hal/dummy/gpio_dummy.cpp
# LIB_SRCS += hal/avr32/gpio_avr32.cpp
LIB_SRCS += hal/linux/file_linux.cpp
LIB_SRCS += hal/linux/serial_udp.cpp
LIB_SRCS += hal/linux/time_keeper.c
LIB_SRCS += hal/linux/udp_client_server.cpp

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
# CC = gcc
OBJCOPY = objcopy

CFLAGS  = -g -O2 -Wall -std=c99

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}

# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
# CXX = g++
OBJCOPY = objcopy

CXXFLAGS  = -g -O2 -Wall -std=c++11

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}

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
# COMMANDS FOR FANCY OUTPUT
# ------------------------------------------------------------------------------
NO_COLOR=\033[0m
OK_COLOR=\033[32;01m
ERROR_COLOR=\033[31;01m
WARN_COLOR=\033[33;01m
 
OK_STRING=$(OK_COLOR)[OK]$(NO_COLOR)
ERROR_STRING=$(ERROR_COLOR)[ERRORS]$(NO_COLOR)
WARN_STRING=$(WARN_COLOR)[WARNINGS]$(NO_COLOR)
 
AWK_CMD = awk '{ printf "%-30s %-10s\n",$$1, $$2; }'
PRINT_ERROR = printf "$@ $(ERROR_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n" && false
PRINT_WARNING = printf "$@ $(WARN_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n"
PRINT_OK = printf "$@ $(OK_STRING)\n" | $(AWK_CMD)
BUILD_CMD = LOG=$$($(CMD) 2>&1) ; if [ $$? -eq 1 ]; then $(PRINT_ERROR); elif [ "$$LOG" != "" ] ; then $(PRINT_WARNING); else $(PRINT_OK); fi;


# ------------------------------------------------------------------------------
# MAKEFILE RULES
# ------------------------------------------------------------------------------

# Main rule
lib: ${OBJS}
	
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

.PHONY: clean
clean:
	@rm -f $(OBJS)
	@$(PRINT_OK)