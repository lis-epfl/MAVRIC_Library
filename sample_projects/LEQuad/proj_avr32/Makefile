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

# Include paths for project folder
SRCS_INC += -I${MAVRIC_SRC}
SRCS_INC += -I${MAVRIC_SRC}/config/

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
LIB_INC += -I$(MAVRIC_LIB)/boards
LIB_INC += -I$(MAVRIC_LIB)/communication
LIB_INC += -I$(MAVRIC_LIB)/control
LIB_INC += -I$(MAVRIC_LIB)/drivers
LIB_INC += -I$(MAVRIC_LIB)/hal/common
LIB_INC += -I$(MAVRIC_LIB)/hal/avr32
LIB_INC += -I$(MAVRIC_LIB)/hal/dummy
LIB_INC += -I$(MAVRIC_LIB)/hal
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/common
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/mavric
LIB_INC += -I$(MAVRIC_LIB)/libs/FatFs/src
LIB_INC += -I$(MAVRIC_LIB)/runtime
LIB_INC += -I$(MAVRIC_LIB)/sensing
LIB_INC += -I$(MAVRIC_LIB)/simulation
LIB_INC += -I$(MAVRIC_LIB)/util
LIB_INC += -I$(MAVRIC_SRC)

# Library source files (*.c and *.cpp)
LIB_SRCS += boards/megafly_rev4.cpp
     
LIB_SRCS += communication/data_logging.cpp            
LIB_SRCS += communication/hud_telemetry.cpp           
LIB_SRCS += communication/mavlink_communication.cpp     
LIB_SRCS += communication/mavlink_message_handler.cpp   
LIB_SRCS += communication/mavlink_stream.cpp            
LIB_SRCS += communication/mavlink_waypoint_handler.cpp  
LIB_SRCS += communication/onboard_parameters.cpp
LIB_SRCS += communication/remote.cpp
LIB_SRCS += communication/remote_telemetry.cpp
LIB_SRCS += communication/state.cpp
LIB_SRCS += communication/state_machine.cpp
LIB_SRCS += communication/state_telemetry.cpp
LIB_SRCS += communication/toggle_logging.cpp
LIB_SRCS += communication/toggle_logging_telemetry.cpp

LIB_SRCS += control/adaptive_parameter.c          
LIB_SRCS += control/attitude_controller.c         
LIB_SRCS += control/attitude_controller_p2.c      
LIB_SRCS += control/attitude_error_estimator.c    
LIB_SRCS += control/joystick.cpp            
LIB_SRCS += control/joystick_telemetry.cpp  
LIB_SRCS += control/manual_control.cpp
LIB_SRCS += control/manual_control_telemetry.cpp
LIB_SRCS += control/navigation.cpp                   
LIB_SRCS += control/pid_controller.c               
LIB_SRCS += control/servos_mix_quadcopter_cross.c  
LIB_SRCS += control/servos_mix_quadcopter_diag.c   
LIB_SRCS += control/stabilisation.c
LIB_SRCS += control/stabilisation_copter.cpp
LIB_SRCS += control/stabilisation_telemetry.cpp
LIB_SRCS += control/vector_field_waypoint.cpp
LIB_SRCS += control/velocity_controller_copter.cpp

LIB_SRCS += drivers/battery.c                    
LIB_SRCS += drivers/barometer.cpp          
LIB_SRCS += drivers/barometer_telemetry.cpp          
LIB_SRCS += drivers/bmp085.cpp                   
LIB_SRCS += drivers/flow.c                  
LIB_SRCS += drivers/gps_ublox.cpp                 
LIB_SRCS += drivers/gps_telemetry.cpp       
LIB_SRCS += drivers/hmc5883l.cpp  
LIB_SRCS += drivers/lsm330dlc.cpp            
LIB_SRCS += drivers/servo.cpp                              
LIB_SRCS += drivers/servos_telemetry.c
LIB_SRCS += drivers/sonar_i2cxl.cpp
LIB_SRCS += drivers/sonar_telemetry.cpp
LIB_SRCS += drivers/spektrum_satellite.cpp

LIB_SRCS += hal/common/dbg.cpp
LIB_SRCS += hal/common/led_gpio.cpp
LIB_SRCS += hal/common/file.cpp
LIB_SRCS += hal/common/serial.cpp
LIB_SRCS += hal/common/dbg.cpp
 
LIB_SRCS += hal/avr32/adc_avr32.cpp          
LIB_SRCS += hal/avr32/adc_int.c          
LIB_SRCS += hal/avr32/analog_monitor.c            
LIB_SRCS += hal/avr32/analog_monitor_telemetry.cpp                  
LIB_SRCS += hal/avr32/dac_dma.c
LIB_SRCS += hal/avr32/i2c_avr32.cpp            
LIB_SRCS += hal/avr32/led.c            
LIB_SRCS += hal/avr32/led_avr32.cpp            
LIB_SRCS += hal/avr32/gpio_avr32.cpp
LIB_SRCS += hal/avr32/piezo_speaker.c        
LIB_SRCS += hal/avr32/pwm_avr32.cpp
LIB_SRCS += hal/avr32/serial_avr32.cpp
LIB_SRCS += hal/avr32/serial_usb_avr32.cpp
LIB_SRCS += hal/avr32/spi_avr32.cpp
LIB_SRCS += hal/avr32/sd_spi.c
LIB_SRCS += hal/avr32/time_keeper.c
LIB_SRCS += hal/avr32/file_flash_avr32.cpp
LIB_SRCS += hal/avr32/file_fat_fs.cpp
LIB_SRCS += hal/avr32/fat_fs_mounting.cpp

LIB_SRCS += hal/dummy/adc_dummy.cpp
LIB_SRCS += hal/dummy/i2c_dummy.cpp
LIB_SRCS += hal/dummy/gpio_dummy.cpp

LIB_SRCS += hal/avr32/FatFs/atadrive.c
LIB_SRCS += hal/avr32/FatFs/sdcard.c
LIB_SRCS += hal/avr32/FatFs/usbdisk.c

LIB_SRCS += libs/FatFs/src/ff.c  
LIB_SRCS += libs/FatFs/src/diskio.c
LIB_SRCS += libs/FatFs/src/option/syscall.c
LIB_SRCS += libs/FatFs/src/option/unicode.c
LIB_SRCS += libs/FatFs/src/option/ccsbcs.c

LIB_SRCS += runtime/scheduler.c  
LIB_SRCS += runtime/scheduler_telemetry.cpp

LIB_SRCS += sensing/ahrs.c            
LIB_SRCS += sensing/ahrs_telemetry.cpp  
LIB_SRCS += sensing/imu.cpp             
LIB_SRCS += sensing/imu_telemetry.cpp                 
LIB_SRCS += sensing/position_estimation.cpp            
LIB_SRCS += sensing/position_estimation_telemetry.cpp
LIB_SRCS += sensing/qfilter.cpp

LIB_SRCS += simulation/simulation.cpp
LIB_SRCS += simulation/dynamic_model_quad_diag.cpp
LIB_SRCS += simulation/accelerometer_sim.cpp
LIB_SRCS += simulation/gyroscope_sim.cpp
LIB_SRCS += simulation/magnetometer_sim.cpp
LIB_SRCS += simulation/barometer_sim.cpp
LIB_SRCS += simulation/sonar_sim.cpp
LIB_SRCS += simulation/gps_sim.cpp

LIB_SRCS += util/coord_conventions.c
LIB_SRCS += util/kalman.c
LIB_SRCS += util/linear_algebra.c
LIB_SRCS += util/print_util.c
LIB_SRCS += util/quick_trig.c
LIB_SRCS += util/string_util.c


# ------------------------------------------------------------------------------
# ATMEL ASF
# ------------------------------------------------------------------------------

LIB_INC += -I${MAVRIC_LIB}/sample_projects/LEQuad/proj_avr32/config
LIB_INC += -I${MAVRIC_LIB}/libs/asf
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/include
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_extract
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_extract/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_get
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_get/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_print
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/programs/data_print/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/resources
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/docsrc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/basic
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/basic/transforms
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/basic/filters
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/basic/operators
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/at32uc/basic/vectors
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/advanced
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/advanced/resampling
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/advanced/adpcm
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/transforms
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/filters
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/operators
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/windowing
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/vectors
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/dsp/dsplib/generic/basic/signal_generation
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/services/delay
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/startup
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/libs
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/libs/dsplib
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/libs/dsplib/include
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/libs/dsplib/at32ucr3fp
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/utils/preprocessor
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/pm
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/adcifa
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/usart
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/twim
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/intc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/ast
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/pevc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/tc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/spi
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/usbc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/pdca
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/cpu
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/cpu/cycle_counter
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/eic
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/dacifb
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/pwm
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/flashc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/gpio
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/twis
LIB_INC += -I${MAVRIC_LIB}/libs/asf/avr32/drivers/scif
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/usb
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/usb/udc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/usb/class
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/usb/class/cdc
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/usb/class/cdc/device
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/clock
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/clock/uc3c
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/serial
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/serial/uc3_usart
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/spi
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/spi/uc3_spi
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/twi
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/twi/uc3_twim
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/sleepmgr
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/services/sleepmgr/uc3
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/utils
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/utils/stdio
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/utils/stdio/stdio_usb
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/utils/stdio/stdio_serial
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/utils/interrupt
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/boards
LIB_INC += -I${MAVRIC_LIB}/libs/asf/common/boards/user_board

LIB_SRCS += libs/asf/avr32/drivers/intc/exception_noNMI.S
LIB_SRCS += libs/asf/avr32/utils/startup/startup_uc3.S
LIB_SRCS += libs/asf/avr32/utils/startup/trampoline_uc3.S
LIB_SRCS += libs/asf/avr32/drivers/adcifa/adcifa.c
LIB_SRCS += libs/asf/avr32/drivers/ast/ast.c
LIB_SRCS += libs/asf/avr32/drivers/dacifb/dacifb.c
LIB_SRCS += libs/asf/avr32/drivers/eic/eic.c
LIB_SRCS += libs/asf/avr32/drivers/flashc/flashc.c
LIB_SRCS += libs/asf/avr32/drivers/gpio/gpio.c
LIB_SRCS += libs/asf/avr32/drivers/intc/intc.c
LIB_SRCS += libs/asf/avr32/drivers/pdca/pdca.c
LIB_SRCS += libs/asf/avr32/drivers/pevc/pevc.c
LIB_SRCS += libs/asf/avr32/drivers/pm/pm_uc3c.c  
LIB_SRCS += libs/asf/avr32/drivers/pm/power_clocks_lib.c
LIB_SRCS += libs/asf/avr32/drivers/pwm/pwm4.c
LIB_SRCS += libs/asf/avr32/drivers/scif/scif_uc3c.c
LIB_SRCS += libs/asf/avr32/drivers/spi/spi.c
LIB_SRCS += libs/asf/avr32/drivers/tc/tc.c
LIB_SRCS += libs/asf/avr32/drivers/twim/twim.c
LIB_SRCS += libs/asf/avr32/drivers/usart/usart.c
LIB_SRCS += libs/asf/avr32/drivers/usbc/usbc_device.c
LIB_SRCS += libs/asf/avr32/services/delay/delay.c
LIB_SRCS += libs/asf/avr32/utils/startup/startup_uc3.S
LIB_SRCS += libs/asf/common/boards/user_board/init.c
LIB_SRCS += libs/asf/common/services/clock/uc3c/osc.c
LIB_SRCS += libs/asf/common/services/clock/uc3c/pll.c
LIB_SRCS += libs/asf/common/services/clock/uc3c/sysclk.c
LIB_SRCS += libs/asf/common/services/sleepmgr/uc3/sleepmgr.c
LIB_SRCS += libs/asf/common/services/spi/uc3_spi/spi_master.c
LIB_SRCS += libs/asf/common/services/usb/class/cdc/device/udi_cdc.c
LIB_SRCS += libs/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c
LIB_SRCS += libs/asf/common/services/usb/udc/udc.c
LIB_SRCS += libs/asf/common/utils/stdio/read.c
LIB_SRCS += libs/asf/common/utils/stdio/write.c
LIB_SRCS += libs/asf/common/utils/stdio/stdio_usb/stdio_usb.c


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
ASFLAGS = -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax 

# Include files from MAVRIC library and source folder
ASFLAGS += -I.
ASFLAGS += ${LIB_INC}
ASFLAGS += ${SRCS_INC}

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
CC = avr32-gcc
CFLAGS  = -O3 -mhard-float -fdata-sections -muse-rodata-section -g2 -pg -p -Wall 
CFLAGS += -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes 
CFLAGS += -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF
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
CXXFLAGS += -g2 -pg -p -Wall -mpart=uc3c1512c -c -Wpointer-arith -mrelax -MD -MP -MF
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

LDFLAGS  = -nostartfiles -Wl,-Map="mavric.map" -Wl,--start-group -lm  
LDGLAGS += -Wl,--end-group -L"src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc"  
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

.PHONY: clean
clean:
	@rm -f $(OBJS) $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin
	@rm -rf build/
	@$(PRINT_OK)