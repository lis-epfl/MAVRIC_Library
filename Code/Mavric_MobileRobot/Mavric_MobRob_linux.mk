##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Mavric_MobRob_linux
ConfigurationName      :=Debug
WorkspacePath          := "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot"
ProjectPath            := "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot"
IntermediateDirectory  :=./Debug_Linux/obj
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Géraud L'Eplattenier
Date                   :=09/03/14
CodeLitePath           :="/home/gleplatt/.codelite"
LinkerName             :=/usr/local/bin/avr32-g++ 
SharedObjectLinkerName :=/usr/local/bin/avr32-g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/../$(ProjectName).elf
Preprocessors          :=$(PreprocessorSwitch)BOARD=USER_BOARD $(PreprocessorSwitch)DSP_OPTIMIZATION=DSP_OPTI_SPEED $(PreprocessorSwitch)DSP_ADPCM $(PreprocessorSwitch)DSP_RESAMPLING $(PreprocessorSwitch)DSP_FILTERS $(PreprocessorSwitch)DSP_OPERATORS $(PreprocessorSwitch)DSP_SIGNAL_GENERATION $(PreprocessorSwitch)DSP_TRANSFORMS $(PreprocessorSwitch)DSP_VECTORS $(PreprocessorSwitch)DSP_WINDOWING $(PreprocessorSwitch)UDD_ENABLE 
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="Mavric_MobRob_linux.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -nostartfiles -Wl,-Map="Mavric.map" -Wl,--start-group -lm  -Wl,--end-group -L"src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc"  -Wl,--gc-sections -mpart=uc3c1512c -Wl,--relax -Wl,-e,_trampoline
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)src $(IncludeSwitch)src/config $(IncludeSwitch)src/asf $(IncludeSwitch)src/asf/avr32 $(IncludeSwitch)src/asf/avr32/services $(IncludeSwitch)src/asf/avr32/services/dsp $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/include $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_extract $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_extract/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_get $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_get/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_print $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/programs/data_print/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/resources $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/docsrc $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/basic $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/basic/transforms $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/basic/filters $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/basic/operators $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/at32uc/basic/vectors $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/advanced $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/advanced/resampling $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/advanced/adpcm $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/transforms $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/filters $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/operators $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/windowing $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/vectors $(IncludeSwitch)src/asf/avr32/services/dsp/dsplib/generic/basic/signal_generation $(IncludeSwitch)src/asf/avr32/services/delay $(IncludeSwitch)src/asf/avr32/utils $(IncludeSwitch)src/asf/avr32/utils/startup $(IncludeSwitch)src/asf/avr32/utils/libs $(IncludeSwitch)src/asf/avr32/utils/libs/dsplib $(IncludeSwitch)src/asf/avr32/utils/libs/dsplib/include $(IncludeSwitch)src/asf/avr32/utils/libs/dsplib/at32ucr3fp $(IncludeSwitch)src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc $(IncludeSwitch)src/asf/avr32/utils/preprocessor $(IncludeSwitch)src/asf/avr32/drivers $(IncludeSwitch)src/asf/avr32/drivers/pm $(IncludeSwitch)src/asf/avr32/drivers/adcifa $(IncludeSwitch)src/asf/avr32/drivers/usart $(IncludeSwitch)src/asf/avr32/drivers/twim $(IncludeSwitch)src/asf/avr32/drivers/intc $(IncludeSwitch)src/asf/avr32/drivers/ast $(IncludeSwitch)src/asf/avr32/drivers/pevc $(IncludeSwitch)src/asf/avr32/drivers/tc $(IncludeSwitch)src/asf/avr32/drivers/spi $(IncludeSwitch)src/asf/avr32/drivers/usbc $(IncludeSwitch)src/asf/avr32/drivers/pdca $(IncludeSwitch)src/asf/avr32/drivers/cpu $(IncludeSwitch)src/asf/avr32/drivers/cpu/cycle_counter $(IncludeSwitch)src/asf/avr32/drivers/eic $(IncludeSwitch)src/asf/avr32/drivers/dacifb $(IncludeSwitch)src/asf/avr32/drivers/pwm $(IncludeSwitch)src/asf/avr32/drivers/flashc $(IncludeSwitch)src/asf/avr32/drivers/gpio $(IncludeSwitch)src/asf/avr32/drivers/twis $(IncludeSwitch)src/asf/avr32/drivers/scif $(IncludeSwitch)src/asf/common $(IncludeSwitch)src/asf/common/services $(IncludeSwitch)src/asf/common/services/usb $(IncludeSwitch)src/asf/common/services/usb/udc $(IncludeSwitch)src/asf/common/services/usb/class $(IncludeSwitch)src/asf/common/services/usb/class/cdc $(IncludeSwitch)src/asf/common/services/usb/class/cdc/device $(IncludeSwitch)src/asf/common/services/clock $(IncludeSwitch)src/asf/common/services/clock/uc3c $(IncludeSwitch)src/asf/common/services/serial $(IncludeSwitch)src/asf/common/services/serial/uc3_usart $(IncludeSwitch)src/asf/common/services/spi $(IncludeSwitch)src/asf/common/services/spi/uc3_spi $(IncludeSwitch)src/asf/common/services/twi $(IncludeSwitch)src/asf/common/services/twi/uc3_twim $(IncludeSwitch)src/asf/common/services/sleepmgr $(IncludeSwitch)src/asf/common/services/sleepmgr/uc3 $(IncludeSwitch)src/asf/common/utils $(IncludeSwitch)src/asf/common/utils/stdio $(IncludeSwitch)src/asf/common/utils/stdio/stdio_usb $(IncludeSwitch)src/asf/common/utils/stdio/stdio_serial $(IncludeSwitch)src/asf/common/utils/interrupt $(IncludeSwitch)src/asf/common/boards $(IncludeSwitch)src/asf/common/boards/user_board $(IncludeSwitch)Library/ $(IncludeSwitch)Library/mavlink $(IncludeSwitch)Library/runtime $(IncludeSwitch)Library/util $(IncludeSwitch)Library/tests $(IncludeSwitch)Library/sensing $(IncludeSwitch)Library/hal $(IncludeSwitch)Library/control $(IncludeSwitch)Library/communication 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/local/bin/avr32-ar rcu
CXX      := /usr/local/bin/avr32-g++ 
CC       := /usr/local/bin/avr32-gcc 
CXXFLAGS :=  -O3 -std=gnu++0x -mhard-float -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c  -Wpointer-arith -mrelax -MD -MP -MF $(Preprocessors)
CFLAGS   :=  -O3 -mhard-float -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF $(Preprocessors)
ASFLAGS  :=  -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax 
AS       := /usr/local/bin/avr32-gcc


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/src_tasks.c$(ObjectSuffix) $(IntermediateDirectory)/src_main.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_boardsupport.c$(ObjectSuffix) $(IntermediateDirectory)/src_mavlink_telemetry.c$(ObjectSuffix) $(IntermediateDirectory)/src_central_data.c$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_stream.c$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(ObjectSuffix) $(IntermediateDirectory)/communication_onboard_parameters.c$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_message_handler.c$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_communication.c$(ObjectSuffix) \
	$(IntermediateDirectory)/communication_console.c$(ObjectSuffix) $(IntermediateDirectory)/communication_state.c$(ObjectSuffix) $(IntermediateDirectory)/communication_hud.c$(ObjectSuffix) $(IntermediateDirectory)/communication_remote.c$(ObjectSuffix) $(IntermediateDirectory)/communication_state_machine.c$(ObjectSuffix) $(IntermediateDirectory)/communication_data_logging.c$(ObjectSuffix) $(IntermediateDirectory)/control_pid_control.c$(ObjectSuffix) $(IntermediateDirectory)/control_navigation.c$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation_copter.c$(ObjectSuffix) $(IntermediateDirectory)/control_adaptive_parameter.c$(ObjectSuffix) \
	$(IntermediateDirectory)/control_stabilisation.c$(ObjectSuffix) $(IntermediateDirectory)/control_attitude_error_estimator.c$(ObjectSuffix) $(IntermediateDirectory)/control_attitude_controller_p2.c$(ObjectSuffix) $(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(ObjectSuffix) $(IntermediateDirectory)/control_joystick_parsing.c$(ObjectSuffix) $(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(ObjectSuffix) $(IntermediateDirectory)/hal_adxl345_driver.c$(ObjectSuffix) $(IntermediateDirectory)/hal_bmp085.c$(ObjectSuffix) $(IntermediateDirectory)/hal_time_keeper.c$(ObjectSuffix) $(IntermediateDirectory)/hal_uart_int.c$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_radar_driver.c$(ObjectSuffix) $(IntermediateDirectory)/hal_radar_module_driver.c$(ObjectSuffix) $(IntermediateDirectory)/hal_led.c$(ObjectSuffix) $(IntermediateDirectory)/hal_spi_buffered.c$(ObjectSuffix) $(IntermediateDirectory)/hal_adc_int.c$(ObjectSuffix) $(IntermediateDirectory)/hal_dac_dma.c$(ObjectSuffix) $(IntermediateDirectory)/hal_ads1274.c$(ObjectSuffix) $(IntermediateDirectory)/hal_analog_monitor.c$(ObjectSuffix) $(IntermediateDirectory)/hal_i2c_driver_int.c$(ObjectSuffix) $(IntermediateDirectory)/hal_piezo_speaker.c$(ObjectSuffix) \
	

Objects1=$(IntermediateDirectory)/hal_i2cxl_sonar.c$(ObjectSuffix) $(IntermediateDirectory)/hal_airspeed_analog.c$(ObjectSuffix) $(IntermediateDirectory)/hal_gps_ublox.c$(ObjectSuffix) $(IntermediateDirectory)/hal_xbee.c$(ObjectSuffix) $(IntermediateDirectory)/hal_hmc5883l.c$(ObjectSuffix) $(IntermediateDirectory)/hal_itg3200.c$(ObjectSuffix) $(IntermediateDirectory)/hal_lsm330dlc.c$(ObjectSuffix) $(IntermediateDirectory)/hal_spektrum_satellite.c$(ObjectSuffix) $(IntermediateDirectory)/hal_servos.c$(ObjectSuffix) $(IntermediateDirectory)/hal_pwm_servos.c$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_sd_spi.c$(ObjectSuffix) $(IntermediateDirectory)/hal_usb_int.c$(ObjectSuffix) $(IntermediateDirectory)/runtime_scheduler.c$(ObjectSuffix) $(IntermediateDirectory)/sensing_imu.c$(ObjectSuffix) $(IntermediateDirectory)/sensing_position_estimation.c$(ObjectSuffix) $(IntermediateDirectory)/sensing_qfilter.c$(ObjectSuffix) $(IntermediateDirectory)/sensing_simulation.c$(ObjectSuffix) $(IntermediateDirectory)/sensing_ahrs.c$(ObjectSuffix) $(IntermediateDirectory)/util_linear_algebra.c$(ObjectSuffix) $(IntermediateDirectory)/util_coord_conventions.c$(ObjectSuffix) \
	$(IntermediateDirectory)/util_sinus.c$(ObjectSuffix) $(IntermediateDirectory)/util_print_util.c$(ObjectSuffix) $(IntermediateDirectory)/util_buffer.c$(ObjectSuffix) $(IntermediateDirectory)/util_kalman.c$(ObjectSuffix) $(IntermediateDirectory)/util_quick_trig.c$(ObjectSuffix) $(IntermediateDirectory)/util_generator.c$(ObjectSuffix) $(IntermediateDirectory)/util_matrixlib_float.c$(ObjectSuffix) $(IntermediateDirectory)/fat_fs_diskio.c$(ObjectSuffix) $(IntermediateDirectory)/fat_fs_ff.c$(ObjectSuffix) 

Objects2=$(IntermediateDirectory)/option_cc932.c$(ObjectSuffix) \
	$(IntermediateDirectory)/option_syscall.c$(ObjectSuffix) $(IntermediateDirectory)/option_unicode.c$(ObjectSuffix) $(IntermediateDirectory)/user_board_init.c$(ObjectSuffix) $(IntermediateDirectory)/stdio_read.c$(ObjectSuffix) $(IntermediateDirectory)/stdio_write.c$(ObjectSuffix) $(IntermediateDirectory)/pm_power_clocks_lib.c$(ObjectSuffix) $(IntermediateDirectory)/pm_pm_uc3c.c$(ObjectSuffix) $(IntermediateDirectory)/adcifa_adcifa.c$(ObjectSuffix) 

Objects3=$(IntermediateDirectory)/usart_usart.c$(ObjectSuffix) $(IntermediateDirectory)/twim_twim.c$(ObjectSuffix) \
	$(IntermediateDirectory)/intc_intc.c$(ObjectSuffix) $(IntermediateDirectory)/intc_exception.S$(ObjectSuffix) $(IntermediateDirectory)/ast_ast.c$(ObjectSuffix) $(IntermediateDirectory)/pevc_pevc.c$(ObjectSuffix) $(IntermediateDirectory)/tc_tc.c$(ObjectSuffix) $(IntermediateDirectory)/spi_spi.c$(ObjectSuffix) $(IntermediateDirectory)/pdca_pdca.c$(ObjectSuffix) $(IntermediateDirectory)/eic_eic.c$(ObjectSuffix) $(IntermediateDirectory)/dacifb_dacifb.c$(ObjectSuffix) $(IntermediateDirectory)/pwm_pwm4.c$(ObjectSuffix) \
	$(IntermediateDirectory)/flashc_flashc.c$(ObjectSuffix) $(IntermediateDirectory)/gpio_gpio.c$(ObjectSuffix) $(IntermediateDirectory)/scif_scif_uc3c.c$(ObjectSuffix) $(IntermediateDirectory)/usbc_usbc_device.c$(ObjectSuffix) $(IntermediateDirectory)/startup_startup_uc3.S$(ObjectSuffix) $(IntermediateDirectory)/startup_trampoline_uc3.S$(ObjectSuffix) $(IntermediateDirectory)/delay_delay.c$(ObjectSuffix) $(IntermediateDirectory)/stdio_usb_stdio_usb.c$(ObjectSuffix) $(IntermediateDirectory)/uc3_sleepmgr.c$(ObjectSuffix) $(IntermediateDirectory)/uc3_spi_spi_master.c$(ObjectSuffix) \
	$(IntermediateDirectory)/uc3c_pll.c$(ObjectSuffix) $(IntermediateDirectory)/uc3c_osc.c$(ObjectSuffix) $(IntermediateDirectory)/uc3c_sysclk.c$(ObjectSuffix) $(IntermediateDirectory)/udc_udc.c$(ObjectSuffix) $(IntermediateDirectory)/device_udi_cdc.c$(ObjectSuffix) $(IntermediateDirectory)/device_udi_cdc_desc.c$(ObjectSuffix) 



Objects=$(Objects0) $(Objects1) $(Objects2) $(Objects3) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	@echo $(Objects1) >> $(ObjectsFileList)
	@echo $(Objects2) >> $(ObjectsFileList)
	@echo $(Objects3) >> $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

PostBuild:
	@echo Executing Post Build commands ...
	avr32-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature  $(IntermediateDirectory)/../$(ProjectName).elf $(IntermediateDirectory)/../$(ProjectName).hex
	avr32-size $(IntermediateDirectory)/../$(ProjectName).elf
	
	@echo Done

$(IntermediateDirectory)/.d:
	@test -d ./Debug_Linux/obj || $(MakeDirCommand) ./Debug_Linux/obj

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/src_tasks.c$(ObjectSuffix): src/tasks.c $(IntermediateDirectory)/src_tasks.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/tasks.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tasks.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tasks.c$(DependSuffix): src/tasks.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tasks.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tasks.c$(DependSuffix) -MM "src/tasks.c"

$(IntermediateDirectory)/src_tasks.c$(PreprocessSuffix): src/tasks.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tasks.c$(PreprocessSuffix) "src/tasks.c"

$(IntermediateDirectory)/src_main.cpp$(ObjectSuffix): src/main.cpp $(IntermediateDirectory)/src_main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_main.cpp$(DependSuffix): src/main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_main.cpp$(DependSuffix) -MM "src/main.cpp"

$(IntermediateDirectory)/src_main.cpp$(PreprocessSuffix): src/main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_main.cpp$(PreprocessSuffix) "src/main.cpp"

$(IntermediateDirectory)/src_boardsupport.c$(ObjectSuffix): src/boardsupport.c $(IntermediateDirectory)/src_boardsupport.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/boardsupport.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_boardsupport.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_boardsupport.c$(DependSuffix): src/boardsupport.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_boardsupport.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_boardsupport.c$(DependSuffix) -MM "src/boardsupport.c"

$(IntermediateDirectory)/src_boardsupport.c$(PreprocessSuffix): src/boardsupport.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_boardsupport.c$(PreprocessSuffix) "src/boardsupport.c"

$(IntermediateDirectory)/src_mavlink_telemetry.c$(ObjectSuffix): src/mavlink_telemetry.c $(IntermediateDirectory)/src_mavlink_telemetry.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/mavlink_telemetry.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_mavlink_telemetry.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_mavlink_telemetry.c$(DependSuffix): src/mavlink_telemetry.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_mavlink_telemetry.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_mavlink_telemetry.c$(DependSuffix) -MM "src/mavlink_telemetry.c"

$(IntermediateDirectory)/src_mavlink_telemetry.c$(PreprocessSuffix): src/mavlink_telemetry.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_mavlink_telemetry.c$(PreprocessSuffix) "src/mavlink_telemetry.c"

$(IntermediateDirectory)/src_central_data.c$(ObjectSuffix): src/central_data.c $(IntermediateDirectory)/src_central_data.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/central_data.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_central_data.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_central_data.c$(DependSuffix): src/central_data.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_central_data.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_central_data.c$(DependSuffix) -MM "src/central_data.c"

$(IntermediateDirectory)/src_central_data.c$(PreprocessSuffix): src/central_data.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_central_data.c$(PreprocessSuffix) "src/central_data.c"

$(IntermediateDirectory)/communication_mavlink_stream.c$(ObjectSuffix): Library/communication/mavlink_stream.c $(IntermediateDirectory)/communication_mavlink_stream.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/mavlink_stream.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_stream.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_stream.c$(DependSuffix): Library/communication/mavlink_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_stream.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_stream.c$(DependSuffix) -MM "Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_mavlink_stream.c$(PreprocessSuffix): Library/communication/mavlink_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_stream.c$(PreprocessSuffix) "Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(ObjectSuffix): Library/communication/mavlink_waypoint_handler.c $(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/mavlink_waypoint_handler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(DependSuffix): Library/communication/mavlink_waypoint_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(DependSuffix) -MM "Library/communication/mavlink_waypoint_handler.c"

$(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(PreprocessSuffix): Library/communication/mavlink_waypoint_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_waypoint_handler.c$(PreprocessSuffix) "Library/communication/mavlink_waypoint_handler.c"

$(IntermediateDirectory)/communication_onboard_parameters.c$(ObjectSuffix): Library/communication/onboard_parameters.c $(IntermediateDirectory)/communication_onboard_parameters.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/onboard_parameters.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_onboard_parameters.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_onboard_parameters.c$(DependSuffix): Library/communication/onboard_parameters.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_onboard_parameters.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_onboard_parameters.c$(DependSuffix) -MM "Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_onboard_parameters.c$(PreprocessSuffix): Library/communication/onboard_parameters.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_onboard_parameters.c$(PreprocessSuffix) "Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_mavlink_message_handler.c$(ObjectSuffix): Library/communication/mavlink_message_handler.c $(IntermediateDirectory)/communication_mavlink_message_handler.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/mavlink_message_handler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_message_handler.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_message_handler.c$(DependSuffix): Library/communication/mavlink_message_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_message_handler.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_message_handler.c$(DependSuffix) -MM "Library/communication/mavlink_message_handler.c"

$(IntermediateDirectory)/communication_mavlink_message_handler.c$(PreprocessSuffix): Library/communication/mavlink_message_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_message_handler.c$(PreprocessSuffix) "Library/communication/mavlink_message_handler.c"

$(IntermediateDirectory)/communication_mavlink_communication.c$(ObjectSuffix): Library/communication/mavlink_communication.c $(IntermediateDirectory)/communication_mavlink_communication.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/mavlink_communication.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_communication.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_communication.c$(DependSuffix): Library/communication/mavlink_communication.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_communication.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_communication.c$(DependSuffix) -MM "Library/communication/mavlink_communication.c"

$(IntermediateDirectory)/communication_mavlink_communication.c$(PreprocessSuffix): Library/communication/mavlink_communication.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_communication.c$(PreprocessSuffix) "Library/communication/mavlink_communication.c"

$(IntermediateDirectory)/communication_console.c$(ObjectSuffix): Library/communication/console.c $(IntermediateDirectory)/communication_console.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/console.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_console.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_console.c$(DependSuffix): Library/communication/console.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_console.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_console.c$(DependSuffix) -MM "Library/communication/console.c"

$(IntermediateDirectory)/communication_console.c$(PreprocessSuffix): Library/communication/console.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_console.c$(PreprocessSuffix) "Library/communication/console.c"

$(IntermediateDirectory)/communication_state.c$(ObjectSuffix): Library/communication/state.c $(IntermediateDirectory)/communication_state.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/state.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_state.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_state.c$(DependSuffix): Library/communication/state.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_state.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_state.c$(DependSuffix) -MM "Library/communication/state.c"

$(IntermediateDirectory)/communication_state.c$(PreprocessSuffix): Library/communication/state.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_state.c$(PreprocessSuffix) "Library/communication/state.c"

$(IntermediateDirectory)/communication_hud.c$(ObjectSuffix): Library/communication/hud.c $(IntermediateDirectory)/communication_hud.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/hud.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_hud.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_hud.c$(DependSuffix): Library/communication/hud.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_hud.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_hud.c$(DependSuffix) -MM "Library/communication/hud.c"

$(IntermediateDirectory)/communication_hud.c$(PreprocessSuffix): Library/communication/hud.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_hud.c$(PreprocessSuffix) "Library/communication/hud.c"

$(IntermediateDirectory)/communication_remote.c$(ObjectSuffix): Library/communication/remote.c $(IntermediateDirectory)/communication_remote.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/remote.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_remote.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_remote.c$(DependSuffix): Library/communication/remote.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_remote.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_remote.c$(DependSuffix) -MM "Library/communication/remote.c"

$(IntermediateDirectory)/communication_remote.c$(PreprocessSuffix): Library/communication/remote.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_remote.c$(PreprocessSuffix) "Library/communication/remote.c"

$(IntermediateDirectory)/communication_state_machine.c$(ObjectSuffix): Library/communication/state_machine.c $(IntermediateDirectory)/communication_state_machine.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/state_machine.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_state_machine.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_state_machine.c$(DependSuffix): Library/communication/state_machine.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_state_machine.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_state_machine.c$(DependSuffix) -MM "Library/communication/state_machine.c"

$(IntermediateDirectory)/communication_state_machine.c$(PreprocessSuffix): Library/communication/state_machine.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_state_machine.c$(PreprocessSuffix) "Library/communication/state_machine.c"

$(IntermediateDirectory)/communication_data_logging.c$(ObjectSuffix): Library/communication/data_logging.c $(IntermediateDirectory)/communication_data_logging.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/communication/data_logging.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_data_logging.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_data_logging.c$(DependSuffix): Library/communication/data_logging.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_data_logging.c$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_data_logging.c$(DependSuffix) -MM "Library/communication/data_logging.c"

$(IntermediateDirectory)/communication_data_logging.c$(PreprocessSuffix): Library/communication/data_logging.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_data_logging.c$(PreprocessSuffix) "Library/communication/data_logging.c"

$(IntermediateDirectory)/control_pid_control.c$(ObjectSuffix): Library/control/pid_control.c $(IntermediateDirectory)/control_pid_control.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/pid_control.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_pid_control.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_pid_control.c$(DependSuffix): Library/control/pid_control.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_pid_control.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_pid_control.c$(DependSuffix) -MM "Library/control/pid_control.c"

$(IntermediateDirectory)/control_pid_control.c$(PreprocessSuffix): Library/control/pid_control.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_pid_control.c$(PreprocessSuffix) "Library/control/pid_control.c"

$(IntermediateDirectory)/control_navigation.c$(ObjectSuffix): Library/control/navigation.c $(IntermediateDirectory)/control_navigation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/navigation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_navigation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_navigation.c$(DependSuffix): Library/control/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_navigation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_navigation.c$(DependSuffix) -MM "Library/control/navigation.c"

$(IntermediateDirectory)/control_navigation.c$(PreprocessSuffix): Library/control/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_navigation.c$(PreprocessSuffix) "Library/control/navigation.c"

$(IntermediateDirectory)/control_stabilisation_copter.c$(ObjectSuffix): Library/control/stabilisation_copter.c $(IntermediateDirectory)/control_stabilisation_copter.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/stabilisation_copter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_copter.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_copter.c$(DependSuffix): Library/control/stabilisation_copter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_stabilisation_copter.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation_copter.c$(DependSuffix) -MM "Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_stabilisation_copter.c$(PreprocessSuffix): Library/control/stabilisation_copter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_copter.c$(PreprocessSuffix) "Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_adaptive_parameter.c$(ObjectSuffix): Library/control/adaptive_parameter.c $(IntermediateDirectory)/control_adaptive_parameter.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/adaptive_parameter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_adaptive_parameter.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_adaptive_parameter.c$(DependSuffix): Library/control/adaptive_parameter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_adaptive_parameter.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_adaptive_parameter.c$(DependSuffix) -MM "Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_adaptive_parameter.c$(PreprocessSuffix): Library/control/adaptive_parameter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_adaptive_parameter.c$(PreprocessSuffix) "Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_stabilisation.c$(ObjectSuffix): Library/control/stabilisation.c $(IntermediateDirectory)/control_stabilisation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/stabilisation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation.c$(DependSuffix): Library/control/stabilisation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_stabilisation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation.c$(DependSuffix) -MM "Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation.c$(PreprocessSuffix): Library/control/stabilisation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation.c$(PreprocessSuffix) "Library/control/stabilisation.c"

$(IntermediateDirectory)/control_attitude_error_estimator.c$(ObjectSuffix): Library/control/attitude_error_estimator.c $(IntermediateDirectory)/control_attitude_error_estimator.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/attitude_error_estimator.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_attitude_error_estimator.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_attitude_error_estimator.c$(DependSuffix): Library/control/attitude_error_estimator.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_attitude_error_estimator.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_attitude_error_estimator.c$(DependSuffix) -MM "Library/control/attitude_error_estimator.c"

$(IntermediateDirectory)/control_attitude_error_estimator.c$(PreprocessSuffix): Library/control/attitude_error_estimator.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_attitude_error_estimator.c$(PreprocessSuffix) "Library/control/attitude_error_estimator.c"

$(IntermediateDirectory)/control_attitude_controller_p2.c$(ObjectSuffix): Library/control/attitude_controller_p2.c $(IntermediateDirectory)/control_attitude_controller_p2.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/attitude_controller_p2.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_attitude_controller_p2.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_attitude_controller_p2.c$(DependSuffix): Library/control/attitude_controller_p2.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_attitude_controller_p2.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_attitude_controller_p2.c$(DependSuffix) -MM "Library/control/attitude_controller_p2.c"

$(IntermediateDirectory)/control_attitude_controller_p2.c$(PreprocessSuffix): Library/control/attitude_controller_p2.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_attitude_controller_p2.c$(PreprocessSuffix) "Library/control/attitude_controller_p2.c"

$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(ObjectSuffix): Library/control/servos_mix_quadcopter_cross.c $(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/servos_mix_quadcopter_cross.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(DependSuffix): Library/control/servos_mix_quadcopter_cross.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(DependSuffix) -MM "Library/control/servos_mix_quadcopter_cross.c"

$(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(PreprocessSuffix): Library/control/servos_mix_quadcopter_cross.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_servos_mix_quadcopter_cross.c$(PreprocessSuffix) "Library/control/servos_mix_quadcopter_cross.c"

$(IntermediateDirectory)/control_joystick_parsing.c$(ObjectSuffix): Library/control/joystick_parsing.c $(IntermediateDirectory)/control_joystick_parsing.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/joystick_parsing.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_joystick_parsing.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_joystick_parsing.c$(DependSuffix): Library/control/joystick_parsing.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_joystick_parsing.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_joystick_parsing.c$(DependSuffix) -MM "Library/control/joystick_parsing.c"

$(IntermediateDirectory)/control_joystick_parsing.c$(PreprocessSuffix): Library/control/joystick_parsing.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_joystick_parsing.c$(PreprocessSuffix) "Library/control/joystick_parsing.c"

$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(ObjectSuffix): Library/control/servos_mix_quadcopter_diag.c $(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/control/servos_mix_quadcopter_diag.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(DependSuffix): Library/control/servos_mix_quadcopter_diag.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(ObjectSuffix) -MF$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(DependSuffix) -MM "Library/control/servos_mix_quadcopter_diag.c"

$(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(PreprocessSuffix): Library/control/servos_mix_quadcopter_diag.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_servos_mix_quadcopter_diag.c$(PreprocessSuffix) "Library/control/servos_mix_quadcopter_diag.c"

$(IntermediateDirectory)/hal_adxl345_driver.c$(ObjectSuffix): Library/hal/adxl345_driver.c $(IntermediateDirectory)/hal_adxl345_driver.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/adxl345_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_adxl345_driver.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_adxl345_driver.c$(DependSuffix): Library/hal/adxl345_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_adxl345_driver.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_adxl345_driver.c$(DependSuffix) -MM "Library/hal/adxl345_driver.c"

$(IntermediateDirectory)/hal_adxl345_driver.c$(PreprocessSuffix): Library/hal/adxl345_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_adxl345_driver.c$(PreprocessSuffix) "Library/hal/adxl345_driver.c"

$(IntermediateDirectory)/hal_bmp085.c$(ObjectSuffix): Library/hal/bmp085.c $(IntermediateDirectory)/hal_bmp085.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/bmp085.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_bmp085.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_bmp085.c$(DependSuffix): Library/hal/bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_bmp085.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_bmp085.c$(DependSuffix) -MM "Library/hal/bmp085.c"

$(IntermediateDirectory)/hal_bmp085.c$(PreprocessSuffix): Library/hal/bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_bmp085.c$(PreprocessSuffix) "Library/hal/bmp085.c"

$(IntermediateDirectory)/hal_time_keeper.c$(ObjectSuffix): Library/hal/time_keeper.c $(IntermediateDirectory)/hal_time_keeper.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/time_keeper.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_time_keeper.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_time_keeper.c$(DependSuffix): Library/hal/time_keeper.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_time_keeper.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_time_keeper.c$(DependSuffix) -MM "Library/hal/time_keeper.c"

$(IntermediateDirectory)/hal_time_keeper.c$(PreprocessSuffix): Library/hal/time_keeper.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_time_keeper.c$(PreprocessSuffix) "Library/hal/time_keeper.c"

$(IntermediateDirectory)/hal_uart_int.c$(ObjectSuffix): Library/hal/uart_int.c $(IntermediateDirectory)/hal_uart_int.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/uart_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_uart_int.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_uart_int.c$(DependSuffix): Library/hal/uart_int.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_uart_int.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_uart_int.c$(DependSuffix) -MM "Library/hal/uart_int.c"

$(IntermediateDirectory)/hal_uart_int.c$(PreprocessSuffix): Library/hal/uart_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_uart_int.c$(PreprocessSuffix) "Library/hal/uart_int.c"

$(IntermediateDirectory)/hal_radar_driver.c$(ObjectSuffix): Library/hal/radar_driver.c $(IntermediateDirectory)/hal_radar_driver.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/radar_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_radar_driver.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_radar_driver.c$(DependSuffix): Library/hal/radar_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_radar_driver.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_radar_driver.c$(DependSuffix) -MM "Library/hal/radar_driver.c"

$(IntermediateDirectory)/hal_radar_driver.c$(PreprocessSuffix): Library/hal/radar_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_radar_driver.c$(PreprocessSuffix) "Library/hal/radar_driver.c"

$(IntermediateDirectory)/hal_radar_module_driver.c$(ObjectSuffix): Library/hal/radar_module_driver.c $(IntermediateDirectory)/hal_radar_module_driver.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/radar_module_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_radar_module_driver.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_radar_module_driver.c$(DependSuffix): Library/hal/radar_module_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_radar_module_driver.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_radar_module_driver.c$(DependSuffix) -MM "Library/hal/radar_module_driver.c"

$(IntermediateDirectory)/hal_radar_module_driver.c$(PreprocessSuffix): Library/hal/radar_module_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_radar_module_driver.c$(PreprocessSuffix) "Library/hal/radar_module_driver.c"

$(IntermediateDirectory)/hal_led.c$(ObjectSuffix): Library/hal/led.c $(IntermediateDirectory)/hal_led.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/led.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_led.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_led.c$(DependSuffix): Library/hal/led.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_led.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_led.c$(DependSuffix) -MM "Library/hal/led.c"

$(IntermediateDirectory)/hal_led.c$(PreprocessSuffix): Library/hal/led.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_led.c$(PreprocessSuffix) "Library/hal/led.c"

$(IntermediateDirectory)/hal_spi_buffered.c$(ObjectSuffix): Library/hal/spi_buffered.c $(IntermediateDirectory)/hal_spi_buffered.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/spi_buffered.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_spi_buffered.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_spi_buffered.c$(DependSuffix): Library/hal/spi_buffered.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_spi_buffered.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_spi_buffered.c$(DependSuffix) -MM "Library/hal/spi_buffered.c"

$(IntermediateDirectory)/hal_spi_buffered.c$(PreprocessSuffix): Library/hal/spi_buffered.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_spi_buffered.c$(PreprocessSuffix) "Library/hal/spi_buffered.c"

$(IntermediateDirectory)/hal_adc_int.c$(ObjectSuffix): Library/hal/adc_int.c $(IntermediateDirectory)/hal_adc_int.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/adc_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_adc_int.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_adc_int.c$(DependSuffix): Library/hal/adc_int.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_adc_int.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_adc_int.c$(DependSuffix) -MM "Library/hal/adc_int.c"

$(IntermediateDirectory)/hal_adc_int.c$(PreprocessSuffix): Library/hal/adc_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_adc_int.c$(PreprocessSuffix) "Library/hal/adc_int.c"

$(IntermediateDirectory)/hal_dac_dma.c$(ObjectSuffix): Library/hal/dac_dma.c $(IntermediateDirectory)/hal_dac_dma.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/dac_dma.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_dac_dma.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_dac_dma.c$(DependSuffix): Library/hal/dac_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_dac_dma.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_dac_dma.c$(DependSuffix) -MM "Library/hal/dac_dma.c"

$(IntermediateDirectory)/hal_dac_dma.c$(PreprocessSuffix): Library/hal/dac_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_dac_dma.c$(PreprocessSuffix) "Library/hal/dac_dma.c"

$(IntermediateDirectory)/hal_ads1274.c$(ObjectSuffix): Library/hal/ads1274.c $(IntermediateDirectory)/hal_ads1274.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/ads1274.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_ads1274.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_ads1274.c$(DependSuffix): Library/hal/ads1274.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_ads1274.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_ads1274.c$(DependSuffix) -MM "Library/hal/ads1274.c"

$(IntermediateDirectory)/hal_ads1274.c$(PreprocessSuffix): Library/hal/ads1274.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_ads1274.c$(PreprocessSuffix) "Library/hal/ads1274.c"

$(IntermediateDirectory)/hal_analog_monitor.c$(ObjectSuffix): Library/hal/analog_monitor.c $(IntermediateDirectory)/hal_analog_monitor.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/analog_monitor.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_analog_monitor.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_analog_monitor.c$(DependSuffix): Library/hal/analog_monitor.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_analog_monitor.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_analog_monitor.c$(DependSuffix) -MM "Library/hal/analog_monitor.c"

$(IntermediateDirectory)/hal_analog_monitor.c$(PreprocessSuffix): Library/hal/analog_monitor.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_analog_monitor.c$(PreprocessSuffix) "Library/hal/analog_monitor.c"

$(IntermediateDirectory)/hal_i2c_driver_int.c$(ObjectSuffix): Library/hal/i2c_driver_int.c $(IntermediateDirectory)/hal_i2c_driver_int.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/i2c_driver_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_i2c_driver_int.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_i2c_driver_int.c$(DependSuffix): Library/hal/i2c_driver_int.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_i2c_driver_int.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_i2c_driver_int.c$(DependSuffix) -MM "Library/hal/i2c_driver_int.c"

$(IntermediateDirectory)/hal_i2c_driver_int.c$(PreprocessSuffix): Library/hal/i2c_driver_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_i2c_driver_int.c$(PreprocessSuffix) "Library/hal/i2c_driver_int.c"

$(IntermediateDirectory)/hal_piezo_speaker.c$(ObjectSuffix): Library/hal/piezo_speaker.c $(IntermediateDirectory)/hal_piezo_speaker.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/piezo_speaker.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_piezo_speaker.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_piezo_speaker.c$(DependSuffix): Library/hal/piezo_speaker.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_piezo_speaker.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_piezo_speaker.c$(DependSuffix) -MM "Library/hal/piezo_speaker.c"

$(IntermediateDirectory)/hal_piezo_speaker.c$(PreprocessSuffix): Library/hal/piezo_speaker.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_piezo_speaker.c$(PreprocessSuffix) "Library/hal/piezo_speaker.c"

$(IntermediateDirectory)/hal_i2cxl_sonar.c$(ObjectSuffix): Library/hal/i2cxl_sonar.c $(IntermediateDirectory)/hal_i2cxl_sonar.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/i2cxl_sonar.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_i2cxl_sonar.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_i2cxl_sonar.c$(DependSuffix): Library/hal/i2cxl_sonar.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_i2cxl_sonar.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_i2cxl_sonar.c$(DependSuffix) -MM "Library/hal/i2cxl_sonar.c"

$(IntermediateDirectory)/hal_i2cxl_sonar.c$(PreprocessSuffix): Library/hal/i2cxl_sonar.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_i2cxl_sonar.c$(PreprocessSuffix) "Library/hal/i2cxl_sonar.c"

$(IntermediateDirectory)/hal_airspeed_analog.c$(ObjectSuffix): Library/hal/airspeed_analog.c $(IntermediateDirectory)/hal_airspeed_analog.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/airspeed_analog.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_airspeed_analog.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_airspeed_analog.c$(DependSuffix): Library/hal/airspeed_analog.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_airspeed_analog.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_airspeed_analog.c$(DependSuffix) -MM "Library/hal/airspeed_analog.c"

$(IntermediateDirectory)/hal_airspeed_analog.c$(PreprocessSuffix): Library/hal/airspeed_analog.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_airspeed_analog.c$(PreprocessSuffix) "Library/hal/airspeed_analog.c"

$(IntermediateDirectory)/hal_gps_ublox.c$(ObjectSuffix): Library/hal/gps_ublox.c $(IntermediateDirectory)/hal_gps_ublox.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/gps_ublox.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_gps_ublox.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_gps_ublox.c$(DependSuffix): Library/hal/gps_ublox.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_gps_ublox.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_gps_ublox.c$(DependSuffix) -MM "Library/hal/gps_ublox.c"

$(IntermediateDirectory)/hal_gps_ublox.c$(PreprocessSuffix): Library/hal/gps_ublox.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_gps_ublox.c$(PreprocessSuffix) "Library/hal/gps_ublox.c"

$(IntermediateDirectory)/hal_xbee.c$(ObjectSuffix): Library/hal/xbee.c $(IntermediateDirectory)/hal_xbee.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/xbee.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_xbee.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_xbee.c$(DependSuffix): Library/hal/xbee.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_xbee.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_xbee.c$(DependSuffix) -MM "Library/hal/xbee.c"

$(IntermediateDirectory)/hal_xbee.c$(PreprocessSuffix): Library/hal/xbee.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_xbee.c$(PreprocessSuffix) "Library/hal/xbee.c"

$(IntermediateDirectory)/hal_hmc5883l.c$(ObjectSuffix): Library/hal/hmc5883l.c $(IntermediateDirectory)/hal_hmc5883l.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/hmc5883l.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_hmc5883l.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_hmc5883l.c$(DependSuffix): Library/hal/hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_hmc5883l.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_hmc5883l.c$(DependSuffix) -MM "Library/hal/hmc5883l.c"

$(IntermediateDirectory)/hal_hmc5883l.c$(PreprocessSuffix): Library/hal/hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_hmc5883l.c$(PreprocessSuffix) "Library/hal/hmc5883l.c"

$(IntermediateDirectory)/hal_itg3200.c$(ObjectSuffix): Library/hal/itg3200.c $(IntermediateDirectory)/hal_itg3200.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/itg3200.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_itg3200.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_itg3200.c$(DependSuffix): Library/hal/itg3200.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_itg3200.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_itg3200.c$(DependSuffix) -MM "Library/hal/itg3200.c"

$(IntermediateDirectory)/hal_itg3200.c$(PreprocessSuffix): Library/hal/itg3200.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_itg3200.c$(PreprocessSuffix) "Library/hal/itg3200.c"

$(IntermediateDirectory)/hal_lsm330dlc.c$(ObjectSuffix): Library/hal/lsm330dlc.c $(IntermediateDirectory)/hal_lsm330dlc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/lsm330dlc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_lsm330dlc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_lsm330dlc.c$(DependSuffix): Library/hal/lsm330dlc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_lsm330dlc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_lsm330dlc.c$(DependSuffix) -MM "Library/hal/lsm330dlc.c"

$(IntermediateDirectory)/hal_lsm330dlc.c$(PreprocessSuffix): Library/hal/lsm330dlc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_lsm330dlc.c$(PreprocessSuffix) "Library/hal/lsm330dlc.c"

$(IntermediateDirectory)/hal_spektrum_satellite.c$(ObjectSuffix): Library/hal/spektrum_satellite.c $(IntermediateDirectory)/hal_spektrum_satellite.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/spektrum_satellite.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_spektrum_satellite.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_spektrum_satellite.c$(DependSuffix): Library/hal/spektrum_satellite.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_spektrum_satellite.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_spektrum_satellite.c$(DependSuffix) -MM "Library/hal/spektrum_satellite.c"

$(IntermediateDirectory)/hal_spektrum_satellite.c$(PreprocessSuffix): Library/hal/spektrum_satellite.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_spektrum_satellite.c$(PreprocessSuffix) "Library/hal/spektrum_satellite.c"

$(IntermediateDirectory)/hal_servos.c$(ObjectSuffix): Library/hal/servos.c $(IntermediateDirectory)/hal_servos.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/servos.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_servos.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_servos.c$(DependSuffix): Library/hal/servos.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_servos.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_servos.c$(DependSuffix) -MM "Library/hal/servos.c"

$(IntermediateDirectory)/hal_servos.c$(PreprocessSuffix): Library/hal/servos.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_servos.c$(PreprocessSuffix) "Library/hal/servos.c"

$(IntermediateDirectory)/hal_pwm_servos.c$(ObjectSuffix): Library/hal/pwm_servos.c $(IntermediateDirectory)/hal_pwm_servos.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/pwm_servos.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_pwm_servos.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_pwm_servos.c$(DependSuffix): Library/hal/pwm_servos.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_pwm_servos.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_pwm_servos.c$(DependSuffix) -MM "Library/hal/pwm_servos.c"

$(IntermediateDirectory)/hal_pwm_servos.c$(PreprocessSuffix): Library/hal/pwm_servos.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_pwm_servos.c$(PreprocessSuffix) "Library/hal/pwm_servos.c"

$(IntermediateDirectory)/hal_sd_spi.c$(ObjectSuffix): Library/hal/sd_spi.c $(IntermediateDirectory)/hal_sd_spi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/sd_spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_sd_spi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_sd_spi.c$(DependSuffix): Library/hal/sd_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_sd_spi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_sd_spi.c$(DependSuffix) -MM "Library/hal/sd_spi.c"

$(IntermediateDirectory)/hal_sd_spi.c$(PreprocessSuffix): Library/hal/sd_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_sd_spi.c$(PreprocessSuffix) "Library/hal/sd_spi.c"

$(IntermediateDirectory)/hal_usb_int.c$(ObjectSuffix): Library/hal/usb_int.c $(IntermediateDirectory)/hal_usb_int.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/usb_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_usb_int.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_usb_int.c$(DependSuffix): Library/hal/usb_int.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_usb_int.c$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_usb_int.c$(DependSuffix) -MM "Library/hal/usb_int.c"

$(IntermediateDirectory)/hal_usb_int.c$(PreprocessSuffix): Library/hal/usb_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_usb_int.c$(PreprocessSuffix) "Library/hal/usb_int.c"

$(IntermediateDirectory)/runtime_scheduler.c$(ObjectSuffix): Library/runtime/scheduler.c $(IntermediateDirectory)/runtime_scheduler.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/runtime/scheduler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/runtime_scheduler.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/runtime_scheduler.c$(DependSuffix): Library/runtime/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/runtime_scheduler.c$(ObjectSuffix) -MF$(IntermediateDirectory)/runtime_scheduler.c$(DependSuffix) -MM "Library/runtime/scheduler.c"

$(IntermediateDirectory)/runtime_scheduler.c$(PreprocessSuffix): Library/runtime/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/runtime_scheduler.c$(PreprocessSuffix) "Library/runtime/scheduler.c"

$(IntermediateDirectory)/sensing_imu.c$(ObjectSuffix): Library/sensing/imu.c $(IntermediateDirectory)/sensing_imu.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/sensing/imu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_imu.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_imu.c$(DependSuffix): Library/sensing/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_imu.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_imu.c$(DependSuffix) -MM "Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_imu.c$(PreprocessSuffix): Library/sensing/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_imu.c$(PreprocessSuffix) "Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_position_estimation.c$(ObjectSuffix): Library/sensing/position_estimation.c $(IntermediateDirectory)/sensing_position_estimation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/sensing/position_estimation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_position_estimation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_position_estimation.c$(DependSuffix): Library/sensing/position_estimation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_position_estimation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_position_estimation.c$(DependSuffix) -MM "Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_position_estimation.c$(PreprocessSuffix): Library/sensing/position_estimation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_position_estimation.c$(PreprocessSuffix) "Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_qfilter.c$(ObjectSuffix): Library/sensing/qfilter.c $(IntermediateDirectory)/sensing_qfilter.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/sensing/qfilter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_qfilter.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_qfilter.c$(DependSuffix): Library/sensing/qfilter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_qfilter.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_qfilter.c$(DependSuffix) -MM "Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_qfilter.c$(PreprocessSuffix): Library/sensing/qfilter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_qfilter.c$(PreprocessSuffix) "Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_simulation.c$(ObjectSuffix): Library/sensing/simulation.c $(IntermediateDirectory)/sensing_simulation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/sensing/simulation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_simulation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_simulation.c$(DependSuffix): Library/sensing/simulation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_simulation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_simulation.c$(DependSuffix) -MM "Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_simulation.c$(PreprocessSuffix): Library/sensing/simulation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_simulation.c$(PreprocessSuffix) "Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_ahrs.c$(ObjectSuffix): Library/sensing/ahrs.c $(IntermediateDirectory)/sensing_ahrs.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/sensing/ahrs.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_ahrs.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_ahrs.c$(DependSuffix): Library/sensing/ahrs.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_ahrs.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_ahrs.c$(DependSuffix) -MM "Library/sensing/ahrs.c"

$(IntermediateDirectory)/sensing_ahrs.c$(PreprocessSuffix): Library/sensing/ahrs.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_ahrs.c$(PreprocessSuffix) "Library/sensing/ahrs.c"

$(IntermediateDirectory)/util_linear_algebra.c$(ObjectSuffix): Library/util/linear_algebra.c $(IntermediateDirectory)/util_linear_algebra.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/linear_algebra.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_linear_algebra.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_linear_algebra.c$(DependSuffix): Library/util/linear_algebra.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_linear_algebra.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_linear_algebra.c$(DependSuffix) -MM "Library/util/linear_algebra.c"

$(IntermediateDirectory)/util_linear_algebra.c$(PreprocessSuffix): Library/util/linear_algebra.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_linear_algebra.c$(PreprocessSuffix) "Library/util/linear_algebra.c"

$(IntermediateDirectory)/util_coord_conventions.c$(ObjectSuffix): Library/util/coord_conventions.c $(IntermediateDirectory)/util_coord_conventions.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/coord_conventions.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_coord_conventions.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_coord_conventions.c$(DependSuffix): Library/util/coord_conventions.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_coord_conventions.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_coord_conventions.c$(DependSuffix) -MM "Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_coord_conventions.c$(PreprocessSuffix): Library/util/coord_conventions.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_coord_conventions.c$(PreprocessSuffix) "Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_sinus.c$(ObjectSuffix): Library/util/sinus.c $(IntermediateDirectory)/util_sinus.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/sinus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_sinus.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_sinus.c$(DependSuffix): Library/util/sinus.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_sinus.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_sinus.c$(DependSuffix) -MM "Library/util/sinus.c"

$(IntermediateDirectory)/util_sinus.c$(PreprocessSuffix): Library/util/sinus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_sinus.c$(PreprocessSuffix) "Library/util/sinus.c"

$(IntermediateDirectory)/util_print_util.c$(ObjectSuffix): Library/util/print_util.c $(IntermediateDirectory)/util_print_util.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/print_util.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_print_util.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_print_util.c$(DependSuffix): Library/util/print_util.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_print_util.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_print_util.c$(DependSuffix) -MM "Library/util/print_util.c"

$(IntermediateDirectory)/util_print_util.c$(PreprocessSuffix): Library/util/print_util.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_print_util.c$(PreprocessSuffix) "Library/util/print_util.c"

$(IntermediateDirectory)/util_buffer.c$(ObjectSuffix): Library/util/buffer.c $(IntermediateDirectory)/util_buffer.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/buffer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_buffer.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_buffer.c$(DependSuffix): Library/util/buffer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_buffer.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_buffer.c$(DependSuffix) -MM "Library/util/buffer.c"

$(IntermediateDirectory)/util_buffer.c$(PreprocessSuffix): Library/util/buffer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_buffer.c$(PreprocessSuffix) "Library/util/buffer.c"

$(IntermediateDirectory)/util_kalman.c$(ObjectSuffix): Library/util/kalman.c $(IntermediateDirectory)/util_kalman.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/kalman.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_kalman.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_kalman.c$(DependSuffix): Library/util/kalman.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_kalman.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_kalman.c$(DependSuffix) -MM "Library/util/kalman.c"

$(IntermediateDirectory)/util_kalman.c$(PreprocessSuffix): Library/util/kalman.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_kalman.c$(PreprocessSuffix) "Library/util/kalman.c"

$(IntermediateDirectory)/util_quick_trig.c$(ObjectSuffix): Library/util/quick_trig.c $(IntermediateDirectory)/util_quick_trig.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/quick_trig.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_quick_trig.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_quick_trig.c$(DependSuffix): Library/util/quick_trig.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_quick_trig.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_quick_trig.c$(DependSuffix) -MM "Library/util/quick_trig.c"

$(IntermediateDirectory)/util_quick_trig.c$(PreprocessSuffix): Library/util/quick_trig.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_quick_trig.c$(PreprocessSuffix) "Library/util/quick_trig.c"

$(IntermediateDirectory)/util_generator.c$(ObjectSuffix): Library/util/generator.c $(IntermediateDirectory)/util_generator.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/generator.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_generator.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_generator.c$(DependSuffix): Library/util/generator.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_generator.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_generator.c$(DependSuffix) -MM "Library/util/generator.c"

$(IntermediateDirectory)/util_generator.c$(PreprocessSuffix): Library/util/generator.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_generator.c$(PreprocessSuffix) "Library/util/generator.c"

$(IntermediateDirectory)/util_matrixlib_float.c$(ObjectSuffix): Library/util/matrixlib_float.c $(IntermediateDirectory)/util_matrixlib_float.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/util/matrixlib_float.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_matrixlib_float.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_matrixlib_float.c$(DependSuffix): Library/util/matrixlib_float.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_matrixlib_float.c$(ObjectSuffix) -MF$(IntermediateDirectory)/util_matrixlib_float.c$(DependSuffix) -MM "Library/util/matrixlib_float.c"

$(IntermediateDirectory)/util_matrixlib_float.c$(PreprocessSuffix): Library/util/matrixlib_float.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_matrixlib_float.c$(PreprocessSuffix) "Library/util/matrixlib_float.c"

$(IntermediateDirectory)/fat_fs_diskio.c$(ObjectSuffix): Library/hal/fat_fs/diskio.c $(IntermediateDirectory)/fat_fs_diskio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/fat_fs/diskio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/fat_fs_diskio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/fat_fs_diskio.c$(DependSuffix): Library/hal/fat_fs/diskio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/fat_fs_diskio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/fat_fs_diskio.c$(DependSuffix) -MM "Library/hal/fat_fs/diskio.c"

$(IntermediateDirectory)/fat_fs_diskio.c$(PreprocessSuffix): Library/hal/fat_fs/diskio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/fat_fs_diskio.c$(PreprocessSuffix) "Library/hal/fat_fs/diskio.c"

$(IntermediateDirectory)/fat_fs_ff.c$(ObjectSuffix): Library/hal/fat_fs/ff.c $(IntermediateDirectory)/fat_fs_ff.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/fat_fs/ff.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/fat_fs_ff.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/fat_fs_ff.c$(DependSuffix): Library/hal/fat_fs/ff.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/fat_fs_ff.c$(ObjectSuffix) -MF$(IntermediateDirectory)/fat_fs_ff.c$(DependSuffix) -MM "Library/hal/fat_fs/ff.c"

$(IntermediateDirectory)/fat_fs_ff.c$(PreprocessSuffix): Library/hal/fat_fs/ff.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/fat_fs_ff.c$(PreprocessSuffix) "Library/hal/fat_fs/ff.c"

$(IntermediateDirectory)/option_cc932.c$(ObjectSuffix): Library/hal/fat_fs/option/cc932.c $(IntermediateDirectory)/option_cc932.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/fat_fs/option/cc932.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/option_cc932.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/option_cc932.c$(DependSuffix): Library/hal/fat_fs/option/cc932.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/option_cc932.c$(ObjectSuffix) -MF$(IntermediateDirectory)/option_cc932.c$(DependSuffix) -MM "Library/hal/fat_fs/option/cc932.c"

$(IntermediateDirectory)/option_cc932.c$(PreprocessSuffix): Library/hal/fat_fs/option/cc932.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/option_cc932.c$(PreprocessSuffix) "Library/hal/fat_fs/option/cc932.c"

$(IntermediateDirectory)/option_syscall.c$(ObjectSuffix): Library/hal/fat_fs/option/syscall.c $(IntermediateDirectory)/option_syscall.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/fat_fs/option/syscall.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/option_syscall.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/option_syscall.c$(DependSuffix): Library/hal/fat_fs/option/syscall.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/option_syscall.c$(ObjectSuffix) -MF$(IntermediateDirectory)/option_syscall.c$(DependSuffix) -MM "Library/hal/fat_fs/option/syscall.c"

$(IntermediateDirectory)/option_syscall.c$(PreprocessSuffix): Library/hal/fat_fs/option/syscall.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/option_syscall.c$(PreprocessSuffix) "Library/hal/fat_fs/option/syscall.c"

$(IntermediateDirectory)/option_unicode.c$(ObjectSuffix): Library/hal/fat_fs/option/unicode.c $(IntermediateDirectory)/option_unicode.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/Library/hal/fat_fs/option/unicode.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/option_unicode.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/option_unicode.c$(DependSuffix): Library/hal/fat_fs/option/unicode.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/option_unicode.c$(ObjectSuffix) -MF$(IntermediateDirectory)/option_unicode.c$(DependSuffix) -MM "Library/hal/fat_fs/option/unicode.c"

$(IntermediateDirectory)/option_unicode.c$(PreprocessSuffix): Library/hal/fat_fs/option/unicode.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/option_unicode.c$(PreprocessSuffix) "Library/hal/fat_fs/option/unicode.c"

$(IntermediateDirectory)/user_board_init.c$(ObjectSuffix): src/asf/common/boards/user_board/init.c $(IntermediateDirectory)/user_board_init.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/boards/user_board/init.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/user_board_init.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/user_board_init.c$(DependSuffix): src/asf/common/boards/user_board/init.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/user_board_init.c$(ObjectSuffix) -MF$(IntermediateDirectory)/user_board_init.c$(DependSuffix) -MM "src/asf/common/boards/user_board/init.c"

$(IntermediateDirectory)/user_board_init.c$(PreprocessSuffix): src/asf/common/boards/user_board/init.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/user_board_init.c$(PreprocessSuffix) "src/asf/common/boards/user_board/init.c"

$(IntermediateDirectory)/stdio_read.c$(ObjectSuffix): src/asf/common/utils/stdio/read.c $(IntermediateDirectory)/stdio_read.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/utils/stdio/read.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_read.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_read.c$(DependSuffix): src/asf/common/utils/stdio/read.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stdio_read.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stdio_read.c$(DependSuffix) -MM "src/asf/common/utils/stdio/read.c"

$(IntermediateDirectory)/stdio_read.c$(PreprocessSuffix): src/asf/common/utils/stdio/read.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_read.c$(PreprocessSuffix) "src/asf/common/utils/stdio/read.c"

$(IntermediateDirectory)/stdio_write.c$(ObjectSuffix): src/asf/common/utils/stdio/write.c $(IntermediateDirectory)/stdio_write.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/utils/stdio/write.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_write.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_write.c$(DependSuffix): src/asf/common/utils/stdio/write.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stdio_write.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stdio_write.c$(DependSuffix) -MM "src/asf/common/utils/stdio/write.c"

$(IntermediateDirectory)/stdio_write.c$(PreprocessSuffix): src/asf/common/utils/stdio/write.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_write.c$(PreprocessSuffix) "src/asf/common/utils/stdio/write.c"

$(IntermediateDirectory)/pm_power_clocks_lib.c$(ObjectSuffix): src/asf/avr32/drivers/pm/power_clocks_lib.c $(IntermediateDirectory)/pm_power_clocks_lib.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/pm/power_clocks_lib.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pm_power_clocks_lib.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pm_power_clocks_lib.c$(DependSuffix): src/asf/avr32/drivers/pm/power_clocks_lib.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pm_power_clocks_lib.c$(ObjectSuffix) -MF$(IntermediateDirectory)/pm_power_clocks_lib.c$(DependSuffix) -MM "src/asf/avr32/drivers/pm/power_clocks_lib.c"

$(IntermediateDirectory)/pm_power_clocks_lib.c$(PreprocessSuffix): src/asf/avr32/drivers/pm/power_clocks_lib.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pm_power_clocks_lib.c$(PreprocessSuffix) "src/asf/avr32/drivers/pm/power_clocks_lib.c"

$(IntermediateDirectory)/pm_pm_uc3c.c$(ObjectSuffix): src/asf/avr32/drivers/pm/pm_uc3c.c $(IntermediateDirectory)/pm_pm_uc3c.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/pm/pm_uc3c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pm_pm_uc3c.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pm_pm_uc3c.c$(DependSuffix): src/asf/avr32/drivers/pm/pm_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pm_pm_uc3c.c$(ObjectSuffix) -MF$(IntermediateDirectory)/pm_pm_uc3c.c$(DependSuffix) -MM "src/asf/avr32/drivers/pm/pm_uc3c.c"

$(IntermediateDirectory)/pm_pm_uc3c.c$(PreprocessSuffix): src/asf/avr32/drivers/pm/pm_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pm_pm_uc3c.c$(PreprocessSuffix) "src/asf/avr32/drivers/pm/pm_uc3c.c"

$(IntermediateDirectory)/adcifa_adcifa.c$(ObjectSuffix): src/asf/avr32/drivers/adcifa/adcifa.c $(IntermediateDirectory)/adcifa_adcifa.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/adcifa/adcifa.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/adcifa_adcifa.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/adcifa_adcifa.c$(DependSuffix): src/asf/avr32/drivers/adcifa/adcifa.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/adcifa_adcifa.c$(ObjectSuffix) -MF$(IntermediateDirectory)/adcifa_adcifa.c$(DependSuffix) -MM "src/asf/avr32/drivers/adcifa/adcifa.c"

$(IntermediateDirectory)/adcifa_adcifa.c$(PreprocessSuffix): src/asf/avr32/drivers/adcifa/adcifa.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/adcifa_adcifa.c$(PreprocessSuffix) "src/asf/avr32/drivers/adcifa/adcifa.c"

$(IntermediateDirectory)/usart_usart.c$(ObjectSuffix): src/asf/avr32/drivers/usart/usart.c $(IntermediateDirectory)/usart_usart.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/usart/usart.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/usart_usart.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/usart_usart.c$(DependSuffix): src/asf/avr32/drivers/usart/usart.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/usart_usart.c$(ObjectSuffix) -MF$(IntermediateDirectory)/usart_usart.c$(DependSuffix) -MM "src/asf/avr32/drivers/usart/usart.c"

$(IntermediateDirectory)/usart_usart.c$(PreprocessSuffix): src/asf/avr32/drivers/usart/usart.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/usart_usart.c$(PreprocessSuffix) "src/asf/avr32/drivers/usart/usart.c"

$(IntermediateDirectory)/twim_twim.c$(ObjectSuffix): src/asf/avr32/drivers/twim/twim.c $(IntermediateDirectory)/twim_twim.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/twim/twim.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/twim_twim.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/twim_twim.c$(DependSuffix): src/asf/avr32/drivers/twim/twim.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/twim_twim.c$(ObjectSuffix) -MF$(IntermediateDirectory)/twim_twim.c$(DependSuffix) -MM "src/asf/avr32/drivers/twim/twim.c"

$(IntermediateDirectory)/twim_twim.c$(PreprocessSuffix): src/asf/avr32/drivers/twim/twim.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/twim_twim.c$(PreprocessSuffix) "src/asf/avr32/drivers/twim/twim.c"

$(IntermediateDirectory)/intc_intc.c$(ObjectSuffix): src/asf/avr32/drivers/intc/intc.c $(IntermediateDirectory)/intc_intc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/intc/intc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/intc_intc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/intc_intc.c$(DependSuffix): src/asf/avr32/drivers/intc/intc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/intc_intc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/intc_intc.c$(DependSuffix) -MM "src/asf/avr32/drivers/intc/intc.c"

$(IntermediateDirectory)/intc_intc.c$(PreprocessSuffix): src/asf/avr32/drivers/intc/intc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/intc_intc.c$(PreprocessSuffix) "src/asf/avr32/drivers/intc/intc.c"

$(IntermediateDirectory)/intc_exception.S$(ObjectSuffix): src/asf/avr32/drivers/intc/exception.S $(IntermediateDirectory)/intc_exception.S$(DependSuffix)
	$(AS) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/intc/exception.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/intc_exception.S$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/intc_exception.S$(DependSuffix): src/asf/avr32/drivers/intc/exception.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/intc_exception.S$(ObjectSuffix) -MF$(IntermediateDirectory)/intc_exception.S$(DependSuffix) -MM "src/asf/avr32/drivers/intc/exception.S"

$(IntermediateDirectory)/intc_exception.S$(PreprocessSuffix): src/asf/avr32/drivers/intc/exception.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/intc_exception.S$(PreprocessSuffix) "src/asf/avr32/drivers/intc/exception.S"

$(IntermediateDirectory)/ast_ast.c$(ObjectSuffix): src/asf/avr32/drivers/ast/ast.c $(IntermediateDirectory)/ast_ast.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/ast/ast.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ast_ast.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ast_ast.c$(DependSuffix): src/asf/avr32/drivers/ast/ast.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ast_ast.c$(ObjectSuffix) -MF$(IntermediateDirectory)/ast_ast.c$(DependSuffix) -MM "src/asf/avr32/drivers/ast/ast.c"

$(IntermediateDirectory)/ast_ast.c$(PreprocessSuffix): src/asf/avr32/drivers/ast/ast.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ast_ast.c$(PreprocessSuffix) "src/asf/avr32/drivers/ast/ast.c"

$(IntermediateDirectory)/pevc_pevc.c$(ObjectSuffix): src/asf/avr32/drivers/pevc/pevc.c $(IntermediateDirectory)/pevc_pevc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/pevc/pevc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pevc_pevc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pevc_pevc.c$(DependSuffix): src/asf/avr32/drivers/pevc/pevc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pevc_pevc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/pevc_pevc.c$(DependSuffix) -MM "src/asf/avr32/drivers/pevc/pevc.c"

$(IntermediateDirectory)/pevc_pevc.c$(PreprocessSuffix): src/asf/avr32/drivers/pevc/pevc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pevc_pevc.c$(PreprocessSuffix) "src/asf/avr32/drivers/pevc/pevc.c"

$(IntermediateDirectory)/tc_tc.c$(ObjectSuffix): src/asf/avr32/drivers/tc/tc.c $(IntermediateDirectory)/tc_tc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/tc/tc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tc_tc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tc_tc.c$(DependSuffix): src/asf/avr32/drivers/tc/tc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/tc_tc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/tc_tc.c$(DependSuffix) -MM "src/asf/avr32/drivers/tc/tc.c"

$(IntermediateDirectory)/tc_tc.c$(PreprocessSuffix): src/asf/avr32/drivers/tc/tc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tc_tc.c$(PreprocessSuffix) "src/asf/avr32/drivers/tc/tc.c"

$(IntermediateDirectory)/spi_spi.c$(ObjectSuffix): src/asf/avr32/drivers/spi/spi.c $(IntermediateDirectory)/spi_spi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/spi/spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/spi_spi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/spi_spi.c$(DependSuffix): src/asf/avr32/drivers/spi/spi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/spi_spi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/spi_spi.c$(DependSuffix) -MM "src/asf/avr32/drivers/spi/spi.c"

$(IntermediateDirectory)/spi_spi.c$(PreprocessSuffix): src/asf/avr32/drivers/spi/spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/spi_spi.c$(PreprocessSuffix) "src/asf/avr32/drivers/spi/spi.c"

$(IntermediateDirectory)/pdca_pdca.c$(ObjectSuffix): src/asf/avr32/drivers/pdca/pdca.c $(IntermediateDirectory)/pdca_pdca.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/pdca/pdca.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pdca_pdca.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pdca_pdca.c$(DependSuffix): src/asf/avr32/drivers/pdca/pdca.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pdca_pdca.c$(ObjectSuffix) -MF$(IntermediateDirectory)/pdca_pdca.c$(DependSuffix) -MM "src/asf/avr32/drivers/pdca/pdca.c"

$(IntermediateDirectory)/pdca_pdca.c$(PreprocessSuffix): src/asf/avr32/drivers/pdca/pdca.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pdca_pdca.c$(PreprocessSuffix) "src/asf/avr32/drivers/pdca/pdca.c"

$(IntermediateDirectory)/eic_eic.c$(ObjectSuffix): src/asf/avr32/drivers/eic/eic.c $(IntermediateDirectory)/eic_eic.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/eic/eic.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/eic_eic.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/eic_eic.c$(DependSuffix): src/asf/avr32/drivers/eic/eic.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/eic_eic.c$(ObjectSuffix) -MF$(IntermediateDirectory)/eic_eic.c$(DependSuffix) -MM "src/asf/avr32/drivers/eic/eic.c"

$(IntermediateDirectory)/eic_eic.c$(PreprocessSuffix): src/asf/avr32/drivers/eic/eic.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/eic_eic.c$(PreprocessSuffix) "src/asf/avr32/drivers/eic/eic.c"

$(IntermediateDirectory)/dacifb_dacifb.c$(ObjectSuffix): src/asf/avr32/drivers/dacifb/dacifb.c $(IntermediateDirectory)/dacifb_dacifb.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/dacifb/dacifb.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dacifb_dacifb.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dacifb_dacifb.c$(DependSuffix): src/asf/avr32/drivers/dacifb/dacifb.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dacifb_dacifb.c$(ObjectSuffix) -MF$(IntermediateDirectory)/dacifb_dacifb.c$(DependSuffix) -MM "src/asf/avr32/drivers/dacifb/dacifb.c"

$(IntermediateDirectory)/dacifb_dacifb.c$(PreprocessSuffix): src/asf/avr32/drivers/dacifb/dacifb.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dacifb_dacifb.c$(PreprocessSuffix) "src/asf/avr32/drivers/dacifb/dacifb.c"

$(IntermediateDirectory)/pwm_pwm4.c$(ObjectSuffix): src/asf/avr32/drivers/pwm/pwm4.c $(IntermediateDirectory)/pwm_pwm4.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/pwm/pwm4.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pwm_pwm4.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pwm_pwm4.c$(DependSuffix): src/asf/avr32/drivers/pwm/pwm4.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pwm_pwm4.c$(ObjectSuffix) -MF$(IntermediateDirectory)/pwm_pwm4.c$(DependSuffix) -MM "src/asf/avr32/drivers/pwm/pwm4.c"

$(IntermediateDirectory)/pwm_pwm4.c$(PreprocessSuffix): src/asf/avr32/drivers/pwm/pwm4.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pwm_pwm4.c$(PreprocessSuffix) "src/asf/avr32/drivers/pwm/pwm4.c"

$(IntermediateDirectory)/flashc_flashc.c$(ObjectSuffix): src/asf/avr32/drivers/flashc/flashc.c $(IntermediateDirectory)/flashc_flashc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/flashc/flashc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flashc_flashc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flashc_flashc.c$(DependSuffix): src/asf/avr32/drivers/flashc/flashc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flashc_flashc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flashc_flashc.c$(DependSuffix) -MM "src/asf/avr32/drivers/flashc/flashc.c"

$(IntermediateDirectory)/flashc_flashc.c$(PreprocessSuffix): src/asf/avr32/drivers/flashc/flashc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flashc_flashc.c$(PreprocessSuffix) "src/asf/avr32/drivers/flashc/flashc.c"

$(IntermediateDirectory)/gpio_gpio.c$(ObjectSuffix): src/asf/avr32/drivers/gpio/gpio.c $(IntermediateDirectory)/gpio_gpio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/gpio/gpio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/gpio_gpio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/gpio_gpio.c$(DependSuffix): src/asf/avr32/drivers/gpio/gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/gpio_gpio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/gpio_gpio.c$(DependSuffix) -MM "src/asf/avr32/drivers/gpio/gpio.c"

$(IntermediateDirectory)/gpio_gpio.c$(PreprocessSuffix): src/asf/avr32/drivers/gpio/gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/gpio_gpio.c$(PreprocessSuffix) "src/asf/avr32/drivers/gpio/gpio.c"

$(IntermediateDirectory)/scif_scif_uc3c.c$(ObjectSuffix): src/asf/avr32/drivers/scif/scif_uc3c.c $(IntermediateDirectory)/scif_scif_uc3c.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/scif/scif_uc3c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/scif_scif_uc3c.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/scif_scif_uc3c.c$(DependSuffix): src/asf/avr32/drivers/scif/scif_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/scif_scif_uc3c.c$(ObjectSuffix) -MF$(IntermediateDirectory)/scif_scif_uc3c.c$(DependSuffix) -MM "src/asf/avr32/drivers/scif/scif_uc3c.c"

$(IntermediateDirectory)/scif_scif_uc3c.c$(PreprocessSuffix): src/asf/avr32/drivers/scif/scif_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/scif_scif_uc3c.c$(PreprocessSuffix) "src/asf/avr32/drivers/scif/scif_uc3c.c"

$(IntermediateDirectory)/usbc_usbc_device.c$(ObjectSuffix): src/asf/avr32/drivers/usbc/usbc_device.c $(IntermediateDirectory)/usbc_usbc_device.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/drivers/usbc/usbc_device.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/usbc_usbc_device.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/usbc_usbc_device.c$(DependSuffix): src/asf/avr32/drivers/usbc/usbc_device.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/usbc_usbc_device.c$(ObjectSuffix) -MF$(IntermediateDirectory)/usbc_usbc_device.c$(DependSuffix) -MM "src/asf/avr32/drivers/usbc/usbc_device.c"

$(IntermediateDirectory)/usbc_usbc_device.c$(PreprocessSuffix): src/asf/avr32/drivers/usbc/usbc_device.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/usbc_usbc_device.c$(PreprocessSuffix) "src/asf/avr32/drivers/usbc/usbc_device.c"

$(IntermediateDirectory)/startup_startup_uc3.S$(ObjectSuffix): src/asf/avr32/utils/startup/startup_uc3.S $(IntermediateDirectory)/startup_startup_uc3.S$(DependSuffix)
	$(AS) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/utils/startup/startup_uc3.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/startup_startup_uc3.S$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/startup_startup_uc3.S$(DependSuffix): src/asf/avr32/utils/startup/startup_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/startup_startup_uc3.S$(ObjectSuffix) -MF$(IntermediateDirectory)/startup_startup_uc3.S$(DependSuffix) -MM "src/asf/avr32/utils/startup/startup_uc3.S"

$(IntermediateDirectory)/startup_startup_uc3.S$(PreprocessSuffix): src/asf/avr32/utils/startup/startup_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/startup_startup_uc3.S$(PreprocessSuffix) "src/asf/avr32/utils/startup/startup_uc3.S"

$(IntermediateDirectory)/startup_trampoline_uc3.S$(ObjectSuffix): src/asf/avr32/utils/startup/trampoline_uc3.S $(IntermediateDirectory)/startup_trampoline_uc3.S$(DependSuffix)
	$(AS) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/utils/startup/trampoline_uc3.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/startup_trampoline_uc3.S$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/startup_trampoline_uc3.S$(DependSuffix): src/asf/avr32/utils/startup/trampoline_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/startup_trampoline_uc3.S$(ObjectSuffix) -MF$(IntermediateDirectory)/startup_trampoline_uc3.S$(DependSuffix) -MM "src/asf/avr32/utils/startup/trampoline_uc3.S"

$(IntermediateDirectory)/startup_trampoline_uc3.S$(PreprocessSuffix): src/asf/avr32/utils/startup/trampoline_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/startup_trampoline_uc3.S$(PreprocessSuffix) "src/asf/avr32/utils/startup/trampoline_uc3.S"

$(IntermediateDirectory)/delay_delay.c$(ObjectSuffix): src/asf/avr32/services/delay/delay.c $(IntermediateDirectory)/delay_delay.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/avr32/services/delay/delay.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/delay_delay.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/delay_delay.c$(DependSuffix): src/asf/avr32/services/delay/delay.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/delay_delay.c$(ObjectSuffix) -MF$(IntermediateDirectory)/delay_delay.c$(DependSuffix) -MM "src/asf/avr32/services/delay/delay.c"

$(IntermediateDirectory)/delay_delay.c$(PreprocessSuffix): src/asf/avr32/services/delay/delay.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/delay_delay.c$(PreprocessSuffix) "src/asf/avr32/services/delay/delay.c"

$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(ObjectSuffix): src/asf/common/utils/stdio/stdio_usb/stdio_usb.c $(IntermediateDirectory)/stdio_usb_stdio_usb.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/utils/stdio/stdio_usb/stdio_usb.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(DependSuffix): src/asf/common/utils/stdio/stdio_usb/stdio_usb.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(DependSuffix) -MM "src/asf/common/utils/stdio/stdio_usb/stdio_usb.c"

$(IntermediateDirectory)/stdio_usb_stdio_usb.c$(PreprocessSuffix): src/asf/common/utils/stdio/stdio_usb/stdio_usb.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_usb_stdio_usb.c$(PreprocessSuffix) "src/asf/common/utils/stdio/stdio_usb/stdio_usb.c"

$(IntermediateDirectory)/uc3_sleepmgr.c$(ObjectSuffix): src/asf/common/services/sleepmgr/uc3/sleepmgr.c $(IntermediateDirectory)/uc3_sleepmgr.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/sleepmgr/uc3/sleepmgr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3_sleepmgr.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3_sleepmgr.c$(DependSuffix): src/asf/common/services/sleepmgr/uc3/sleepmgr.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/uc3_sleepmgr.c$(ObjectSuffix) -MF$(IntermediateDirectory)/uc3_sleepmgr.c$(DependSuffix) -MM "src/asf/common/services/sleepmgr/uc3/sleepmgr.c"

$(IntermediateDirectory)/uc3_sleepmgr.c$(PreprocessSuffix): src/asf/common/services/sleepmgr/uc3/sleepmgr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3_sleepmgr.c$(PreprocessSuffix) "src/asf/common/services/sleepmgr/uc3/sleepmgr.c"

$(IntermediateDirectory)/uc3_spi_spi_master.c$(ObjectSuffix): src/asf/common/services/spi/uc3_spi/spi_master.c $(IntermediateDirectory)/uc3_spi_spi_master.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/spi/uc3_spi/spi_master.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3_spi_spi_master.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3_spi_spi_master.c$(DependSuffix): src/asf/common/services/spi/uc3_spi/spi_master.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/uc3_spi_spi_master.c$(ObjectSuffix) -MF$(IntermediateDirectory)/uc3_spi_spi_master.c$(DependSuffix) -MM "src/asf/common/services/spi/uc3_spi/spi_master.c"

$(IntermediateDirectory)/uc3_spi_spi_master.c$(PreprocessSuffix): src/asf/common/services/spi/uc3_spi/spi_master.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3_spi_spi_master.c$(PreprocessSuffix) "src/asf/common/services/spi/uc3_spi/spi_master.c"

$(IntermediateDirectory)/uc3c_pll.c$(ObjectSuffix): src/asf/common/services/clock/uc3c/pll.c $(IntermediateDirectory)/uc3c_pll.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/clock/uc3c/pll.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_pll.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_pll.c$(DependSuffix): src/asf/common/services/clock/uc3c/pll.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/uc3c_pll.c$(ObjectSuffix) -MF$(IntermediateDirectory)/uc3c_pll.c$(DependSuffix) -MM "src/asf/common/services/clock/uc3c/pll.c"

$(IntermediateDirectory)/uc3c_pll.c$(PreprocessSuffix): src/asf/common/services/clock/uc3c/pll.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_pll.c$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/pll.c"

$(IntermediateDirectory)/uc3c_osc.c$(ObjectSuffix): src/asf/common/services/clock/uc3c/osc.c $(IntermediateDirectory)/uc3c_osc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/clock/uc3c/osc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_osc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_osc.c$(DependSuffix): src/asf/common/services/clock/uc3c/osc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/uc3c_osc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/uc3c_osc.c$(DependSuffix) -MM "src/asf/common/services/clock/uc3c/osc.c"

$(IntermediateDirectory)/uc3c_osc.c$(PreprocessSuffix): src/asf/common/services/clock/uc3c/osc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_osc.c$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/osc.c"

$(IntermediateDirectory)/uc3c_sysclk.c$(ObjectSuffix): src/asf/common/services/clock/uc3c/sysclk.c $(IntermediateDirectory)/uc3c_sysclk.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/clock/uc3c/sysclk.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_sysclk.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_sysclk.c$(DependSuffix): src/asf/common/services/clock/uc3c/sysclk.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/uc3c_sysclk.c$(ObjectSuffix) -MF$(IntermediateDirectory)/uc3c_sysclk.c$(DependSuffix) -MM "src/asf/common/services/clock/uc3c/sysclk.c"

$(IntermediateDirectory)/uc3c_sysclk.c$(PreprocessSuffix): src/asf/common/services/clock/uc3c/sysclk.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_sysclk.c$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/sysclk.c"

$(IntermediateDirectory)/udc_udc.c$(ObjectSuffix): src/asf/common/services/usb/udc/udc.c $(IntermediateDirectory)/udc_udc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/usb/udc/udc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/udc_udc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/udc_udc.c$(DependSuffix): src/asf/common/services/usb/udc/udc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/udc_udc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/udc_udc.c$(DependSuffix) -MM "src/asf/common/services/usb/udc/udc.c"

$(IntermediateDirectory)/udc_udc.c$(PreprocessSuffix): src/asf/common/services/usb/udc/udc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/udc_udc.c$(PreprocessSuffix) "src/asf/common/services/usb/udc/udc.c"

$(IntermediateDirectory)/device_udi_cdc.c$(ObjectSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc.c $(IntermediateDirectory)/device_udi_cdc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/usb/class/cdc/device/udi_cdc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/device_udi_cdc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/device_udi_cdc.c$(DependSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/device_udi_cdc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/device_udi_cdc.c$(DependSuffix) -MM "src/asf/common/services/usb/class/cdc/device/udi_cdc.c"

$(IntermediateDirectory)/device_udi_cdc.c$(PreprocessSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/device_udi_cdc.c$(PreprocessSuffix) "src/asf/common/services/usb/class/cdc/device/udi_cdc.c"

$(IntermediateDirectory)/device_udi_cdc_desc.c$(ObjectSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c $(IntermediateDirectory)/device_udi_cdc_desc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/media/gleplatt/DATA/LIS/Maveric/maveric/Code/Mavric_MobileRobot/src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/device_udi_cdc_desc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/device_udi_cdc_desc.c$(DependSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/device_udi_cdc_desc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/device_udi_cdc_desc.c$(DependSuffix) -MM "src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c"

$(IntermediateDirectory)/device_udi_cdc_desc.c$(PreprocessSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/device_udi_cdc_desc.c$(PreprocessSuffix) "src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) ./Debug_Linux/obj/*$(ObjectSuffix)
	$(RM) ./Debug_Linux/obj/*$(DependSuffix)
	$(RM) $(OutputFile)
	$(RM) ".build-debug/Mavric_MobRob_linux"


