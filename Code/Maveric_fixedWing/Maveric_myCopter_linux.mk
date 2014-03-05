##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Maveric_myCopter_linux
ConfigurationName      :=Debug
WorkspacePath          := "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing"
ProjectPath            := "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing"
IntermediateDirectory  :=./Debug_Linux/obj
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Julien Lecoeur
Date                   :=03/05/14
CodeLitePath           :="/home/julien/.codelite"
LinkerName             :=avr32-g++
SharedObjectLinkerName :=avr32-g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=
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
ObjectsFileList        :="Maveric_myCopter_linux.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -nostartfiles -Wl,-Map="DualRadar.map" -Wl,--start-group -lm  -Wl,--end-group -L"src/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc"  -Wl,--gc-sections -mpart=uc3c1512c -Wl,--relax -Wl,-e,_trampoline
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
AR       := avr32-ar rcu
CXX      := avr32-g++
CC       := avr32-gcc
CXXFLAGS :=  -O1 -std=gnu++0x -mhard-float -ffunction-sections -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c  -Wpointer-arith -mrelax -MD -MP -MF $(Preprocessors)
CFLAGS   :=  -O1 -mhard-float -ffunction-sections -fdata-sections -muse-rodata-section -g2 -pg -p -Wall -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF $(Preprocessors)
ASFLAGS  :=  -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax 
AS       := avr32-gcc


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix) $(IntermediateDirectory)/src_tasks$(ObjectSuffix) $(IntermediateDirectory)/src_main$(ObjectSuffix) $(IntermediateDirectory)/src_central_data$(ObjectSuffix) $(IntermediateDirectory)/src_boardsupport$(ObjectSuffix) $(IntermediateDirectory)/tests_test_small_matrix$(ObjectSuffix) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) $(IntermediateDirectory)/tests_test_quick_trig$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) \
	$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IntermediateDirectory)/control_orca$(ObjectSuffix) $(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IntermediateDirectory)/hal_lsm330dlc_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_adxl345_driver$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_compass_hmc5883l$(ObjectSuffix) $(IntermediateDirectory)/hal_bmp085$(ObjectSuffix) $(IntermediateDirectory)/hal_time_keeper$(ObjectSuffix) $(IntermediateDirectory)/hal_uart_int$(ObjectSuffix) $(IntermediateDirectory)/hal_amplifiers$(ObjectSuffix) $(IntermediateDirectory)/hal_radar_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_radar_module_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_led$(ObjectSuffix) $(IntermediateDirectory)/hal_spi_buffered$(ObjectSuffix) $(IntermediateDirectory)/hal_itg3200_driver$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_adc_int$(ObjectSuffix) $(IntermediateDirectory)/hal_dac_dma$(ObjectSuffix) $(IntermediateDirectory)/hal_ads1274$(ObjectSuffix) $(IntermediateDirectory)/hal_servo_pwm$(ObjectSuffix) $(IntermediateDirectory)/hal_analog_monitor$(ObjectSuffix) $(IntermediateDirectory)/hal_remote_dsm2$(ObjectSuffix) $(IntermediateDirectory)/hal_i2c_driver_int$(ObjectSuffix) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) 

Objects1=$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) \
	$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IntermediateDirectory)/sensing_estimator$(ObjectSuffix) $(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IntermediateDirectory)/util_linear_algebra$(ObjectSuffix) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IntermediateDirectory)/util_buffer$(ObjectSuffix) \
	$(IntermediateDirectory)/util_kalman$(ObjectSuffix) $(IntermediateDirectory)/util_quick_trig$(ObjectSuffix) $(IntermediateDirectory)/util_generator$(ObjectSuffix) 

Objects2=$(IntermediateDirectory)/user_board_init$(ObjectSuffix) $(IntermediateDirectory)/stdio_read$(ObjectSuffix) $(IntermediateDirectory)/stdio_write$(ObjectSuffix) $(IntermediateDirectory)/pm_power_clocks_lib$(ObjectSuffix) $(IntermediateDirectory)/pm_pm_uc3c$(ObjectSuffix) $(IntermediateDirectory)/adcifa_adcifa$(ObjectSuffix) $(IntermediateDirectory)/usart_usart$(ObjectSuffix) \
	$(IntermediateDirectory)/twim_twim$(ObjectSuffix) $(IntermediateDirectory)/intc_intc$(ObjectSuffix) $(IntermediateDirectory)/intc_exception$(ObjectSuffix) $(IntermediateDirectory)/ast_ast$(ObjectSuffix) $(IntermediateDirectory)/pevc_pevc$(ObjectSuffix) $(IntermediateDirectory)/tc_tc$(ObjectSuffix) $(IntermediateDirectory)/spi_spi$(ObjectSuffix) $(IntermediateDirectory)/pdca_pdca$(ObjectSuffix) $(IntermediateDirectory)/eic_eic$(ObjectSuffix) $(IntermediateDirectory)/dacifb_dacifb$(ObjectSuffix) \
	$(IntermediateDirectory)/pwm_pwm4$(ObjectSuffix) $(IntermediateDirectory)/flashc_flashc$(ObjectSuffix) $(IntermediateDirectory)/gpio_gpio$(ObjectSuffix) $(IntermediateDirectory)/scif_scif_uc3c$(ObjectSuffix) $(IntermediateDirectory)/usbc_usbc_device$(ObjectSuffix) $(IntermediateDirectory)/startup_startup_uc3$(ObjectSuffix) $(IntermediateDirectory)/startup_trampoline_uc3$(ObjectSuffix) $(IntermediateDirectory)/delay_delay$(ObjectSuffix) $(IntermediateDirectory)/stdio_usb_stdio_usb$(ObjectSuffix) $(IntermediateDirectory)/uc3_sleepmgr$(ObjectSuffix) \
	$(IntermediateDirectory)/uc3_spi_spi_master$(ObjectSuffix) 

Objects3=$(IntermediateDirectory)/uc3c_pll$(ObjectSuffix) $(IntermediateDirectory)/uc3c_osc$(ObjectSuffix) $(IntermediateDirectory)/uc3c_sysclk$(ObjectSuffix) $(IntermediateDirectory)/udc_udc$(ObjectSuffix) $(IntermediateDirectory)/device_udi_cdc$(ObjectSuffix) $(IntermediateDirectory)/device_udi_cdc_desc$(ObjectSuffix) 



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
$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix): src/mavlink_actions.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/mavlink_actions.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix): src/mavlink_actions.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix) "src/mavlink_actions.c"

$(IntermediateDirectory)/src_tasks$(ObjectSuffix): src/tasks.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/tasks.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tasks$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tasks$(PreprocessSuffix): src/tasks.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tasks$(PreprocessSuffix) "src/tasks.c"

$(IntermediateDirectory)/src_main$(ObjectSuffix): src/main.cpp 
	$(CXX) $(IncludePCH) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_main$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_main$(PreprocessSuffix): src/main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_main$(PreprocessSuffix) "src/main.cpp"

$(IntermediateDirectory)/src_central_data$(ObjectSuffix): src/central_data.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/central_data.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_central_data$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_central_data$(PreprocessSuffix): src/central_data.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_central_data$(PreprocessSuffix) "src/central_data.c"

$(IntermediateDirectory)/src_boardsupport$(ObjectSuffix): src/boardsupport.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/boardsupport.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_boardsupport$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_boardsupport$(PreprocessSuffix): src/boardsupport.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_boardsupport$(PreprocessSuffix) "src/boardsupport.c"

$(IntermediateDirectory)/tests_test_small_matrix$(ObjectSuffix): Library/tests/test_small_matrix.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/tests/test_small_matrix.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tests_test_small_matrix$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tests_test_small_matrix$(PreprocessSuffix): Library/tests/test_small_matrix.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tests_test_small_matrix$(PreprocessSuffix) "Library/tests/test_small_matrix.c"

$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix): Library/tests/test_maths.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/tests/test_maths.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix): Library/tests/test_maths.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix) "Library/tests/test_maths.c"

$(IntermediateDirectory)/tests_test_quick_trig$(ObjectSuffix): Library/tests/test_quick_trig.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/tests/test_quick_trig.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tests_test_quick_trig$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tests_test_quick_trig$(PreprocessSuffix): Library/tests/test_quick_trig.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tests_test_quick_trig$(PreprocessSuffix) "Library/tests/test_quick_trig.c"

$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix): Library/communication/mavlink_stream.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/communication/mavlink_stream.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix): Library/communication/mavlink_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix) "Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix): Library/communication/onboard_parameters.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/communication/onboard_parameters.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix): Library/communication/onboard_parameters.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix) "Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix): Library/communication/mavlink_waypoint_handler.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/communication/mavlink_waypoint_handler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix): Library/communication/mavlink_waypoint_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix) "Library/communication/mavlink_waypoint_handler.c"

$(IntermediateDirectory)/control_pid_control$(ObjectSuffix): Library/control/pid_control.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/pid_control.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_pid_control$(PreprocessSuffix): Library/control/pid_control.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix) "Library/control/pid_control.c"

$(IntermediateDirectory)/control_orca$(ObjectSuffix): Library/control/orca.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/orca.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_orca$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_orca$(PreprocessSuffix): Library/control/orca.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_orca$(PreprocessSuffix) "Library/control/orca.c"

$(IntermediateDirectory)/control_navigation$(ObjectSuffix): Library/control/navigation.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/navigation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_navigation$(PreprocessSuffix): Library/control/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix) "Library/control/navigation.c"

$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix): Library/control/stabilisation_hybrid.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/stabilisation_hybrid.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix): Library/control/stabilisation_hybrid.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix) "Library/control/stabilisation_hybrid.c"

$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix): Library/control/stabilisation_copter.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/stabilisation_copter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix): Library/control/stabilisation_copter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix) "Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix): Library/control/adaptive_parameter.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/adaptive_parameter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix): Library/control/adaptive_parameter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix) "Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix): Library/control/stabilisation.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/control/stabilisation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix): Library/control/stabilisation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix) "Library/control/stabilisation.c"

$(IntermediateDirectory)/hal_lsm330dlc_driver$(ObjectSuffix): Library/hal/lsm330dlc_driver.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/lsm330dlc_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_lsm330dlc_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_lsm330dlc_driver$(PreprocessSuffix): Library/hal/lsm330dlc_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_lsm330dlc_driver$(PreprocessSuffix) "Library/hal/lsm330dlc_driver.c"

$(IntermediateDirectory)/hal_adxl345_driver$(ObjectSuffix): Library/hal/adxl345_driver.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/adxl345_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_adxl345_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_adxl345_driver$(PreprocessSuffix): Library/hal/adxl345_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_adxl345_driver$(PreprocessSuffix) "Library/hal/adxl345_driver.c"

$(IntermediateDirectory)/hal_compass_hmc5883l$(ObjectSuffix): Library/hal/compass_hmc5883l.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/compass_hmc5883l.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_compass_hmc5883l$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_compass_hmc5883l$(PreprocessSuffix): Library/hal/compass_hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_compass_hmc5883l$(PreprocessSuffix) "Library/hal/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_bmp085$(ObjectSuffix): Library/hal/bmp085.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/bmp085.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_bmp085$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_bmp085$(PreprocessSuffix): Library/hal/bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_bmp085$(PreprocessSuffix) "Library/hal/bmp085.c"

$(IntermediateDirectory)/hal_time_keeper$(ObjectSuffix): Library/hal/time_keeper.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/time_keeper.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_time_keeper$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_time_keeper$(PreprocessSuffix): Library/hal/time_keeper.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_time_keeper$(PreprocessSuffix) "Library/hal/time_keeper.c"

$(IntermediateDirectory)/hal_uart_int$(ObjectSuffix): Library/hal/uart_int.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/uart_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_uart_int$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_uart_int$(PreprocessSuffix): Library/hal/uart_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_uart_int$(PreprocessSuffix) "Library/hal/uart_int.c"

$(IntermediateDirectory)/hal_amplifiers$(ObjectSuffix): Library/hal/amplifiers.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/amplifiers.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_amplifiers$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_amplifiers$(PreprocessSuffix): Library/hal/amplifiers.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_amplifiers$(PreprocessSuffix) "Library/hal/amplifiers.c"

$(IntermediateDirectory)/hal_radar_driver$(ObjectSuffix): Library/hal/radar_driver.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/radar_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_radar_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_radar_driver$(PreprocessSuffix): Library/hal/radar_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_radar_driver$(PreprocessSuffix) "Library/hal/radar_driver.c"

$(IntermediateDirectory)/hal_radar_module_driver$(ObjectSuffix): Library/hal/radar_module_driver.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/radar_module_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_radar_module_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_radar_module_driver$(PreprocessSuffix): Library/hal/radar_module_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_radar_module_driver$(PreprocessSuffix) "Library/hal/radar_module_driver.c"

$(IntermediateDirectory)/hal_led$(ObjectSuffix): Library/hal/led.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/led.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_led$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_led$(PreprocessSuffix): Library/hal/led.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_led$(PreprocessSuffix) "Library/hal/led.c"

$(IntermediateDirectory)/hal_spi_buffered$(ObjectSuffix): Library/hal/spi_buffered.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/spi_buffered.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_spi_buffered$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_spi_buffered$(PreprocessSuffix): Library/hal/spi_buffered.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_spi_buffered$(PreprocessSuffix) "Library/hal/spi_buffered.c"

$(IntermediateDirectory)/hal_itg3200_driver$(ObjectSuffix): Library/hal/itg3200_driver.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/itg3200_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_itg3200_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_itg3200_driver$(PreprocessSuffix): Library/hal/itg3200_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_itg3200_driver$(PreprocessSuffix) "Library/hal/itg3200_driver.c"

$(IntermediateDirectory)/hal_adc_int$(ObjectSuffix): Library/hal/adc_int.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/adc_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_adc_int$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_adc_int$(PreprocessSuffix): Library/hal/adc_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_adc_int$(PreprocessSuffix) "Library/hal/adc_int.c"

$(IntermediateDirectory)/hal_dac_dma$(ObjectSuffix): Library/hal/dac_dma.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/dac_dma.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_dac_dma$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_dac_dma$(PreprocessSuffix): Library/hal/dac_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_dac_dma$(PreprocessSuffix) "Library/hal/dac_dma.c"

$(IntermediateDirectory)/hal_ads1274$(ObjectSuffix): Library/hal/ads1274.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/ads1274.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_ads1274$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_ads1274$(PreprocessSuffix): Library/hal/ads1274.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_ads1274$(PreprocessSuffix) "Library/hal/ads1274.c"

$(IntermediateDirectory)/hal_servo_pwm$(ObjectSuffix): Library/hal/servo_pwm.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/servo_pwm.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_servo_pwm$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_servo_pwm$(PreprocessSuffix): Library/hal/servo_pwm.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_servo_pwm$(PreprocessSuffix) "Library/hal/servo_pwm.c"

$(IntermediateDirectory)/hal_analog_monitor$(ObjectSuffix): Library/hal/analog_monitor.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/analog_monitor.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_analog_monitor$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_analog_monitor$(PreprocessSuffix): Library/hal/analog_monitor.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_analog_monitor$(PreprocessSuffix) "Library/hal/analog_monitor.c"

$(IntermediateDirectory)/hal_remote_dsm2$(ObjectSuffix): Library/hal/remote_dsm2.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/remote_dsm2.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_remote_dsm2$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_remote_dsm2$(PreprocessSuffix): Library/hal/remote_dsm2.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_remote_dsm2$(PreprocessSuffix) "Library/hal/remote_dsm2.c"

$(IntermediateDirectory)/hal_i2c_driver_int$(ObjectSuffix): Library/hal/i2c_driver_int.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/hal/i2c_driver_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_i2c_driver_int$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_i2c_driver_int$(PreprocessSuffix): Library/hal/i2c_driver_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_i2c_driver_int$(PreprocessSuffix) "Library/hal/i2c_driver_int.c"

$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix): Library/runtime/scheduler.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/runtime/scheduler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix): Library/runtime/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix) "Library/runtime/scheduler.c"

$(IntermediateDirectory)/sensing_imu$(ObjectSuffix): Library/sensing/imu.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/imu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_imu$(PreprocessSuffix): Library/sensing/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix) "Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix): Library/sensing/position_estimation.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/position_estimation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix): Library/sensing/position_estimation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix) "Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix): Library/sensing/qfilter.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/qfilter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix): Library/sensing/qfilter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix) "Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_estimator$(ObjectSuffix): Library/sensing/estimator.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/estimator.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_estimator$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix): Library/sensing/estimator.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix) "Library/sensing/estimator.c"

$(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix): Library/sensing/neighbor_selection.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/neighbor_selection.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix): Library/sensing/neighbor_selection.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix) "Library/sensing/neighbor_selection.c"

$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix): Library/sensing/gps_ublox.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/gps_ublox.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix): Library/sensing/gps_ublox.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix) "Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix): Library/sensing/simulation.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/sensing/simulation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix): Library/sensing/simulation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix) "Library/sensing/simulation.c"

$(IntermediateDirectory)/util_linear_algebra$(ObjectSuffix): Library/util/linear_algebra.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/linear_algebra.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_linear_algebra$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_linear_algebra$(PreprocessSuffix): Library/util/linear_algebra.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_linear_algebra$(PreprocessSuffix) "Library/util/linear_algebra.c"

$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix): Library/util/coord_conventions.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/coord_conventions.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix): Library/util/coord_conventions.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix) "Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_sinus$(ObjectSuffix): Library/util/sinus.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/sinus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_sinus$(PreprocessSuffix): Library/util/sinus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix) "Library/util/sinus.c"

$(IntermediateDirectory)/util_print_util$(ObjectSuffix): Library/util/print_util.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/print_util.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_print_util$(PreprocessSuffix): Library/util/print_util.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix) "Library/util/print_util.c"

$(IntermediateDirectory)/util_buffer$(ObjectSuffix): Library/util/buffer.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/buffer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_buffer$(PreprocessSuffix): Library/util/buffer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix) "Library/util/buffer.c"

$(IntermediateDirectory)/util_kalman$(ObjectSuffix): Library/util/kalman.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/kalman.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_kalman$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_kalman$(PreprocessSuffix): Library/util/kalman.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_kalman$(PreprocessSuffix) "Library/util/kalman.c"

$(IntermediateDirectory)/util_quick_trig$(ObjectSuffix): Library/util/quick_trig.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/quick_trig.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_quick_trig$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix): Library/util/quick_trig.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix) "Library/util/quick_trig.c"

$(IntermediateDirectory)/util_generator$(ObjectSuffix): Library/util/generator.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/Library/util/generator.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_generator$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_generator$(PreprocessSuffix): Library/util/generator.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_generator$(PreprocessSuffix) "Library/util/generator.c"

$(IntermediateDirectory)/user_board_init$(ObjectSuffix): src/asf/common/boards/user_board/init.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/boards/user_board/init.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/user_board_init$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/user_board_init$(PreprocessSuffix): src/asf/common/boards/user_board/init.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/user_board_init$(PreprocessSuffix) "src/asf/common/boards/user_board/init.c"

$(IntermediateDirectory)/stdio_read$(ObjectSuffix): src/asf/common/utils/stdio/read.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/utils/stdio/read.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_read$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_read$(PreprocessSuffix): src/asf/common/utils/stdio/read.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_read$(PreprocessSuffix) "src/asf/common/utils/stdio/read.c"

$(IntermediateDirectory)/stdio_write$(ObjectSuffix): src/asf/common/utils/stdio/write.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/utils/stdio/write.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_write$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_write$(PreprocessSuffix): src/asf/common/utils/stdio/write.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_write$(PreprocessSuffix) "src/asf/common/utils/stdio/write.c"

$(IntermediateDirectory)/pm_power_clocks_lib$(ObjectSuffix): src/asf/avr32/drivers/pm/power_clocks_lib.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/pm/power_clocks_lib.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pm_power_clocks_lib$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pm_power_clocks_lib$(PreprocessSuffix): src/asf/avr32/drivers/pm/power_clocks_lib.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pm_power_clocks_lib$(PreprocessSuffix) "src/asf/avr32/drivers/pm/power_clocks_lib.c"

$(IntermediateDirectory)/pm_pm_uc3c$(ObjectSuffix): src/asf/avr32/drivers/pm/pm_uc3c.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/pm/pm_uc3c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pm_pm_uc3c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pm_pm_uc3c$(PreprocessSuffix): src/asf/avr32/drivers/pm/pm_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pm_pm_uc3c$(PreprocessSuffix) "src/asf/avr32/drivers/pm/pm_uc3c.c"

$(IntermediateDirectory)/adcifa_adcifa$(ObjectSuffix): src/asf/avr32/drivers/adcifa/adcifa.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/adcifa/adcifa.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/adcifa_adcifa$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/adcifa_adcifa$(PreprocessSuffix): src/asf/avr32/drivers/adcifa/adcifa.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/adcifa_adcifa$(PreprocessSuffix) "src/asf/avr32/drivers/adcifa/adcifa.c"

$(IntermediateDirectory)/usart_usart$(ObjectSuffix): src/asf/avr32/drivers/usart/usart.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/usart/usart.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/usart_usart$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/usart_usart$(PreprocessSuffix): src/asf/avr32/drivers/usart/usart.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/usart_usart$(PreprocessSuffix) "src/asf/avr32/drivers/usart/usart.c"

$(IntermediateDirectory)/twim_twim$(ObjectSuffix): src/asf/avr32/drivers/twim/twim.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/twim/twim.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/twim_twim$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/twim_twim$(PreprocessSuffix): src/asf/avr32/drivers/twim/twim.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/twim_twim$(PreprocessSuffix) "src/asf/avr32/drivers/twim/twim.c"

$(IntermediateDirectory)/intc_intc$(ObjectSuffix): src/asf/avr32/drivers/intc/intc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/intc/intc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/intc_intc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/intc_intc$(PreprocessSuffix): src/asf/avr32/drivers/intc/intc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/intc_intc$(PreprocessSuffix) "src/asf/avr32/drivers/intc/intc.c"

$(IntermediateDirectory)/intc_exception$(ObjectSuffix): src/asf/avr32/drivers/intc/exception.S 
	$(AS) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/intc/exception.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/intc_exception$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/intc_exception$(PreprocessSuffix): src/asf/avr32/drivers/intc/exception.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/intc_exception$(PreprocessSuffix) "src/asf/avr32/drivers/intc/exception.S"

$(IntermediateDirectory)/ast_ast$(ObjectSuffix): src/asf/avr32/drivers/ast/ast.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/ast/ast.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ast_ast$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ast_ast$(PreprocessSuffix): src/asf/avr32/drivers/ast/ast.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ast_ast$(PreprocessSuffix) "src/asf/avr32/drivers/ast/ast.c"

$(IntermediateDirectory)/pevc_pevc$(ObjectSuffix): src/asf/avr32/drivers/pevc/pevc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/pevc/pevc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pevc_pevc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pevc_pevc$(PreprocessSuffix): src/asf/avr32/drivers/pevc/pevc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pevc_pevc$(PreprocessSuffix) "src/asf/avr32/drivers/pevc/pevc.c"

$(IntermediateDirectory)/tc_tc$(ObjectSuffix): src/asf/avr32/drivers/tc/tc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/tc/tc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tc_tc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tc_tc$(PreprocessSuffix): src/asf/avr32/drivers/tc/tc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tc_tc$(PreprocessSuffix) "src/asf/avr32/drivers/tc/tc.c"

$(IntermediateDirectory)/spi_spi$(ObjectSuffix): src/asf/avr32/drivers/spi/spi.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/spi/spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/spi_spi$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/spi_spi$(PreprocessSuffix): src/asf/avr32/drivers/spi/spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/spi_spi$(PreprocessSuffix) "src/asf/avr32/drivers/spi/spi.c"

$(IntermediateDirectory)/pdca_pdca$(ObjectSuffix): src/asf/avr32/drivers/pdca/pdca.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/pdca/pdca.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pdca_pdca$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pdca_pdca$(PreprocessSuffix): src/asf/avr32/drivers/pdca/pdca.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pdca_pdca$(PreprocessSuffix) "src/asf/avr32/drivers/pdca/pdca.c"

$(IntermediateDirectory)/eic_eic$(ObjectSuffix): src/asf/avr32/drivers/eic/eic.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/eic/eic.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/eic_eic$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/eic_eic$(PreprocessSuffix): src/asf/avr32/drivers/eic/eic.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/eic_eic$(PreprocessSuffix) "src/asf/avr32/drivers/eic/eic.c"

$(IntermediateDirectory)/dacifb_dacifb$(ObjectSuffix): src/asf/avr32/drivers/dacifb/dacifb.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/dacifb/dacifb.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dacifb_dacifb$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dacifb_dacifb$(PreprocessSuffix): src/asf/avr32/drivers/dacifb/dacifb.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dacifb_dacifb$(PreprocessSuffix) "src/asf/avr32/drivers/dacifb/dacifb.c"

$(IntermediateDirectory)/pwm_pwm4$(ObjectSuffix): src/asf/avr32/drivers/pwm/pwm4.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/pwm/pwm4.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pwm_pwm4$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pwm_pwm4$(PreprocessSuffix): src/asf/avr32/drivers/pwm/pwm4.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pwm_pwm4$(PreprocessSuffix) "src/asf/avr32/drivers/pwm/pwm4.c"

$(IntermediateDirectory)/flashc_flashc$(ObjectSuffix): src/asf/avr32/drivers/flashc/flashc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/flashc/flashc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flashc_flashc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flashc_flashc$(PreprocessSuffix): src/asf/avr32/drivers/flashc/flashc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flashc_flashc$(PreprocessSuffix) "src/asf/avr32/drivers/flashc/flashc.c"

$(IntermediateDirectory)/gpio_gpio$(ObjectSuffix): src/asf/avr32/drivers/gpio/gpio.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/gpio/gpio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/gpio_gpio$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/gpio_gpio$(PreprocessSuffix): src/asf/avr32/drivers/gpio/gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/gpio_gpio$(PreprocessSuffix) "src/asf/avr32/drivers/gpio/gpio.c"

$(IntermediateDirectory)/scif_scif_uc3c$(ObjectSuffix): src/asf/avr32/drivers/scif/scif_uc3c.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/scif/scif_uc3c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/scif_scif_uc3c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/scif_scif_uc3c$(PreprocessSuffix): src/asf/avr32/drivers/scif/scif_uc3c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/scif_scif_uc3c$(PreprocessSuffix) "src/asf/avr32/drivers/scif/scif_uc3c.c"

$(IntermediateDirectory)/usbc_usbc_device$(ObjectSuffix): src/asf/avr32/drivers/usbc/usbc_device.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/drivers/usbc/usbc_device.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/usbc_usbc_device$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/usbc_usbc_device$(PreprocessSuffix): src/asf/avr32/drivers/usbc/usbc_device.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/usbc_usbc_device$(PreprocessSuffix) "src/asf/avr32/drivers/usbc/usbc_device.c"

$(IntermediateDirectory)/startup_startup_uc3$(ObjectSuffix): src/asf/avr32/utils/startup/startup_uc3.S 
	$(AS) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/utils/startup/startup_uc3.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/startup_startup_uc3$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/startup_startup_uc3$(PreprocessSuffix): src/asf/avr32/utils/startup/startup_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/startup_startup_uc3$(PreprocessSuffix) "src/asf/avr32/utils/startup/startup_uc3.S"

$(IntermediateDirectory)/startup_trampoline_uc3$(ObjectSuffix): src/asf/avr32/utils/startup/trampoline_uc3.S 
	$(AS) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/utils/startup/trampoline_uc3.S" $(ASFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/startup_trampoline_uc3$(ObjectSuffix) -I$(IncludePath)
$(IntermediateDirectory)/startup_trampoline_uc3$(PreprocessSuffix): src/asf/avr32/utils/startup/trampoline_uc3.S
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/startup_trampoline_uc3$(PreprocessSuffix) "src/asf/avr32/utils/startup/trampoline_uc3.S"

$(IntermediateDirectory)/delay_delay$(ObjectSuffix): src/asf/avr32/services/delay/delay.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/avr32/services/delay/delay.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/delay_delay$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/delay_delay$(PreprocessSuffix): src/asf/avr32/services/delay/delay.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/delay_delay$(PreprocessSuffix) "src/asf/avr32/services/delay/delay.c"

$(IntermediateDirectory)/stdio_usb_stdio_usb$(ObjectSuffix): src/asf/common/utils/stdio/stdio_usb/stdio_usb.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/utils/stdio/stdio_usb/stdio_usb.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stdio_usb_stdio_usb$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stdio_usb_stdio_usb$(PreprocessSuffix): src/asf/common/utils/stdio/stdio_usb/stdio_usb.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stdio_usb_stdio_usb$(PreprocessSuffix) "src/asf/common/utils/stdio/stdio_usb/stdio_usb.c"

$(IntermediateDirectory)/uc3_sleepmgr$(ObjectSuffix): src/asf/common/services/sleepmgr/uc3/sleepmgr.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/sleepmgr/uc3/sleepmgr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3_sleepmgr$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3_sleepmgr$(PreprocessSuffix): src/asf/common/services/sleepmgr/uc3/sleepmgr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3_sleepmgr$(PreprocessSuffix) "src/asf/common/services/sleepmgr/uc3/sleepmgr.c"

$(IntermediateDirectory)/uc3_spi_spi_master$(ObjectSuffix): src/asf/common/services/spi/uc3_spi/spi_master.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/spi/uc3_spi/spi_master.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3_spi_spi_master$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3_spi_spi_master$(PreprocessSuffix): src/asf/common/services/spi/uc3_spi/spi_master.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3_spi_spi_master$(PreprocessSuffix) "src/asf/common/services/spi/uc3_spi/spi_master.c"

$(IntermediateDirectory)/uc3c_pll$(ObjectSuffix): src/asf/common/services/clock/uc3c/pll.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/clock/uc3c/pll.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_pll$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_pll$(PreprocessSuffix): src/asf/common/services/clock/uc3c/pll.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_pll$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/pll.c"

$(IntermediateDirectory)/uc3c_osc$(ObjectSuffix): src/asf/common/services/clock/uc3c/osc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/clock/uc3c/osc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_osc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_osc$(PreprocessSuffix): src/asf/common/services/clock/uc3c/osc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_osc$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/osc.c"

$(IntermediateDirectory)/uc3c_sysclk$(ObjectSuffix): src/asf/common/services/clock/uc3c/sysclk.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/clock/uc3c/sysclk.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/uc3c_sysclk$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/uc3c_sysclk$(PreprocessSuffix): src/asf/common/services/clock/uc3c/sysclk.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/uc3c_sysclk$(PreprocessSuffix) "src/asf/common/services/clock/uc3c/sysclk.c"

$(IntermediateDirectory)/udc_udc$(ObjectSuffix): src/asf/common/services/usb/udc/udc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/usb/udc/udc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/udc_udc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/udc_udc$(PreprocessSuffix): src/asf/common/services/usb/udc/udc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/udc_udc$(PreprocessSuffix) "src/asf/common/services/usb/udc/udc.c"

$(IntermediateDirectory)/device_udi_cdc$(ObjectSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/usb/class/cdc/device/udi_cdc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/device_udi_cdc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/device_udi_cdc$(PreprocessSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/device_udi_cdc$(PreprocessSuffix) "src/asf/common/services/usb/class/cdc/device/udi_cdc.c"

$(IntermediateDirectory)/device_udi_cdc_desc$(ObjectSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c 
	$(CC) $(SourceSwitch) "/media/julien/Data/Documents/Robots/maveric/Code/Maveric_fixedWing/src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/device_udi_cdc_desc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/device_udi_cdc_desc$(PreprocessSuffix): src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/device_udi_cdc_desc$(PreprocessSuffix) "src/asf/common/services/usb/class/cdc/device/udi_cdc_desc.c"

##
## Clean
##
clean:
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_main$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_main$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_main$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_boardsupport$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_boardsupport$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_boardsupport$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_small_matrix$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_small_matrix$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_small_matrix$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_quick_trig$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_quick_trig$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_quick_trig$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_lsm330dlc_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_lsm330dlc_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_lsm330dlc_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_adxl345_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_adxl345_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_adxl345_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_compass_hmc5883l$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_compass_hmc5883l$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_compass_hmc5883l$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_bmp085$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_bmp085$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_bmp085$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_time_keeper$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_time_keeper$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_time_keeper$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_uart_int$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_uart_int$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_uart_int$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_amplifiers$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_amplifiers$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_amplifiers$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_module_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_module_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_radar_module_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_led$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_led$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_led$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_spi_buffered$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_spi_buffered$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_spi_buffered$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_itg3200_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_itg3200_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_itg3200_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_adc_int$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_adc_int$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_adc_int$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_dac_dma$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_dac_dma$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_dac_dma$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_ads1274$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_ads1274$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_ads1274$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_servo_pwm$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_servo_pwm$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_servo_pwm$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_analog_monitor$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_analog_monitor$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_analog_monitor$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_remote_dsm2$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_remote_dsm2$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_remote_dsm2$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_i2c_driver_int$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_i2c_driver_int$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_i2c_driver_int$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_linear_algebra$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_linear_algebra$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_linear_algebra$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_buffer$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_buffer$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_kalman$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_kalman$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_kalman$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_generator$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_generator$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_generator$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/user_board_init$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/user_board_init$(DependSuffix)
	$(RM) $(IntermediateDirectory)/user_board_init$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/stdio_read$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/stdio_read$(DependSuffix)
	$(RM) $(IntermediateDirectory)/stdio_read$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/stdio_write$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/stdio_write$(DependSuffix)
	$(RM) $(IntermediateDirectory)/stdio_write$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pm_power_clocks_lib$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pm_power_clocks_lib$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pm_power_clocks_lib$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pm_pm_uc3c$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pm_pm_uc3c$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pm_pm_uc3c$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/adcifa_adcifa$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/adcifa_adcifa$(DependSuffix)
	$(RM) $(IntermediateDirectory)/adcifa_adcifa$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/usart_usart$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/usart_usart$(DependSuffix)
	$(RM) $(IntermediateDirectory)/usart_usart$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/twim_twim$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/twim_twim$(DependSuffix)
	$(RM) $(IntermediateDirectory)/twim_twim$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/intc_intc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/intc_intc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/intc_intc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/intc_exception$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/intc_exception$(DependSuffix)
	$(RM) $(IntermediateDirectory)/intc_exception$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/ast_ast$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/ast_ast$(DependSuffix)
	$(RM) $(IntermediateDirectory)/ast_ast$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pevc_pevc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pevc_pevc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pevc_pevc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tc_tc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tc_tc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tc_tc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/spi_spi$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/spi_spi$(DependSuffix)
	$(RM) $(IntermediateDirectory)/spi_spi$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pdca_pdca$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pdca_pdca$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pdca_pdca$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/eic_eic$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/eic_eic$(DependSuffix)
	$(RM) $(IntermediateDirectory)/eic_eic$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/dacifb_dacifb$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/dacifb_dacifb$(DependSuffix)
	$(RM) $(IntermediateDirectory)/dacifb_dacifb$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/pwm_pwm4$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/pwm_pwm4$(DependSuffix)
	$(RM) $(IntermediateDirectory)/pwm_pwm4$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/flashc_flashc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/flashc_flashc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/flashc_flashc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/gpio_gpio$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/gpio_gpio$(DependSuffix)
	$(RM) $(IntermediateDirectory)/gpio_gpio$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/scif_scif_uc3c$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/scif_scif_uc3c$(DependSuffix)
	$(RM) $(IntermediateDirectory)/scif_scif_uc3c$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/usbc_usbc_device$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/usbc_usbc_device$(DependSuffix)
	$(RM) $(IntermediateDirectory)/usbc_usbc_device$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/startup_startup_uc3$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/startup_startup_uc3$(DependSuffix)
	$(RM) $(IntermediateDirectory)/startup_startup_uc3$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/startup_trampoline_uc3$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/startup_trampoline_uc3$(DependSuffix)
	$(RM) $(IntermediateDirectory)/startup_trampoline_uc3$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/delay_delay$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/delay_delay$(DependSuffix)
	$(RM) $(IntermediateDirectory)/delay_delay$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/stdio_usb_stdio_usb$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/stdio_usb_stdio_usb$(DependSuffix)
	$(RM) $(IntermediateDirectory)/stdio_usb_stdio_usb$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/uc3_sleepmgr$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/uc3_sleepmgr$(DependSuffix)
	$(RM) $(IntermediateDirectory)/uc3_sleepmgr$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/uc3_spi_spi_master$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/uc3_spi_spi_master$(DependSuffix)
	$(RM) $(IntermediateDirectory)/uc3_spi_spi_master$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_pll$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_pll$(DependSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_pll$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_osc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_osc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_osc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_sysclk$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_sysclk$(DependSuffix)
	$(RM) $(IntermediateDirectory)/uc3c_sysclk$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/udc_udc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/udc_udc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/udc_udc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc_desc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc_desc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/device_udi_cdc_desc$(PreprocessSuffix)
	$(RM) $(OutputFile)
	$(RM) ".build-debug/Maveric_myCopter_linux"


