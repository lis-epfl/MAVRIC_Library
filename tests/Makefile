##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Library_tests
ConfigurationName      :=Debug
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
WorkspacePath          := ".."
ProjectPath            := "."
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=felix
Date                   :=02/14/2014
CodeLitePath           :="~/.codelite"
LinkerName             :=gcc
ArchiveTool            :=ar rcus
SharedObjectLinkerName :=gcc -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.o.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
CompilerName           :=gcc
C_CompilerName         :=gcc
OutputFile             :=$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
MakeDirCommand         :=mkdir -p
CmpOptions             := -g -std=c99 -lm $(Preprocessors)
LinkOptions            :=  -lm
IncludePath            :=  "$(IncludeSwitch)." "$(IncludeSwitch)." "$(IncludeSwitch)../util" "$(IncludeSwitch)../communication" "$(IncludeSwitch)../control" "$(IncludeSwitch)../hal_emu" "$(IncludeSwitch)../mavlink" "$(IncludeSwitch)../runtime" "$(IncludeSwitch)../sensing" "$(IncludeSwitch)config" "$(IncludeSwitch).." 
RcIncludePath          :=
Libs                   :=
LibPath                := "$(LibraryPathSwitch)." 


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects=$(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IntermediateDirectory)/util_quick_trig$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_joystick$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) \
	$(IntermediateDirectory)/test_maths$(ObjectSuffix) $(IntermediateDirectory)/main$(ObjectSuffix) $(IntermediateDirectory)/config_central_data$(ObjectSuffix) $(IntermediateDirectory)/config_boardsupport$(ObjectSuffix) $(IntermediateDirectory)/test_small_matrix$(ObjectSuffix) $(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) \
	$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) 

##
## Main Build Targets 
##
all: $(OutputFile)

$(OutputFile): makeDirStep $(Objects)
	@$(MakeDirCommand) $(@D)
	$(LinkerName) $(OutputSwitch)$(OutputFile) $(Objects) $(LibPath) $(Libs) $(LinkOptions)

makeDirStep:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/util_buffer$(ObjectSuffix): ../util/buffer.c $(IntermediateDirectory)/util_buffer$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/buffer.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_buffer$(DependSuffix): ../util/buffer.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_buffer$(ObjectSuffix) -MF$(IntermediateDirectory)/util_buffer$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/buffer.c"

$(IntermediateDirectory)/util_buffer$(PreprocessSuffix): ../util/buffer.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/buffer.c"

$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix): ../util/coord_conventions.c $(IntermediateDirectory)/util_coord_conventions$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_coord_conventions$(DependSuffix): ../util/coord_conventions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) -MF$(IntermediateDirectory)/util_coord_conventions$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix): ../util/coord_conventions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_sinus$(ObjectSuffix): ../util/sinus.c $(IntermediateDirectory)/util_sinus$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/sinus.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_sinus$(DependSuffix): ../util/sinus.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_sinus$(ObjectSuffix) -MF$(IntermediateDirectory)/util_sinus$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/sinus.c"

$(IntermediateDirectory)/util_sinus$(PreprocessSuffix): ../util/sinus.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/sinus.c"

$(IntermediateDirectory)/util_print_util$(ObjectSuffix): ../util/print_util.c $(IntermediateDirectory)/util_print_util$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/print_util.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_print_util$(DependSuffix): ../util/print_util.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_print_util$(ObjectSuffix) -MF$(IntermediateDirectory)/util_print_util$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/print_util.c"

$(IntermediateDirectory)/util_print_util$(PreprocessSuffix): ../util/print_util.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/print_util.c"

$(IntermediateDirectory)/util_quick_trig$(ObjectSuffix): ../util/quick_trig.c $(IntermediateDirectory)/util_quick_trig$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/quick_trig.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_quick_trig$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_quick_trig$(DependSuffix): ../util/quick_trig.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_quick_trig$(ObjectSuffix) -MF$(IntermediateDirectory)/util_quick_trig$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/quick_trig.c"

$(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix): ../util/quick_trig.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/quick_trig.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix): ../hal_emu/radar_module_driver.c $(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix): ../hal_emu/radar_module_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix): ../hal_emu/radar_module_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix): ../hal_emu/analog_monitor.c $(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/analog_monitor.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix): ../hal_emu/analog_monitor.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/analog_monitor.c"

$(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix): ../hal_emu/analog_monitor.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/analog_monitor.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix): ../hal_emu/time_keeper.c $(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix): ../hal_emu/time_keeper.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix): ../hal_emu/time_keeper.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix): ../hal_emu/bmp085.c $(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix): ../hal_emu/bmp085.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix): ../hal_emu/bmp085.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix): ../hal_emu/servo_pwm.c $(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix): ../hal_emu/servo_pwm.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix): ../hal_emu/servo_pwm.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix): ../hal_emu/compass_hmc5883l.c $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix): ../hal_emu/compass_hmc5883l.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix): ../hal_emu/compass_hmc5883l.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix): ../hal_emu/itg3200.c $(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix): ../hal_emu/itg3200.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix): ../hal_emu/itg3200.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200.c"

$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix): ../hal_emu/led.c $(IntermediateDirectory)/hal_emu_led$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_led$(DependSuffix): ../hal_emu/led.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_led$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix): ../hal_emu/led.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix): ../hal_emu/lsm330dlc_driver.c $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/lsm330dlc_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix): ../hal_emu/lsm330dlc_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/lsm330dlc_driver.c"

$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix): ../hal_emu/lsm330dlc_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/lsm330dlc_driver.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix): ../hal_emu/adxl345_driver.c $(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix): ../hal_emu/adxl345_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix): ../hal_emu/adxl345_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix): ../hal_emu/joystick_rc.c $(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix): ../hal_emu/joystick_rc.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix): ../hal_emu/joystick_rc.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/hal_emu_joystick$(ObjectSuffix): ../hal_emu/joystick.c $(IntermediateDirectory)/hal_emu_joystick$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_joystick$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_joystick$(DependSuffix): ../hal_emu/joystick.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_joystick$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_joystick$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick.c"

$(IntermediateDirectory)/hal_emu_joystick$(PreprocessSuffix): ../hal_emu/joystick.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_joystick$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix): ../hal_emu/udp_stream.c $(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix): ../hal_emu/udp_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix): ../hal_emu/udp_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix): ../communication/mavlink_stream.c $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix): ../communication/mavlink_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix): ../communication/mavlink_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix): ../communication/onboard_parameters.c $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix): ../communication/onboard_parameters.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix): ../communication/onboard_parameters.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/test_maths$(ObjectSuffix): test_maths.c $(IntermediateDirectory)/test_maths$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/test_maths$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/test_maths$(DependSuffix): test_maths.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/test_maths$(ObjectSuffix) -MF$(IntermediateDirectory)/test_maths$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c"

$(IntermediateDirectory)/test_maths$(PreprocessSuffix): test_maths.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/test_maths$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c"

$(IntermediateDirectory)/main$(ObjectSuffix): main.c $(IntermediateDirectory)/main$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/main.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/main$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main$(DependSuffix): main.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/main$(ObjectSuffix) -MF$(IntermediateDirectory)/main$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/main.c"

$(IntermediateDirectory)/main$(PreprocessSuffix): main.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/main.c"

$(IntermediateDirectory)/config_central_data$(ObjectSuffix): config/central_data.c $(IntermediateDirectory)/config_central_data$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/config/central_data.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/config_central_data$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/config_central_data$(DependSuffix): config/central_data.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/config_central_data$(ObjectSuffix) -MF$(IntermediateDirectory)/config_central_data$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/config/central_data.c"

$(IntermediateDirectory)/config_central_data$(PreprocessSuffix): config/central_data.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/config_central_data$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/config/central_data.c"

$(IntermediateDirectory)/config_boardsupport$(ObjectSuffix): config/boardsupport.c $(IntermediateDirectory)/config_boardsupport$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/config/boardsupport.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/config_boardsupport$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/config_boardsupport$(DependSuffix): config/boardsupport.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/config_boardsupport$(ObjectSuffix) -MF$(IntermediateDirectory)/config_boardsupport$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/config/boardsupport.c"

$(IntermediateDirectory)/config_boardsupport$(PreprocessSuffix): config/boardsupport.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/config_boardsupport$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/config/boardsupport.c"

$(IntermediateDirectory)/test_small_matrix$(ObjectSuffix): test_small_matrix.c $(IntermediateDirectory)/test_small_matrix$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/test_small_matrix.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/test_small_matrix$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/test_small_matrix$(DependSuffix): test_small_matrix.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/test_small_matrix$(ObjectSuffix) -MF$(IntermediateDirectory)/test_small_matrix$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/test_small_matrix.c"

$(IntermediateDirectory)/test_small_matrix$(PreprocessSuffix): test_small_matrix.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/test_small_matrix$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/test_small_matrix.c"

$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix): ../sensing/qfilter.c $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_qfilter$(DependSuffix): ../sensing/qfilter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_qfilter$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix): ../sensing/qfilter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix): ../sensing/position_estimation.c $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix): ../sensing/position_estimation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix): ../sensing/position_estimation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix): ../sensing/simulation.c $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_simulation$(DependSuffix): ../sensing/simulation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_simulation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix): ../sensing/simulation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_imu$(ObjectSuffix): ../sensing/imu.c $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/imu.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_imu$(DependSuffix): ../sensing/imu.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_imu$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_imu$(PreprocessSuffix): ../sensing/imu.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix): ../sensing/gps_ublox.c $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix): ../sensing/gps_ublox.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix): ../sensing/gps_ublox.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix): ../control/stabilisation_copter.c $(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_copter.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix): ../control/stabilisation_copter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix): ../control/stabilisation_copter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix): ../control/adaptive_parameter.c $(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/adaptive_parameter.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix): ../control/adaptive_parameter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) -MF$(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix): ../control/adaptive_parameter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix): ../control/stabilisation.c $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation$(DependSuffix): ../control/stabilisation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix): ../control/stabilisation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix): ../control/stabilisation_hybrid.c $(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_hybrid.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix): ../control/stabilisation_hybrid.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/stabilisation_hybrid.c"

$(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix): ../control/stabilisation_hybrid.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_hybrid.c"

$(IntermediateDirectory)/control_pid_control$(ObjectSuffix): ../control/pid_control.c $(IntermediateDirectory)/control_pid_control$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/pid_control.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_pid_control$(DependSuffix): ../control/pid_control.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_pid_control$(ObjectSuffix) -MF$(IntermediateDirectory)/control_pid_control$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/pid_control.c"

$(IntermediateDirectory)/control_pid_control$(PreprocessSuffix): ../control/pid_control.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/pid_control.c"

$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix): ../runtime/scheduler.c $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/runtime_scheduler$(DependSuffix): ../runtime/scheduler.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) -MF$(IntermediateDirectory)/runtime_scheduler$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c"

$(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix): ../runtime/scheduler.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) $(IntermediateDirectory)/util_buffer$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_buffer$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(DependSuffix)
	$(RM) $(IntermediateDirectory)/util_quick_trig$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/test_maths$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/test_maths$(DependSuffix)
	$(RM) $(IntermediateDirectory)/test_maths$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/main$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/main$(DependSuffix)
	$(RM) $(IntermediateDirectory)/main$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/config_central_data$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/config_central_data$(DependSuffix)
	$(RM) $(IntermediateDirectory)/config_central_data$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/config_boardsupport$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/config_boardsupport$(DependSuffix)
	$(RM) $(IntermediateDirectory)/config_boardsupport$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/test_small_matrix$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/test_small_matrix$(DependSuffix)
	$(RM) $(IntermediateDirectory)/test_small_matrix$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix)
	$(RM) $(OutputFile)


