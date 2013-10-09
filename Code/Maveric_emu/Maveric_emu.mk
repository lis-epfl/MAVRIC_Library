##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Maveric_emu
ConfigurationName      :=Debug
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
WorkspacePath          := "/home/felix/Projects/maveric/Code/Maveric_emu"
ProjectPath            := "/home/felix/Projects/maveric/Code/Maveric_emu"
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=felix
Date                   :=10/09/2013
CodeLitePath           :="/home/felix/.codelite"
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
OutputFile             :=maveric
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
MakeDirCommand         :=mkdir -p
CmpOptions             := -g -O0 -Wall -std=gnu99 $(Preprocessors)
LinkOptions            :=  -lm
IncludePath            :=  "$(IncludeSwitch)." "$(IncludeSwitch)../Library" "$(IncludeSwitch)../Library/communication" "$(IncludeSwitch)../Library/control" "$(IncludeSwitch)../Library/hal_emu" "$(IncludeSwitch)../Library/mavlink" "$(IncludeSwitch)../Library/mavlink/include/" "$(IncludeSwitch)../Library/runtime" "$(IncludeSwitch)../Library/sensing" "$(IncludeSwitch)../Library/util" "$(IncludeSwitch)../Library/tests" "$(IncludeSwitch)./config" "$(IncludeSwitch)./Debug" 
RcIncludePath          :=
Libs                   :=
LibPath                := "$(LibraryPathSwitch)." "$(LibraryPathSwitch)./Debug" 


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects=$(IntermediateDirectory)/boardsupport$(ObjectSuffix) $(IntermediateDirectory)/main$(ObjectSuffix) $(IntermediateDirectory)/mavlink_actions$(ObjectSuffix) $(IntermediateDirectory)/tasks$(ObjectSuffix) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix) \
	$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IntermediateDirectory)/sensing_waypoint_navigation$(ObjectSuffix) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IntermediateDirectory)/sensing_estimator$(ObjectSuffix) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IntermediateDirectory)/control_control$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) \
	$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) 

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
$(IntermediateDirectory)/boardsupport$(ObjectSuffix): boardsupport.c $(IntermediateDirectory)/boardsupport$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/boardsupport.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/boardsupport$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/boardsupport$(DependSuffix): boardsupport.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/boardsupport$(ObjectSuffix) -MF$(IntermediateDirectory)/boardsupport$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Maveric_emu/boardsupport.c"

$(IntermediateDirectory)/boardsupport$(PreprocessSuffix): boardsupport.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/boardsupport$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Maveric_emu/boardsupport.c"

$(IntermediateDirectory)/main$(ObjectSuffix): main.c $(IntermediateDirectory)/main$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/main.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/main$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main$(DependSuffix): main.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/main$(ObjectSuffix) -MF$(IntermediateDirectory)/main$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Maveric_emu/main.c"

$(IntermediateDirectory)/main$(PreprocessSuffix): main.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Maveric_emu/main.c"

$(IntermediateDirectory)/mavlink_actions$(ObjectSuffix): mavlink_actions.c $(IntermediateDirectory)/mavlink_actions$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/mavlink_actions.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/mavlink_actions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/mavlink_actions$(DependSuffix): mavlink_actions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/mavlink_actions$(ObjectSuffix) -MF$(IntermediateDirectory)/mavlink_actions$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Maveric_emu/mavlink_actions.c"

$(IntermediateDirectory)/mavlink_actions$(PreprocessSuffix): mavlink_actions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/mavlink_actions$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Maveric_emu/mavlink_actions.c"

$(IntermediateDirectory)/tasks$(ObjectSuffix): tasks.c $(IntermediateDirectory)/tasks$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/tasks.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/tasks$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tasks$(DependSuffix): tasks.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/tasks$(ObjectSuffix) -MF$(IntermediateDirectory)/tasks$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Maveric_emu/tasks.c"

$(IntermediateDirectory)/tasks$(PreprocessSuffix): tasks.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tasks$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Maveric_emu/tasks.c"

$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix): ../Library/runtime/scheduler.c $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/runtime_scheduler$(DependSuffix): ../Library/runtime/scheduler.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) -MF$(IntermediateDirectory)/runtime_scheduler$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c"

$(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix): ../Library/runtime/scheduler.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c"

$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix): ../Library/util/coord_conventions.c $(IntermediateDirectory)/util_coord_conventions$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_coord_conventions$(DependSuffix): ../Library/util/coord_conventions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) -MF$(IntermediateDirectory)/util_coord_conventions$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix): ../Library/util/coord_conventions.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_sinus$(ObjectSuffix): ../Library/util/sinus.c $(IntermediateDirectory)/util_sinus$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/sinus.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_sinus$(DependSuffix): ../Library/util/sinus.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_sinus$(ObjectSuffix) -MF$(IntermediateDirectory)/util_sinus$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/sinus.c"

$(IntermediateDirectory)/util_sinus$(PreprocessSuffix): ../Library/util/sinus.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/sinus.c"

$(IntermediateDirectory)/util_print_util$(ObjectSuffix): ../Library/util/print_util.c $(IntermediateDirectory)/util_print_util$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/print_util.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_print_util$(DependSuffix): ../Library/util/print_util.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_print_util$(ObjectSuffix) -MF$(IntermediateDirectory)/util_print_util$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/print_util.c"

$(IntermediateDirectory)/util_print_util$(PreprocessSuffix): ../Library/util/print_util.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/print_util.c"

$(IntermediateDirectory)/util_buffer$(ObjectSuffix): ../Library/util/buffer.c $(IntermediateDirectory)/util_buffer$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/buffer.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_buffer$(DependSuffix): ../Library/util/buffer.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/util_buffer$(ObjectSuffix) -MF$(IntermediateDirectory)/util_buffer$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/util/buffer.c"

$(IntermediateDirectory)/util_buffer$(PreprocessSuffix): ../Library/util/buffer.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/util/buffer.c"

$(IntermediateDirectory)/sensing_imu$(ObjectSuffix): ../Library/sensing/imu.c $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/imu.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_imu$(DependSuffix): ../Library/sensing/imu.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_imu$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_imu$(PreprocessSuffix): ../Library/sensing/imu.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix): ../Library/sensing/qfilter.c $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_qfilter$(DependSuffix): ../Library/sensing/qfilter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_qfilter$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix): ../Library/sensing/qfilter.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix): ../Library/sensing/gps_ublox.c $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix): ../Library/sensing/gps_ublox.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix): ../Library/sensing/gps_ublox.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_waypoint_navigation$(ObjectSuffix): ../Library/sensing/waypoint_navigation.c $(IntermediateDirectory)/sensing_waypoint_navigation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/waypoint_navigation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_waypoint_navigation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_waypoint_navigation$(DependSuffix): ../Library/sensing/waypoint_navigation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_waypoint_navigation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_waypoint_navigation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/waypoint_navigation.c"

$(IntermediateDirectory)/sensing_waypoint_navigation$(PreprocessSuffix): ../Library/sensing/waypoint_navigation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_waypoint_navigation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/waypoint_navigation.c"

$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix): ../Library/sensing/simulation.c $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_simulation$(DependSuffix): ../Library/sensing/simulation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_simulation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix): ../Library/sensing/simulation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_estimator$(ObjectSuffix): ../Library/sensing/estimator.c $(IntermediateDirectory)/sensing_estimator$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/estimator.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_estimator$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_estimator$(DependSuffix): ../Library/sensing/estimator.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_estimator$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_estimator$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/estimator.c"

$(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix): ../Library/sensing/estimator.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/estimator.c"

$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix): ../Library/sensing/position_estimation.c $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix): ../Library/sensing/position_estimation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix): ../Library/sensing/position_estimation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c"

$(IntermediateDirectory)/control_navigation$(ObjectSuffix): ../Library/control/navigation.c $(IntermediateDirectory)/control_navigation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/navigation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_navigation$(DependSuffix): ../Library/control/navigation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_navigation$(ObjectSuffix) -MF$(IntermediateDirectory)/control_navigation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/navigation.c"

$(IntermediateDirectory)/control_navigation$(PreprocessSuffix): ../Library/control/navigation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/navigation.c"

$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix): ../Library/control/stabilisation.c $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation$(DependSuffix): ../Library/control/stabilisation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix): ../Library/control/stabilisation.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c"

$(IntermediateDirectory)/control_control$(ObjectSuffix): ../Library/control/control.c $(IntermediateDirectory)/control_control$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/control.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/control_control$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_control$(DependSuffix): ../Library/control/control.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/control_control$(ObjectSuffix) -MF$(IntermediateDirectory)/control_control$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/control/control.c"

$(IntermediateDirectory)/control_control$(PreprocessSuffix): ../Library/control/control.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_control$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/control/control.c"

$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix): ../Library/communication/mavlink_stream.c $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix): ../Library/communication/mavlink_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix): ../Library/communication/mavlink_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix): ../Library/communication/onboard_parameters.c $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix): ../Library/communication/onboard_parameters.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix): ../Library/communication/onboard_parameters.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix): ../Library/hal_emu/adxl345_driver.c $(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix): ../Library/hal_emu/adxl345_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix): ../Library/hal_emu/adxl345_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix): ../Library/hal_emu/compass_hmc5883l.c $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix): ../Library/hal_emu/compass_hmc5883l.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix): ../Library/hal_emu/compass_hmc5883l.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix): ../Library/hal_emu/bmp085.c $(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix): ../Library/hal_emu/bmp085.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix): ../Library/hal_emu/bmp085.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix): ../Library/hal_emu/radar_module_driver.c $(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix): ../Library/hal_emu/radar_module_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix): ../Library/hal_emu/radar_module_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix): ../Library/hal_emu/servo_pwm.c $(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix): ../Library/hal_emu/servo_pwm.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix): ../Library/hal_emu/servo_pwm.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix): ../Library/hal_emu/led.c $(IntermediateDirectory)/hal_emu_led$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_led$(DependSuffix): ../Library/hal_emu/led.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_led$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix): ../Library/hal_emu/led.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix): ../Library/hal_emu/itg3200_driver.c $(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200_driver.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix): ../Library/hal_emu/itg3200_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200_driver.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix): ../Library/hal_emu/itg3200_driver.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200_driver.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix): ../Library/hal_emu/time_keeper.c $(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix): ../Library/hal_emu/time_keeper.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix): ../Library/hal_emu/time_keeper.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix): ../Library/hal_emu/udp_stream.c $(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix): ../Library/hal_emu/udp_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix): ../Library/hal_emu/udp_stream.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix): ../Library/hal_emu/joystick_rc.c $(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix): ../Library/hal_emu/joystick_rc.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix): ../Library/hal_emu/joystick_rc.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix): ../Library/tests/test_maths.c $(IntermediateDirectory)/tests_test_maths$(DependSuffix)
	$(C_CompilerName) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c" $(CmpOptions) $(ObjectSwitch)$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tests_test_maths$(DependSuffix): ../Library/tests/test_maths.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) -MT$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) -MF$(IntermediateDirectory)/tests_test_maths$(DependSuffix) -MM "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c"

$(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix): ../Library/tests/test_maths.c
	@$(C_CompilerName) $(CmpOptions) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix) "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) $(IntermediateDirectory)/boardsupport$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/boardsupport$(DependSuffix)
	$(RM) $(IntermediateDirectory)/boardsupport$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/main$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/main$(DependSuffix)
	$(RM) $(IntermediateDirectory)/main$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/mavlink_actions$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/mavlink_actions$(DependSuffix)
	$(RM) $(IntermediateDirectory)/mavlink_actions$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tasks$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tasks$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tasks$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(RM) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix)
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
	$(RM) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_waypoint_navigation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_waypoint_navigation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_waypoint_navigation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_estimator$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_control$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_control$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_control$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix)
	$(RM) $(OutputFile)


