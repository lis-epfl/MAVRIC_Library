##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=Maveric_emu
ConfigurationName      :=Debug
WorkspacePath          := "/home/felix/Projects/maveric/Code/Maveric_emu"
ProjectPath            := "/home/felix/Projects/maveric/Code/Maveric_emu"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=felix
Date                   :=04/15/14
CodeLitePath           :="/home/felix/.codelite"
LinkerName             :=gcc
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
OutputFile             :=maveric
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
ObjectsFileList        :="Maveric_emu.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -lm
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)../Library $(IncludeSwitch)../Library/communication $(IncludeSwitch)../Library/control $(IncludeSwitch)../Library/hal_emu $(IncludeSwitch)../Library/mavlink $(IncludeSwitch)../Library/mavlink/include/ $(IncludeSwitch)../Library/runtime $(IncludeSwitch)../Library/sensing $(IncludeSwitch)../Library/util $(IncludeSwitch)../Library/tests $(IncludeSwitch)./config $(IncludeSwitch)../Maveric_myCopter/src $(IncludeSwitch)./Debug 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)./Debug 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := ar rcus
CXX      := gcc
CC       := gcc
CXXFLAGS :=  -g -O0 -Wall -std=gnu99 $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall -std=gnu99 $(Preprocessors)
ASFLAGS  := 
AS       := as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/boardsupport$(ObjectSuffix) $(IntermediateDirectory)/main$(ObjectSuffix) $(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix) $(IntermediateDirectory)/src_tasks$(ObjectSuffix) $(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IntermediateDirectory)/sensing_imu$(ObjectSuffix) \
	$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IntermediateDirectory)/control_orca$(ObjectSuffix) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) \
	$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) \
	$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) $(IntermediateDirectory)/hal_emu_piezo_speaker$(ObjectSuffix) 

Objects1=$(IntermediateDirectory)/src_central_data$(ObjectSuffix) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) \
	



Objects=$(Objects0) $(Objects1) 

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
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/boardsupport$(ObjectSuffix): boardsupport.c $(IntermediateDirectory)/boardsupport$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/boardsupport.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/boardsupport$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/boardsupport$(DependSuffix): boardsupport.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/boardsupport$(ObjectSuffix) -MF$(IntermediateDirectory)/boardsupport$(DependSuffix) -MM "boardsupport.c"

$(IntermediateDirectory)/boardsupport$(PreprocessSuffix): boardsupport.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/boardsupport$(PreprocessSuffix) "boardsupport.c"

$(IntermediateDirectory)/main$(ObjectSuffix): main.c $(IntermediateDirectory)/main$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_emu/main.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main$(DependSuffix): main.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main$(ObjectSuffix) -MF$(IntermediateDirectory)/main$(DependSuffix) -MM "main.c"

$(IntermediateDirectory)/main$(PreprocessSuffix): main.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main$(PreprocessSuffix) "main.c"

$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix): ../Maveric_myCopter/src/mavlink_actions.c $(IntermediateDirectory)/src_mavlink_actions$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_myCopter/src/mavlink_actions.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_mavlink_actions$(DependSuffix): ../Maveric_myCopter/src/mavlink_actions.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix) -MF$(IntermediateDirectory)/src_mavlink_actions$(DependSuffix) -MM "../Maveric_myCopter/src/mavlink_actions.c"

$(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix): ../Maveric_myCopter/src/mavlink_actions.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix) "../Maveric_myCopter/src/mavlink_actions.c"

$(IntermediateDirectory)/src_tasks$(ObjectSuffix): ../Maveric_myCopter/src/tasks.c $(IntermediateDirectory)/src_tasks$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_myCopter/src/tasks.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tasks$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tasks$(DependSuffix): ../Maveric_myCopter/src/tasks.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tasks$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tasks$(DependSuffix) -MM "../Maveric_myCopter/src/tasks.c"

$(IntermediateDirectory)/src_tasks$(PreprocessSuffix): ../Maveric_myCopter/src/tasks.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tasks$(PreprocessSuffix) "../Maveric_myCopter/src/tasks.c"

$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix): ../Library/runtime/scheduler.c $(IntermediateDirectory)/runtime_scheduler$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/runtime/scheduler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/runtime_scheduler$(DependSuffix): ../Library/runtime/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/runtime_scheduler$(ObjectSuffix) -MF$(IntermediateDirectory)/runtime_scheduler$(DependSuffix) -MM "../Library/runtime/scheduler.c"

$(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix): ../Library/runtime/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/runtime_scheduler$(PreprocessSuffix) "../Library/runtime/scheduler.c"

$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix): ../Library/util/coord_conventions.c $(IntermediateDirectory)/util_coord_conventions$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/coord_conventions.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_coord_conventions$(DependSuffix): ../Library/util/coord_conventions.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_coord_conventions$(ObjectSuffix) -MF$(IntermediateDirectory)/util_coord_conventions$(DependSuffix) -MM "../Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix): ../Library/util/coord_conventions.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_coord_conventions$(PreprocessSuffix) "../Library/util/coord_conventions.c"

$(IntermediateDirectory)/util_sinus$(ObjectSuffix): ../Library/util/sinus.c $(IntermediateDirectory)/util_sinus$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/sinus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_sinus$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_sinus$(DependSuffix): ../Library/util/sinus.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_sinus$(ObjectSuffix) -MF$(IntermediateDirectory)/util_sinus$(DependSuffix) -MM "../Library/util/sinus.c"

$(IntermediateDirectory)/util_sinus$(PreprocessSuffix): ../Library/util/sinus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_sinus$(PreprocessSuffix) "../Library/util/sinus.c"

$(IntermediateDirectory)/util_print_util$(ObjectSuffix): ../Library/util/print_util.c $(IntermediateDirectory)/util_print_util$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/print_util.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_print_util$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_print_util$(DependSuffix): ../Library/util/print_util.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_print_util$(ObjectSuffix) -MF$(IntermediateDirectory)/util_print_util$(DependSuffix) -MM "../Library/util/print_util.c"

$(IntermediateDirectory)/util_print_util$(PreprocessSuffix): ../Library/util/print_util.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_print_util$(PreprocessSuffix) "../Library/util/print_util.c"

$(IntermediateDirectory)/util_buffer$(ObjectSuffix): ../Library/util/buffer.c $(IntermediateDirectory)/util_buffer$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/util/buffer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_buffer$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_buffer$(DependSuffix): ../Library/util/buffer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_buffer$(ObjectSuffix) -MF$(IntermediateDirectory)/util_buffer$(DependSuffix) -MM "../Library/util/buffer.c"

$(IntermediateDirectory)/util_buffer$(PreprocessSuffix): ../Library/util/buffer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_buffer$(PreprocessSuffix) "../Library/util/buffer.c"

$(IntermediateDirectory)/sensing_imu$(ObjectSuffix): ../Library/sensing/imu.c $(IntermediateDirectory)/sensing_imu$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/imu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_imu$(DependSuffix): ../Library/sensing/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_imu$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_imu$(DependSuffix) -MM "../Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_imu$(PreprocessSuffix): ../Library/sensing/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_imu$(PreprocessSuffix) "../Library/sensing/imu.c"

$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix): ../Library/sensing/qfilter.c $(IntermediateDirectory)/sensing_qfilter$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/qfilter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_qfilter$(DependSuffix): ../Library/sensing/qfilter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_qfilter$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_qfilter$(DependSuffix) -MM "../Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix): ../Library/sensing/qfilter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_qfilter$(PreprocessSuffix) "../Library/sensing/qfilter.c"

$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix): ../Library/sensing/gps_ublox.c $(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/gps_ublox.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix): ../Library/sensing/gps_ublox.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_gps_ublox$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_gps_ublox$(DependSuffix) -MM "../Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix): ../Library/sensing/gps_ublox.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_gps_ublox$(PreprocessSuffix) "../Library/sensing/gps_ublox.c"

$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix): ../Library/sensing/simulation.c $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/simulation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_simulation$(DependSuffix): ../Library/sensing/simulation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_simulation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_simulation$(DependSuffix) -MM "../Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix): ../Library/sensing/simulation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix) "../Library/sensing/simulation.c"

$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix): ../Library/sensing/position_estimation.c $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/position_estimation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix): ../Library/sensing/position_estimation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_position_estimation$(DependSuffix) -MM "../Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix): ../Library/sensing/position_estimation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix) "../Library/sensing/position_estimation.c"

$(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix): ../Library/sensing/neighbor_selection.c $(IntermediateDirectory)/sensing_neighbor_selection$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/sensing/neighbor_selection.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensing_neighbor_selection$(DependSuffix): ../Library/sensing/neighbor_selection.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix) -MF$(IntermediateDirectory)/sensing_neighbor_selection$(DependSuffix) -MM "../Library/sensing/neighbor_selection.c"

$(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix): ../Library/sensing/neighbor_selection.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix) "../Library/sensing/neighbor_selection.c"

$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix): ../Library/control/adaptive_parameter.c $(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/adaptive_parameter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix): ../Library/control/adaptive_parameter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix) -MF$(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix) -MM "../Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix): ../Library/control/adaptive_parameter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix) "../Library/control/adaptive_parameter.c"

$(IntermediateDirectory)/control_navigation$(ObjectSuffix): ../Library/control/navigation.c $(IntermediateDirectory)/control_navigation$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/navigation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_navigation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_navigation$(DependSuffix): ../Library/control/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_navigation$(ObjectSuffix) -MF$(IntermediateDirectory)/control_navigation$(DependSuffix) -MM "../Library/control/navigation.c"

$(IntermediateDirectory)/control_navigation$(PreprocessSuffix): ../Library/control/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix) "../Library/control/navigation.c"

$(IntermediateDirectory)/control_orca$(ObjectSuffix): ../Library/control/orca.c $(IntermediateDirectory)/control_orca$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/orca.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_orca$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_orca$(DependSuffix): ../Library/control/orca.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_orca$(ObjectSuffix) -MF$(IntermediateDirectory)/control_orca$(DependSuffix) -MM "../Library/control/orca.c"

$(IntermediateDirectory)/control_orca$(PreprocessSuffix): ../Library/control/orca.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_orca$(PreprocessSuffix) "../Library/control/orca.c"

$(IntermediateDirectory)/control_pid_control$(ObjectSuffix): ../Library/control/pid_control.c $(IntermediateDirectory)/control_pid_control$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/pid_control.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_pid_control$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_pid_control$(DependSuffix): ../Library/control/pid_control.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_pid_control$(ObjectSuffix) -MF$(IntermediateDirectory)/control_pid_control$(DependSuffix) -MM "../Library/control/pid_control.c"

$(IntermediateDirectory)/control_pid_control$(PreprocessSuffix): ../Library/control/pid_control.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix) "../Library/control/pid_control.c"

$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix): ../Library/control/stabilisation.c $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation$(DependSuffix): ../Library/control/stabilisation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_stabilisation$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation$(DependSuffix) -MM "../Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix): ../Library/control/stabilisation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix) "../Library/control/stabilisation.c"

$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix): ../Library/control/stabilisation_copter.c $(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_copter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix): ../Library/control/stabilisation_copter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix) -MM "../Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix): ../Library/control/stabilisation_copter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix) "../Library/control/stabilisation_copter.c"

$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix): ../Library/control/stabilisation_hybrid.c $(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/control/stabilisation_hybrid.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix): ../Library/control/stabilisation_hybrid.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix) -MF$(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix) -MM "../Library/control/stabilisation_hybrid.c"

$(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix): ../Library/control/stabilisation_hybrid.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix) "../Library/control/stabilisation_hybrid.c"

$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix): ../Library/communication/mavlink_stream.c $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_stream.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix): ../Library/communication/mavlink_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix) -MM "../Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix): ../Library/communication/mavlink_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix) "../Library/communication/mavlink_stream.c"

$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix): ../Library/communication/onboard_parameters.c $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/onboard_parameters.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix): ../Library/communication/onboard_parameters.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix) -MM "../Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix): ../Library/communication/onboard_parameters.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix) "../Library/communication/onboard_parameters.c"

$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix): ../Library/communication/mavlink_waypoint_handler.c $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/communication/mavlink_waypoint_handler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(DependSuffix): ../Library/communication/mavlink_waypoint_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix) -MF$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(DependSuffix) -MM "../Library/communication/mavlink_waypoint_handler.c"

$(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix): ../Library/communication/mavlink_waypoint_handler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix) "../Library/communication/mavlink_waypoint_handler.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix): ../Library/hal_emu/adxl345_driver.c $(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/adxl345_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix): ../Library/hal_emu/adxl345_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_adxl345_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_adxl345_driver$(DependSuffix) -MM "../Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix): ../Library/hal_emu/adxl345_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_adxl345_driver$(PreprocessSuffix) "../Library/hal_emu/adxl345_driver.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix): ../Library/hal_emu/compass_hmc5883l.c $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/compass_hmc5883l.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix): ../Library/hal_emu/compass_hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(DependSuffix) -MM "../Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix): ../Library/hal_emu/compass_hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_compass_hmc5883l$(PreprocessSuffix) "../Library/hal_emu/compass_hmc5883l.c"

$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix): ../Library/hal_emu/bmp085.c $(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/bmp085.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix): ../Library/hal_emu/bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_bmp085$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_bmp085$(DependSuffix) -MM "../Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix): ../Library/hal_emu/bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_bmp085$(PreprocessSuffix) "../Library/hal_emu/bmp085.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix): ../Library/hal_emu/radar_module_driver.c $(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/radar_module_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix): ../Library/hal_emu/radar_module_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_radar_module_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_radar_module_driver$(DependSuffix) -MM "../Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix): ../Library/hal_emu/radar_module_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_radar_module_driver$(PreprocessSuffix) "../Library/hal_emu/radar_module_driver.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix): ../Library/hal_emu/servo_pwm.c $(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/servo_pwm.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix): ../Library/hal_emu/servo_pwm.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_servo_pwm$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_servo_pwm$(DependSuffix) -MM "../Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix): ../Library/hal_emu/servo_pwm.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_servo_pwm$(PreprocessSuffix) "../Library/hal_emu/servo_pwm.c"

$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix): ../Library/hal_emu/led.c $(IntermediateDirectory)/hal_emu_led$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/led.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_led$(DependSuffix): ../Library/hal_emu/led.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_led$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_led$(DependSuffix) -MM "../Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix): ../Library/hal_emu/led.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_led$(PreprocessSuffix) "../Library/hal_emu/led.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix): ../Library/hal_emu/itg3200_driver.c $(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/itg3200_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix): ../Library/hal_emu/itg3200_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_itg3200_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_itg3200_driver$(DependSuffix) -MM "../Library/hal_emu/itg3200_driver.c"

$(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix): ../Library/hal_emu/itg3200_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_itg3200_driver$(PreprocessSuffix) "../Library/hal_emu/itg3200_driver.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix): ../Library/hal_emu/time_keeper.c $(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/time_keeper.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix): ../Library/hal_emu/time_keeper.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_time_keeper$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_time_keeper$(DependSuffix) -MM "../Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix): ../Library/hal_emu/time_keeper.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_time_keeper$(PreprocessSuffix) "../Library/hal_emu/time_keeper.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix): ../Library/hal_emu/udp_stream.c $(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/udp_stream.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix): ../Library/hal_emu/udp_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_udp_stream$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_udp_stream$(DependSuffix) -MM "../Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix): ../Library/hal_emu/udp_stream.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_udp_stream$(PreprocessSuffix) "../Library/hal_emu/udp_stream.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix): ../Library/hal_emu/joystick_rc.c $(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/joystick_rc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix): ../Library/hal_emu/joystick_rc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_joystick_rc$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_joystick_rc$(DependSuffix) -MM "../Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix): ../Library/hal_emu/joystick_rc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_joystick_rc$(PreprocessSuffix) "../Library/hal_emu/joystick_rc.c"

$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix): ../Library/hal_emu/analog_monitor.c $(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/analog_monitor.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix): ../Library/hal_emu/analog_monitor.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix) -MM "../Library/hal_emu/analog_monitor.c"

$(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix): ../Library/hal_emu/analog_monitor.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix) "../Library/hal_emu/analog_monitor.c"

$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix): ../Library/hal_emu/lsm330dlc_driver.c $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/lsm330dlc_driver.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix): ../Library/hal_emu/lsm330dlc_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix) -MM "../Library/hal_emu/lsm330dlc_driver.c"

$(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix): ../Library/hal_emu/lsm330dlc_driver.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix) "../Library/hal_emu/lsm330dlc_driver.c"

$(IntermediateDirectory)/hal_emu_piezo_speaker$(ObjectSuffix): ../Library/hal_emu/piezo_speaker.c $(IntermediateDirectory)/hal_emu_piezo_speaker$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/hal_emu/piezo_speaker.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/hal_emu_piezo_speaker$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/hal_emu_piezo_speaker$(DependSuffix): ../Library/hal_emu/piezo_speaker.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/hal_emu_piezo_speaker$(ObjectSuffix) -MF$(IntermediateDirectory)/hal_emu_piezo_speaker$(DependSuffix) -MM "../Library/hal_emu/piezo_speaker.c"

$(IntermediateDirectory)/hal_emu_piezo_speaker$(PreprocessSuffix): ../Library/hal_emu/piezo_speaker.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/hal_emu_piezo_speaker$(PreprocessSuffix) "../Library/hal_emu/piezo_speaker.c"

$(IntermediateDirectory)/src_central_data$(ObjectSuffix): ../Maveric_myCopter/src/central_data.c $(IntermediateDirectory)/src_central_data$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Maveric_myCopter/src/central_data.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_central_data$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_central_data$(DependSuffix): ../Maveric_myCopter/src/central_data.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_central_data$(ObjectSuffix) -MF$(IntermediateDirectory)/src_central_data$(DependSuffix) -MM "../Maveric_myCopter/src/central_data.c"

$(IntermediateDirectory)/src_central_data$(PreprocessSuffix): ../Maveric_myCopter/src/central_data.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_central_data$(PreprocessSuffix) "../Maveric_myCopter/src/central_data.c"

$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix): ../Library/tests/test_maths.c $(IntermediateDirectory)/tests_test_maths$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/felix/Projects/maveric/Code/Library/tests/test_maths.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/tests_test_maths$(DependSuffix): ../Library/tests/test_maths.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/tests_test_maths$(ObjectSuffix) -MF$(IntermediateDirectory)/tests_test_maths$(DependSuffix) -MM "../Library/tests/test_maths.c"

$(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix): ../Library/tests/test_maths.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix) "../Library/tests/test_maths.c"


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
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_mavlink_actions$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_tasks$(PreprocessSuffix)
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
	$(RM) $(IntermediateDirectory)/sensing_simulation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_simulation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_position_estimation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(DependSuffix)
	$(RM) $(IntermediateDirectory)/sensing_neighbor_selection$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_adaptive_parameter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_navigation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_orca$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_pid_control$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_copter$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(DependSuffix)
	$(RM) $(IntermediateDirectory)/control_stabilisation_hybrid$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_stream$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_onboard_parameters$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(DependSuffix)
	$(RM) $(IntermediateDirectory)/communication_mavlink_waypoint_handler$(PreprocessSuffix)
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
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_analog_monitor$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_lsm330dlc_driver$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_piezo_speaker$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_piezo_speaker$(DependSuffix)
	$(RM) $(IntermediateDirectory)/hal_emu_piezo_speaker$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(DependSuffix)
	$(RM) $(IntermediateDirectory)/src_central_data$(PreprocessSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(ObjectSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(DependSuffix)
	$(RM) $(IntermediateDirectory)/tests_test_maths$(PreprocessSuffix)
	$(RM) $(OutputFile)
	$(RM) ".build-debug/Maveric_emu"


