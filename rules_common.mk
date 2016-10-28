####################################################################################################
# PROJECT NAME
####################################################################################################
# Ensure project name is defined
ifndef PROJ_NAME
PROJ_NAME = MAVRIC
endif

# Add to compile flags
CFLAGS   += -DPROJECT_NAME=\"$(PROJ_NAME)\"
CXXFLAGS += -DPROJECT_NAME=\"$(PROJ_NAME)\"

# Report
$(info Project name     = $(PROJ_NAME))


####################################################################################################
# GIT HASH
####################################################################################################
# Find git hash for project (current folder)
PROJECT_GIT_HASH := $(shell git describe --abbrev=7 --always)
# Find git hash for library
MAVRIC_GIT_HASH  := $(shell cd $(MAVRIC_LIB) && git describe --abbrev=7 --always)

# Add to compile flags
CFLAGS   += -DMAVRIC_GIT_HASH=\"$(MAVRIC_GIT_HASH)\"
CFLAGS   += -DPROJECT_GIT_HASH=\"$(PROJECT_GIT_HASH)\"
CXXFLAGS += -DMAVRIC_GIT_HASH=\"$(MAVRIC_GIT_HASH)\"
CXXFLAGS += -DPROJECT_GIT_HASH=\"$(PROJECT_GIT_HASH)\"

# Report
$(info Project git hash = $(PROJECT_GIT_HASH))
$(info Mavric git hash  = $(MAVRIC_GIT_HASH))
$(info Mavric version   = $(shell git describe --tags))


####################################################################################################
# COMMON SOURCE FILES
####################################################################################################
LIB_SRCS += communication/data_logging.cpp
LIB_SRCS += communication/data_logging_telemetry.cpp
LIB_SRCS += communication/hud_telemetry.cpp
LIB_SRCS += communication/mavlink_message_handler.cpp
LIB_SRCS += communication/mavlink_stream.cpp
LIB_SRCS += communication/mavlink_waypoint_handler.cpp
#LIB_SRCS += communication/mavlink_waypoint_handler_swarm.cpp
LIB_SRCS += communication/onboard_parameters.cpp
LIB_SRCS += communication/periodic_telemetry.cpp


LIB_SRCS += control/altitude_controller.cpp
LIB_SRCS += control/attitude_controller_p2.cpp
LIB_SRCS += control/attitude_error_estimator.cpp
LIB_SRCS += control/pid_controller.cpp
LIB_SRCS += control/stabilisation.cpp
# LIB_SRCS += control/stabilisation_wing.cpp
LIB_SRCS += control/servos_mix.cpp
LIB_SRCS += control/servos_mix_quadcopter_diag.cpp
LIB_SRCS += control/servos_mix_quadcopter_cross.cpp
LIB_SRCS += control/servos_mix_wing.cpp
LIB_SRCS += control/servos_mix_ywing.cpp
LIB_SRCS += control/gimbal_controller.cpp
LIB_SRCS += control/gimbal_controller_telemetry.cpp

LIB_SRCS += drivers/airspeed_analog.cpp
LIB_SRCS += drivers/battery.cpp
LIB_SRCS += drivers/barometer.cpp
LIB_SRCS += drivers/barometer_telemetry.cpp
LIB_SRCS += drivers/barometer_bmp085.cpp
LIB_SRCS += drivers/barometer_ms5611.cpp
LIB_SRCS += drivers/flow_px4.cpp
LIB_SRCS += drivers/gps_ublox.cpp
LIB_SRCS += drivers/gps_telemetry.cpp
LIB_SRCS += drivers/gps_mocap.cpp
LIB_SRCS += drivers/hmc5883l.cpp
LIB_SRCS += drivers/lsm330dlc.cpp
LIB_SRCS += drivers/mpu_6050.cpp
LIB_SRCS += drivers/mpu_9250.cpp
LIB_SRCS += drivers/px4flow_i2c.cpp
LIB_SRCS += drivers/px4flow_telemetry.cpp
LIB_SRCS += drivers/servo.cpp
LIB_SRCS += drivers/servos_telemetry.cpp
LIB_SRCS += drivers/sonar_i2cxl.cpp
LIB_SRCS += drivers/sonar_telemetry.cpp
LIB_SRCS += drivers/spektrum_satellite.cpp
LIB_SRCS += drivers/state_display.cpp
LIB_SRCS += drivers/state_display_megafly_rev4.cpp
LIB_SRCS += drivers/state_display_sparky_v2.cpp

LIB_SRCS += drones/lequad.cpp

LIB_SRCS += hal/common/dbg.cpp
LIB_SRCS += hal/common/led_gpio.cpp
LIB_SRCS += hal/common/file.cpp
LIB_SRCS += hal/common/serial.cpp

LIB_SRCS += manual_control/joystick.cpp
LIB_SRCS += manual_control/joystick_telemetry.cpp
LIB_SRCS += manual_control/manual_control.cpp
LIB_SRCS += manual_control/manual_control_telemetry.cpp
LIB_SRCS += manual_control/remote.cpp
LIB_SRCS += manual_control/remote_telemetry.cpp

LIB_SRCS += mission/mission_planner.cpp
LIB_SRCS += mission/mission_handler_hold_position.cpp
LIB_SRCS += mission/mission_handler_landing.cpp
LIB_SRCS += mission/mission_handler_manual.cpp
LIB_SRCS += mission/mission_handler_navigating.cpp
LIB_SRCS += mission/mission_handler_on_ground.cpp
LIB_SRCS += mission/mission_handler_registry.cpp
LIB_SRCS += mission/mission_handler_takeoff.cpp
LIB_SRCS += mission/waypoint.cpp

LIB_SRCS += navigation/dubin.cpp
LIB_SRCS += navigation/vector_field_waypoint.cpp

LIB_SRCS += runtime/scheduler.cpp
LIB_SRCS += runtime/scheduler_task.cpp
LIB_SRCS += runtime/scheduler_telemetry.cpp

LIB_SRCS += sensing/ahrs.cpp
LIB_SRCS += sensing/ahrs_ekf.cpp
LIB_SRCS += sensing/ahrs_ekf_mocap.cpp
LIB_SRCS += sensing/ahrs_telemetry.cpp
LIB_SRCS += sensing/ahrs_madgwick.cpp
LIB_SRCS += sensing/altitude_estimation.cpp
LIB_SRCS += sensing/imu.cpp
LIB_SRCS += sensing/imu_telemetry.cpp
LIB_SRCS += sensing/ins.cpp
LIB_SRCS += sensing/ins_complementary.cpp
LIB_SRCS += sensing/ins_kf.cpp
LIB_SRCS += sensing/ins_telemetry.cpp
LIB_SRCS += sensing/qfilter.cpp

LIB_SRCS += simulation/accelerometer_sim.cpp
LIB_SRCS += simulation/barometer_sim.cpp
LIB_SRCS += simulation/dynamic_model_quad_diag.cpp
LIB_SRCS += simulation/dynamic_model_telemetry.cpp
LIB_SRCS += simulation/flow_sim.cpp
LIB_SRCS += simulation/gps_sim.cpp
LIB_SRCS += simulation/gyroscope_sim.cpp
LIB_SRCS += simulation/ins_ahrs_groundtruth.cpp
LIB_SRCS += simulation/magnetometer_sim.cpp
LIB_SRCS += simulation/simulation.cpp
LIB_SRCS += simulation/sonar_sim.cpp

LIB_SRCS += status/geofence_cylinder.cpp
LIB_SRCS += status/state.cpp
LIB_SRCS += status/state_machine.cpp
LIB_SRCS += status/state_telemetry.cpp

LIB_SRCS += util/coord_conventions.cpp
LIB_SRCS += util/matrix.cpp
LIB_SRCS += util/print_util.cpp
LIB_SRCS += util/quick_trig.cpp
LIB_SRCS += util/raytracing.cpp
LIB_SRCS += util/string_util.cpp
