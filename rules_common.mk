LIB_SRCS += communication/data_logging.cpp
LIB_SRCS += communication/data_logging_telemetry.cpp            
LIB_SRCS += communication/hud_telemetry.cpp           
LIB_SRCS += communication/mavlink_communication.cpp     
LIB_SRCS += communication/mavlink_message_handler.cpp   
LIB_SRCS += communication/mavlink_stream.cpp            
LIB_SRCS += communication/mavlink_waypoint_handler.cpp
LIB_SRCS += communication/mavlink_waypoint_handler_swarm.cpp
LIB_SRCS += communication/onboard_parameters.cpp
LIB_SRCS += communication/remote.cpp
LIB_SRCS += communication/remote_telemetry.cpp
LIB_SRCS += communication/state.cpp
LIB_SRCS += communication/state_machine.cpp
LIB_SRCS += communication/state_telemetry.cpp

LIB_SRCS += control/adaptive_parameter.c
LIB_SRCS += control/altitude_controller.cpp
LIB_SRCS += control/attitude_controller.c
LIB_SRCS += control/attitude_controller_p2.c
LIB_SRCS += control/attitude_error_estimator.c
LIB_SRCS += control/joystick.cpp
LIB_SRCS += control/joystick_telemetry.cpp
LIB_SRCS += control/manual_control.cpp
LIB_SRCS += control/manual_control_telemetry.cpp
LIB_SRCS += control/navigation.cpp
LIB_SRCS += control/pid_controller.c
LIB_SRCS += control/servos_mix_quadcopter_cross.cpp
LIB_SRCS += control/servos_mix_quadcopter_diag.cpp
LIB_SRCS += control/stabilisation.c
LIB_SRCS += control/stabilisation_copter.cpp
LIB_SRCS += control/stabilisation_telemetry.cpp
LIB_SRCS += control/vector_field_waypoint.cpp
LIB_SRCS += control/velocity_controller_copter.cpp
LIB_SRCS += control/gimbal_controller.cpp
LIB_SRCS += control/gimbal_controller_telemetry.cpp

LIB_SRCS += drivers/battery.cpp
LIB_SRCS += drivers/barometer.cpp          
LIB_SRCS += drivers/barometer_telemetry.cpp          
LIB_SRCS += drivers/bmp085.cpp                   
LIB_SRCS += drivers/flow.cpp
LIB_SRCS += drivers/gps_ublox.cpp                 
LIB_SRCS += drivers/gps_telemetry.cpp       
LIB_SRCS += drivers/hmc5883l.cpp  
LIB_SRCS += drivers/lsm330dlc.cpp            
LIB_SRCS += drivers/servo.cpp                              
LIB_SRCS += drivers/servos_telemetry.cpp
LIB_SRCS += drivers/sonar_i2cxl.cpp
LIB_SRCS += drivers/sonar_telemetry.cpp
LIB_SRCS += drivers/spektrum_satellite.cpp

LIB_SRCS += hal/common/dbg.cpp
LIB_SRCS += hal/common/led_gpio.cpp
LIB_SRCS += hal/common/file.cpp
LIB_SRCS += hal/common/serial.cpp
LIB_SRCS += hal/common/dbg.cpp

LIB_SRCS += runtime/scheduler.cpp
LIB_SRCS += runtime/scheduler_task.cpp
LIB_SRCS += runtime/scheduler_telemetry.cpp

LIB_SRCS += sensing/ahrs.c
LIB_SRCS += sensing/ahrs_telemetry.cpp
LIB_SRCS += sensing/altitude_estimation.cpp
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
LIB_SRCS += util/matrix.cpp
LIB_SRCS += util/print_util.c
LIB_SRCS += util/quick_trig.c

LIB_SRCS += util/raytracing.cpp
LIB_SRCS += util/string_util.cpp
