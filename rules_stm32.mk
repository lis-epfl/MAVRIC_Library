CFLAGS   += -D__MAVRIC_ENDIAN_LITTLE__
CXXFLAGS += -D__MAVRIC_ENDIAN_LITTLE__

LIB_SRCS += boards/mavrimini.cpp

LIB_SRCS += drivers/state_display_mavrimini.cpp

LIB_SRCS += hal/stm32/time_keeper.c
LIB_SRCS += hal/stm32/gpio_stm32.cpp
LIB_SRCS += hal/stm32/serial_stm32.cpp
LIB_SRCS += hal/stm32/serial_usb_stm32.cpp
LIB_SRCS += hal/stm32/i2c_stm32.cpp
LIB_SRCS += hal/stm32/pwm_stm32.cpp
