CFLAGS   += -D__MAVRIC_ENDIAN_LITTLE__
CXXFLAGS += -D__MAVRIC_ENDIAN_LITTLE__

LIB_SRCS += hal/chibios/gpio_chibios.cpp
LIB_SRCS += hal/chibios/i2c_chibios.cpp
LIB_SRCS += hal/chibios/pwm_chibios.cpp
LIB_SRCS += hal/chibios/serial_chibios.cpp
LIB_SRCS += hal/chibios/spi_chibios.cpp
LIB_SRCS += hal/chibios/stubs.cpp
LIB_SRCS += hal/chibios/time_keeper.cpp
