CFLAGS   += -D__MAVRIC_ENDIAN_LITTLE__
CXXFLAGS += -D__MAVRIC_ENDIAN_LITTLE__

LIB_SRCS += boards/mavrinux.cpp

LIB_SRCS += hal/linux/file_linux.cpp
LIB_SRCS += hal/linux/serial_udp.cpp
LIB_SRCS += hal/linux/time_keeper.c
LIB_SRCS += hal/linux/serial_linux_io.cpp