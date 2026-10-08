CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES = -I. -I../../Components/DroneControl/Inc \
	-I../../Components/DroneProtocol/Inc -I../../Components/Mahony/Inc
SOURCES = test_drone_control.c \
	../../Components/Mahony/Src/mahony.c \
	../../Components/RateControl/Src/rate_control.c \
	../../Components/MotorMixer/Src/motor_mixer.c \
	../../Components/Motor/Src/motor.c \
	../../Components/DroneProtocol/Src/dp_protocol.c \
	../../Components/DroneProtocol/Src/dp_bytes.c \
	../../Components/DroneProtocol/Src/dp_cobs.c
HEADERS = ../../Components/DroneControl/Src/drone_control.c \
	../../Components/DroneControl/Inc/drone_control.h \
	../../Components/RateControl/Inc/rate_control.h stm32h7xx_hal.h

TARGET ?= /tmp/drone_test_control

.PHONY: all test clean
all: test

test: $(TARGET)
	$(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -lm -o $@

clean:
	$(RM) $(TARGET)
