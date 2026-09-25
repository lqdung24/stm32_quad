CC ?= cc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES := -I. \
	-I../../Components/Motor/Inc \
	-I../../Components/MotorPWM/Inc \
	-I../../Components/MotorDshot/Inc \
	-I../../Components/MotorDshot300/Inc \
	-I../../Components/PwmTimer/Inc
SOURCES := test_motor_output.c \
	../../Components/Motor/Src/motor.c \
	../../Components/MotorPWM/Src/motor_pwm.c \
	../../Components/MotorDshot/Src/motor_dshot.c

HEADERS := ../../Components/MotorDshot/Inc/motor_dshot.h \
	../../Components/MotorDshot300/Inc/motor_dshot300.h \
	../../Components/App/Inc/app_dshot_test.h

.PHONY: all clean

all: test_motor_output
	./test_motor_output

test_motor_output: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -lm -o $@

clean:
	$(RM) test_motor_output
