CC ?= cc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES := -I../../Components/MotorMixer/Inc

.PHONY: all clean

all: test_motor_mixer
	./test_motor_mixer

test_motor_mixer: test_motor_mixer.c ../../Components/MotorMixer/Src/motor_mixer.c
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

clean:
	$(RM) test_motor_mixer
