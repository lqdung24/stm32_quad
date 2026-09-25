CC ?= cc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES := -I. -I../../Components/PwmTimer/Inc

.PHONY: all clean

all: test_pwm_timer
	./test_pwm_timer

test_pwm_timer: test_pwm_timer.c ../../Components/PwmTimer/Src/pwm_timer.c
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

clean:
	$(RM) test_pwm_timer
