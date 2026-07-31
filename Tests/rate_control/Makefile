CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES = -I../../Components/DroneControl/Inc
SOURCES = test_rate_control.c \
	../../Components/DroneControl/Src/rate_control.c

.PHONY: all test clean

all: test

test: test_rate_control
	./test_rate_control

test_rate_control: $(SOURCES)
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -lm -o $@

clean:
	$(RM) test_rate_control
