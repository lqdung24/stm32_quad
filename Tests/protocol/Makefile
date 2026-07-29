CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES = -I../../Components/DroneProtocol/Inc
SOURCES = test_drone_protocol.c \
	../../Components/DroneProtocol/Src/drone_protocol.c \
	../../Components/DroneProtocol/Src/drone_cobs.c

.PHONY: all test clean

all: test

test: test_drone_protocol
	./test_drone_protocol

test_drone_protocol: $(SOURCES)
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -o $@

clean:
	$(RM) test_drone_protocol
