CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES = -I../../Components/DroneProtocol/Inc
SOURCES = test_drone_protocol.c \
	../../Components/DroneProtocol/Src/dp_protocol.c \
	../../Components/DroneProtocol/Src/dp_bytes.c \
	../../Components/DroneProtocol/Src/dp_cobs.c

.PHONY: all test clean

all: test

test: test_drone_protocol
	./test_drone_protocol

test_drone_protocol: $(SOURCES)
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -o $@

clean:
	$(RM) test_drone_protocol
