CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic
INCLUDES = -I../../Components/Mahony/Inc
SOURCES = test_mahony.c \
	../../Components/Mahony/Src/mahony.c
TARGET ?= /tmp/drone_test_mahony

.PHONY: all test clean

all: test

test: $(TARGET)
	$(TARGET)

$(TARGET): $(SOURCES) ../../Components/Mahony/Inc/mahony.h
	$(CC) $(CFLAGS) $(INCLUDES) $(SOURCES) -lm -o $@

clean:
	$(RM) $(TARGET)
