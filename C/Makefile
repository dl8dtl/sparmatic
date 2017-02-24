CC = avr-gcc
CXX = avr-g++

MCU = atmega169p

CDEFS += -DF_CPU=8E6

CFLAGS = -Os -g -std=gnu99 -mmcu=$(MCU) -I. -Wall -Wextra $(CDEFS)
CXXFLAGS = -Os -g -std=c++11 -mmcu=$(MCU) -I. -Wall -Wextra $(CDEFS)

OBJS = comet.o
LIBS = -lm
SRCS = $(OBJS:.o=.cpp)

all: comet.elf

comet.elf: $(OBJS)
	$(CC) $(CFLAGS) -o comet.elf $(OBJS) $(LIBS)

clean:
	rm -rf $(OBJS) comet.elf

program: comet.elf
	avrdude -B1 -c jtag3 -p atmega169p -U comet.elf

comet.o: comet.c defines.h
