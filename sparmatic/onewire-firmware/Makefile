include device.mk
include programmer.mk

.PHONY: fuses all flash clean erase serial

BUILDDIR = build
SRCDIR = src

OBJECTS = $(patsubst $(SRCDIR)%, $(BUILDDIR)%, $(addsuffix .o, $(basename $(shell find $(SRCDIR) -type f -name '*.c') $(shell find $(SRCDIR) -type f -name '*.S'))))
HEADERS = $(wildcard $(SRCDIR)/*.h) $(wildcard $(SRCDIR)/*/*.h)

GCC_OPT += -std=gnu99 
GCC_OPT += -W -Wall -pedantic
GCC_OPT += -DF_CPU=$(FCPU)
GCC_OPT += -mmcu=$(MCU)
GCC_OPT += -O2
GCC_OPT += -ffreestanding
GCC_OPT += -Wl,--relax

TOOLCHAIN_PREFIX = avr
GCC = $(TOOLCHAIN_PREFIX)-gcc
ELFSIZE = $(TOOLCHAIN_PREFIX)-size
OBJCOPY = $(TOOLCHAIN_PREFIX)-objcopy

ELFOUT = $(BUILDDIR)/out.elf
HEXOUT = $(BUILDDIR)/out.hex


all: $(HEXOUT)
	make flash

build:
	@echo "Building objects $(OBJECTS) -> $(ELFOUT) -> $(HEXOUT)"
	$(MAKE) $(HEXOUT)

fuses: 
	$(call FLASH, 100, $(FUSES))

erase:
	$(call FLASH, 100, -e)

flash: $(HEXOUT)
	$(call FLASH, $(FLASH_SPEED), -U flash:w:$(HEXOUT):i)

$(HEXOUT): $(ELFOUT)
	$(OBJCOPY) -R .eeprom -O ihex -v $(ELFOUT) $(HEXOUT) 

$(ELFOUT): $(OBJECTS)
	$(GCC) $(GCC_OPT) $(CFLAGS) $(LFLAGS) $(OBJECTS) -o $(ELFOUT) 
	$(ELFSIZE) $(ELFOUT) 

$(BUILDDIR)/%.o: $(SRCDIR)/%.S $(HEADERS) Makefile
	@ test -e $(dir $@) || mkdir -p $(dir $@)
	$(GCC) $(GCC_OPT) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.c $(HEADERS) Makefile
	@ test -e $(dir $@) || mkdir -p $(dir $@)
	$(GCC) $(GCC_OPT) $(CFLAGS) $(INCLUDES) -c $< -o $@ 

$(SRCDIR)/serial.h:
	$(MAKE) serial

clean:
	-rm $(OBJECTS)
	-rm $(ELFOUT) 
	-rm $(HEXOUT)
	-rm -r $(BUILDDIR)

distclean: clean
	-rm $(SRCDIR)/serial.h
