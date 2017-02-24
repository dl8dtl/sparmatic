# MCU for gcc -mmcu
MCU = atmega169pa

# Part for avrdude -p
PART = m169

# CPU Frequency in Hz
FCPU = 4000000UL

# maximum SCK period in us
FLASH_SPEED = 1

# fuses to be written on make fuses
FUSES = -U lfuse:w:0xfd:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
