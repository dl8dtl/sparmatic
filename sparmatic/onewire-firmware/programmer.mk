PROGRAMMER = -c avrispmkII -P usb

FLASH = avrdude $(PROGRAMMER) -B $(1) -p $(PART) $(2) 2>&1
