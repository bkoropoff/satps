CPU_FLAGS := -DF_CPU=16000000UL -mmcu=atmega32u4
CFLAGS := -Wall -Werror $(CPU_FLAGS) -Os

OBJECTS = main.o
HEADERS =


all: satps.hex

upload: satps.hex
	avrdude -v -patmega32u4 -cavrisp -P/dev/ttyACM0 -b9600 -Uflash:w:$<:i

clean:
	rm -f $(OBJECTS) satps.hex satps

satps.hex: satps
	avr-objcopy -O ihex -R .eeprom $< $@

satps: $(OBJECTS)
	avr-gcc $(CFLAGS) -o $@ $(OBJECTS)

.c.o:
	avr-gcc $(CFLAGS) -c -o $@ $<

$(OBJECTS): $(HEADERS)

.PHONY: upload clean
