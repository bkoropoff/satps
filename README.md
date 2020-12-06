# Saturn to PS controller adapter

This allows using a digital Saturn controller with a Playstation or
Playstation 2.  It's based on an ATMega32U4 since that's what I had
sitting around, but any sufficiently fast AVR microcontroller should
work.  The MISO line requires a Schottky diode (anode facing PS,
cathode facing MCU) to convert the push-pull SPI output to open
collector.

Use `avr-gcc` version 7.2 or later, or the generated code will
not be efficient enough to keep up with the console.  Note that
LTS versions of Ubuntu are currently shipping old versions, so
you will need to obtain a newer version elsewhere (e.g. the
version bundled with Arduino).
