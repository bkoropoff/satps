# Saturn to PS controller adapter

This allows using a digital Saturn controller with a Playstation or
Playstation 2.  It's based on an ATMega32U4 since that's what I had
sitting around, but any sufficiently fast AVR microcontroller should
work.  The MISO line requires a Schottky diode (anode facing PS,
cathode facing MCU) to convert the push-pull SPI output to open
collector.
