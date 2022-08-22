# Dementha 2022

A ring of leds activated by an accelerometer

Display design is a charlieplexed matrix of leds.

Microcontroller is an attiny24a.

The accelerometer is an LIS2DH (virtually the same as the more popular LIS3DH without ADCs).

The acclerometer is driven via bit-banging SPI. Since there weren't enough pins left on the microcontroller, one pin is shared between CS and interrupt, biased against the interrupt with a resistor.

The leds are all on PORTA. This makes it easy to define leds as bits.

Since the layout of the LEDs on the PCB is irregular, their arrangement was optimized via imperfect brute force hill climbing optimization.
