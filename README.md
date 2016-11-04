# AdaFruit TFTLCD Library (ThingTank fork)
## Introduction
This is a library for the various AdaFruit TFT displays. Their webpage contains all the information on tutorials & wiring diagrams. These displays use 8-bit parallel to communicate, 12 or 13 pins are required to interface (RST is optional).

Place the Adafruit_TFT library folder your <arduinosketchfolder>/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE

Also requires the Adafruit_GFX library for Arduino. https://github.com/adafruit/Adafruit-GFX-Library

## About this fork
Fork code currently being pimped by Tim Jacobs (not completed yet) to add support for Adafruit Feather M0 / Arduino MKR1000 devices, or any other device running a SAMD21 processor.
```
Work is still in progress - code not tested yet
```
I've also added code that should make this library portable to other Arduino devices, not supported by AdaFruit. That code uses the standard Arduino communication (digitalRead, digitalWrite, ...) so is much slower compared to the optimized AdaFruit code for supported devices. You're probably better of using the SPI variation of the TFT LCD's in that case, but hey, the code is there if you need it.

## Original credits
Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution
