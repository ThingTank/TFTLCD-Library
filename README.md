# AdaFruit TFTLCD Library (ThingTank fork)
## Introduction
This is a library for the various AdaFruit TFT displays. Their webpage contains all the information on tutorials & wiring diagrams for the "classic" Arduino's. For SAMD21 variants, consult `pin_magic.h`. These displays use 8-bit parallel to communicate, 12 or 13 pins are required to interface (RST is optional).

Also requires the Adafruit_GFX library for Arduino. https://github.com/adafruit/Adafruit-GFX-Library

## About this fork
Fork code currently being pimped by Tim Jacobs / ThingTank to add support for the [Arduino Zero (Pro)](https://www.arduino.cc/en/Main/ArduinoBoardZero) / [SparkFun SAMD21 Dev Breakout](https://www.sparkfun.com/products/13672), the [Adafruit Feather M0](https://www.adafruit.com/product/2772) / [Arduino MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) devices, or any other device running a SAMD21 processor. I'm using the [AdaFruit 2.8" TFT CapTouch board](https://www.adafruit.com/products/2090) for testing.
```
Work is still in progress - library not completed yet - use at your own risk! 
```
I've also added code that should make this library portable to other Arduino devices, not supported by AdaFruit. That code uses the standard Arduino communication (digitalRead, digitalWrite, ...) so is much slower compared to the optimized AdaFruit code for supported devices. You're probably better of using the SPI variation of the TFT LCD's in that case, but hey, the code is there if you need it.

Speed benchmark in microseconds, using the "GraphicsTest" example, on different SAMD21 boards:

Test | Generic code | AdaFruit Feather M0 code | SparkFun SAMD21 Dev code
:--- | ---: | ---: | ---:
Screen fill             | 8799787   | 2876835 | 576785
Text                    | 937758    | 467650 | 271590
Lines                   | 8669701   | 4558891 | 2867372
Horiz/Vert Lines        | 1093055   | 400925 | 83923
Rectangles (outline)    | 716133    | 267931 | 63451
Rectangles (filled)     | 26619628  | 9795900 | 1800202
Circles (filled)        | 5139809   | 2182144 | 877089
Circles (outline)       | 3785683   | 1992094 | 1251726
Triangles (outline)     | 2755664   | 1445209 | 909307
Triangles (filled)      | 9542007   | 3632513 | 942129
Rounded rects (outline) | 1641708   | 779013 | 412223
Rounded rects (filled)  | 29925709  | 10835993 | 2155870

The big difference between the "mini" SAMD21 boards like the MKR1000/AdaFruit Feather M0 and the bigger brothers Arduino Zero/Sparkfun SAMD21 Dev is that the design of the mini boards is compact, not exposing all the datapins of the SAMD21 processor - that is not necessarily a bad thing, just to be clear :). The bigger boards do expose all pins, which allows us to use some tricks in bit-shifting since we can neatly line up all 8 communication bits (saving on a dozen of operations per read & write operation to the TFT screen).

## Motivation
You'll quickly realize when working with an [Arduino MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) or [AdaFruit Feather M0](https://www.adafruit.com/product/2772) that the TFT display in 8-bit parallel mode pretty much takes up every pin you have available on the breakout board. Nonetheless, we have a use case where we want to connect these LCDs to the more modern SAMD processors of Atmel like on the [MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) and [AdaFruit Feather M0 board](https://www.adafruit.com/product/2772).

On top, there are other boards available with the SAMD21 that expose more pins (have a look at the [SparkFun SAMD21 Dev Breakout](https://www.sparkfun.com/products/13672), which has 5 more digital pins available). So for everybody trying to get one of the AdaFruit LCD's working on those boards, here is a good place to get started.

## Current status (14/Nov/2016)
Added code for the "bigger brother" boards that allow to do some bitshifting tricks, providing an additional factor 5-6x speed increase compared to the previous optimizations. We're now roughly 10x faster than the generic code, using the standard Arduino functions. You **must** use the pin layouts below in order to get that working.

## Pin layouts per board
The following PIN connections are coded into `pin_magic.h`. You need to uncomment the appropriate pin config for your specific SAMD21 board in order to get the library working. If your board is not listed, you can use a more generic SAMD21 config (customize your pin layout), and if your board is totally not a SAMD21 board but something else, you can use the *"Arduino Generic"* code and pin layout. The latter is entirely customizable to however you want to wire your TFT screen.

TFT Pin | MKR1000 | Arduino Zero 
--- | --- | ---
D0 | A4 | D2
D1 | D12 | D5
D2 | D11 | D11
D3 | D10 | D13
D4 | D9 | D10
D5 | D6 | D12
D6 | D5 | D6
D7 | A5 |  D7
RD | A0 | A0
WR | A1 | A1
C/D | A2 | A2
CS | A3 | A3
RESET | Not connected | A4

Equivalent boards:
 * MKR1000 pin layout also works on: AdaFruit Feather M0
 * Arduino Zero pin layout also works on: Sparkfun SAMD21
 * Generic SAMD21 pin layout: same as MKR1000 pin layout
 * Generic pin layout: customize in `pin_magic.h`

## Original credits
Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Originally written by Limor Fried/Ladyada for Adafruit Industries.

Original credits: see [readme.txt](readme.txt)
