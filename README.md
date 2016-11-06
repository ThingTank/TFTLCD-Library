# AdaFruit TFTLCD Library (ThingTank fork)
## Introduction
This is a library for the various AdaFruit TFT displays. Their webpage contains all the information on tutorials & wiring diagrams. These displays use 8-bit parallel to communicate, 12 or 13 pins are required to interface (RST is optional).

Also requires the Adafruit_GFX library for Arduino. https://github.com/adafruit/Adafruit-GFX-Library

## About this fork
Fork code currently being pimped by Tim Jacobs / ThingTank to add support for [Adafruit Feather M0](https://www.adafruit.com/product/2772) / [Arduino MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) devices, or any other device running a SAMD21 processor. I'm using the [AdaFruit 2.8" TFT CapTouch board](https://www.adafruit.com/products/2090) for testing.
```
Work is still in progress - library not completed yet - use at your own risk! 
```
I've also added code that should make this library portable to other Arduino devices, not supported by AdaFruit. That code uses the standard Arduino communication (digitalRead, digitalWrite, ...) so is much slower compared to the optimized AdaFruit code for supported devices. You're probably better of using the SPI variation of the TFT LCD's in that case, but hey, the code is there if you need it.

Speed benchmark in microseconds, using the "GraphicsTest" example, on an AdaFruit Feather M0 Basic Proto board:

Test | Generic code | SAMD21 code
:--- | ---: | ---:
Screen fill             | 8799787   | 2876835
Text                    | 937758    | 467650
Lines                   | 8669701   | 4558891
Horiz/Vert Lines        | 1093055   | 400925
Rectangles (outline)    | 716133    | 267931
Rectangles (filled)     | 26619628  | 9795900
Circles (filled)        | 5139809   | 2182144
Circles (outline)       | 3785683   | 1992094
Triangles (outline)     | 2755664   | 1445209
Triangles (filled)      | 9542007   | 3632513
Rounded rects (outline) | 1641708   | 779013
Rounded rects (filled)  | 29925709  | 10835993

## Motivation
You'll quickly realize when working with an [Arduino MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) or [AdaFruit Feather M0](https://www.adafruit.com/product/2772) that the TFT display in 8-bit parallel mode pretty much takes up every pin you have available on the breakout board. Nonetheless, we have a use case where we want to connect these LCDs to the more modern SAMD processors of Atmel like on the [MKR1000](https://www.arduino.cc/en/Main/ArduinoMKR1000) and [AdaFruit Feather M0 board](https://www.adafruit.com/product/2772). On top, there are other boards available with the SAMD21 that expose more pins (have a look at the [SparkFun SAMD21 Dev Breakout](https://www.sparkfun.com/products/13672), which has 5 more digital pins available). So for everybody trying to get one of the AdaFruit LCD's working on those boards, here is a good place to get started.

## Current status (6/Nov/2016)
Optimized the generic code specifically for the SAMD21, using some basic ASF functions. That yielded a speed increase of a factor 2-3x compared to the generic code. More optimization and hence speed increase is possible and needed.

I'm using the following connections in the code, currently partially hard coded (ironing that out):

TFT Pin | Feather M0 pin
--- | ---
D0 | A4
D1 | D12
D2 | D11
D3 | D10
D4 | D9
D5 | D6
D6 | D5
D7 | A5
RD | A0
WR | A1
C/D | A2
CS | A3
RESET | Not connected

The next days I'll have a look at replacing part by part the "good behaving" Arduino code by some nasty ASF code to speed things up.

## Original credits
Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Originally written by Limor Fried/Ladyada for Adafruit Industries.

Original credits: see [readme.txt](readme.txt)
