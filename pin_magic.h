#ifndef _pin_magic_
#define _pin_magic_

// This header file serves two purposes:
//
// 1) Isolate non-portable MCU port- and pin-specific identifiers and
//    operations so the library code itself remains somewhat agnostic
//    (PORTs and pin numbers are always referenced through macros).
//
// 2) GCC doesn't always respect the "inline" keyword, so this is a
//    ham-fisted manner of forcing the issue to minimize function calls.
//    This sometimes makes the library a bit bigger than before, but fast++.
//    However, because they're macros, we need to be SUPER CAREFUL about
//    parameters -- for example, write8(x) may expand to multiple PORT
//    writes that all refer to x, so it needs to be a constant or fixed
//    variable and not something like *ptr++ (which, after macro
//    expansion, may increment the pointer repeatedly and run off into
//    la-la land).  Macros also give us fine-grained control over which
//    operations are inlined on which boards (balancing speed against
//    available program space).

// When using the TFT shield, control and data pins exist in set physical
// locations, but the ports and bitmasks corresponding to each vary among
// boards.  A separate set of pin definitions is given for each supported
// board type.
// When using the TFT breakout board, control pins are configurable but
// the data pins are still fixed -- making every data pin configurable
// would be much too slow.  The data pin layouts are not the same between
// the shield and breakout configurations -- for the latter, pins were
// chosen to keep the tutorial wiring manageable more than making optimal
// use of ports and bitmasks.  So there's a second set of pin definitions
// given for each supported board.

// Shield pin usage:
// LCD Data Bit :    7    6    5    4    3    2    1    0
// Digital pin #:    7    6   13    4   11   10    9    8
// Uno port/pin :  PD7  PD6  PB5  PD4  PB3  PB2  PB1  PB0
// Mega port/pin:  PH4  PH3  PB7  PG5  PB5  PB4  PH6  PH5
// Leo port/pin :  PE6  PD7  PC7  PD4  PB7  PB6  PB5  PB4
// Due port/pin : PC23 PC24 PB27 PC26  PD7 PC29 PC21 PC22

// Breakout pin usage:
// LCD Data Bit :   7   6   5   4   3   2   1   0
// Uno dig. pin :   7   6   5   4   3   2   9   8
// Uno port/pin : PD7 PD6 PD5 PD4 PD3 PD2 PB1 PB0
// Mega dig. pin:  29  28  27  26  25  24  23  22
// Mega port/pin: PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0 (one contiguous PORT)
// Leo dig. pin :   7   6   5   4   3   2   9   8
// Leo port/pin : PE6 PD7 PC6 PD4 PD0 PD1 PB5 PB4
// Due dig. pin :  40  39  38  37  36  35  34  33
// Due port/pin : PC8 PC7 PC6 PC5 PC4 PC3 PC2 PC1 (one contiguous PORT. -ish…)

// Pixel read operations require a minimum 400 nS delay from RD_ACTIVE
// to polling the input pins.  At 16 MHz, one machine cycle is 62.5 nS.
// This code burns 7 cycles (437.5 nS) doing nothing; the RJMPs are
// equivalent to two NOPs each, final NOP burns the 7th cycle, and the
// last line is a radioactive mutant emoticon.
#define AVRDELAY7        \
  asm volatile(       \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "nop"      "\n"   \
    ::);

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)

 // Arduino Uno, Duemilanove, etc.

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  // LCD control lines:
  // RD (read), WR (write), CD (command/data), CS (chip select)
  #define RD_PORT PORTC				/*pin A0 */
  #define WR_PORT PORTC				/*pin A1 */
  #define CD_PORT PORTC				/*pin A2 */
  #define CS_PORT PORTC				/*pin A3 */
  #define RD_MASK B00000001
  #define WR_MASK B00000010
  #define CD_MASK B00000100
  #define CS_MASK B00001000

  // These are macros for I/O operations...

  // Write 8-bit value to LCD data lines
  #define write8inline(d) {                          \
    PORTD = (PORTD & B00101111) | ((d) & B11010000); \
    PORTB = (PORTB & B11010000) | ((d) & B00101111); \
    WR_STROBE; } // STROBEs are defined later

  // Read 8-bit value from LCD data lines.  The signle argument
  // is a destination variable; this isn't a function and doesn't
  // return a value in the conventional sense.
  #define read8inline(result) {                       \
    RD_ACTIVE;                                        \
    AVRDELAY7;                                           \
    result = (PIND & B11010000) | (PINB & B00101111); \
    RD_IDLE; }

  // These set the PORT directions as required before the write and read
  // operations.  Because write operations are much more common than reads,
  // the data-reading functions in the library code set the PORT(s) to
  // input before a read, and restore them back to the write state before
  // returning.  This avoids having to set it for output inside every
  // drawing method.  The default state has them initialized for writes.
  #define setWriteDirInline() { DDRD |=  B11010000; DDRB |=  B00101111; }
  #define setReadDirInline()  { DDRD &= ~B11010000; DDRB &= ~B00101111; }

  // Control signals are ACTIVE LOW (idle is HIGH)
  // Command/Data: LOW = command, HIGH = data
  // These are single-instruction operations and always inline
  #define RD_ACTIVE  RD_PORT &= ~RD_MASK
  #define RD_IDLE    RD_PORT |=  RD_MASK
  #define WR_ACTIVE  WR_PORT &= ~WR_MASK
  #define WR_IDLE    WR_PORT |=  WR_MASK
  #define CD_COMMAND CD_PORT &= ~CD_MASK
  #define CD_DATA    CD_PORT |=  CD_MASK
  #define CS_ACTIVE  CS_PORT &= ~CS_MASK
  #define CS_IDLE    CS_PORT |=  CS_MASK


 #else // Uno w/Breakout board

  #define write8inline(d) {                          \
    CS_ACTIVE;                                       \                  // yarda
    PORTD = (PORTD & B00000011) | ((d) & B11111100); \
    PORTB = (PORTB & B11111100) | ((d) & B00000011); \
    WR_STROBE;                                       \                  // yarda
    CS_IDLE; }
  #define read8inline(result) {                       \
    CS_ACTIVE;                                       \                  // yarda
    RD_ACTIVE;                                        \
    AVRDELAY7;                                           \
    result = (PIND & B11111100) | (PINB & B00000011); \
    RD_IDLE;                                        \                   // yarda
    CS_IDLE; }
  #define setWriteDirInline() { DDRD |=  B11111100; DDRB |=  B00000011; }
  #define setReadDirInline()  { DDRD &= ~B11111100; DDRB &= ~B00000011; }

  // When using the TFT breakout board, control pins are configurable.
  #define RD_ACTIVE  *rdPort &=  rdPinUnset
  #define RD_IDLE    *rdPort |=  rdPinSet
  #define WR_ACTIVE  *wrPort &=  wrPinUnset
  #define WR_IDLE    *wrPort |=  wrPinSet
  #define CD_COMMAND *cdPort &=  cdPinUnset
  #define CD_DATA    *cdPort |=  cdPinSet
  #define CS_ACTIVE  *csPort &=  csPinUnset
  #define CS_IDLE    *csPort |=  csPinSet


 #endif

  // As part of the inline control, macros reference other macros...if any
  // of these are left undefined, an equivalent function version (non-inline)
  // is declared later.  The Uno has a moderate amount of program space, so
  // only write8() is inlined -- that one provides the most performance
  // benefit, but unfortunately also generates the most bloat.  This is
  // why only certain cases are inlined for each board.
  #define write8 write8inline


#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

 // Arduino Mega, ADK, etc.

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  #define RD_PORT PORTF
  #define WR_PORT PORTF
  #define CD_PORT PORTF
  #define CS_PORT PORTF
  #define RD_MASK B00000001
  #define WR_MASK B00000010
  #define CD_MASK B00000100
  #define CS_MASK B00001000

  #define write8inline(d) {                                              \
    PORTH = (PORTH&B10000111)|(((d)&B11000000)>>3)|(((d)&B00000011)<<5); \
    PORTB = (PORTB&B01001111)|(((d)&B00101100)<<2);                      \
    PORTG = (PORTG&B11011111)|(((d)&B00010000)<<1);                      \
    WR_STROBE; }
  #define read8inline(result) {                                      \
    RD_ACTIVE;                                                       \
    AVRDELAY7;                                                          \
    result = ((PINH & B00011000) << 3) | ((PINB & B10110000) >> 2) | \
             ((PING & B00100000) >> 1) | ((PINH & B01100000) >> 5);  \
    RD_IDLE; }
  #define setWriteDirInline() {                                   \
    DDRH |=  B01111000; DDRB |=  B10110000; DDRG |=  B00100000; }
  #define setReadDirInline()  {                                   \
    DDRH &= ~B01111000; DDRB &= ~B10110000; DDRG &= ~B00100000; }

    // Control signals are ACTIVE LOW (idle is HIGH)
    // Command/Data: LOW = command, HIGH = data
    // These are single-instruction operations and always inline
    #define RD_ACTIVE  RD_PORT &= ~RD_MASK
    #define RD_IDLE    RD_PORT |=  RD_MASK
    #define WR_ACTIVE  WR_PORT &= ~WR_MASK
    #define WR_IDLE    WR_PORT |=  WR_MASK
    #define CD_COMMAND CD_PORT &= ~CD_MASK
    #define CD_DATA    CD_PORT |=  CD_MASK
    #define CS_ACTIVE  CS_PORT &= ~CS_MASK
    #define CS_IDLE    CS_PORT |=  CS_MASK

 #else // Mega w/Breakout board

  #define write8inline(d)   { PORTA = (d); WR_STROBE; }
  #define read8inline(result) { \
    RD_ACTIVE;                  \
    AVRDELAY7;                     \
    result = PINA;              \
    RD_IDLE; }
  #define setWriteDirInline() DDRA  = 0xff
  #define setReadDirInline()  DDRA  = 0

  // When using the TFT breakout board, control pins are configurable.
  #define RD_ACTIVE  *rdPort &=  rdPinUnset
  #define RD_IDLE    *rdPort |=  rdPinSet
  #define WR_ACTIVE  *wrPort &=  wrPinUnset
  #define WR_IDLE    *wrPort |=  wrPinSet
  #define CD_COMMAND *cdPort &=  cdPinUnset
  #define CD_DATA    *cdPort |=  cdPinSet
  #define CS_ACTIVE  *csPort &=  csPinUnset
  #define CS_IDLE    *csPort |=  csPinSet

 #endif

  // All of the functions are inlined on the Arduino Mega.  When using the
  // breakout board, the macro versions aren't appreciably larger than the
  // function equivalents, and they're super simple and fast.  When using
  // the shield, the macros become pretty complicated...but this board has
  // so much code space, the macros are used anyway.  If you need to free
  // up program space, some macros can be removed, at a minor cost in speed.
  #define write8            write8inline
  #define read8             read8inline
  #define setWriteDir       setWriteDirInline
  #define setReadDir        setReadDirInline
  #define writeRegister8    writeRegister8inline
  #define writeRegister16   writeRegister16inline
  #define writeRegisterPair writeRegisterPairInline


#elif defined(__AVR_ATmega32U4__)

 // Arduino Leonardo

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  #define RD_PORT PORTF
  #define WR_PORT PORTF
  #define CD_PORT PORTF
  #define CS_PORT PORTF
  #define RD_MASK B10000000
  #define WR_MASK B01000000
  #define CD_MASK B00100000
  #define CS_MASK B00010000

  #define write8inline(d) {                                                   \
    PORTE = (PORTE & B10111111) | (((d) & B10000000)>>1);                     \
    PORTD = (PORTD & B01101111) | (((d) & B01000000)<<1) | ((d) & B00010000); \
    PORTC = (PORTC & B01111111) | (((d) & B00100000)<<2);                     \
    PORTB = (PORTB & B00001111) | (((d) & B00001111)<<4);                     \
    WR_STROBE; }
  #define read8inline(result) {                                      \
    RD_ACTIVE;                                                       \
    AVRDELAY7;                                                          \
    result = ((PINE & B01000000) << 1) | ((PIND & B10000000) >> 1) | \
             ((PINC & B10000000) >> 2) | ((PINB & B11110000) >> 4) | \
              (PIND & B00010000);                                    \
    RD_IDLE; }
  #define setWriteDirInline() {               \
    DDRE |=  B01000000; DDRD |=  B10010000;   \
    DDRC |=  B10000000; DDRB |=  B11110000; }
  #define setReadDirInline() {                \
    DDRE &= ~B01000000; DDRD &= ~B10010000;   \
    DDRC &= ~B10000000; DDRB &= ~B11110000; }

    // Control signals are ACTIVE LOW (idle is HIGH)
    // Command/Data: LOW = command, HIGH = data
    // These are single-instruction operations and always inline
    #define RD_ACTIVE  RD_PORT &= ~RD_MASK
    #define RD_IDLE    RD_PORT |=  RD_MASK
    #define WR_ACTIVE  WR_PORT &= ~WR_MASK
    #define WR_IDLE    WR_PORT |=  WR_MASK
    #define CD_COMMAND CD_PORT &= ~CD_MASK
    #define CD_DATA    CD_PORT |=  CD_MASK
    #define CS_ACTIVE  CS_PORT &= ~CS_MASK
    #define CS_IDLE    CS_PORT |=  CS_MASK

 #else // Leonardo w/Breakout board

  #define write8inline(d) {                                                   \
    uint8_t dr1 = (d) >> 1, dl1 = (d) << 1;                                   \
    PORTE = (PORTE & B10111111) | (dr1 & B01000000);                          \
    PORTD = (PORTD & B01101100) | (dl1 & B10000000) | (((d) & B00001000)>>3) |\
                                  (dr1 & B00000010) |  ((d) & B00010000);     \
    PORTC = (PORTC & B10111111) | (dl1 & B01000000);                          \
    PORTB = (PORTB & B11001111) |(((d) & B00000011)<<4);                      \
    WR_STROBE; }
  #define read8inline(result) {                                       \
    RD_ACTIVE;                                                        \
    AVRDELAY7;                                                           \
    result = (((PINE & B01000000) | (PIND & B00000010)) << 1) |       \
             (((PINC & B01000000) | (PIND & B10000000)) >> 1) |       \
              ((PIND & B00000001) << 3) | ((PINB & B00110000) >> 4) | \
               (PIND & B00010000);                                    \
    RD_IDLE; }
  #define setWriteDirInline() {               \
    DDRE |=  B01000000; DDRD |=  B10010011;   \
    DDRC |=  B01000000; DDRB |=  B00110000; }
  #define setReadDirInline() {                \
    DDRE &= ~B01000000; DDRD &= ~B10010011;   \
    DDRC &= ~B01000000; DDRB &= ~B00110000; }

    // When using the TFT breakout board, control pins are configurable.
    #define RD_ACTIVE  *rdPort &=  rdPinUnset
    #define RD_IDLE    *rdPort |=  rdPinSet
    #define WR_ACTIVE  *wrPort &=  wrPinUnset
    #define WR_IDLE    *wrPort |=  wrPinSet
    #define CD_COMMAND *cdPort &=  cdPinUnset
    #define CD_DATA    *cdPort |=  cdPinSet
    #define CS_ACTIVE  *csPort &=  csPinUnset
    #define CS_IDLE    *csPort |=  csPinSet


 #endif

  // On the Leonardo, only the write8() macro is used -- though even that
  // might be excessive given the code size and available program space
  // on this board.  You may need to disable this to get any sizable
  // program to compile.
  #define write8 write8inline


#elif defined(__SAM3X8E__)

// Arduino Due

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  #define RD_PORT PIOA				/*pin A0 */
  #define WR_PORT PIOA				/*pin A1 */
  #define CD_PORT PIOA				/*pin A2 */
  #define CS_PORT PIOA				/*pin A3 */
  #define RD_MASK 0x00010000
  #define WR_MASK 0x01000000
  #define CD_MASK 0x00800000
  #define CS_MASK 0x00400000

  #define write8inline(d) { \
   PIO_Set(PIOD, (((d) & 0x08)<<(7-3))); \
   PIO_Clear(PIOD, (((~d) & 0x08)<<(7-3))); \
   PIO_Set(PIOC, (((d) & 0x01)<<(22-0)) | (((d) & 0x02)<<(21-1))| (((d) & 0x04)<<(29-2))| (((d) & 0x10)<<(26-4))| (((d) & 0x40)<<(24-6))| (((d) & 0x80)<<(23-7))); \
   PIO_Clear(PIOC, (((~d) & 0x01)<<(22-0)) | (((~d) & 0x02)<<(21-1))| (((~d) & 0x04)<<(29-2))| (((~d) & 0x10)<<(26-4))| (((~d) & 0x40)<<(24-6))| (((~d) & 0x80)<<(23-7))); \
   PIO_Set(PIOB, (((d) & 0x20)<<(27-5))); \
   PIO_Clear(PIOB, (((~d) & 0x20)<<(27-5))); \
   WR_STROBE; }

  #define read8inline(result) { \
   RD_ACTIVE;   \
   delayMicroseconds(1);      \
   result = (((PIOC->PIO_PDSR & (1<<23)) >> (23-7)) | ((PIOC->PIO_PDSR & (1<<24)) >> (24-6)) | \
             ((PIOB->PIO_PDSR & (1<<27)) >> (27-5)) | ((PIOC->PIO_PDSR & (1<<26)) >> (26-4)) | \
             ((PIOD->PIO_PDSR & (1<< 7)) >> ( 7-3)) | ((PIOC->PIO_PDSR & (1<<29)) >> (29-2)) | \
             ((PIOC->PIO_PDSR & (1<<21)) >> (21-1)) | ((PIOC->PIO_PDSR & (1<<22)) >> (22-0))); \
   RD_IDLE;}

  #define setWriteDirInline() { \
   PIOD->PIO_MDDR |=  0x00000080; /*PIOD->PIO_SODR =  0x00000080;*/ PIOD->PIO_OER |=  0x00000080; PIOD->PIO_PER |=  0x00000080; \
   PIOC->PIO_MDDR |=  0x25E00000; /*PIOC->PIO_SODR =  0x25E00000;*/ PIOC->PIO_OER |=  0x25E00000; PIOC->PIO_PER |=  0x25E00000; \
   PIOB->PIO_MDDR |=  0x08000000; /*PIOB->PIO_SODR =  0x08000000;*/ PIOB->PIO_OER |=  0x08000000; PIOB->PIO_PER |=  0x08000000; }

  #define setReadDirInline() { \
	  pmc_enable_periph_clk( ID_PIOD ) ;	  pmc_enable_periph_clk( ID_PIOC ) ;	  pmc_enable_periph_clk( ID_PIOB ) ; \
   PIOD->PIO_PUDR |=  0x00000080; PIOD->PIO_IFDR |=  0x00000080; PIOD->PIO_ODR |=  0x00000080; PIOD->PIO_PER |=  0x00000080; \
   PIOC->PIO_PUDR |=  0x25E00000; PIOC->PIO_IFDR |=  0x25E00000; PIOC->PIO_ODR |=  0x25E00000; PIOC->PIO_PER |=  0x25E00000; \
   PIOB->PIO_PUDR |=  0x08000000; PIOB->PIO_IFDR |=  0x08000000; PIOB->PIO_ODR |=  0x08000000; PIOB->PIO_PER |=  0x08000000; }

   // Control signals are ACTIVE LOW (idle is HIGH)
   // Command/Data: LOW = command, HIGH = data
   // These are single-instruction operations and always inline
   #define RD_ACTIVE  RD_PORT->PIO_CODR |= RD_MASK
   #define RD_IDLE    RD_PORT->PIO_SODR |= RD_MASK
   #define WR_ACTIVE  WR_PORT->PIO_CODR |= WR_MASK
   #define WR_IDLE    WR_PORT->PIO_SODR |= WR_MASK
   #define CD_COMMAND CD_PORT->PIO_CODR |= CD_MASK
   #define CD_DATA    CD_PORT->PIO_SODR |= CD_MASK
   #define CS_ACTIVE  CS_PORT->PIO_CODR |= CS_MASK
   #define CS_IDLE    CS_PORT->PIO_SODR |= CS_MASK


#else // Due w/Breakout board

    #define write8inline(d) { \
		PIO_Set(PIOC, (((d) & 0xFF)<<1)); \
		PIO_Clear(PIOC, (((~d) & 0xFF)<<1)); \
		WR_STROBE; }

    #define read8inline(result) { \
		RD_ACTIVE;   \
		delayMicroseconds(1);      \
		result = ((PIOC->PIO_PDSR & 0x1FE) >> 1); \
		RD_IDLE;}

    #define setWriteDirInline() { \
	    PIOC->PIO_MDDR |=  0x000001FE; /*PIOC->PIO_SODR |=  0x000001FE;*/ PIOC->PIO_OER |=  0x000001FE; PIOC->PIO_PER |=  0x000001FE; }

    #define setReadDirInline() { \
		pmc_enable_periph_clk( ID_PIOC ) ; \
		PIOC->PIO_PUDR |=  0x000001FE; PIOC->PIO_IFDR |=  0x000001FE; PIOC->PIO_ODR |=  0x000001FE; PIOC->PIO_PER |=  0x000001FE; }

    // When using the TFT breakout board, control pins are configurable.
    #define RD_ACTIVE	rdPort->PIO_CODR |= rdPinSet		//PIO_Clear(rdPort, rdPinSet)
    #define RD_IDLE		rdPort->PIO_SODR |= rdPinSet		//PIO_Set(rdPort, rdPinSet)
    #define WR_ACTIVE	wrPort->PIO_CODR |= wrPinSet		//PIO_Clear(wrPort, wrPinSet)
    #define WR_IDLE		wrPort->PIO_SODR |= wrPinSet		//PIO_Set(wrPort, wrPinSet)
    #define CD_COMMAND	cdPort->PIO_CODR |= cdPinSet		//PIO_Clear(cdPort, cdPinSet)
    #define CD_DATA		cdPort->PIO_SODR |= cdPinSet		//PIO_Set(cdPort, cdPinSet)
    #define CS_ACTIVE	csPort->PIO_CODR |= csPinSet		//PIO_Clear(csPort, csPinSet)
    #define CS_IDLE		csPort->PIO_SODR |= csPinSet		//PIO_Set(csPort, csPinSet)

 #endif

#elif defined(__SAMD21G18A__)
    // =============================================================================
    // SAMD21 TFTLCD IMPLEMENATION - GENERAL NOTES (ThingTank TIJA)
    // =============================================================================
    // Fix for using the Shield with SAMD21 based boards like:
    // Arduino MKR1000, Adafruit Feather M0, ...
    //
    // Some explanations are in order...
    // * You can always communicate to specific digital/analog pins using standard
    //   Arduino functions like digitalWrite(), analogRead(), ... These functions
    //   include some overhead which makes them """"slow"""".
    // * Another way to set pins is to use the memory addresses of the GPIO pins
    //   directly. This is much faster, but obviously much more complicated.
    //
    // The Adafruit code, specifically for 8-bit LCD displays like this one, where
    // you want speed... writes directly to the Atmel registers to control pins
    // and setting them high/low.
    //
    // So to go for speed, we need to do some mapping of the pins where we
    // connected the TFT display to memory addresses. This is done in the following
    // lines which define the PORT and BITMASK to address a specific register.
    //
    // The first part are the LCD control lines that need to be assigned:
    // - RD = Read -> to be wired to pin A0
    // - WR = Write -> to be wired to pin A1
    // - CD = Command/Data -> to be wired to pin A2
    // - CS = Chip Select -> to be wired to pin A3
    //
    // The following code was updated, very heavily inspired by:
    // http://forum.arduino.cc/index.php?topic=334073.msg2308565#msg2308565
    //
    // The analogPinToBitMask and other macros are defined in:
    // packages\arduino\hardware\samd\<version>\variants\mkr1000\variants.h
    // packages\adafruit\hardware\samd\1.0.13\variants\zero_radio\variants.h
    // etc...

    // =============================================================================
    // BOARD SELECTION
    // =============================================================================
    // The next section defines which SAMD21 based board is used. This has a
    // very significant speed impact because some boards allow much more
    // efficient communications with the TFT LCD display when using a specific
    // wiring.
    //
    // Uncomment below the appropriate board for your use case. If all else
    // fails, we'll fall back to slightly more generic code which unfortunately
    // is a bit slower (but still about 2-3 times faster than the very generic
    // code at the end of this file).

    // *****************************************************************************
    // ARDUINO ZERO / ZERO PRO
    // SPARKFUN SAMD21 DEV BREAKOUT (not the mini!)
    // *****************************************************************************
    // Pin connection diagram - regular mode
    // +---------+--------------+--------------
    // + LCD Pin | Arduino pin  | SAMD21G18 PIN
    // +---------+--------------+--------------
    // |  LCD0   | D2           | PA14
    // |  LCD1   | D5           | PA15
    // |  LCD2   | D11          | PA16
    // |  LCD3   | D13          | PA17
    // |  LCD4   | D10          | PA18
    // |  LCD5   | D12          | PA19
    // |  LCD6   | D6           | PA20
    // |  LCD7   | D7           | PA21
    // +---------+--------------+--------------
    // | RD      | A0           | PA02
    // | WR      | A1           | PB08
    // | CD      | A2           | PB09
    // | CS      | A3           | PA04
    // | Reset   | A4           | PA05
    // +---------+--------------+--------------

    // Uncomment the following line to use the above board & pin configuration:
    #define SAMD21LCD_ZEROBOARD

    // On top of the following, you can uncomment the HARDCORE mode for those
    // of you hard core enough to sacrifice the standard I2C bus. This
    // uses the following wiring:
    //
    // +---------+--------------+--------------
    // + LCD Pin | Arduino pin  | SAMD21G18 PIN
    // +---------+--------------+--------------
    // |  LCD0   | D11          | PA16
    // |  LCD1   | D13          | PA17
    // |  LCD2   | D10          | PA18
    // |  LCD3   | D12          | PA19
    // |  LCD4   | D6           | PA20
    // |  LCD5   | D7           | PA21
    // |  LCD6   | SDA = D20    | PA22
    // |  LCD7   | SCL = D21    | PA23
    // +---------+--------------+--------------
    // | RD      | A0           | PA02
    // | WR      | A1           | PB08
    // | CD      | A2           | PB09
    // | CS      | A3           | PA04
    // | Reset   | A4           | PA05
    // +---------+--------------+--------------
    #define SAMD21_LCDZEROBOARD_HARDCORE



    // *****************************************************************************
    // ADAFRUIT FEATHER M0 PROTO
    // *****************************************************************************
    // Pin connection diagram
    // +---------+--------------+--------------
    // + LCD Pin | Arduino pin  | SAMD21G18 PIN
    // +---------+--------------+--------------
    // |  LCD0   | A4           | PA05
    // |  LCD1   | D12          | PA19
    // |  LCD2   | D11          | PA16
    // |  LCD3   | D10          | PA18
    // |  LCD4   | D9           | PA07
    // |  LCD5   | D6           | PA20
    // |  LCD6   | D5           | PA15
    // |  LCD7   | A5           | PB02
    // +---------+--------------+--------------
    // | RD      | A0           | PA02
    // | WR      | A1           | PB08
    // | CD      | A2           | PB09
    // | CS      | A3           | PA04
    // | Reset   | No connect   | N/C
    // +---------+--------------+--------------

    // Uncomment the following line to use the above board & pin configuration:
    //#define SAMD21_FEATHERM0BOARD


    // *****************************************************************************
    // GENERIC SAMD21 BOARD
    // *****************************************************************************
    // Pin connection: see AdaFruit Feather M0 Proto pins
    //
    // If you leave all the above board definitions commented, the generic code
    // will automatically be selected...

    // =============================================================================
    // BOARD SPECIFIC CODE
    // =============================================================================

    #ifdef SAMD21LCD_ZEROBOARD

        #ifndef SAMD21_LCDZEROBOARD_HARDCORE
            #pragma message "Adafruit_TFTLCD: Using Arduino Zero SAMD21 (regular) implementation"

            #define LCD_PORTMASK (PORT_PA14 | PORT_PA15 | PORT_PA16 | PORT_PA17 | PORT_PA18 | PORT_PA19 | PORT_PA20 | PORT_PA21)

            // Reading data
            #define read8inline(result) { \
             RD_ACTIVE;   \
             delayMicroseconds(1);      \
             result = ((REG_PORT_IN0 & LCD_PORTMASK) >> 14); \
             RD_IDLE; }

             // Writing data
             #define write8inline(d) { \
               (REG_PORT_OUTSET0 = ((d) << 14)); \
               (REG_PORT_OUTCLR0 = ((~d) << 14)); \
               WR_STROBE; }

               // Port directions
               //
               // From SAMD 21 Guide (http://www.atmel.com/Images/Atmel-42181-SAM-D21_Datasheet.pdf)
               // Section 23.6.2.2
               // =================
               // To use pin number y as an output, write bit y of the DIR register to '1'. This can also be done by writing bit
               // y in the DIRSET register to '1' - this will avoid disturbing the configuration of other pins in that group. The
               // y bit in the OUT register must be written to the desired output value.
               // Similarly, writing an OUTSET bit to '1' will set the corresponding bit in the OUT register to '1'. Writing a bit
               // in OUTCLR to '1' will set that bit in OUT to zero. Writing a bit in OUTTGL to '1' will toggle that bit in OUT.
               //
               // To use pin y as an input, bit y in the DIR register must be written to '0'. This can also be done by writing
               // bit y in the DIRCLR register to '1' - this will avoid disturbing the configuration of other pins in that group.
               // The input value can be read from bit y in register IN as soon as the INEN bit in the Pin Configuration
               // register (PINCFGy.INEN) is written to '1'.
               //
               // Therefore, we only need to use DIRSET and DIRCLR to set input/output directions. For reading, we also
               // need to set INEN. For that we used the WRCONFIG register.

               #define setWriteDirInline() { \
                 REG_PORT_DIRSET0 = LCD_PORTMASK; \
               }

               // The Write Configuration register (WRCONFIG) requires the
               // pins to to grouped into two 16-bit half-words - split them out here
               #define LCD_LOWERPINMASK (LCD_PORTMASK & 0xFFFF)
               #define LCD_UPPERPINMASK (LCD_PORTMASK & 0xFFFF0000 >> 16)

               #define setReadDirInline() { \
                 REG_PORT_WRCONFIG0 = (PORT_WRCONFIG_PINMASK(LCD_LOWERPINMASK) | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_WRPINCFG); \
                 REG_PORT_WRCONFIG0 = (PORT_WRCONFIG_PINMASK(LCD_UPPERPINMASK) | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_HWSEL); \
                 REG_PORT_DIRCLR0 = LCD_PORTMASK; \
               }
        #else
            #pragma message "Adafruit_TFTLCD: Using Arduino Zero SAMD21 (h4rdc0r3) implementation"

            // Hardcore mode, optimizes further by splitting the 32-bit port register into 8 bit parts,
            // for example:
            //    REG_PORT_IN0+1 for accessing PA08-15
            //    REG_PORT_IN0+2 for accessing PA16-23
            // That allows to use 8 bit operations instead of full 32 bit operations, see:
            // http://community.atmel.com/comment/1676586#comment-1676586
            //
            // Unfortunately that means we need to sacrifice the I2C port. It will be sadly missed...
            #define LCD_PORTMASK (PORT_PA16 | PORT_PA17 | PORT_PA18 | PORT_PA19 | PORT_PA20 | PORT_PA21 | PORT_PA22 | PORT_PA23)

            // Reading data
            #define read8inline(result) { \
             RD_ACTIVE;   \
             delayMicroseconds(1);      \
             result = REG_ACCESS(RoReg8, ((void*)&REG_PORT_IN0)+2); \
             RD_IDLE; }

             // Writing data
             #define write8inline(d) { \
               REG_ACCESS(RwReg8, ((void*)&REG_PORT_OUTSET0)+2) = (RwReg8)(d); \
               REG_ACCESS(RwReg8, ((void*)&REG_PORT_OUTCLR0)+2) = (RwReg8)(~d); \
               WR_STROBE; }

             // In this case, there is only an upper pin mask...
             #define LCD_UPPERPINMASK (LCD_PORTMASK & 0xFFFF0000 >> 16)

             #define setReadDirInline() { \
               REG_PORT_WRCONFIG0 = (PORT_WRCONFIG_PINMASK(LCD_UPPERPINMASK) | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PULLEN | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_HWSEL); \
               REG_PORT_DIRCLR0 = LCD_PORTMASK; \
             }

            //  #define setWriteDirInline() { \
            //    REG_PORT_WRCONFIG0 = ((PORT_WRCONFIG_PINMASK(LCD_UPPERPINMASK) | PORT_WRCONFIG_INEN |  PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_HWSEL) & (~(PORT_WRCONFIG_PULLEN))); \
            //    REG_PORT_DIRSET0 = LCD_PORTMASK; \
            //  }

            // TODO: For some reason the WRCONFIG does not work when using
            // the I2C pins. I guess more configuration work is needed to
            // get them to work... fallback to generic code for now...

            // Pin definitions
            #define SAMD21_LCDDATA1 11
            #define SAMD21_LCDDATA2 13
            #define SAMD21_LCDDATA3 10
            #define SAMD21_LCDDATA4 12
            #define SAMD21_LCDDATA5 6
            #define SAMD21_LCDDATA6 7
            #define SAMD21_LCDDATA7 20
            #define SAMD21_LCDDATA8 21

            // Port definitions
            #define SAMD21_LCD1PORT digitalPinToPort(SAMD21_LCDDATA1)
            #define SAMD21_LCD2PORT digitalPinToPort(SAMD21_LCDDATA2)
            #define SAMD21_LCD3PORT digitalPinToPort(SAMD21_LCDDATA3)
            #define SAMD21_LCD4PORT digitalPinToPort(SAMD21_LCDDATA4)
            #define SAMD21_LCD5PORT digitalPinToPort(SAMD21_LCDDATA5)
            #define SAMD21_LCD6PORT digitalPinToPort(SAMD21_LCDDATA6)
            #define SAMD21_LCD7PORT digitalPinToPort(SAMD21_LCDDATA7)
            #define SAMD21_LCD8PORT digitalPinToPort(SAMD21_LCDDATA8)

            #define digitalPinToPortPin(P) g_APinDescription[P].ulPin

            #define setWriteDirInline() { \
                 SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.INEN = 1; \
                 SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.PULLEN = 0; \
                 SAMD21_LCD1PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA1); \
                 SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.INEN = 1; \
                 SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.PULLEN = 0; \
                 SAMD21_LCD2PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA2); \
                 SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.INEN = 1; \
                 SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.PULLEN = 0; \
                 SAMD21_LCD3PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA3); \
                 SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.INEN = 1; \
                 SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.PULLEN = 0; \
                 SAMD21_LCD4PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA4); \
                 SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.INEN = 1; \
                 SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.PULLEN = 0; \
                 SAMD21_LCD5PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA5); \
                 SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.INEN = 1; \
                 SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.PULLEN = 0; \
                 SAMD21_LCD6PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA6); \
                 SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.INEN = 1; \
                 SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.PULLEN = 0; \
                 SAMD21_LCD7PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA7); \
                 SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.INEN = 1; \
                 SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.PULLEN = 0; \
                 SAMD21_LCD8PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA8); }
        #endif




    #elif defined SAMD21_FEATHERM0BOARD
        #pragma message "Adafruit_TFTLCD: Using AdaFruit Feather M0 SAMD21 implementation"

        // Pin definitions
        #define SAMD21_LCDDATA1 PIN_A4
        #define SAMD21_LCDDATA2 12
        #define SAMD21_LCDDATA3 11
        #define SAMD21_LCDDATA4 10
        #define SAMD21_LCDDATA5 9
        #define SAMD21_LCDDATA6 6
        #define SAMD21_LCDDATA7 5
        #define SAMD21_LCDDATA8 PIN_A5

        // Port definitions
        #define SAMD21_LCD1PORT digitalPinToPort(SAMD21_LCDDATA1)
        #define SAMD21_LCD2PORT digitalPinToPort(SAMD21_LCDDATA2)
        #define SAMD21_LCD3PORT digitalPinToPort(SAMD21_LCDDATA3)
        #define SAMD21_LCD4PORT digitalPinToPort(SAMD21_LCDDATA4)
        #define SAMD21_LCD5PORT digitalPinToPort(SAMD21_LCDDATA5)
        #define SAMD21_LCD6PORT digitalPinToPort(SAMD21_LCDDATA6)
        #define SAMD21_LCD7PORT digitalPinToPort(SAMD21_LCDDATA7)
        #define SAMD21_LCD8PORT digitalPinToPort(SAMD21_LCDDATA8)

        #define digitalPinToPortPin(P) g_APinDescription[P].ulPin

        // Reading data
        #define read8inline(result) { \
         RD_ACTIVE;   \
         delayMicroseconds(1);      \
         result = ( \
           ((REG_PORT_IN1 & PORT_PB02) << -(2-7)) | \
           ((REG_PORT_IN0 & PORT_PA15) >> (15-6)) | \
           ((REG_PORT_IN0 & PORT_PA20) >> (20-5)) | \
           ((REG_PORT_IN0 & PORT_PA07) >> (7-4)) | \
           ((REG_PORT_IN0 & PORT_PA18) >> (18-3)) | \
           ((REG_PORT_IN0 & PORT_PA16) >> (16-2)) | \
           ((REG_PORT_IN0 & PORT_PA19) >> (19-1)) | \
           ((REG_PORT_IN0 & PORT_PA05) >> (5-0)) \
         ); \
         RD_IDLE; }

         // Writing data
         #define portFlexOutputRegister(port,val) ((val>0) ? portOutputSetRegister(port) : portOutputClearRegister(port))
         #define write8inline(d) { \
           *portFlexOutputRegister(SAMD21_LCD1PORT,((d) & 0x01)) = digitalPinToBitMask(SAMD21_LCDDATA1); \
           *portFlexOutputRegister(SAMD21_LCD2PORT,((d) & 0x02)) = digitalPinToBitMask(SAMD21_LCDDATA2); \
           *portFlexOutputRegister(SAMD21_LCD3PORT,((d) & 0x04)) = digitalPinToBitMask(SAMD21_LCDDATA3); \
           *portFlexOutputRegister(SAMD21_LCD4PORT,((d) & 0x08)) = digitalPinToBitMask(SAMD21_LCDDATA4); \
           *portFlexOutputRegister(SAMD21_LCD5PORT,((d) & 0x10)) = digitalPinToBitMask(SAMD21_LCDDATA5); \
           *portFlexOutputRegister(SAMD21_LCD6PORT,((d) & 0x20)) = digitalPinToBitMask(SAMD21_LCDDATA6); \
           *portFlexOutputRegister(SAMD21_LCD7PORT,((d) & 0x40)) = digitalPinToBitMask(SAMD21_LCDDATA7); \
           *portFlexOutputRegister(SAMD21_LCD8PORT,((d) & 0x80)) = digitalPinToBitMask(SAMD21_LCDDATA8); \
           WR_STROBE; }

         // Port directions
         #define setWriteDirInline() { \
           SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.INEN = 1; \
           SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.PULLEN = 0; \
           SAMD21_LCD1PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA1); \
           SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.INEN = 1; \
           SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.PULLEN = 0; \
           SAMD21_LCD2PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA2); \
           SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.INEN = 1; \
           SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.PULLEN = 0; \
           SAMD21_LCD3PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA3); \
           SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.INEN = 1; \
           SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.PULLEN = 0; \
           SAMD21_LCD4PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA4); \
           SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.INEN = 1; \
           SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.PULLEN = 0; \
           SAMD21_LCD5PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA5); \
           SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.INEN = 1; \
           SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.PULLEN = 0; \
           SAMD21_LCD6PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA6); \
           SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.INEN = 1; \
           SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.PULLEN = 0; \
           SAMD21_LCD7PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA7); \
           SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.INEN = 1; \
           SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.PULLEN = 0; \
           SAMD21_LCD8PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA8); }

         #define setReadDirInline() { \
           SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD1PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA1);  \
           SAMD21_LCD1PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA1);  \
           SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD2PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA2);  \
           SAMD21_LCD2PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA2);  \
           SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD3PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA3);  \
           SAMD21_LCD3PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA3);  \
           SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD4PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA4);  \
           SAMD21_LCD4PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA4);  \
           SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD5PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA5);  \
           SAMD21_LCD5PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA5);  \
           SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD6PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA6);  \
           SAMD21_LCD6PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA6);  \
           SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD7PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA7);  \
           SAMD21_LCD7PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA7);  \
           SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
           SAMD21_LCD8PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA8);  \
           SAMD21_LCD8PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA8); }


    #else  // GENERIC SAMD21 BOARD
        #pragma message "Adafruit_TFTLCD: Using generic SAMD21 implementation"

        // Pin definitions
        #define SAMD21_LCDDATA1 PIN_A4
        #define SAMD21_LCDDATA2 12
        #define SAMD21_LCDDATA3 11
        #define SAMD21_LCDDATA4 10
        #define SAMD21_LCDDATA5 9
        #define SAMD21_LCDDATA6 6
        #define SAMD21_LCDDATA7 5
        #define SAMD21_LCDDATA8 PIN_A5

        // Use SAMD21_GENERIC
        #define SAMD21_LCD1PORT digitalPinToPort(SAMD21_LCDDATA1)
        #define SAMD21_LCD2PORT digitalPinToPort(SAMD21_LCDDATA2)
        #define SAMD21_LCD3PORT digitalPinToPort(SAMD21_LCDDATA3)
        #define SAMD21_LCD4PORT digitalPinToPort(SAMD21_LCDDATA4)
        #define SAMD21_LCD5PORT digitalPinToPort(SAMD21_LCDDATA5)
        #define SAMD21_LCD6PORT digitalPinToPort(SAMD21_LCDDATA6)
        #define SAMD21_LCD7PORT digitalPinToPort(SAMD21_LCDDATA7)
        #define SAMD21_LCD8PORT digitalPinToPort(SAMD21_LCDDATA8)

        #define digitalPinToPortPin(P) g_APinDescription[P].ulPin

        // How to read?
        // - First, set READ control pin active
        // - Read the 8 digital pins and form the result
        // - Set the READ control pin idle again

        // How to read a bit?
        // -> First, we get the INPUT register associated to a specific port:
        //       SAMD21_LCD7PORT->IN.reg
        //    This input register has 1 or 0 on specific locations, defined by
        //    the bitmask of that port.
        // -> So we do an AND with the port's bitmask to do a digital read:
        //       SAMD21_LCD7PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA7)
        // -> This has a 1 or 0 on the location of the SAMD21 port corresponding to that PIN,
        //    so we do some bit shifting to make sure this is at the proper location...

        #define read8inline(result) { \
         RD_ACTIVE;   \
         delayMicroseconds(1);      \
         result = ( \
           ((SAMD21_LCD8PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA8)) << -(digitalPinToPortPin(SAMD21_LCDDATA8)-7)) | \
           ((SAMD21_LCD7PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA7)) >> (digitalPinToPortPin(SAMD21_LCDDATA7)-6)) | \
           ((SAMD21_LCD6PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA6)) >> (digitalPinToPortPin(SAMD21_LCDDATA6)-5)) | \
           ((SAMD21_LCD5PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA5)) >> (digitalPinToPortPin(SAMD21_LCDDATA5)-4)) | \
           ((SAMD21_LCD4PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA4)) >> (digitalPinToPortPin(SAMD21_LCDDATA4)-3)) | \
           ((SAMD21_LCD3PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA3)) >> (digitalPinToPortPin(SAMD21_LCDDATA3)-2)) | \
           ((SAMD21_LCD2PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA2)) >> (digitalPinToPortPin(SAMD21_LCDDATA2)-1)) | \
           ((SAMD21_LCD1PORT->IN.reg & digitalPinToBitMask(SAMD21_LCDDATA1)) >> (digitalPinToPortPin(SAMD21_LCDDATA1)-0)) \
         ); \
         RD_IDLE; }
         // TODO: this is not as generic as it should be, as it takes the actual pin into account (first line)...

         // The following macro returns the OUTCLR register if we need to write a 0 to the pin, or the OUTSET register otherwise
         #define portFlexOutputRegister(port,val) ((val>0) ? portOutputSetRegister(port) : portOutputClearRegister(port))

         // How to write 8 bits?
         // - Set all the bits that need to be set to HIGH
         // - Clear all other bits to LOW
         // - Then signal LCD that data is to be read, that is the WR_STROBE macro
         #define write8inline(d) { \
           *portFlexOutputRegister(SAMD21_LCD1PORT,((d) & 0x01)) = digitalPinToBitMask(SAMD21_LCDDATA1); \
           *portFlexOutputRegister(SAMD21_LCD2PORT,((d) & 0x02)) = digitalPinToBitMask(SAMD21_LCDDATA2); \
           *portFlexOutputRegister(SAMD21_LCD3PORT,((d) & 0x04)) = digitalPinToBitMask(SAMD21_LCDDATA3); \
           *portFlexOutputRegister(SAMD21_LCD4PORT,((d) & 0x08)) = digitalPinToBitMask(SAMD21_LCDDATA4); \
           *portFlexOutputRegister(SAMD21_LCD5PORT,((d) & 0x10)) = digitalPinToBitMask(SAMD21_LCDDATA5); \
           *portFlexOutputRegister(SAMD21_LCD6PORT,((d) & 0x20)) = digitalPinToBitMask(SAMD21_LCDDATA6); \
           *portFlexOutputRegister(SAMD21_LCD7PORT,((d) & 0x40)) = digitalPinToBitMask(SAMD21_LCDDATA7); \
           *portFlexOutputRegister(SAMD21_LCD8PORT,((d) & 0x80)) = digitalPinToBitMask(SAMD21_LCDDATA8); \
           WR_STROBE; }

         // These set the PORT directions as required before the write and read
         // operations.  Because write operations are much more common than reads,
         // the data-reading functions in the library code set the PORT(s) to
         // input before a read, and restore them back to the write state before
         // returning.  This avoids having to set it for output inside every
         // drawing method.  The default state has them initialized for writes.
         //
         // Implementation note: the following two macro's are horrible. They are
         // just copies of the relevant parts of pinMode() from wiring_digital.c

         #define setWriteDirInline() { \
            SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.INEN = 1; \
            SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].bit.PULLEN = 0; \
            SAMD21_LCD1PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA1); \
            SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.INEN = 1; \
            SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].bit.PULLEN = 0; \
            SAMD21_LCD2PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA2); \
            SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.INEN = 1; \
            SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].bit.PULLEN = 0; \
            SAMD21_LCD3PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA3); \
            SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.INEN = 1; \
            SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].bit.PULLEN = 0; \
            SAMD21_LCD4PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA4); \
            SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.INEN = 1; \
            SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].bit.PULLEN = 0; \
            SAMD21_LCD5PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA5); \
            SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.INEN = 1; \
            SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].bit.PULLEN = 0; \
            SAMD21_LCD6PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA6); \
            SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.INEN = 1; \
            SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].bit.PULLEN = 0; \
            SAMD21_LCD7PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA7); \
            SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.INEN = 1; \
            SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].bit.PULLEN = 0; \
            SAMD21_LCD8PORT->DIRSET.reg = digitalPinToBitMask(SAMD21_LCDDATA8); }

         #define setReadDirInline() { \
            SAMD21_LCD1PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA1)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD1PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA1);  \
            SAMD21_LCD1PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA1);  \
            SAMD21_LCD2PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA2)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD2PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA2);  \
            SAMD21_LCD2PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA2);  \
            SAMD21_LCD3PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA3)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD3PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA3);  \
            SAMD21_LCD3PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA3);  \
            SAMD21_LCD4PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA4)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD4PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA4);  \
            SAMD21_LCD4PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA4);  \
            SAMD21_LCD5PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA5)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD5PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA5);  \
            SAMD21_LCD5PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA5);  \
            SAMD21_LCD6PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA6)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD6PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA6);  \
            SAMD21_LCD6PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA6);  \
            SAMD21_LCD7PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA7)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD7PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA7);  \
            SAMD21_LCD7PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA7);  \
            SAMD21_LCD8PORT->PINCFG[digitalPinToPortPin(SAMD21_LCDDATA8)].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN); \
            SAMD21_LCD8PORT->DIRCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA8);  \
            SAMD21_LCD8PORT->OUTCLR.reg = digitalPinToBitMask(SAMD21_LCDDATA8); }

    #endif

    // =============================================================================
    // SHARED CODE FOR ALL SAMD21 BOARDS
    // =============================================================================

    #define RD_PORT digitalPinToPort(PIN_A0)    /*pin A0 */
    #define WR_PORT digitalPinToPort(PIN_A1)    /*pin A1 */
    #define CD_PORT digitalPinToPort(PIN_A2)    /*pin A2 */
    #define CS_PORT digitalPinToPort(PIN_A3)    /*pin A3 */
    #define RD_MASK digitalPinToBitMask(PIN_A0)
    #define WR_MASK digitalPinToBitMask(PIN_A1)
    #define CD_MASK digitalPinToBitMask(PIN_A2)
    #define CS_MASK digitalPinToBitMask(PIN_A3)

    // ThingTank TIJA: We define a helper MACRO to set or clear a bit in the
    // PORT OUTPUT register
    // See: https://forum.arduino.cc/index.php?topic=334073.msg2328725#msg2328725
    #ifndef portOutputClearRegister
      #define portOutputClearRegister(port) (&(port->OUTCLR.reg))
    #endif
    #ifndef portOutputSetRegister
      #define portOutputSetRegister(port) (&(port->OUTSET.reg))
    #endif

    // Control signals are ACTIVE LOW (idle is HIGH)
    // Command/Data: LOW = command, HIGH = data
    // These are single-instruction operations and always inline
    #define RD_ACTIVE  *portOutputClearRegister(RD_PORT) = RD_MASK
    #define RD_IDLE    *portOutputSetRegister(RD_PORT) = RD_MASK
    #define WR_ACTIVE  *portOutputClearRegister(WR_PORT) = WR_MASK
    #define WR_IDLE    *portOutputSetRegister(WR_PORT) = WR_MASK
    #define CD_COMMAND *portOutputClearRegister(CD_PORT) = CD_MASK
    #define CD_DATA    *portOutputSetRegister(CD_PORT) = CD_MASK
    #define CS_ACTIVE  *portOutputClearRegister(CS_PORT) = CS_MASK
    #define CS_IDLE    *portOutputSetRegister(CS_PORT) = CS_MASK

    // ThingTank TIJA: Here, you can define which commands will be used "inline"
    // (which means: this macro is expanded at the location where the command is
    // used) and which commands will be defined as a function by Adafruit_TFTLCD.cpp
    // (see code almost at the end of the CPP file).
    //
    // Using it "inline" has the advantage that it is a bit faster (no function call)
    // but takes up more program space as the code is duplicated a dozen times.
    // Using them as functions however is only slight slower (it's just a function
    // call for crying out loud).
    //
    // For "fast" ARM processors such as the SAMD21, you would need to be crazy
    // to want to use them as inline functions (waste of space) so we're going
    // to comment all the following lines, and make sure Adafruit_TFTLCD.cpp
    // defines them as functions...

    //#define write8            write8inline
    //#define read8             read8inline
    //#define setWriteDir       setWriteDirInline
    //#define setReadDir        setReadDirInline
    //#define writeRegister8    writeRegister8inline
    //#define writeRegister16   writeRegister16inline
    //#define writeRegisterPair writeRegisterPairInline





#else
    // =============================================================================
    // GENERIC TFTLCD IMPLEMENATION - GENERAL NOTES (ThingTank TIJA)
    // =============================================================================
    // This is a rewrite of all of the above using standard Arduino functions.
    // That'll make sure that this library can be used, but probably not at the
    // high speed you are going for when using 8-bit parallel processing.
    //
    // I've written it here just to get this working on all Arduino boards.
    //
    // You can change the PIN layout below to fit your needs...
    // Examples:
    //  - To use a digital pin like D10, simply enter "10":
    //     #define TTLCD_DATA5 10
    //  - To use an analog pin, like A5, simply enter "PIN_A5":
    //     #define TTLCD_DATA7 PIN_A5
    //
    #pragma message "Adafruit_TFTLCD: Using generic implementation"

    #define TTLCD_DATA0 11
    #define TTLCD_DATA1 13
    #define TTLCD_DATA2 10
    #define TTLCD_DATA3 12
    #define TTLCD_DATA4 6
    #define TTLCD_DATA5 7
    #define TTLCD_DATA6 20
    #define TTLCD_DATA7 21

    // #define TTLCD_DATA0 2
    // #define TTLCD_DATA1 5
    // #define TTLCD_DATA2 11
    // #define TTLCD_DATA3 13
    // #define TTLCD_DATA4 10
    // #define TTLCD_DATA5 12
    // #define TTLCD_DATA6 6
    // #define TTLCD_DATA7 7


    #define TTLCD_READ PIN_A0
    #define TTLCD_WRITE PIN_A1
    #define TTLCD_CMDDATA PIN_A2
    #define TTLCD_CSELECT PIN_A3

    // Control signals are ACTIVE LOW (idle is HIGH)
    // Command/Data: LOW = command, HIGH = data
    // These are single-instruction operations and always inline
    #define RD_ACTIVE  digitalWrite(TTLCD_READ, LOW)
    #define RD_IDLE    digitalWrite(TTLCD_READ, HIGH)
    #define WR_ACTIVE  digitalWrite(TTLCD_WRITE, LOW)
    #define WR_IDLE    digitalWrite(TTLCD_WRITE, HIGH)
    #define CD_COMMAND digitalWrite(TTLCD_CMDDATA, LOW)
    #define CD_DATA    digitalWrite(TTLCD_CMDDATA, HIGH)
    #define CS_ACTIVE  digitalWrite(TTLCD_CSELECT, LOW)
    #define CS_IDLE    digitalWrite(TTLCD_CSELECT, HIGH)

    #define read8inline(result) { \
     RD_ACTIVE;   \
     delayMicroseconds(1);      \
     result = ((digitalRead(TTLCD_DATA7) << 7) | (digitalRead(TTLCD_DATA6) << 6) | (digitalRead(TTLCD_DATA5) << 5) | (digitalRead(TTLCD_DATA4) << 4) | \
               (digitalRead(TTLCD_DATA3) << 3) | (digitalRead(TTLCD_DATA2) << 2) | (digitalRead(TTLCD_DATA1) << 1) | (digitalRead(TTLCD_DATA0) << 0)); \
     RD_IDLE; }

     #define write8inline(d) { \
       digitalWrite(TTLCD_DATA0,(d) & 0x01); \
       digitalWrite(TTLCD_DATA1,(d) & 0x02); \
       digitalWrite(TTLCD_DATA2,(d) & 0x04); \
       digitalWrite(TTLCD_DATA3,(d) & 0x08); \
       digitalWrite(TTLCD_DATA4,(d) & 0x10); \
       digitalWrite(TTLCD_DATA5,(d) & 0x20); \
       digitalWrite(TTLCD_DATA6,(d) & 0x40); \
       digitalWrite(TTLCD_DATA7,(d) & 0x80); \
       WR_STROBE; }

     #define setWriteDirInline() { \
       pinMode(TTLCD_DATA0, OUTPUT); \
       pinMode(TTLCD_DATA1, OUTPUT); \
       pinMode(TTLCD_DATA2, OUTPUT); \
       pinMode(TTLCD_DATA3, OUTPUT); \
       pinMode(TTLCD_DATA4, OUTPUT); \
       pinMode(TTLCD_DATA5, OUTPUT); \
       pinMode(TTLCD_DATA6, OUTPUT); \
       pinMode(TTLCD_DATA7, OUTPUT); \
     }

     #define setReadDirInline() { \
       pinMode(TTLCD_DATA0, INPUT); \
       pinMode(TTLCD_DATA1, INPUT); \
       pinMode(TTLCD_DATA2, INPUT); \
       pinMode(TTLCD_DATA3, INPUT); \
       pinMode(TTLCD_DATA4, INPUT); \
       pinMode(TTLCD_DATA5, INPUT); \
       pinMode(TTLCD_DATA6, INPUT); \
       pinMode(TTLCD_DATA7, INPUT); \
     }

// END: ThingTank TIJA contribution
#endif




// Data write strobe, ~2 instructions and always inline
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

// These higher-level operations are usually functionalized,
// except on Mega where's there's gobs and gobs of program space.

// Set value of TFT register: 8-bit address, 8-bit value
#define writeRegister8inline(a, d) { \
  CD_COMMAND; write8(a); CD_DATA; write8(d); }

// Set value of TFT register: 16-bit address, 16-bit value
// See notes at top about macro expansion, hence hi & lo temp vars
#define writeRegister16inline(a, d) { \
  uint8_t hi, lo; \
  hi = (a) >> 8; lo = (a); CD_COMMAND; write8(hi); write8(lo); \
  hi = (d) >> 8; lo = (d); CD_DATA   ; write8(hi); write8(lo); }

// Set value of 2 TFT registers: Two 8-bit addresses (hi & lo), 16-bit value
#define writeRegisterPairInline(aH, aL, d) { \
  uint8_t hi = (d) >> 8, lo = (d); \
  CD_COMMAND; write8(aH); CD_DATA; write8(hi); \
  CD_COMMAND; write8(aL); CD_DATA; write8(lo); }

#endif // _pin_magic_
