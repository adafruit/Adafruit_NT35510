/*!
* @file Adafruit_NT35510.cpp
*
* @mainpage Adafruit NT35510 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for Adafruit's NT35510 driver for the
* Arduino platform.
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include "Adafruit_NT35510.h"
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #ifndef RASPI
    #include "wiring_private.h"
  #endif
#endif
#include <limits.h>

#if defined (ARDUINO_ARCH_ARC32) || defined (ARDUINO_MAXIM)
  #define SPI_DEFAULT_FREQ  16000000
// Teensy 3.0, 3.1/3.2, 3.5, 3.6
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined (__AVR__) || defined(TEENSYDUINO)
  #define SPI_DEFAULT_FREQ  8000000
#elif defined(ESP8266) || defined(ESP32)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined(RASPI)
  #define SPI_DEFAULT_FREQ  80000000
#elif defined(ARDUINO_ARCH_STM32F1)
  #define SPI_DEFAULT_FREQ  36000000
#else
  #define SPI_DEFAULT_FREQ  24000000  ///< Default SPI data clock frequency
#endif

#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Row/Column exchange
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit NT35510 driver with software SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_NT35510::Adafruit_NT35510(int8_t cs, int8_t dc, int8_t mosi,
  int8_t sclk, int8_t rst, int8_t miso) : Adafruit_SPITFT(NT35510_TFTWIDTH,
  NT35510_TFTHEIGHT, cs, dc, mosi, sclk, rst, miso) {
}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit NT35510 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_NT35510::Adafruit_NT35510(int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_SPITFT(NT35510_TFTWIDTH, NT35510_TFTHEIGHT, cs, dc, rst) {
}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit NT35510 driver with hardware SPI using
            a specific SPI peripheral (not necessarily default).
    @param  spiClass  Pointer to SPI peripheral (e.g. &SPI or &SPI1).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and
                      CS is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_NT35510::Adafruit_NT35510(
  SPIClass *spiClass, int8_t dc, int8_t cs, int8_t rst) :
  Adafruit_SPITFT(NT35510_TFTWIDTH, NT35510_TFTHEIGHT, spiClass, cs, dc, rst) {
}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit NT35510 driver using parallel interface.
    @param  busWidth  If tft16 (enumeration in Adafruit_SPITFT.h), is a
                      16-bit interface, else 8-bit.
    @param  d0        Data pin 0 (MUST be a byte- or word-aligned LSB of a
                      PORT register -- pins 1-n are extrapolated from this).
    @param  wr        Write strobe pin # (required).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and CS
                      is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
    @param  rd        Read strobe pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_NT35510::Adafruit_NT35510(tftBusWidth busWidth,
  int8_t d0, int8_t wr, int8_t dc, int8_t cs, int8_t rst, int8_t rd) :
  Adafruit_SPITFT(NT35510_TFTWIDTH, NT35510_TFTHEIGHT, busWidth,
    d0, wr, dc, cs, rst, rd) {
}

// Decompose a 16-bit display command into high & low uint8_t's for array
#define CmdHiLo(_x_) (_x_ >> 8), (_x_ & 0xFF)

// Adapted from EastRising example code:
static const uint8_t PROGMEM
initCmd1[] = {
  // Most of these registers are not in the datasheet
  // 35510h	
  CmdHiLo(0xF000), 5, 0x55, 0xAA, 0x52, 0x08, 0x01,
  // AVDD Set AVDD 5.2V
  CmdHiLo(0xB000), 3, 0x0D, 0x0D, 0x0D,
  // AVDD ratio
  CmdHiLo(0xB600), 3, 0x34, 0x34, 0x34,
  // AVEE  -5.2V
  CmdHiLo(0xB100), 3, 0x0D, 0x0D, 0x0D,
  // AVEE ratio
  CmdHiLo(0xB700), 3, 0x34, 0x34, 0x34,
  // VCL  -2.5V
  CmdHiLo(0xB200), 3, 0x00, 0x00, 0x00,
  // VCL ratio
  CmdHiLo(0xB800), 3, 0x24, 0x24, 0x24,
  // VGH  15V
  CmdHiLo(0xBF00), 1, 0x01,
  CmdHiLo(0xB300), 3, 0x0F, 0x0F, 0x0F,
  // VGH  ratio
  CmdHiLo(0xB900), 3, 0x34, 0x34, 0x34,
  // VGL_REG  -10V
  CmdHiLo(0xB500), 1, 0x08,
  CmdHiLo(0xB500), 2, 0x08, 0x08,
  CmdHiLo(0xC200), 1, 0x03,
  // VGLX  ratio
  CmdHiLo(0xBA00), 3, 0x24, 0x24, 0x24,
  // VGMP/VGSP 4.5V/0V
  CmdHiLo(0xBC00), 3, 0x00, 0x78, 0x00,
  // VGMN/VGSN -4.5V/0V
  CmdHiLo(0xBD00), 3, 0x00, 0x78, 0x00,
  // VCOM  -1.325V
  CmdHiLo(0xBE00), 2, 0x00, 0x89,
  // Gamma Setting	 
  CmdHiLo(0xD100), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  CmdHiLo(0xD200), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  CmdHiLo(0xD300), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  CmdHiLo(0xD400), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  CmdHiLo(0xD500), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  CmdHiLo(0xD600), 52,
    0x00, 0x2D, 0x00, 0x2E, 0x00, 0x32, 0x00, 0x44, 0x00, 0x53,
    0x00, 0x88, 0x00, 0xB6, 0x00, 0xF3, 0x01, 0x22, 0x01, 0x64,
    0x01, 0x92, 0x01, 0xD4, 0x02, 0x07, 0x02, 0x08, 0x02, 0x34,
    0x02, 0x5F, 0x02, 0x78, 0x02, 0x94, 0x02, 0xA6, 0x02, 0xBB,
    0x02, 0xCA, 0x02, 0xDB, 0x02, 0xE8, 0x02, 0xF9, 0x03, 0x1F,
    0x03, 0x7F,
  // LV2 Page 0 enable
  CmdHiLo(0xF000), 5, 0x55, 0xAA, 0x52, 0x08, 0x00,
  // Display control
  CmdHiLo(0xB100), 2, 0xCC, 0x00,
  0, },
initCmdHSDLCD[] = { // IPS
  CmdHiLo(0xB500), 1, 0x6b,
  0, },
initCmdCPTLCD[] = { // TN
  CmdHiLo(0xB500), 1, 0x50,
  0, },
initCmd2[] = {
  // Source hold time
  CmdHiLo(0xB600), 1, 0x05,
  // Set Gate EQ
  CmdHiLo(0xB700), 2, 0x70, 0x70,
  // Source EQ control (Mode 2)
  CmdHiLo(0xB800), 4, 0x01, 0x03, 0x03, 0x03,
  // INVERSION MODE
  CmdHiLo(0xBC00), 3, 0x02, 0x00, 0x00,
  // Timing control
  CmdHiLo(0xC900), 5, 0xD0, 0x02, 0x50, 0x50, 0x50,
  CmdHiLo(NT35510_TEON)  ,   1, 0x00, // Tearing effect line on
  CmdHiLo(NT35510_COLMOD),   1, 0x55, // Data format 16 bits
  CmdHiLo(NT35510_MADCTL),   1, 0x00, // Memory access control
  CmdHiLo(NT35510_SLPOUT), 128,       // Sleep out, no args, delay 120 ms
  CmdHiLo(NT35510_DISPON), 128,       // Display on, no args, delay 120 ms
  0,
};

/**************************************************************************/
/*!
    @brief  Companion code to the initialization tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_NT35510::displayInit(const uint8_t *addr) {
    uint8_t        cmdHi, cmdLo, x, numArgs;
    while((cmdHi = pgm_read_byte(addr++)) > 0) {
        cmdLo    = pgm_read_byte(addr++);
        x        = pgm_read_byte(addr++);
        numArgs  = x & 0x7F;
        sendCommand16((cmdHi << 8) | cmdLo, addr, numArgs);
        addr += numArgs;
        if(x & 0x80) delay(120);
    }
}

/**************************************************************************/
/*!
    @brief  Initialize NT35510 chip
    Connects to the NT35510 over SPI and sends initialization commands.
    @param  freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_NT35510::begin(uint32_t freq) {

    if(!freq) freq = SPI_DEFAULT_FREQ;
    initSPI(freq);

    if(_rst < 0) {                      // If no hardware reset pin...
        sendCommand16(NT35510_SWRESET); // Engage software reset
        delay(150);
    }

    // Note: hardware reset in Adafruit_SPITFT.cpp is currently
    // 100, 100, 200 ms for high, low, high. EastRising example
    // library uses 200, 800, 800 for same. May need to change
    // SPITFT if screen fails to init.

    displayInit(initCmd1);      // Common init, part 1
    //displayInit(initCmdHSDLCD); // IPS-specific init
    displayInit(initCmdCPTLCD); // TN-specific init
    displayInit(initCmd2);      // Common init, continued

    _width  = NT35510_TFTWIDTH;
    _height = NT35510_TFTHEIGHT;
}

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   r  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_NT35510::setRotation(uint8_t r) {
    uint8_t m;
    switch(r & 3) {
        case 0:
            m       = MADCTL_BGR;
            _width  = NT35510_TFTWIDTH;
            _height = NT35510_TFTHEIGHT;
            break;
        case 1:
            m       = (MADCTL_MV | MADCTL_MX | MADCTL_BGR);
            _width  = NT35510_TFTHEIGHT;
            _height = NT35510_TFTWIDTH;
            break;
        case 2:
            m       = (MADCTL_MX | MADCTL_MY | MADCTL_BGR);
            _width  = NT35510_TFTWIDTH;
            _height = NT35510_TFTHEIGHT;
            break;
        case 3:
            m       = (MADCTL_MV | MADCTL_MY | MADCTL_BGR);
            _width  = NT35510_TFTHEIGHT;
            _height = NT35510_TFTWIDTH;
            break;
    }

    sendCommand16(NT35510_MADCTL, &m, 1);
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_NT35510::invertDisplay(bool invert) {
    sendCommand16(invert ? NT35510_INVON : NT35510_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM
             with the next chunk of SPI data writes. The NT35510 will
             automatically wrap the data as each row is filled.
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void Adafruit_NT35510::setAddrWindow(
  uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = (x1 + w - 1),
             y2 = (y1 + h - 1);
    // Unfortunately yes, it DOES seem to be necessary
    // to writeCommand for each byte out.
#if 1
    // This is SUPER rigged for 16-bit parallel interface
    // with SAMD51 (has port set/clear registers)
    SPI_DC_LOW(); // Command
    *(volatile uint16_t *)tft8.writePort = NT35510_CASET;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH(); // Data
    *(volatile uint16_t *)tft8.writePort = x1 >> 8;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_CASET + 1;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = x1 & 0xFF;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_CASET + 2;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = x2 >> 8;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_CASET + 3;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = x2 & 0xFF;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_RASET;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = y1 >> 8;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_RASET + 1;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = y1 & 0xFF;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_RASET + 2;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = y2 >> 8;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_RASET + 3;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
    *(volatile uint16_t *)tft8.writePort = y2 & 0xFF;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;

    SPI_DC_LOW();
    *(volatile uint16_t *)tft8.writePort = NT35510_RAMWR;
    *tft8.wrPortClr = tft8.wrPinMask; *tft8.wrPortSet = tft8.wrPinMask;
    SPI_DC_HIGH();
#else
    writeCommand16(NT35510_CASET); // Column address set
    SPI_WRITE16(x1 >> 8);
    writeCommand16(NT35510_CASET + 1);
    SPI_WRITE16(x1 & 0xFF);
    writeCommand16(NT35510_CASET + 2);
    SPI_WRITE16(x2 >> 8);
    writeCommand16(NT35510_CASET + 3);
    SPI_WRITE16(x2 & 0xFF);
    writeCommand16(NT35510_RASET); // Row address set
    SPI_WRITE16(y1 >> 8);
    writeCommand16(NT35510_RASET + 1);
    SPI_WRITE16(y1 & 0xFF);
    writeCommand16(NT35510_RASET + 2);
    SPI_WRITE16(y2 >> 8);
    writeCommand16(NT35510_RASET + 3);
    SPI_WRITE16(y2 & 0xFF);
    writeCommand16(NT35510_RAMWR); // Write to RAM
#endif
}

