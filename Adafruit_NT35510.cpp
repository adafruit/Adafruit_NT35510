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
#define MADCTL_MV  0x20  ///< Reverse Mode
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


static const uint8_t PROGMEM initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

/**************************************************************************/
/*!
    @brief   Initialize NT35510 chip
    Connects to the NT35510 over SPI and sends initialization commands.
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_NT35510::begin(uint32_t freq) {

    if(!freq) freq = SPI_DEFAULT_FREQ;
    initSPI(freq);

    if(_rst < 0) {                     // If no hardware reset pin...
        sendCommand(ILI9341_SWRESET); // Engage software reset
        delay(150);
    }

    uint8_t        cmd, x, numArgs;
    const uint8_t *addr = initcmd;
    while((cmd = pgm_read_byte(addr++)) > 0) {
        x = pgm_read_byte(addr++);
        numArgs = x & 0x7F;
        sendCommand(cmd, addr, numArgs);
        addr += numArgs;
        if(x & 0x80) delay(150);
    }

    _width  = NT35510_TFTWIDTH;
    _height = NT35510_TFTHEIGHT;
}


/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_NT35510::setRotation(uint8_t m) {
    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            m = (MADCTL_MX | MADCTL_BGR);
            _width  = NT35510_TFTWIDTH;
            _height = NT35510_TFTHEIGHT;
            break;
        case 1:
            m = (MADCTL_MV | MADCTL_BGR);
            _width  = NT35510_TFTHEIGHT;
            _height = NT35510_TFTWIDTH;
            break;
        case 2:
            m = (MADCTL_MY | MADCTL_BGR);
            _width  = NT35510_TFTWIDTH;
            _height = NT35510_TFTHEIGHT;
            break;
        case 3:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            _width  = NT35510_TFTHEIGHT;
            _height = NT35510_TFTWIDTH;
            break;
    }

    sendCommand(NT35510_MADCTL, &m, 1);
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_NT35510::invertDisplay(bool invert) {
    sendCommand(invert ? ILI9341_INVON : ILI9341_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void Adafruit_NT35510::scrollTo(uint16_t y) {
    uint8_t data[2];
    data[0] = y >> 8;
    data[1] = y & 0xff;
    sendCommand(ILI9341_VSCRSADD, (uint8_t*) data, 2);
}

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void Adafruit_NT35510::setScrollMargins(uint16_t top, uint16_t bottom) {
  // TFA+VSA+BFA must equal 320
  if (top + bottom <= NT35510_TFTHEIGHT) {
    uint16_t middle = NT35510_TFTHEIGHT - top + bottom;
    uint8_t data[6];
    data[0] = top >> 8;
    data[1] = top & 0xff;
    data[2] = middle >> 8;
    data[3] = middle & 0xff;
    data[4] = bottom >> 8;
    data[5] = bottom & 0xff;
    sendCommand(ILI9341_VSCRDEF, (uint8_t*) data, 6);
  }
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
    writeCommand(ILI9341_CASET); // Column address set
    SPI_WRITE16(x1);
    SPI_WRITE16(x2);
    writeCommand(ILI9341_PASET); // Row address set
    SPI_WRITE16(y1);
    SPI_WRITE16(y2);
    writeCommand(ILI9341_RAMWR); // Write to RAM
}

/**************************************************************************/
/*!
    @brief   Read 8 bits of data from NT35510 configuration memory. NOT from
             RAM! This is highly undocumented/supported, it's really a hack
             but kinda works?
    @param   commandByte  The command register to read data from
    @param   index  The byte index into the command to read from
    @return  Unsigned 8-bit data read from NT35510 register
 */
/**************************************************************************/
uint8_t Adafruit_NT35510::readcommand8(uint8_t commandByte, uint8_t index) {
  uint8_t data = 0x10 + index;
  sendCommand(0xD9, &data, 1); // Set Index Register
  return Adafruit_SPITFT::readcommand8(commandByte);
}
