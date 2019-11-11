/*!
* @file Adafruit_NT35510.h
*
* This is the documentation for Adafruit's NT35510 driver for the
* Arduino platform.
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* BSD license, all text here must be included in any redistribution.
*
*/

#ifndef _ADAFRUIT_NT35510_H_
#define _ADAFRUIT_NT35510_H_

#include "Arduino.h"
#include "Print.h"
#include <SPI.h>
#include "Adafruit_GFX.h"
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>

#define NT35510_TFTWIDTH   800      ///< NT35510 max TFT width
#define NT35510_TFTHEIGHT  480      ///< NT35510 max TFT height

#define ILI9341_NOP        0x00     ///< No-op register
#define ILI9341_SWRESET    0x01     ///< Software reset register
#define ILI9341_RDDID      0x04     ///< Read display identification information
#define ILI9341_RDDST      0x09     ///< Read Display Status

#define ILI9341_SLPIN      0x10     ///< Enter Sleep Mode
#define ILI9341_SLPOUT     0x11     ///< Sleep Out
#define ILI9341_PTLON      0x12     ///< Partial Mode ON
#define ILI9341_NORON      0x13     ///< Normal Display Mode ON

#define ILI9341_RDMODE     0x0A     ///< Read Display Power Mode
#define ILI9341_RDMADCTL   0x0B     ///< Read Display MADCTL
#define ILI9341_RDPIXFMT   0x0C     ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT   0x0D     ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F     ///< Read Display Self-Diagnostic Result

#define ILI9341_INVOFF     0x20     ///< Display Inversion OFF
#define ILI9341_INVON      0x21     ///< Display Inversion ON
#define ILI9341_GAMMASET   0x26     ///< Gamma Set
#define ILI9341_DISPOFF    0x28     ///< Display OFF
#define ILI9341_DISPON     0x29     ///< Display ON

#define ILI9341_CASET      0x2A     ///< Column Address Set
#define ILI9341_PASET      0x2B     ///< Page Address Set
#define ILI9341_RAMWR      0x2C     ///< Memory Write
#define ILI9341_RAMRD      0x2E     ///< Memory Read

#define ILI9341_PTLAR      0x30     ///< Partial Area
#define ILI9341_VSCRDEF    0x33     ///< Vertical Scrolling Definition
#define ILI9341_MADCTL     0x36     ///< Memory Access Control
#define ILI9341_VSCRSADD   0x37     ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT     0x3A     ///< COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1    0xB1     ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2    0xB2     ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3    0xB3     ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR     0xB4     ///< Display Inversion Control
#define ILI9341_DFUNCTR    0xB6     ///< Display Function Control

#define ILI9341_PWCTR1     0xC0     ///< Power Control 1
#define ILI9341_PWCTR2     0xC1     ///< Power Control 2
#define ILI9341_PWCTR3     0xC2     ///< Power Control 3
#define ILI9341_PWCTR4     0xC3     ///< Power Control 4
#define ILI9341_PWCTR5     0xC4     ///< Power Control 5
#define ILI9341_VMCTR1     0xC5     ///< VCOM Control 1
#define ILI9341_VMCTR2     0xC7     ///< VCOM Control 2

#define ILI9341_RDID1      0xDA     ///< Read ID 1
#define ILI9341_RDID2      0xDB     ///< Read ID 2
#define ILI9341_RDID3      0xDC     ///< Read ID 3
#define ILI9341_RDID4      0xDD     ///< Read ID 4

#define ILI9341_GMCTRP1    0xE0     ///< Positive Gamma Correction
#define ILI9341_GMCTRN1    0xE1     ///< Negative Gamma Correction
//#define ILI9341_PWCTR6     0xFC

// Color definitions
#define NT35510_BLACK       0x0000  ///<   0,   0,   0
#define NT35510_NAVY        0x000F  ///<   0,   0, 123
#define NT35510_DARKGREEN   0x03E0  ///<   0, 125,   0
#define NT35510_DARKCYAN    0x03EF  ///<   0, 125, 123
#define NT35510_MAROON      0x7800  ///< 123,   0,   0
#define NT35510_PURPLE      0x780F  ///< 123,   0, 123
#define NT35510_OLIVE       0x7BE0  ///< 123, 125,   0
#define NT35510_LIGHTGREY   0xC618  ///< 198, 195, 198
#define NT35510_DARKGREY    0x7BEF  ///< 123, 125, 123
#define NT35510_BLUE        0x001F  ///<   0,   0, 255
#define NT35510_GREEN       0x07E0  ///<   0, 255,   0
#define NT35510_CYAN        0x07FF  ///<   0, 255, 255
#define NT35510_RED         0xF800  ///< 255,   0,   0
#define NT35510_MAGENTA     0xF81F  ///< 255,   0, 255
#define NT35510_YELLOW      0xFFE0  ///< 255, 255,   0
#define NT35510_WHITE       0xFFFF  ///< 255, 255, 255
#define NT35510_ORANGE      0xFD20  ///< 255, 165,   0
#define NT35510_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define NT35510_PINK        0xFC18  ///< 255, 130, 198

/**************************************************************************/
/*!
@brief Class to manage hardware interface with NT35510 chipset
*/
/**************************************************************************/

class Adafruit_NT35510 : public Adafruit_SPITFT {
    public:
        Adafruit_NT35510(int8_t _CS, int8_t _DC, int8_t _MOSI, int8_t _SCLK,
          int8_t _RST = -1, int8_t _MISO = -1);
        Adafruit_NT35510(int8_t _CS, int8_t _DC, int8_t _RST = -1);
#if !defined(ESP8266)
        Adafruit_NT35510(SPIClass *spiClass, int8_t dc,
          int8_t cs = -1, int8_t rst = -1);
#endif // end !ESP8266
        Adafruit_NT35510(tftBusWidth busWidth, int8_t d0, int8_t wr, int8_t dc,
          int8_t cs = -1, int8_t rst = -1, int8_t rd = -1);

        void    begin(uint32_t freq=0);
        void    setRotation(uint8_t r);
        void    invertDisplay(bool i);
        void    scrollTo(uint16_t y);
        void    setScrollMargins(uint16_t top, uint16_t bottom);

        // Transaction API not used by GFX
        void    setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

        uint8_t readcommand8(uint8_t reg, uint8_t index=0);
};

#endif // _ADAFRUIT_NT35510_H_
