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

#define NT35510_TFTWIDTH   480      ///< NT35510 max TFT width
#define NT35510_TFTHEIGHT  800      ///< NT35510 max TFT height

#define NT35510_NOP        0x0000   ///< No-op
#define NT35510_SWRESET    0x0100   ///< Software reset
#define NT35510_RDDID      0x0400   ///< Read display ID (0x0400 - 0x0402)
#define NT35510_RDNUMED    0x0500   ///< Read number of errors (DSI only)
#define NT35510_RDDPM      0x0A00   ///< Read Display Power Mode
#define NT35510_RDDMADCTL  0x0B00   ///< Read Display MADCTL
#define NT35510_RDDCOLMOD  0x0C00   ///< Read Display Pixel Format
#define NT35510_RDDIM      0x0D00   ///< Read Display Image Mode
#define NT35510_RDDSM      0x0E00   ///< Read Display Signal Mode
#define NT35510_RDDSDR     0x0F00   ///< Read Display Self-Diagnostic Result
#define NT35510_SLPIN      0x1000   ///< Enter Sleep Mode
#define NT35510_SLPOUT     0x1100   ///< Sleep Out
#define NT35510_PTLON      0x1200   ///< Partial Mode ON
#define NT35510_NORON      0x1300   ///< Normal Display Mode ON
#define NT35510_INVOFF     0x2000   ///< Display Inversion OFF
#define NT35510_INVON      0x2100   ///< Display Inversion ON
#define NT35510_ALLPOFF    0x2200   ///< All pixels off
#define NT35510_ALLPON     0x2300   ///< All pixels on
#define NT35510_GAMSET     0x2600   ///< Gamma Set
#define NT35510_DISPOFF    0x2800   ///< Display OFF
#define NT35510_DISPON     0x2900   ///< Display ON
#define NT35510_CASET      0x2A00   ///< Column Address Set (0x2A00 - 0x2A03)
#define NT35510_RASET      0x2B00   ///< Row Address Set (0x2B00 - 0x2B03)
#define NT35510_RAMWR      0x2C00   ///< Memory Write
#define NT35510_RAMRD      0x2E00   ///< Memory Read
#define NT35510_PTLAR      0x3000   ///< Partial Area (0x3000 - 0x3003)
#define NT35510_TEOFF      0x3400   ///< Tearing effect line off
#define NT35510_TEON       0x3500   ///< Tearing effect line on
#define NT35510_MADCTL     0x3600   ///< Memory Access Control
#define NT35510_IDMOFF     0x3800   ///< Idle mode off
#define NT35510_IDMON      0x3900   ///< Idle mode on
#define NT35510_COLMOD     0x3A00   ///< Interface pixel format
#define NT35510_RAMWRC     0x3C00   ///< Memory write continue
#define NT35510_RAMRDC     0x3E00   ///< Memory read continue
#define NT35510_STESL      0x4400   ///< Set tearing effect line (0x4400-4401)
#define NT35510_GSL        0x4500   ///< Get scan line (0x4500 - 0x4501)
#define NT35510_DPCKRGB    0x4A00   ///< Display clock in RGB interface
#define NT35510_DSTBON     0x4F00   ///< Deep standby mode on
#define NT35510_WRPFD      0x5000   ///< Write profile value for display
#define NT35510_WRDISBV    0x5100   ///< Write display brightness
#define NT35510_RDDISBV    0x5200   ///< Read display brightness
#define NT35510_WRCTRLD    0x5300   ///< Write CTRL display
#define NT35510_RDCTRLD    0x5400   ///< Read CTRL display
#define NT35510_WRCABC     0x5500   ///< Write content adaptive brightness
#define NT35510_RDCABC     0x5600   ///< Read content adaptive brightness
#define NT35510_WRHYSTE    0x5700   ///< Write hysteresis (0x5700 - 0x573F)
#define NT35510_WRGAMMASET 0x5800   ///< Write gamma setting (0x5800 - 0x5807)
#define NT35510_RDFSVM     0x5A00   ///< Read FS value MSBs
#define NT35510_RDFSVL     0x5B00   ///< Read FS value LSBs
#define NT35510_RDMFFSVM   0x5C00   ///< Read median filter FS value MSBs
#define NT35510_RDMFFSVL   0x5D00   ///< Read median filter FS value LSBs
#define NT35510_WRCABCMB   0x5E00   ///< Write CABC minimum brightness
#define NT35510_RDCABCMB   0x5F00   ///< Read CABC minimum brightness
#define NT35510_WRLSCC     0x6500   ///< Write light sensor comp (0x6500-6501)
#define NT35510_RDLSCCM    0x6600   ///< Read light sensor value MSBs
#define NT35510_RDLSCCL    0x6700   ///< Read light sensor value LSBs
#define NT35510_RDBWLB     0x7000   ///< Read black/white low bits
#define NT35510_RDBkx      0x7100   ///< Read Bkx
#define NT35510_RDBky      0x7200   ///< Read Bky
#define NT35510_RDWx       0x7300   ///< Read Wx
#define NT35510_RDWy       0x7400   ///< Read Wy
#define NT35510_RDRGLB     0x7500   ///< Read red/green low bits
#define NT35510_RDRx       0x7600   ///< Read Rx
#define NT35510_RDRy       0x7700   ///< Read Ry
#define NT35510_RDGx       0x7800   ///< Read Gx
#define NT35510_RDGy       0x7900   ///< Read Gy
#define NT35510_RDBALB     0x7A00   ///< Read blue/acolor low bits
#define NT35510_RDBx       0x7B00   ///< Read Bx
#define NT35510_RDBy       0x7C00   ///< Read By
#define NT35510_RDAx       0x7D00   ///< Read Ax
#define NT35510_RDAy       0x7E00   ///< Read Ay
#define NT35510_RDDDBS     0xA100   ///< Read DDB start (0xA100 - 0xA104)
#define NT35510_RDDDBC     0xA800   ///< Read DDB continue (0xA800 - 0xA804)
#define NT35510_RDFCS      0xAA00   ///< Read first checksum
#define NT35510_RDCCS      0xAF00   ///< Read continue checksum
#define NT35510_RDID1      0xDA00   ///< Read ID1 value
#define NT35510_RDID2      0xDB00   ///< Read ID2 value
#define NT35510_RDID3      0xDC00   ///< Read ID3 value

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

    // Transaction API not used by GFX
    void    setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  private:
    void    displayInit(const uint8_t *addr);
};

#endif // _ADAFRUIT_NT35510_H_
