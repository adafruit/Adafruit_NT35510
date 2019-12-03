#include "Adafruit_GFX.h"
#include "Adafruit_NT35510.h"

#define TFT_D0        14 // Data bit 0 pin (MUST be on PORT 16-bit boundary) (UART3_TX)

// 80 = sd miso = PORTB29 ("topmost" of 4 wires) - yellow
// 81 = sd sck  = PORTB27 - red
// 82 = sd mosi = PORTB26 - brown
// 83 = sd cs   = PORTB28 ("lowest" of 4 wires) - orange
// 95 = sd cd   = PORTB31 - blue
// SWO has no Arduino pin # - must do manually - green
// (hacked variant.cpp to add pin 96 on SWO, PORTB30)

#define TFT_BACKLIGHT A9
#define TFT_RD        A10 // Read-strobe pin
#define TFT_DC        A11 // Data/command pin
#define TFT_WR        A12 // Write-strobe pin (CCL-inverted timer output) (PORTB9 on PyPortal = CCL/OUT[2])
// On Grand Central, PORTB9 is D58, one of the additional ADC pins = A12
#define TFT_CS        A13 // Chip-select pin
#define TFT_RST       A14 // Reset pin

/*

Pin 14  PORTB16
Pin 15  PORTB17
Pin 8   PORTB18
Pin 29  PORTB19
Pin 20  PORTB20
Pin 21  PORTB21
Pin 62  PORTB20 ???
Pin 63  PORTB21 ???
Pin 10  PORTB22
Pin 11  PORTB23
Pin 1   PORTB24
Pin 0   PORTB25
Pin 82  PORTB26
Pin 81  PORTB27
Pin 83  PORTB28
Pin 80  PORTB29
SWO 96  PORTB30
CD 95   PORTB31

Also set up pins 95, 96 as outputs
*/

// NT35510 with 8-bit parallel interface:
Adafruit_NT35510 tft(tft16bitbus,
  TFT_D0, TFT_WR, TFT_DC, TFT_CS, TFT_RST, TFT_RD);

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("NT35510 Test!"); 

// Problem in SPITFT.cpp if multiple Arduino pin numbers are assigned to the same port,
// the code there stops after 16, should keep going. So there's this...
  // 96 (SWO, PORTB30) is not normally in the pin list, need to edit variant.cpp
  pinMode(95, OUTPUT); digitalWrite(95, LOW);
  pinMode(96, OUTPUT); digitalWrite(96, LOW);

  // Turn on backlight (required on PyPortal)
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  tft.begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand16(NT35510_RDDPM);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand16(NT35510_RDDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand16(NT35510_RDDCOLMOD);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand16(NT35510_RDDIM);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand16(NT35510_RDDSDR);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 

  tft.setRotation(0);

  Serial.println(F("Benchmark                Time (microseconds)"));
  delay(10);
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(NT35510_CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(NT35510_RED, NT35510_BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(NT35510_GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(NT35510_YELLOW, NT35510_MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, NT35510_MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, NT35510_WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  Serial.println(F("Done!"));
}


void loop(void) {
  for(uint8_t rotation=0; rotation<4; rotation++) {
    tft.setRotation(rotation);
    testText();
    delay(1000);
  }
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(NT35510_BLACK);
  yield();
  tft.fillScreen(NT35510_RED);
  yield();
  tft.fillScreen(NT35510_GREEN);
  yield();
  tft.fillScreen(NT35510_BLUE);
  yield();
  tft.fillScreen(NT35510_BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(NT35510_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(NT35510_WHITE);  tft.setTextSize(1);
  tft.println(F("Hello World!"));
  tft.setTextColor(NT35510_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(NT35510_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(NT35510_GREEN);
  tft.setTextSize(5);
  tft.println(F("Groop"));
  tft.setTextSize(2);
  tft.println(F("I implore thee,"));
  tft.setTextSize(1);
  tft.println(F("my foonting turlingdromes."));
  tft.println(F("And hooptiously drangle me"));
  tft.println(F("with crinkly bindlewurdles,"));
  tft.println(F("Or I will rend thee"));
  tft.println(F("in the gobberwarts"));
  tft.println(F("with my blurglecruncheon,"));
  tft.println(F("see if I don't!"));
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(NT35510_BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(NT35510_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(NT35510_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(NT35510_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(NT35510_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(NT35510_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(NT35510_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(NT35510_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(NT35510_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(NT35510_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(NT35510_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(NT35510_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}
