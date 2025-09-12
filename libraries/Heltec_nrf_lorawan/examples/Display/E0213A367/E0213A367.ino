#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "epd/GxEPD2_213_E0213A367.h"

#if defined(MESH_SOLAR)&&(SPI_INTERFACES_COUNT!=2)
#error "To reuse pins, please modify SPI_INTERFACES_COUNT=2."
#endif

// HT_E0213A367(40 _rst, 13 _dc, 41 _cs, 11 _busy, 12 _sck, 2 _mosi, int8_t _miso, uint32_t _freq = 6000000, DISPLAY_GEOMETRY g = GEOMETRY_250_122)

GxEPD2_213_E0213A367 lowLevel = GxEPD2_213_E0213A367(41 , 13 , 40 , 11 , SPI1);
GxEPD2_BW<GxEPD2_213_E0213A367, GxEPD2_213_E0213A367::HEIGHT> display =  GxEPD2_BW<GxEPD2_213_E0213A367, GxEPD2_213_E0213A367::HEIGHT>(lowLevel);

void setup()
{
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);

  SPI1.begin();
  display.init();
  helloWorld();
  display.hibernate();
}

const char HelloWorld[] = "Hello World!";

void helloWorld()
{
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(HelloWorld);
  }
  while (display.nextPage());
}

void loop() {};
