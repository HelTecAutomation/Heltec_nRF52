#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "epd/GxEPD2_213_B74.h"

GxEPD2_213_B74 lowLevel = GxEPD2_213_B74(PIN_DISPLAY_CS, PIN_DISPLAY_DC, PIN_DISPLAY_RST, PIN_DISPLAY_BUSY , SPI1);
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display =  GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT>(lowLevel);

void setup()
{

  SPI1.begin();
  display.init();
  helloWorld();
  display.hibernate();
}

const char HelloWorld[] = "Hello World!";

void helloWorld()
{
  display.setRotation(3);
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
