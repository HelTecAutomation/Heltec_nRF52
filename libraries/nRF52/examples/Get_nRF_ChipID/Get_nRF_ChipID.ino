#include "ht_nrf_board.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

uint64_t ChipID = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ChipID = getID();

  pinMode(PIN_TFT_VDD_CTL,OUTPUT); //TFT VDD ENABLE
  digitalWrite(PIN_TFT_VDD_CTL,0);
  pinMode(PIN_TFT_LEDA_CTL,OUTPUT); //LEDA ENABLE
  digitalWrite(PIN_TFT_LEDA_CTL,0);

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(40000000);

  tft.init(135, 240);           // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setCursor(0, 0);
  tft.println("nRF52 ChipID:");
  tft.setCursor(0, 60);
  tft.printf("%04x",(uint16_t)(ChipID>>32));
  tft.printf("%08x\r\n",(uint32_t)ChipID);
}

void loop()
{
  ChipID = getID();
  Serial.printf("nRF ChipID=%04x",(uint16_t)(ChipID>>32));
  Serial.printf("%08x\r\n",(uint32_t)ChipID);//print Low 4bytes.
  delay(4000);
}
