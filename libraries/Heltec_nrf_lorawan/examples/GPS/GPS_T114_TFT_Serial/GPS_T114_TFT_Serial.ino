#include "Arduino.h"
#include "SPI.h"
#include "heltec_nrf_lorawan.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "TinyGPS.h"

#if !defined(ARDUINO_HT_N5262) || defined(HT_N5262_E213) || defined(HT_N5262G) || defined(MESH_SOLAR)
#error "GPS_T114_TFT_Serial is intended for Mesh Node T114 (HT-n5262)."
#endif

#define GPS_POWER_PIN 21
#define GPS_RESET_PIN 38
#define GPS_BAUDRATE_T114 9600
#define SERIAL_BAUDRATE 115200
#define LED_ON_VALUE LOW

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
TinyGPSPlus GPS;

bool gpsStarted = false;
bool firstFixSeen = false;
uint32_t gpsStartTime = 0;
uint32_t firstFixSeconds = 0;
uint32_t lastUpdate = 0;

static String formatDate()
{
  if (!GPS.date.isValid())
  {
    return String("--");
  }

  char str[16];
  snprintf(str, sizeof(str), "%04d-%02d-%02d", GPS.date.year(), GPS.date.month(), GPS.date.day());
  return String(str);
}

static String formatTime()
{
  if (!GPS.time.isValid())
  {
    return String("--");
  }

  char str[16];
  snprintf(str, sizeof(str), "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second());
  return String(str);
}

static void startDisplay()
{
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);
  digitalWrite(PIN_TFT_LEDA_CTL, TFT_LEDA_ENABLE);

  tft.init(135, 240);
  tft.setRotation(3);
  tft.setSPISpeed(40000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextWrap(false);
}

static void startGps()
{
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, HIGH);
  delay(10);

  pinMode(GPS_RESET_PIN, OUTPUT);
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(100);
  digitalWrite(GPS_RESET_PIN, HIGH);

  Serial2.begin(GPS_BAUDRATE_T114);
  gpsStartTime = millis();
  gpsStarted = true;
}

static bool hasRecentFix()
{
  return GPS.location.isValid() && GPS.location.age() < 1000;
}

static void updateFirstFix(bool hasFix)
{
  if (hasFix && !firstFixSeen)
  {
    firstFixSeen = true;
    firstFixSeconds = (millis() - gpsStartTime) / 1000;
  }
}

static void drawDisplay(bool hasFix)
{
  tft.setTextSize(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(hasFix ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setCursor(220, 0);
  tft.println(hasFix ? "A" : "V");

  tft.setTextColor(firstFixSeen ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(200, 100);
  tft.println(String(firstFixSeen ? firstFixSeconds : ((millis() - gpsStartTime) / 1000)));
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  if (GPS.date.isValid())
  {
    tft.setCursor(0, 0);
    tft.println(formatDate());
  }

  if (GPS.time.isValid())
  {
    tft.setCursor(120, 0);
    tft.println(formatTime());
  }

  if (GPS.location.isValid())
  {
    tft.setCursor(0, 20);
    tft.print("lat : ");
    tft.println(String(GPS.location.lat(), 6));

    tft.setCursor(0, 40);
    tft.print("lon : ");
    tft.println(String(GPS.location.lng(), 6));
  }

  if (GPS.altitude.isValid())
  {
    tft.setCursor(0, 60);
    tft.print("alt: ");
    tft.println(String(GPS.altitude.meters(), 2));
  }

  if (GPS.hdop.isValid())
  {
    tft.setCursor(0, 80);
    tft.print("hdop: ");
    tft.println(String(GPS.hdop.hdop(), 2));
  }

  if (GPS.speed.isValid())
  {
    tft.setCursor(0, 100);
    tft.print("speed: ");
    tft.print(String(GPS.speed.kmph(), 2));
    tft.println(" km/h");
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  startDisplay();
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, !LED_ON_VALUE);

  startGps();
}

void loop()
{
  if (!gpsStarted)
  {
    startGps();
  }

  while (Serial2.available() > 0)
  {
    uint8_t data = Serial2.read();
    Serial.write(data);
    GPS.encode(data);
  }

  if (millis() - lastUpdate >= 1000)
  {
    lastUpdate = millis();
    bool hasFix = hasRecentFix();
    updateFirstFix(hasFix);
    digitalWrite(PIN_LED1, hasFix ? LED_ON_VALUE : !LED_ON_VALUE);
    drawDisplay(hasFix);
  }
}
