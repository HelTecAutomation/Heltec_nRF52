#include "Arduino.h"
#include "SPI.h"
#include "heltec_nrf_lorawan.h"
// Install Arduino_GFX separately; it is not bundled with this hardware package.
#include <Arduino_GFX_Library.h>
#include "TinyGPS.h"

#if !defined(HELTEC_MESH_POCKET_V2)
#error "GPS_Mesh_Pocket_V2_TFT_Serial is intended for Mesh Pocket V2 (HT-mesh-pocket-v2)."
#endif

#define SERIAL_BAUDRATE 115200
#define DISPLAY_ROTATION 3
#define DISPLAY_IPS true
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 220
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_SPI_FREQUENCY 4000000

Arduino_DataBus *bus = new Arduino_HWSPI(PIN_TFT_DC, PIN_TFT_CS, &SPI1, true);
Arduino_GFX *tft = new Arduino_NV3001B(bus, PIN_TFT_RST, DISPLAY_ROTATION, DISPLAY_IPS, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                       DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
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
  digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);
  delay(100);

  pinMode(PIN_IO_CSA, OUTPUT);
  digitalWrite(PIN_IO_CSA, HIGH);
  delay(100);

  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_LEDA_CTL, TFT_LEDA_ENABLE);
  delay(100);

  if (!tft->begin(DISPLAY_SPI_FREQUENCY))
  {
    Serial.println("tft->begin() failed!");
  }

  tft->setRotation(DISPLAY_ROTATION);
  tft->fillScreen(RGB565_BLACK);
  tft->setTextSize(2);
  tft->setTextWrap(false);
}

static void startGps()
{
  Serial.print("GPS power pin: ");
  Serial.print(PIN_GPS_EN);
  Serial.print(", reset pin: ");
  Serial.print(PIN_GPS_RESET);
  Serial.print(", RX/TX: ");
  Serial.print(PIN_SERIAL1_RX);
  Serial.print("/");
  Serial.println(PIN_SERIAL1_TX);

  pinMode(PIN_GPS_EN, OUTPUT);
  digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
  delay(10);

  pinMode(PIN_GPS_RESET, OUTPUT);
  digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE);
  delay(100);
  digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);

  Serial1.setPins(PIN_SERIAL1_RX, PIN_SERIAL1_TX);
  Serial1.begin(GPS_BAUDRATE);
  Serial.print("GPS Serial1 started at ");
  Serial.println(GPS_BAUDRATE);
  Serial.println("GPS raw output follows:");

  gpsStartTime = millis();
  gpsStarted = true;
}

static void startUsbSerial()
{
  Serial.begin(SERIAL_BAUDRATE);

  uint32_t start = millis();
  while (!Serial && (millis() - start < 2000))
  {
    delay(10);
  }

  Serial.println();
  Serial.println("GPS Mesh Pocket V2 TFT Serial");
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
  tft->setTextSize(1);
  tft->fillScreen(RGB565_BLACK);

  tft->setTextColor(hasFix ? RGB565_GREEN : RGB565_WHITE);
  tft->setCursor(200, 0);
  tft->println(hasFix ? "A" : "V");

  tft->setTextColor(firstFixSeen ? RGB565_GREEN : RGB565_WHITE);
  tft->setTextSize(2);
  tft->setCursor(170, 100);
  tft->println(String(firstFixSeen ? firstFixSeconds : ((millis() - gpsStartTime) / 1000)));
  tft->setTextSize(1);
  tft->setTextColor(RGB565_WHITE);

  if (GPS.date.isValid())
  {
    tft->setCursor(0, 0);
    tft->println(formatDate());
  }

  if (GPS.time.isValid())
  {
    tft->setCursor(110, 0);
    tft->println(formatTime());
  }

  if (GPS.location.isValid())
  {
    tft->setCursor(0, 20);
    tft->print("lat : ");
    tft->println(String(GPS.location.lat(), 6));

    tft->setCursor(0, 40);
    tft->print("lon : ");
    tft->println(String(GPS.location.lng(), 6));
  }

  if (GPS.altitude.isValid())
  {
    tft->setCursor(0, 60);
    tft->print("alt: ");
    tft->println(String(GPS.altitude.meters(), 2));
  }

  if (GPS.hdop.isValid())
  {
    tft->setCursor(0, 80);
    tft->print("hdop: ");
    tft->println(String(GPS.hdop.hdop(), 2));
  }

  if (GPS.speed.isValid())
  {
    tft->setCursor(0, 100);
    tft->print("speed: ");
    tft->print(String(GPS.speed.kmph(), 2));
    tft->println(" km/h");
  }
}

void setup()
{
  startUsbSerial();
  startDisplay();
  startGps();
}

void loop()
{
  if (!gpsStarted)
  {
    startGps();
  }

  while (Serial1.available() > 0)
  {
    uint8_t data = Serial1.read();
    Serial.write(data);
    GPS.encode(data);
  }

  if (millis() - lastUpdate >= 1000)
  {
    lastUpdate = millis();
    bool hasFix = hasRecentFix();
    updateFirstFix(hasFix);
    drawDisplay(hasFix);
  }
}
