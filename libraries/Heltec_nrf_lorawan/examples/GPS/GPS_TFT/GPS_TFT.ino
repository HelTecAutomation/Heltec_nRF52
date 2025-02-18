/*
 * GPS Tracker with Display and LED Indicators
 *
 * This program interfaces with a GPS module using Serial2 communication 
 * and displays the GPS data on an ST7789 TFT display. It shows the current 
 * date, time, latitude, longitude, altitude, horizontal dilution of precision 
 * (HDOP), and speed on the screen. The LED (connected to pin 35) serves as 
 * an indicator to show GPS status—blinking when a valid GPS location is 
 * received and solid when the GPS signal is locked. The screen will display 
 * real-time updates, and the GPS test can be started by setting the `gps_test_start` variable to `true`.
 * 
 * Features:
 * - Displays GPS data (latitude, longitude, altitude, etc.)
 * - Indicates GPS lock status with an LED (Green: valid location, White: invalid location)
 * - Updates display with date and time
 * - Shows the GPS signal strength and status (A for valid, V for invalid)
 * 
 * The TFT display uses the ST7789 driver and is configured for 240x135 resolution.
 */

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include "SPI.h"
#include "heltec_nrf_lorawan.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "TinyGPS.h"

#define LED 35
#define LED_ON_VALUE LOW

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
TinyGPSPlus GPS;

bool gps_test_start = true; // 设置为 true 以启动 GPS 测试
bool first_get_location = true;
uint32_t gps_get_time = 0;
bool gps_started = false;

static void st7789Start() {
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL, LOW);  // 启用 TFT VDD
    digitalWrite(PIN_TFT_LEDA_CTL, LOW); // 启用 TFT LEDA
    tft.init(135, 240);                  // 初始化 ST7789 240x135
    tft.setRotation(3);
    tft.setSPISpeed(40000000);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2); // 12*16
    tft.setTextWrap(false);
}

void setup() {
    InternalFS.begin();
    st7789Start();
    pinMode(LED, OUTPUT);
    digitalWrite(LED, !LED_ON_VALUE);

    // 初始化 Serial2 用于 GPS 通信
    Serial2.begin(9600);
}

void loop() {
    if (gps_test_start) {
        if (!gps_started) {
            gps_started = true;
            pinMode(21, OUTPUT);
            digitalWrite(21, 1);
            delay(10);
            pinMode(38, OUTPUT);
            digitalWrite(38, 0);
            delay(100);
            digitalWrite(38, 1);
        }

        uint32_t starttime = millis();
        while ((millis() - starttime) < 1000) {
            while (Serial2.available() > 0) {
                uint8_t a = Serial2.read();
                GPS.encode(a);
            }
        }

        tft.setTextSize(1);
        tft.fillScreen(ST77XX_BLACK);

        if (gps_get_time != 0) {
            if (gps_get_time < 90) {
                tft.setTextColor(ST77XX_GREEN);
            } else if (gps_get_time < 180) {
                tft.setTextColor(ST77XX_YELLOW);
            } else {
                tft.setTextColor(ST77XX_RED);
            }
            tft.setTextSize(2);
            tft.setCursor(200, 100);
            tft.println(String(gps_get_time));
            tft.setTextSize(1);
        }

        if (GPS.location.isValid() && GPS.location.age() < 1000) {
            digitalWrite(LED, LED_ON_VALUE);
            tft.setTextColor(ST77XX_GREEN);
            tft.setCursor(220, 0);
            tft.println("A");
            if (first_get_location) {
                first_get_location = false;
                gps_get_time = (millis() - starttime) / 1000;
            }
        } else {
            tft.setCursor(220, 0);
            tft.setTextColor(ST77XX_WHITE);
            tft.println("V");
            digitalWrite(LED, !LED_ON_VALUE);
        }

        char str[30];
        if (GPS.date.isValid()) {
            int index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
            str[index] = 0;
            tft.setCursor(0, 0);
            tft.println(str);
        }

        if (GPS.time.isValid()) {
            int index = sprintf(str, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second());
            str[index] = 0;
            tft.setCursor(120, 0);
            tft.println(str);
        }

        if (GPS.location.isValid()) {
            int index = sprintf(str, "lat :  %d.%04d", (int)GPS.location.lat(), (int)(fabs(GPS.location.lat() - (int)GPS.location.lat()) * 10000));
            str[index] = 0;
            tft.setCursor(0, 20);
            tft.println(str);

            index = sprintf(str, "lon : %d.%04d", (int)GPS.location.lng(), (int)(fabs(GPS.location.lng() - (int)GPS.location.lng()) * 10000));
            str[index] = 0;
            tft.setCursor(0, 40);
            tft.println(str);
        }

        if (GPS.altitude.isValid()) {
            int index = sprintf(str, "alt: %d.%02d", (int)GPS.altitude.meters(), (int)(fabs(GPS.altitude.meters() - (int)GPS.altitude.meters())) * 100);
            str[index] = 0;
            tft.setCursor(0, 60);
            tft.println(str);
        }

        if (GPS.hdop.isValid()) {
            int index = sprintf(str, "hdop: %d.%02d", (int)GPS.hdop.hdop(), (int)(fabs(GPS.hdop.hdop() - (int)GPS.hdop.hdop()) * 100));
            str[index] = 0;
            tft.setCursor(0, 80);
            tft.println(str);
        }

        if (GPS.speed.isValid()) {
            int index = sprintf(str, "speed: %d.%02d km/h", (int)GPS.speed.kmph(), (int)(fabs(GPS.speed.kmph() - (int)GPS.speed.kmph()) * 100));
            str[index] = 0;
            tft.setCursor(0, 100);
            tft.println(str);
        }
    }
}