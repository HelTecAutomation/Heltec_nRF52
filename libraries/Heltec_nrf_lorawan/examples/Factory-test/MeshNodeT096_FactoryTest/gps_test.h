#include "Arduino.h"
#include <Adafruit_ST7735.h>

#define GNSS_UART Serial2

extern Adafruit_ST7735 tft;
void initGsvCaptors();
void gps_test();
#define LED 28
#define LED_ON_VALUE LOW
extern bool gps_state;
extern bool gps_test_state;
extern uint32_t gps_get_time;
extern uint32_t gps_start_time;
extern bool first_get_location;
extern void sendFullClearOnce();