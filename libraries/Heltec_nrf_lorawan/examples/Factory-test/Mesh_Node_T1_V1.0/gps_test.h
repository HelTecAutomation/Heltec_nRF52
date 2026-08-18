#include "Arduino.h"
#include <Adafruit_ST7735.h>
extern Adafruit_ST7735 st7735;

#define GNSS_UART Serial1

void initGsvCaptors();
void gps_test();
#define LED 16
#define LED_ON_VALUE LOW
extern bool gps_state;
extern bool gps_test_state;
extern uint32_t gps_get_time;
extern uint32_t gps_start_time;
extern bool first_get_location;
extern void sendFullClearOnce();