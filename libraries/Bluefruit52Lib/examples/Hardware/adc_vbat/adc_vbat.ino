#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

#if defined ARDUINO_NRF52840_CIRCUITPLAY
#define  PIN_VBAT          A8   // this is just a mock read, we'll use the light sensor, so we can run the test
#endif

uint32_t vbat_pin = 4;             // A7 for feather nRF52832, A6 for nRF52840

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

//#ifdef NRF52840_XXAA
#define VBAT_DIVIDER      (0.204F)          // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (4.90F)          // Compensation factor for the VBAT divider
// #else
// #define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
// #define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
// #endif

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


float readVBAT(void) {
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

uint8_t mvToPercent(float mvolts) {
  if (mvolts < 3300) return 0;             // Voltage less than 3.3V, battery level 0%
  
  if (mvolts < 3600) {                     // Between 3.3V and 3.6V, the battery level ranges from 0% to 20%
    mvolts -= 3300;
    return mvolts / 15;                    // Each 30mV corresponds to 1% of electricity consumption
  }

  if (mvolts < 3700) {                     // Between 3.6V and 3.7V, the battery level ranges from 20% to 40%
    mvolts -= 3600;
    return 20 + (mvolts * 0.2);            // Each 5mV corresponds to 1% of electricity consumption
  }

  if (mvolts < 3800) {                     // Between 3.7V and 3.8V, the battery level ranges from 40% to 60%
    mvolts -= 3700;
    return 40 + (mvolts * 0.2);            // Each 5mV corresponds to 1% of electricity consumption
  }

  if (mvolts < 3900) {                     // Between 3.8V and 3.9V, the battery level ranges from 60% to 80%
    mvolts -= 3800;
    return 60 + (mvolts * 0.2);            // Each 5mV corresponds to 1% of electricity consumption
  }

  if (mvolts < 4200) {                     // Between 3.9V and 4.2V, the power level ranges from 80% to 100%
    mvolts -= 3900;
    return 80 + (mvolts * 0.0666);         // Every 3mV corresponds to 1% of electricity consumption
  }

  return 100;                              // Voltage greater than or equal to 4.2V, with 100% battery capacity
}

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  pinMode(6,1); //Enable ADC
  digitalWrite(6,1) ;
  // Get a single ADC sample and throw it away
  readVBAT();
}

void loop() {
  // Get a raw ADC reading
  float vbat_mv = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_mv);

  // Display the results

  Serial.print("LIPO = ");
  Serial.print(vbat_mv);
  Serial.print(" mV (");
  Serial.print(vbat_per);
  Serial.println("%)");

  delay(1000);
}

