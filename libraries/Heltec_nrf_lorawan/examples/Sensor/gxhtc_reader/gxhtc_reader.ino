/* Heltec Automation  GXHTC Sensors test example
 *
 * Function:
 * temperature and humidity sensor
 * 
 * HelTec AutoMation, Chengdu, China
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
 * 
*/
#include "Wire.h"
#include "GXHTC.h"

GXHTC gxhtc;
void setup() {
  Serial.begin(115200);  // start serial for output
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  gxhtc.begin(7,8);
}

void loop() {
  gxhtc.read_data();
  Serial.print("Temperature:");
  Serial.print(gxhtc.g_temperature);
  Serial.print("  Humidity:");
  Serial.println(gxhtc.g_humidity);
  Serial.printf("id = %X\r\n",gxhtc.read_id());

  delay(3000);
}