#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include <SPI.h>
#include "heltec_nrf_lorawan.h"
#include "TinyGPS.h"

TinyGPSPlus gps;
#define VGNSS_CTRL 21
void VextON(void)
{
  pinMode(PIN_VEXT_CTL,OUTPUT);
  digitalWrite(PIN_VEXT_CTL, HIGH);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(PIN_VEXT_CTL,OUTPUT);
  digitalWrite(PIN_VEXT_CTL, LOW);
}

void gps_test(void)
{
	char gps_buf[64];
    uint8_t gps_buf_index = 0;
	pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,HIGH);
	Serial1.begin(115200);    
	Serial.println("gps_test");
	delay(100);
	while(1)
	{
		if((Serial1.available()>0) &&(gps_buf_index < (sizeof(gps_buf) - 1)))
		{
            gps_buf[gps_buf_index++] = Serial1.read();
		}
		else
        {
            if(gps_buf_index > 0) {
                Serial.write(gps_buf, gps_buf_index);
            }
            gps_buf_index = 0;
            delay(1);
		}
	}
}

void setup()
{
	Serial.begin(115200);
	VextON();
	gps_test();
}

void loop()
{
    delay(1000);
}