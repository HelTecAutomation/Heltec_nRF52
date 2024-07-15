/* Heltec Automation Receive communication test example
 *
 * Function:
 * 1. Receive the same frequency band lora signal program
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 */

#include "display.h"
#include "heltec_nrf_lorawan.h"

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

uint8_t isDispayOn=0;
extern DeviceClass_t loraWanClass;

uint16_t batteryInMV()
{
  int adcin = PIN_BAT_ADC;   //battery adc pin 4
  int adcvalue = 0;
  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);
  float mv_per_lsb = 3000.0F / 4096.0F;  //12-bit ADC with 3.0V input range
  pinMode(PIN_BAT_ADC_CTL, OUTPUT);      //battery adc can be read only ctrl pin 6 set to high 
  digitalWrite(PIN_BAT_ADC_CTL, 1);      
  delay(10);
  adcvalue = analogRead(adcin);
  digitalWrite(6, 0);
  uint16_t v = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9);
  debug_printf("battry volt : %d\r\n", v);
  return v;
}


void displayStart()
{
	if(isDispayOn==0)
	{
		isDispayOn = 1;
		pinMode(PIN_TFT_VDD_CTL,OUTPUT);
		pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
		digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  //set pin 3 low to enable tft vdd
    digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);//set pin 3 low to enable tft ledA
    tft.init(135, 240);           // Init ST7789 240x135
    tft.setRotation(3);
    tft.setSPISpeed(40000000);
    tft.fillScreen(ST77XX_BLACK);
	}
}


void displayEnd()
{
	if(isDispayOn)
	{
		isDispayOn = 0;
		SPI1.end();
    digitalWrite(PIN_TFT_CS,0);
    digitalWrite(PIN_TFT_DC,0);
    digitalWrite(PIN_TFT_RST,0);
		digitalWrite(PIN_TFT_VDD_CTL,!TFT_VDD_ENABLE);
		digitalWrite(PIN_TFT_LEDA_CTL,!TFT_LEDA_ENABLE);
	}
}

void displayJoining()
{
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);//12*16
  String tmp="OTAA JOINING";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2);
  tft.println(tmp);
	delay(1000);
}

void displayJoined()
{
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);//12*16
  String tmp="OTAA JOINED";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2);
  tft.println(tmp);
	delay(1000);
}

void displayJoinFailed()
{
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  String tmp="JOIN FAILED";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2-16);
  tft.println(tmp);
  tmp="RETRY JOIN IN 30S";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2+16);
  tft.println(tmp);
	delay(2000);
}

void displaySending()
{
  batteryInMV();
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);//12*16
  String tmp="SENDING DATA";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2);
  tft.println(tmp);
}

void displayAck(int16_t rssi,int8_t snr)
{
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);//12*16
  String tmp="ACK RECEIVED";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2-16);
  tft.println(tmp);
  tmp="RSSI "+String(rssi)+", SNR "+String(snr);
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2+16);
  tft.println(tmp);
	if(loraWanClass==CLASS_A)
	{
      tmp="Into deep sleep in 2S";
      tft.setTextSize(1);//6*8
      tft.setCursor(tft.width()/2-tmp.length()/2*6, tft.height()-8);
      tft.println(tmp);
    	delay(2000);
  }
}
void displayInit()
{
	displayStart();
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);//12*16
  String tmp="LORAWAN STARTING!";
  tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2); // A char width is TextSize*6, height is TextSize*8
  tft.println(tmp);
	delay(2000);
}

