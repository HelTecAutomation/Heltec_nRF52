#include "Arduino.h"
#include "heltec_nrf_lorawan.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             -9        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

int8_t power = TX_OUTPUT_POWER;
bool interrupt_flag = false;
String powerStr,freqStr;
void interrupt_GPIO0(void)
{
    interrupt_flag = true;
}

void interrupt_handle(void)
{
	if(interrupt_flag)
	{
		interrupt_flag = false;
		if(digitalRead(PIN_BUTTON1)==0)
		{
				delay(500);
				if(digitalRead(PIN_BUTTON1)==0)
				{
	          digitalWrite(PIN_LED1, HIGH);
            power += 1;
            Radio.SetTxConfig( MODEM_LORA, power, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
            tft.fillScreen(ST7735_BLACK);
            powerStr = "power: "+String(power,DEC)+" dBm";
            freqStr = "freq: "+String(RF_FREQUENCY/1000000,DEC)+" MHz";
            tft.setCursor(0, 0);
            tft.print(powerStr);
            tft.setCursor(0, 30);
            tft.print(freqStr);
				}
		}
	}
}

void setup(void) {
  Serial.begin(115200);
  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  pinMode(PIN_LED1 ,OUTPUT);
	pinMode(PIN_TFT_VDD_CTL,OUTPUT);
	pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
	digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  
  digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);
  delay(1000);
	tft.initR(INITR_MINI160x80_PLUGIN);
	tft.setRotation(1);
	tft.setSPISpeed(40000000);
	tft.fillScreen(ST7735_BLACK);
	tft.setTextSize(1);
	tft.setTextWrap(false);
  tft.invertDisplay(true);

  txNumber=0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  attachInterrupt(PIN_BUTTON1, interrupt_GPIO0, FALLING);


    tft.fillScreen(ST7735_BLACK);
    powerStr = "power: "+String(power,DEC)+" dBm";
    freqStr = "freq: "+String(RF_FREQUENCY/1000000,DEC)+" MHz";
    tft.setCursor(0, 0);
    tft.print(powerStr);
    tft.setCursor(0, 30);
    tft.print(freqStr);
}



void loop()
{
	digitalWrite(PIN_LED1, LOW);
	if(lora_idle == true)
	{
    interrupt_handle();
    delay(1000);
		txNumber += 0.01;
		sprintf(txpacket,"Hello world number %0.2f",txNumber);  //start a package
   
		Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

		Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
    lora_idle = false;
	}
  Radio.IrqProcess( );
}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}  
