/*
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include <SPI.h>
#include "heltec_nrf_lorawan.h"
#include "TinyGPS.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735

Adafruit_ST7735 st7735 = Adafruit_ST7735(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

typedef enum 
{
	LORA_TEST_INIT,
	LORA_COMMUNICATION_TEST,
	DEEPSLEEP_TEST,
	GPS_TEST,
}test_status_t;

TinyGPSPlus gps;
#define VGNSS_CTRL PIN_GPS_EN
test_status_t  test_status;
bool resendflag=false;
bool deepsleepflag=false;
bool interrupt_flag = false;
/********************************* lora  *********************************************/
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             10        // dBm

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

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber = 0;
int16_t rxNumber = 0;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String packet;
String send_num;

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;
int16_t RssiDetection = 0;

uint8_t TxDone=0;
void OnTxDone( void )
{
	Serial.print("TX done......");
  TxDone++;
	state=STATE_RX;
}

void OnTxTimeout( void )
{
	Radio.Sleep( );
	Serial.print("TX Timeout......");
	state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	rxNumber++;
	Rssi=rssi;
	rxSize=size;
	memcpy(rxpacket, payload, size );
	rxpacket[size]='\0';
	Radio.Sleep( );
	Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
	Serial.println("wait to send next packet");
	receiveflag = true;
	state=STATE_TX;
}

void OnRxTimeout()
{
  Radio.Sleep();
  state=STATE_TX;
  Serial.println("RX Timeout......");
}

void OnRxError()
{
  Radio.Sleep();
  state=STATE_TX;
  Serial.println("RX Error......");
}
void lora_init(void)
{
	txNumber=0;
	Rssi=0;
	rxNumber = 0;
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.RxError = OnRxError;
	RadioEvents.RxTimeout = OnRxTimeout;

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
	state=STATE_TX;
	st7735.fillScreen(ST7735_BLACK);
	packet ="waiting lora data!";
	st7735.setCursor(0, 10);
	st7735.println(packet);
}


/********************************* lora  *********************************************/

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
			if(rxNumber < 2)
			{
				delay(500);
				if(digitalRead(PIN_BUTTON1)==0)
				{
					test_status = GPS_TEST;
				}
				else
				{
					resendflag=true;
				}
			}
			else
			{
				test_status = DEEPSLEEP_TEST;
				// deepsleepflag=true;
			}
		}
	}
}

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


void enter_deepsleep(void)
{
	Radio.Sleep();
	SPI.end();
	pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,HIGH);
	Serial2.end();
	Serial.end();
	pinMode(PIN_LED1,OUTPUT);
	digitalWrite(PIN_LED1,LOW);
	VextOFF();
	SPI1.end();
	pinMode(LORA_PA_POWER,OUTPUT);
	digitalWrite(LORA_PA_POWER,LOW);
	pinMode(RADIO_DIO_1,ANALOG);
	pinMode(RADIO_NSS,ANALOG);
	pinMode(RADIO_RESET,ANALOG);
	pinMode(RADIO_BUSY,ANALOG);

	nrf_gpio_cfg_default(VEXT_ENABLE);
    nrf_gpio_cfg_default(ST7735_CS);
    nrf_gpio_cfg_default(ST7735_RS);
    nrf_gpio_cfg_default(ST7735_SDA);
    nrf_gpio_cfg_default(ST7735_SCK);
    nrf_gpio_cfg_default(ST7735_RESET);
    nrf_gpio_cfg_default(ST7735_BL);

    nrf_gpio_cfg_default(PIN_LED1);

    // nrf_gpio_cfg_default(LORA_PA_POWER);
    pinMode(LORA_PA_POWER, OUTPUT);
    digitalWrite(LORA_PA_POWER, LOW);

    nrf_gpio_cfg_default(LORA_PA_CSD);
    nrf_gpio_cfg_default(LORA_PA_CTX);

    digitalWrite(PIN_BAT_ADC_CTL, LOW);

    nrf_gpio_cfg_default(RADIO_NSS);
    nrf_gpio_cfg_default(RADIO_DIO_1);
    nrf_gpio_cfg_default(RADIO_BUSY);
    nrf_gpio_cfg_default(RADIO_RESET);

    nrf_gpio_cfg_default(PIN_SPI_MISO);
    nrf_gpio_cfg_default(PIN_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_SPI_SCK);

    nrf_gpio_cfg_default(PIN_GPS_PPS);
    nrf_gpio_cfg_default(PIN_GPS_RESET);
    nrf_gpio_cfg_default(PIN_GPS_EN);
    nrf_gpio_cfg_default(GPS_TX_PIN);
    nrf_gpio_cfg_default(GPS_RX_PIN);

	// vTaskSuspend(checkUserkey1kHandle);
	sd_power_system_off(); // Enter System OFF mode (this function will not return)
	delay(portMAX_DELAY);
}

void lora_status_handle(void)
{
	if(resendflag)
	{
		state = STATE_TX;
		resendflag = false;
	}

	if(receiveflag && (state==LOWPOWER) )
	{
		receiveflag = false;
		packet ="Rdata:";
		int i = 0;
		while(i < rxSize)
		{
			packet += rxpacket[i];
			i++;
		}
		// packSize = "R_Size:";
		// packSize += String(rxSize,DEC);
		String packSize = "R_rssi:";
		packSize += String(Rssi,DEC);
		send_num = "send num:";
		send_num += String(txNumber,DEC);
		st7735.fillScreen(ST7735_BLACK);
		delay(100);
    st7735.setCursor(0, 0);
    st7735.println(packet);
    st7735.setCursor(0, 40);
    st7735.println(packSize);
    st7735.setCursor(0, 60);
    st7735.println(send_num);

		if((rxNumber%2)==0)
		{
			digitalWrite(PIN_LED1, HIGH);  
		}
	}
	switch(state)
	{
		case STATE_TX:
			delay(1000);
			txNumber++;
			sprintf(txpacket,"hello %d,Rssi:%d",txNumber,Rssi);
			Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
			Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
			state=LOWPOWER;
			break;
		case STATE_RX:
			Serial.println("into RX mode");
			Radio.Rx( 0 );
			state=LOWPOWER;
			break;
		case LOWPOWER:
      // TimerLowPowerHandler();
			Radio.IrqProcess( );
			break;
		default:
			break;
	}
}

void gps_test(void)
{
	uint32_t clear_num = 0;
	uint32_t last_second=0;
	pinMode(VGNSS_CTRL,OUTPUT);
	digitalWrite(VGNSS_CTRL,LOW);
	Serial2.begin(115200);    
	Serial.println("gps_test");
	st7735.fillScreen(ST7735_BLACK);
	delay(100);
  st7735.setCursor(0, 0);
  st7735.println("gps_test");
	while(1)
	{
		if(Serial2.available()>0)
		{
			if(Serial2.peek()!='\n')
			{
				char c = Serial2.read();
				gps.encode(c);
				Serial.write(c);
			}
			else
			{
				Serial.println();
				Serial2.read();
				st7735.fillScreen(ST7735_BLACK);
				st7735.setCursor(0, 0);
				st7735.println("gps_detected");
				if(gps.time.second()==0)
				{
					continue;
				}
				String time_str = (String)gps.time.hour() + ":" + (String)gps.time.minute() + ":" + (String)gps.time.second()+ ":"+(String)gps.time.centisecond();
				st7735.setCursor(0,15);
				st7735.println(time_str);
				String latitude = "LAT: " + (String)gps.location.lat();
				st7735.setCursor(0, 30);
				st7735.println(latitude);
				String longitude  = "LON: "+  (String)gps.location.lng();
				st7735.setCursor(0, 45);
				st7735.println(longitude);

				Serial.printf(" %02d:%02d:%02d.%02d",gps.time.hour(),gps.time.minute(),gps.time.second(),gps.time.centisecond());
				Serial.print("LAT: ");
				Serial.print(gps.location.lat(),6);
				Serial.print(", LON: ");
				Serial.print(gps.location.lng(),6);
				Serial.println();
				if(last_second != gps.time.second())
				{
					last_second = gps.time.second();
					delay(1000);
					while(Serial2.read()>0);
				}
				else
				{
					delay(10);
					clear_num++;
					if(clear_num%5==0)
					{
						while(Serial2.read()>0);
					}
				}
			}
		}
	}
}

void setup()
{
	Serial.begin(115200);
	InternalFS.begin();
	boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
	VextON();
	delay(100);
	pinMode(PIN_TFT_VDD_CTL,OUTPUT);
	pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
	digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  
    digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);
	st7735.initR(INITR_MINI160x80_PLUGIN);
	st7735.setRotation(1);
	st7735.setSPISpeed(40000000);
	st7735.fillScreen(ST7735_BLACK);
	st7735.setTextSize(1);//12*16
	st7735.setTextWrap(false);
  	st7735.invertDisplay(true);

	attachInterrupt(PIN_BUTTON1,interrupt_GPIO0,FALLING);
	resendflag=false;
	deepsleepflag=false;
	interrupt_flag = false;

	pinMode(PIN_LED1 ,OUTPUT);
	digitalWrite(PIN_LED1, LOW);
	test_status = LORA_TEST_INIT;

}

void loop()
{
	interrupt_handle();
	switch (test_status)
	{
		case LORA_TEST_INIT:
		{
			lora_init();
			test_status = LORA_COMMUNICATION_TEST;
			break;
		}
		case LORA_COMMUNICATION_TEST:
		{
			lora_status_handle();
			break;
		}
		case DEEPSLEEP_TEST:
		{
			enter_deepsleep();
			break;
		}
		case GPS_TEST:
		{
			gps_test();
			break;
		}
		default:
			break;
	}
}