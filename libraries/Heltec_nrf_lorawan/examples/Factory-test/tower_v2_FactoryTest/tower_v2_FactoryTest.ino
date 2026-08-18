#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "Arduino.h"
#include <SPI.h>
#include "heltec_nrf_lorawan.h"
#include "TinyGPS.h"

typedef enum 
{
	LORA_TEST_INIT,
	LORA_COMMUNICATION_TEST,
	DEEPSLEEP_TEST,
	GPS_TEST,
}test_status_t;

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
	packet ="waiting lora data!";
	Serial.println(packet);
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
				resendflag=true;
			}
			else
			{
				test_status = DEEPSLEEP_TEST;
			}
		}
	}
}

void enter_deepsleep(void)
{
	Serial.println("entering deepsleep mode...");
	delay(100);
	Radio.Sleep();
	SPI.end();

	Serial1.end();
	Serial.end();

	pinMode(RADIO_DIO_1,ANALOG);
	pinMode(RADIO_NSS,ANALOG);
	pinMode(RADIO_RESET,ANALOG);
	pinMode(RADIO_BUSY,ANALOG);

    pinMode(PIN_BAT_ADC_CTL, OUTPUT);
    digitalWrite(PIN_BAT_ADC_CTL, LOW);

    nrf_gpio_cfg_default(PIN_SPI_MISO);
    nrf_gpio_cfg_default(PIN_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_SPI_SCK);

    nrf_gpio_cfg_default(PIN_WIRE_SDA);
    nrf_gpio_cfg_default(PIN_WIRE_SCL);

    // nrf_gpio_cfg_default(SX126X_CS);
    nrf_gpio_cfg_default(SX126X_DIO1);
    nrf_gpio_cfg_default(SX126X_BUSY);
    nrf_gpio_cfg_default(SX126X_RESET);

    pinMode(PIN_GPS_EN, OUTPUT);
    digitalWrite(PIN_GPS_EN, HIGH);

    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, HIGH);

    pinMode(LORA_KCT8103L_EN, OUTPUT);
    digitalWrite(LORA_KCT8103L_EN, LOW);

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
		String packSize = "R_rssi:";
		packSize += String(Rssi,DEC);
		send_num = "send num:";
		send_num += String(txNumber,DEC);

		if((rxNumber%2)==0)
		{
			digitalWrite(PIN_LED1, LOW);  
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


void setup()
{
	Serial.begin(115200);
	InternalFS.begin();
	boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
	delay(100);

	attachInterrupt(PIN_BUTTON1,interrupt_GPIO0,FALLING);
	resendflag=false;
	deepsleepflag=false;
	interrupt_flag = false;

	pinMode(PIN_LED1 ,OUTPUT);
	digitalWrite(PIN_LED1, HIGH);
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
		default:
			break;
	}
}