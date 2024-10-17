#include <NRF_LoRaWan_APP.h>
#include <Arduino.h>
#include <SPI.h>


#if(LoraWan_RGB==1)
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

#if defined( REGION_EU868 )
#include "loramac/region/RegionEU868.h"
#elif defined( REGION_EU433 )
#include "loramac/region/RegionEU433.h"
#elif defined( REGION_KR920 )
#include "loramac/region/RegionKR920.h"
#elif defined( REGION_AS923) || defined( REGION_AS923_AS1) || defined( REGION_AS923_AS2)
#include "loramac/region/RegionAS923.h"
#endif


#if(LoraWan_RGB==1)
void turnOnRGB(uint32_t color,uint32_t time)
{
	uint8_t red,green,blue;
	red=(uint8_t)(color>>16);
	green=(uint8_t)(color>>8);
	blue=(uint8_t)color;
	pinMode(PIN_VEXT_CTL,OUTPUT);
	digitalWrite(PIN_VEXT_CTL,VEXT_ENABLE); //SET POWER
	delay(5);
	pixels.begin(); // INITIALIZE RGB strip object (REQUIRED)
	pixels.clear(); // Set all pixel colors to 'off'
	for(int i=0;i<NEOPIXEL_NUM;i++)
	{
		pixels.setPixelColor(i, pixels.Color(red, green, blue));
		pixels.show();   // Send the updated pixel colors to the hardware.
	}
	if(time>0)
	{
		delay(time);
	}
}

void turnOffRGB(void)
{
	turnOnRGB(0,0);
	digitalWrite(PIN_VEXT_CTL,!VEXT_ENABLE);
}
#else
void turnOnRGB(uint32_t color,uint32_t time)
{
	return;
}

void turnOffRGB(void)
{
	return;
}
#endif

int8_t defaultDrForNoAdr = 3;


uint8_t power_for_noAdr=17;
uint16_t rx1_delay=1000;
uint16_t rx2_delay=2000;
uint8_t rx2_dr=0;
uint32_t rx2_freq=505300000;
uint8_t rx2dr_type=0;
uint8_t rx2freq_type=0;
uint8_t region_index=0;



const char * region_str[15]={"CN470","IN865","EU868","US915","AU915","KR920","AS923_1","AS923_2","NULL","NULL","NULL","NULL","NULL","NULL","NULL"};
const uint8_t region_array[15]={LORAMAC_REGION_CN470,LORAMAC_REGION_IN865,LORAMAC_REGION_EU868,LORAMAC_REGION_US915,LORAMAC_REGION_AU915,LORAMAC_REGION_KR920,LORAMAC_REGION_AS923_AS1,LORAMAC_REGION_AS923_AS2,0,0,0,0,0,0};
const uint8_t def_rx2dr[15] = {CN470_RX_WND_2_DR,IN865_RX_WND_2_DR,EU868_RX_WND_2_DR,US915_RX_WND_2_DR,AU915_RX_WND_2_DR,KR920_RX_WND_2_DR,AS923_RX_WND_2_DR,AS923_RX_WND_2_DR,0,0,0,0,0,0,0};
const uint32_t def_rx2freq[15] = {CN470_RX_WND_2_FREQ,IN865_RX_WND_2_FREQ,EU868_RX_WND_2_FREQ,US915_RX_WND_2_FREQ,AU915_RX_WND_2_FREQ,KR920_RX_WND_2_FREQ,AS923_RX_WND_2_FREQ,AS923_RX_WND_2_FREQ,0,0,0,0,0,0,0}; 

uint8_t debugLevel=LoRaWAN_DEBUG_LEVEL;

/*AT mode, auto into low power mode*/
bool autoLPM = true;

/*loraWan current Dr when adr disabled*/
int8_t currentDrForNoAdr;

/*!
 * User application data size
 */
uint8_t appDataSize = 0;

/*!
 * User application data
 */
uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];


/*!
 * Defines the application data transmission duty cycle
 */
uint32_t txDutyCycleTime ;

/*!
 * Timer to handle the application data transmission duty cycle
 */
TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool nextTx = true;


enum eDeviceState_LoraWan deviceState=DEVICE_STATE_INIT;


/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
bool SendFrame( void )
{
	lwan_dev_params_update();
	
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;
	LORAWANLOG;
	if( LoRaMacQueryTxPossible( appDataSize, &txInfo ) != LORAMAC_STATUS_OK )
	{
		// Send empty frame in order to flush MAC commands
		debug_printf("payload length error ...\r\n");
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = currentDrForNoAdr;
		//return false;
	}
	else
	{
		if( isTxConfirmed == false )
		{
			debug_printf("unconfirmed uplink sending ...\r\n");
			mcpsReq.Type = MCPS_UNCONFIRMED;
			mcpsReq.Req.Unconfirmed.fPort = appPort;
			mcpsReq.Req.Unconfirmed.fBuffer = appData;
			mcpsReq.Req.Unconfirmed.fBufferSize = appDataSize;
			mcpsReq.Req.Unconfirmed.Datarate = currentDrForNoAdr;
		}
		else
		{
			debug_printf("confirmed uplink sending ...\r\n");
			mcpsReq.Type = MCPS_CONFIRMED;
			mcpsReq.Req.Confirmed.fPort = appPort;
			mcpsReq.Req.Confirmed.fBuffer = appData;
			mcpsReq.Req.Confirmed.fBufferSize = appDataSize;
			mcpsReq.Req.Confirmed.NbTrials = confirmedNbTrials;
			mcpsReq.Req.Confirmed.Datarate = currentDrForNoAdr;
		}
	}

	if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
	{
		return false;
	}
	return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
	MibRequestConfirm_t mibReq;
	LoRaMacStatus_t status;

	TimerStop( &TxNextPacketTimer );

	mibReq.Type = MIB_NETWORK_JOINED;
	status = LoRaMacMibGetRequestConfirm( &mibReq );

	if( status == LORAMAC_STATUS_OK )
	{
		if( mibReq.Param.IsNetworkJoined == true )
		{
			deviceState = DEVICE_STATE_SEND;
			nextTx = true;
		}
		else
		{
			// Network not joined yet. Try to join again
			MlmeReq_t mlmeReq;
			mlmeReq.Type = MLME_JOIN;
			mlmeReq.Req.Join.DevEui = devEui;
			mlmeReq.Req.Join.AppEui = appEui;
			mlmeReq.Req.Join.AppKey = appKey;
			mlmeReq.Req.Join.NbTrials = 1;

			if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
			{
				deviceState = DEVICE_STATE_SLEEP;
			}
			else
			{
				deviceState = DEVICE_STATE_CYCLE;
			}
		}
	}
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
	if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
	{
		switch( mcpsConfirm->McpsRequest )
		{
			case MCPS_UNCONFIRMED:
			{
				// Check Datarate
				// Check TxPower
				break;
			}
			case MCPS_CONFIRMED:
			{
				// Check Datarate
				// Check TxPower
				// Check AckReceived
				// Check NbTrials
				break;
			}
			case MCPS_PROPRIETARY:
			{
				break;
			}
			default:
				break;
		}
	}
	nextTx = true;
}





void __attribute__((weak)) downLinkAckHandle(int16_t rssi,int8_t snr)
{
	//debug_printf("ack received\r\n");
}

void __attribute__((weak)) downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
	debug_printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
	debug_printf("+REV DATA:");
	for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
	{
		debug_printf("%02X",mcpsIndication->Buffer[i]);
	}
	debug_printf("\r\n");
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
	if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
	{
		return;
	}


#if (LoraWan_RGB==1)
		turnOnRGB(COLOR_RECEIVED, 200);
		turnOffRGB();
#endif

	LORAWANLOG;
	debug_printf( "received ");
	switch( mcpsIndication->McpsIndication )
	{
		case MCPS_UNCONFIRMED:
		{
			debug_printf( "unconfirmed ");
			break;
		}
		case MCPS_CONFIRMED:
		{
			debug_printf( "confirmed ");
			OnTxNextPacketTimerEvent( );
			break;
		}
		case MCPS_PROPRIETARY:
		{
			debug_printf( "proprietary ");
			break;
		}
		case MCPS_MULTICAST:
		{
			debug_printf( "multicast ");
			break;
		}
		default:
			break;
	}
	debug_printf( "downlink: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,(int)mcpsIndication->RxDoneDatarate);

	if(mcpsIndication->AckReceived)
	{
		downLinkAckHandle(mcpsIndication->Rssi, mcpsIndication->Snr);
	}

	if( mcpsIndication->RxData == true )
	{
		downLinkDataHandle(mcpsIndication);
	}

	// Check Multicast
	// Check Port
	// Check Datarate
	// Check FramePending
	if( mcpsIndication->FramePending == true )
	{
		// The server signals that it has pending data to be sent.
		// We schedule an uplink as soon as possible to flush the server.
		OnTxNextPacketTimerEvent( );
	}
	// Check Buffer
	// Check BufferSize
	// Check Rssi
	// Check Snr
	// Check RxSlot
}


void __attribute__((weak)) dev_time_updated()
{
	debug_printf("device time updated\r\n");
}

void __attribute__((weak)) otaaJoindHandle(bool joined)
{
	//debug_printf("ack received\r\n");
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
	switch( mlmeConfirm->MlmeRequest )
	{
		case MLME_JOIN:
		{
			if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
			{

#if (LoraWan_RGB==1)
				turnOnRGB(COLOR_JOINED,500);
				turnOffRGB();
#endif

				LORAWANLOG;
				debug_printf("joined\r\n");
				otaaJoindHandle(true);
				deviceState = DEVICE_STATE_SEND;

			}
			else
			{
				uint32_t rejoin_delay = 30000;
				debug_printf("join failed, join again at 30s later\r\n");
				otaaJoindHandle(false);

				delay(5);
				TimerSetValue( &TxNextPacketTimer, rejoin_delay );
				TimerStart( &TxNextPacketTimer );
			}
			break;
		}
		case MLME_LINK_CHECK:
		{
			if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
			{
				// Check DemodMargin
				// Check NbGateways
			}
			break;
		}
		case MLME_DEVICE_TIME:
		{
			if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
			{
				dev_time_updated();
			}
			break;
		}
		default:
			break;
	}
	nextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
	switch( mlmeIndication->MlmeIndication )
	{
		case MLME_SCHEDULE_UPLINK:
		{// The MAC signals that we shall provide an uplink as soon as possible
			OnTxNextPacketTimerEvent( );
			break;
		}
		default:
			break;
	}
}


void lwan_dev_params_update( void )
{



	MibRequestConfirm_t mibReq;

	mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
	mibReq.Param.ChannelsMask = userChannelsMask;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_CHANNELS_MASK;
	mibReq.Param.ChannelsMask = userChannelsMask;
	LoRaMacMibSetRequestConfirm(&mibReq);
}

void print_Hex(uint8_t *para,uint8_t size)
{
	for(int i=0;i<size;i++)
	{
		debug_printf("%02X",para[i]);
	}
}

void printDevParam(void)
{

	debug_printf("+REGION=%s\r\n",region_str[region_index]);
	debug_printf("+OTAA=%d\r\n",overTheAirActivation);
	debug_printf("+Class=%X\r\n",loraWanClass+10);
	debug_printf("+ADR=%d\r\n",loraWanAdr);
	debug_printf("+DR=%d\r\n",defaultDrForNoAdr);
	debug_printf("+Power=%d\r\n",power_for_noAdr);
	debug_printf("+IsTxConfirmed=%d\r\n",isTxConfirmed);
	debug_printf("+AppPort=%d\r\n",appPort);
	debug_printf("+DutyCycle=%u\r\n",appTxDutyCycle);
	debug_printf("+Retry=%d\r\n",confirmedNbTrials);
//	String ch=channeltoString();
//	debug_printf("+ChMask=%s\r\n",ch.c_str());
	//printf("+ChMask=%04X%04X%04X%04X%04X%04X\r\n",userChannelsMask[5],userChannelsMask[4],userChannelsMask[3],userChannelsMask[2],userChannelsMask[1],	userChannelsMask[0]);
	debug_printf("+DevEui=");print_Hex(devEui,8); debug_printf("(For OTAA Mode)\r\n");
	debug_printf("+AppEui=");print_Hex(appEui,8); debug_printf("(For OTAA Mode)\r\n");
	debug_printf("+AppKey=");print_Hex(appKey,16);	debug_printf("(For OTAA Mode)\r\n");
	debug_printf("+NwkSKey=");print_Hex(nwkSKey,16);	debug_printf("(For ABP Mode)\r\n");
	debug_printf("+AppSKey=");print_Hex(appSKey,16);	debug_printf("(For ABP Mode)\r\n");
	debug_printf("+DevAddr=%08X(For ABP Mode)\r\n",devAddr);
	debug_printf("+RX1DELAY=%d\r\n",rx1_delay/1000);
	debug_printf("+RX2DELAY=%d\r\n",rx2_delay/1000);
	debug_printf("+RX2DR_TYPE=%d\r\n",rx2dr_type);
	debug_printf("+RX2FREQ_TYPE=%d\r\n",rx2freq_type);
	debug_printf("+RX2DR=%d\r\n",rx2_dr);
	debug_printf("+RX2FREQ=%d\r\n",rx2_freq);
}


LoRaMacPrimitives_t LoRaMacPrimitive;
LoRaMacCallback_t LoRaMacCallback;

void LoRaWanClass::generateDeveuiByChipID()
{
	uint32_t uniqueId[2];
#if defined(ESP_PLATFORM)
	uint64_t id = getID();
	uniqueId[0]=(uint32_t)(id>>32);
	uniqueId[1]=(uint32_t)id;
#endif
	for(int i=0;i<8;i++)
	{
		if(i<4)
			devEui[i] = (uniqueId[1]>>(8*(3-i)))&0xFF;
		else
			devEui[i] = (uniqueId[0]>>(8*(7-i)))&0xFF;
	}
}


void LoRaWanClass::init(DeviceClass_t lorawanClass,LoRaMacRegion_t region)
{

	
	debug_printf("\r\n");
	
	LORAWANLOG;
	debug_printf("LoRaWAN ");
	switch(region)
	{
		case LORAMAC_REGION_AS923_AS1:
			debug_printf("AS923(AS1:922.0-923.4MHz)");
			break;
		case LORAMAC_REGION_AS923_AS2:
			debug_printf("AS923(AS2:923.2-924.6MHz)");
			break;
		case LORAMAC_REGION_AU915:
			debug_printf("AU915");
			break;
		case LORAMAC_REGION_CN470:
			debug_printf("CN470");
			break;
		case LORAMAC_REGION_CN779:
			debug_printf("CN779");
			break;
		case LORAMAC_REGION_EU433:
			debug_printf("EU433");
			break;
		case LORAMAC_REGION_EU868:
			debug_printf("EU868");
			break;
		case LORAMAC_REGION_KR920:
			debug_printf("KR920");
			break;
		case LORAMAC_REGION_IN865:
			debug_printf("IN865");
			break;
		case LORAMAC_REGION_US915:
			debug_printf("US915");
			break;
		case LORAMAC_REGION_US915_HYBRID:
			debug_printf("US915_HYBRID ");
			break;
		default:
			break;
	}
	debug_printf(" Class %X start!\r\n\r\n",loraWanClass+10);
	printDevParam();

	if(region == LORAMAC_REGION_AS923_AS1 || region == LORAMAC_REGION_AS923_AS2)
		region = LORAMAC_REGION_AS923;
	MibRequestConfirm_t mibReq;

	LoRaMacPrimitive.MacMcpsConfirm = McpsConfirm;
	LoRaMacPrimitive.MacMcpsIndication = McpsIndication;
	LoRaMacPrimitive.MacMlmeConfirm = MlmeConfirm;
	LoRaMacPrimitive.MacMlmeIndication = MlmeIndication;
	LoRaMacCallback.GetBatteryLevel = BoardGetBatteryLevel;
	LoRaMacCallback.GetTemperatureLevel = NULL;
	LoRaMacInitialization( &LoRaMacPrimitive, &LoRaMacCallback,region);
	TimerStop( &TxNextPacketTimer );
	TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
	
    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = loraWanAdr;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
    LoRaMacMibSetRequestConfirm( &mibReq );

    lwan_dev_params_update();

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm( &mibReq );

    if(loraWanClass != mibReq.Param.Class)
    {
    mibReq.Param.Class = loraWanClass;
    LoRaMacMibSetRequestConfirm( &mibReq );
    }

    deviceState = DEVICE_STATE_JOIN;


	if(loraWanRegion==LORAMAC_REGION_EU868)
	{
	LoRaMacChannelAdd( 3, ( ChannelParams_t )EU868_LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )EU868_LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )EU868_LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )EU868_LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )EU868_LC8 );
	}
	else if(loraWanRegion==LORAMAC_REGION_EU433)
	{
	LoRaMacChannelAdd( 3, ( ChannelParams_t )EU433_LC4 ); 
	LoRaMacChannelAdd( 4, ( ChannelParams_t )EU433_LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )EU433_LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )EU433_LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )EU433_LC8 );
	}
	else if(loraWanRegion==LORAMAC_REGION_KR920)
	{
	LoRaMacChannelAdd( 3, ( ChannelParams_t )KR920_LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )KR920_LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )KR920_LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )KR920_LC7 );
	}
	else if(loraWanRegion==LORAMAC_REGION_AS923_AS1)
	{
	LoRaMacChannelAdd( 2, ( ChannelParams_t )AS923_1_LC3 );
	LoRaMacChannelAdd( 3, ( ChannelParams_t )AS923_1_LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )AS923_1_LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )AS923_1_LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )AS923_1_LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )AS923_1_LC8 );
	}
	else if(loraWanRegion==LORAMAC_REGION_AS923_AS2)
	{
	LoRaMacChannelAdd( 2, ( ChannelParams_t )AS923_2_LC3 );
	LoRaMacChannelAdd( 3, ( ChannelParams_t )AS923_2_LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )AS923_2_LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )AS923_2_LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )AS923_2_LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )AS923_2_LC8 );
	}

}


void LoRaWanClass::join()
{
	if( overTheAirActivation )
	{
		LORAWANLOG;
		debug_printf("joining...\r\n");
		MlmeReq_t mlmeReq;
		
		mlmeReq.Type = MLME_JOIN;

		mlmeReq.Req.Join.DevEui = devEui;
		mlmeReq.Req.Join.AppEui = appEui;
		mlmeReq.Req.Join.AppKey = appKey;
		mlmeReq.Req.Join.NbTrials = 1;

		if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
		{
			deviceState = DEVICE_STATE_SLEEP;
		}
		else
		{
			deviceState = DEVICE_STATE_CYCLE;
		}
	}
	else
	{
		MibRequestConfirm_t mibReq;

		mibReq.Type = MIB_NET_ID;
		mibReq.Param.NetID = LORAWAN_NETWORK_ID;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_DEV_ADDR;
		mibReq.Param.DevAddr = devAddr;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_NWK_SKEY;
		mibReq.Param.NwkSKey = nwkSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_APP_SKEY;
		mibReq.Param.AppSKey = appSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_NETWORK_JOINED;
		mibReq.Param.IsNetworkJoined = true;
		LoRaMacMibSetRequestConfirm( &mibReq );
		
		deviceState = DEVICE_STATE_SEND;
	}
}

void LoRaWanClass::send()
{
	if( nextTx == true )
	{
		MibRequestConfirm_t mibReq;

		mibReq.Type = MIB_ADR;
		mibReq.Param.AdrEnable = loraWanAdr;
		LoRaMacMibSetRequestConfirm( &mibReq );
		
		mibReq.Type = MIB_DEVICE_CLASS;
		LoRaMacMibGetRequestConfirm( &mibReq );


		if(loraWanClass != mibReq.Param.Class)
		{
			mibReq.Param.Class = loraWanClass;
			LoRaMacMibSetRequestConfirm( &mibReq );
		}
		nextTx = SendFrame( );
	}
}

void LoRaWanClass::cycle(uint32_t dutyCycle)
{
	TimerSetValue( &TxNextPacketTimer, dutyCycle );
	TimerStart( &TxNextPacketTimer );
}

void LoRaWanClass::sleep(DeviceClass_t classMode)
{
	TimerLowPowerHandler();
	Radio.IrqProcess();
}

void LoRaWanClass::setDefaultDR(int8_t dataRate)
{
	defaultDrForNoAdr = dataRate;
}

uint8_t getHardwareVersion()
{
  uint8_t hard_ver;
  
  flash_nrf5x_read(&hard_ver,HARD_VERSION_ADDR,1);
  if(hard_ver==0XFF)
  {
    hard_ver=0;
  }
  return hard_ver;
}

void setHardwareVersion(uint8_t hard_ver)
{
  uint8_t data_temp[17];
  data_temp[0]=hard_ver;
  flash_nrf5x_read(&data_temp[1],HT_LICENSE_ADDR,16);
  flash_nrf5x_erase(HT_LICENSE_ADDR_BASE);
  flash_nrf5x_write(HARD_VERSION_ADDR,(uint8_t *)data_temp,17);
  flash_nrf5x_flush();
}
LoRaWanClass LoRaWAN;

