#include "TinyGPS.h"
#include "gps_test.h"
TinyGPSPlus gps;

// 星座类型
// 三选一打开宏定义
// #define GPS_L76K
// #define GPS_LC760CA
// #define GPS_LC760Z
#define GPS_UC6580

#ifdef GPS_L76K
#define BDGSV  "BDGSV"
#define GPGSV  "GPGSV"
#define GPS_BAUDRATE 9600
#elif defined GPS_LC760CA
#define BDGSV  "GBGSV"
#define GPGSV  "GPGSV"
#define GLGSV  "GLGSV"
#define GAGSV  "GAGSV"
#define GPS_BAUDRATE 115200
#elif defined GPS_LC760Z
#define BDGSV  "GBGSV"
#define GPGSV  "GPGSV"
#define GQGSV  "GQGSV"   // QZSS
#define GAGSV  "GAGSV"
#define GPS_BAUDRATE 115200
#elif defined GPS_UC6580
#define BDGSV  "GBGSV"
#define GPGSV  "GPGSV"
#define GLGSV  "GLGSV"
#define GAGSV  "GAGSV"
#define GQGSV  "GQGSV"
#define GPS_BAUDRATE 115200
#endif


enum CONSTELLATION
{
    CONST_GPS,
    CONST_BDS,
    CONST_GLONASS,
    CONST_GALILEO,
    CONST_QZSS     // GQGSV 使用
};

#define MAX_SAT_ALL 80
// 单颗卫星信息
struct SatInfo
{
    bool active;
    uint8_t constType;   // CONSTELLATION
    uint16_t prn;
    int elevation;
    int azimuth;
    int snr;
} satList[MAX_SAT_ALL];

//=====【新增】各个星座独立最大SNR =====
int maxSnr_GPS = 0;
int maxSnr_BDS = 0;
int maxSnr_GLO = 0;
int maxSnr_GAL = 0;
int maxSnr_QZS = 0;

// 一轮采集是否已经启动
bool cycleStarted = false;
// 防重复打印锁
bool printLocked = false;

// ===================== GSV捕获器定义 =====================
// GPS GPGSV
#ifdef GPGSV
TinyGPSCustom gpsMsgTotal(gps, GPGSV, 1);
TinyGPSCustom gpsMsgNum(gps, GPGSV, 2);
TinyGPSCustom gpsSatCount(gps, GPGSV, 3);
TinyGPSCustom gpsSatNo[4];
TinyGPSCustom gpsElev[4];
TinyGPSCustom gpsAzi[4];
TinyGPSCustom gpsSnr[4];
#endif

// BDS
#ifdef BDGSV
TinyGPSCustom bdsMsgTotal(gps, BDGSV, 1);
TinyGPSCustom bdsMsgNum(gps, BDGSV, 2);
TinyGPSCustom bdsSatCount(gps, BDGSV, 3);
TinyGPSCustom bdsSatNo[4];
TinyGPSCustom bdsElev[4];
TinyGPSCustom bdsAzi[4];
TinyGPSCustom bdsSnr[4];
#endif

// GLONASS GLGSV
#ifdef GLGSV
TinyGPSCustom gloMsgTotal(gps, GLGSV, 1);
TinyGPSCustom gloMsgNum(gps, GLGSV, 2);
TinyGPSCustom gloSatCount(gps, GLGSV, 3);
TinyGPSCustom gloSatNo[4];
TinyGPSCustom gloElev[4];
TinyGPSCustom gloAzi[4];
TinyGPSCustom gloSnr[4];
#endif

// QZSS GQGSV【独立一套，不和GLGSV共享】
#ifdef GQGSV
TinyGPSCustom qzsMsgTotal(gps, GQGSV, 1);
TinyGPSCustom qzsMsgNum(gps, GQGSV, 2);
TinyGPSCustom qzsSatCount(gps, GQGSV, 3);
TinyGPSCustom qzsSatNo[4];
TinyGPSCustom qzsElev[4];
TinyGPSCustom qzsAzi[4];
TinyGPSCustom qzsSnr[4];
#endif

// Galileo GAGSV
#ifdef GAGSV
TinyGPSCustom galMsgTotal(gps, GAGSV, 1);
TinyGPSCustom galMsgNum(gps, GAGSV, 2);
TinyGPSCustom galSatCount(gps, GAGSV, 3);
TinyGPSCustom galSatNo[4];
TinyGPSCustom galElev[4];
TinyGPSCustom galAzi[4];
TinyGPSCustom galSnr[4];
#endif

// 本轮各个星座接收完成标记
bool gpsCycleDone = false;
bool bdsCycleDone = false;
bool gloCycleDone = false;
bool galCycleDone = false;
bool qzsCycleDone = false;


// 函数声明
void initGsvCaptors();
void parseOneGsv(TinyGPSCustom &msgTotal, TinyGPSCustom &msgNum,
                 TinyGPSCustom satNo[], TinyGPSCustom elev[], TinyGPSCustom azi[], TinyGPSCustom snr[],
                 uint8_t constType);
void printAllSatellites();
bool checkAllConstellationComplete();
void resetCycleFlag();

// 初始化所有GSV字段绑定
void initGsvCaptors()
{
#ifdef GPS_UC6580
    sendFullClearOnce();
#endif
    for (int i = 0; i < 4; i++)
    {
#ifdef GPGSV
        gpsSatNo[i].begin(gps, GPGSV, 4 + 4 * i);
        gpsElev[i].begin(gps, GPGSV, 5 + 4 * i);
        gpsAzi[i].begin(gps, GPGSV, 6 + 4 * i);
        gpsSnr[i].begin(gps, GPGSV, 7 + 4 * i);
#endif

#ifdef BDGSV
        bdsSatNo[i].begin(gps, BDGSV, 4 + 4 * i);
        bdsElev[i].begin(gps, BDGSV, 5 + 4 * i);
        bdsAzi[i].begin(gps, BDGSV, 6 + 4 * i);
        bdsSnr[i].begin(gps, BDGSV, 7 + 4 * i);
#endif

#ifdef GLGSV
        gloSatNo[i].begin(gps, GLGSV, 4 + 4 * i);
        gloElev[i].begin(gps, GLGSV, 5 + 4 * i);
        gloAzi[i].begin(gps, GLGSV, 6 + 4 * i);
        gloSnr[i].begin(gps, GLGSV, 7 + 4 * i);
#endif

#ifdef GQGSV
        qzsSatNo[i].begin(gps, GQGSV, 4 + 4 * i);
        qzsElev[i].begin(gps, GQGSV, 5 + 4 * i);
        qzsAzi[i].begin(gps, GQGSV, 6 + 4 * i);
        qzsSnr[i].begin(gps, GQGSV, 7 + 4 * i);
#endif

#ifdef GAGSV
        galSatNo[i].begin(gps, GAGSV, 4 + 4 * i);
        galElev[i].begin(gps, GAGSV, 5 + 4 * i);
        galAzi[i].begin(gps, GAGSV, 6 + 4 * i);
        galSnr[i].begin(gps, GAGSV, 7 + 4 * i);
#endif
    }
    memset(satList, 0, sizeof(satList));
}

// 判断所有启用星座是否全部接收完毕
bool checkAllConstellationComplete()
{
    bool res = true;
#ifdef GPGSV
    res &= gpsCycleDone;
#endif
#ifdef BDGSV
    res &= bdsCycleDone;
#endif
#ifdef GLGSV
    res &= gloCycleDone;
#endif
#ifdef GQGSV
    res &= qzsCycleDone;
#endif
#ifdef GAGSV
    res &= galCycleDone;
#endif
    return res;
}

// 重置一轮接收标志
void resetCycleFlag()
{
#ifdef GPGSV
    gpsCycleDone = false;
#endif
#ifdef BDGSV
    bdsCycleDone = false;
#endif
#ifdef GLGSV
    gloCycleDone = false;
#endif
#ifdef GQGSV
    qzsCycleDone = false;
#endif
#ifdef GAGSV
    galCycleDone = false;
#endif
    cycleStarted = false;
    printLocked = false;
    //=====【新增】每轮清空各星座最大SNR =====
    //maxSnr_GPS = 0;
    //maxSnr_BDS = 0;
    //maxSnr_GLO = 0;
    //maxSnr_GAL = 0;
    //maxSnr_QZS = 0;
}

// 通用解析一段GSV报文
void parseOneGsv(TinyGPSCustom &msgTotal, TinyGPSCustom &msgNum,
                 TinyGPSCustom satNo[], TinyGPSCustom elev[], TinyGPSCustom azi[], TinyGPSCustom snr[],
                 uint8_t constType)
{
    if (!cycleStarted)
    {
        memset(satList, 0, sizeof(satList));
        cycleStarted = true;
    }

    for (int i = 0; i < 4; i++)
    {
        String prnStr = satNo[i].value();
        if (prnStr.length() == 0) continue;
        uint16_t prn = atoi(prnStr.c_str());
        if (prn == 0) continue;

        String snrStr = snr[i].value();
        int cnr = snrStr.length() ? atoi(snrStr.c_str()) : 0;
        if(cnr <= 0)
            continue;

        // 查找已有卫星或空闲槽位
        int slot = -1;
        for (int k = 0; k < MAX_SAT_ALL; k++)
        {
            if (satList[k].active && satList[k].constType == constType && satList[k].prn == prn)
            {
                slot = k; break;
            }
        }
        if (slot == -1)
        {
            for (int k = 0; k < MAX_SAT_ALL; k++)
            {
                if (!satList[k].active)
                {
                    slot = k; break;
                }
            }
        }
        if (slot < 0) return;

        satList[slot].active = true;
        satList[slot].constType = constType;
        satList[slot].prn = prn;
        satList[slot].elevation = atoi(elev[i].value());
        satList[slot].azimuth = atoi(azi[i].value());
        satList[slot].snr = cnr;
    }

    int totalMsg = atoi(msgTotal.value());
    int curMsg = atoi(msgNum.value());
    if (totalMsg == curMsg)
    {
        switch (constType)
        {
            case CONST_GPS:     gpsCycleDone = true; break;
            case CONST_BDS:     bdsCycleDone = true; break;
            case CONST_GLONASS: gloCycleDone = true; break;
            case CONST_QZSS:    qzsCycleDone = true; break;
            case CONST_GALILEO: galCycleDone = true; break;
        }

        if (checkAllConstellationComplete() && !printLocked)
        {
            printLocked = true;
            //=====【新增】遍历卫星，统计每个星座最大SNR =====
            for(int i=0; i<MAX_SAT_ALL; i++)
            {
                if(!satList[i].active) continue;
                int snrVal = satList[i].snr;
                if(snrVal>50)
                {
                    continue;
                }
                switch(satList[i].constType)
                {
                    case CONST_GPS:
                        if(snrVal > maxSnr_GPS) maxSnr_GPS = snrVal;
                        break;
                    case CONST_BDS:
                        if(snrVal > maxSnr_BDS) maxSnr_BDS = snrVal;
                        break;
                    case CONST_GLONASS:
                        if(snrVal > maxSnr_GLO) maxSnr_GLO = snrVal;
                        break;
                    case CONST_GALILEO:
                        if(snrVal > maxSnr_GAL) maxSnr_GAL = snrVal;
                        break;
                    case CONST_QZSS:
                        if(snrVal > maxSnr_QZS) maxSnr_QZS = snrVal;
                        break;
                }
            }

            printAllSatellites();
            resetCycleFlag();
        }
    }
}

void printAllSatellites()
{
    uint16_t cntGps = 0, cntBds = 0, cntGlo = 0, cntGal = 0, cntQzs = 0;
    uint16_t totalVisibleSat = 0;
    uint16_t snrOver25 = 0;

    Serial.println("======================================");
    for (int i = 0; i < MAX_SAT_ALL; i++)
    {
        if (!satList[i].active) continue;

        totalVisibleSat++;
        if (satList[i].snr > 35)
        {
            snrOver25++;
        }

        const char *conName;
        switch (satList[i].constType)
        {
            case CONST_GPS:     conName = "GPS"; cntGps++; break;
            case CONST_BDS:     conName = "BDS"; cntBds++; break;
            case CONST_GLONASS: conName = "GLO"; cntGlo++; break;
            case CONST_QZSS:    conName = "QZS"; cntQzs++; break;
            case CONST_GALILEO: conName = "GAL"; cntGal++; break;
            default: conName = "??";
        }
        Serial.print(conName);
        Serial.print(" PRN:");
        Serial.print(satList[i].prn);
        Serial.print(" SNR:");
        Serial.print(satList[i].snr);
        Serial.print(" EL:");
        Serial.print(satList[i].elevation);
        Serial.print(" AZ:");
        Serial.println(satList[i].azimuth);
    }

    Serial.print("分星座数量 | GPS="); Serial.print(cntGps);
    Serial.print(" BDS="); Serial.print(cntBds);
#ifdef GLGSV
    Serial.print(" GLO="); Serial.print(cntGlo);
#endif
#ifdef GQGSV
    Serial.print(" QZS="); Serial.print(cntQzs);
#endif
#ifdef GAGSV
    Serial.print(" GAL="); Serial.print(cntGal);
#endif
    Serial.println();

    Serial.println("--------【全局汇总（不区分星座）】--------");
    Serial.print("✅ 全部可见卫星总数：");
    Serial.println(totalVisibleSat);

    Serial.print("✅ 参与定位解算卫星：");
    Serial.println(gps.satellites.value());

    Serial.print("✅ SNR > 25dB 卫星总数：");
    Serial.println(snrOver25);

    //=====【新增串口打印各个星座最大SNR】=====
    Serial.println("--------【各星座独立最大SNR】--------");
    Serial.print("GPS Max SNR:"); Serial.println(maxSnr_GPS);
    Serial.print("BDS Max SNR:"); Serial.println(maxSnr_BDS);
#ifdef GLGSV
    Serial.print("GLO Max SNR:"); Serial.println(maxSnr_GLO);
#endif
#ifdef GAGSV
    Serial.print("GAL Max SNR:"); Serial.println(maxSnr_GAL);
#endif
#ifdef GQGSV
    Serial.print("QZS Max SNR:"); Serial.println(maxSnr_QZS);
#endif

    Serial.print("✅ GPS定位有效：");
    if(gps.location.isValid()){
        Serial.println("YES");
        Serial.print("纬度(Lat):");
        Serial.print(gps.location.lat(),6);
        Serial.print(" 经度(Lon):");
        Serial.println(gps.location.lng(),6);
        Serial.print("海拔Alt:");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");
    }else{
        Serial.println("NO");
        Serial.println("暂无有效坐标");
    }

    Serial.println("======================================\n");
}

void satellite_check()
{
    while (GNSS_UART.available() > 0)
    {
        char c=GNSS_UART.read();
        Serial.print(c);
        gps.encode(c);
    }
#ifdef GPGSV
    if (gpsMsgTotal.isUpdated())
        parseOneGsv(gpsMsgTotal, gpsMsgNum, gpsSatNo, gpsElev, gpsAzi, gpsSnr, CONST_GPS);
#endif

#ifdef BDGSV
    if (bdsMsgTotal.isUpdated())
        parseOneGsv(bdsMsgTotal, bdsMsgNum, bdsSatNo, bdsElev, bdsAzi, bdsSnr, CONST_BDS);
#endif

#ifdef GLGSV
    if (gloMsgTotal.isUpdated())
        parseOneGsv(gloMsgTotal, gloMsgNum, gloSatNo, gloElev, gloAzi, gloSnr, CONST_GLONASS);
#endif

#ifdef GQGSV
    if (qzsMsgTotal.isUpdated())
        parseOneGsv(qzsMsgTotal, qzsMsgNum, qzsSatNo, qzsElev, qzsAzi, qzsSnr, CONST_QZSS);
#endif

#ifdef GAGSV
    if (galMsgTotal.isUpdated())
        parseOneGsv(galMsgTotal, galMsgNum, galSatNo, galElev, galAzi, galSnr, CONST_GALILEO);
#endif
}


bool gps_state=false;
void gps_test(void)
{
    uint32_t starttime = millis();
    while( (millis()-starttime) < 1000 )
    {
      satellite_check();
    }
    uint32_t t = millis()-gps_start_time;
    if(gps_state==false)
    {
      gps_get_time = t/1000;
    }
    // st7735.st7735_write_str(0, 0, (String)"gps_test", Font_11x18, ST7735_WHITE);
    // String time_str = (String)gps.time.hour() + ":" + (String)gps.time.minute() + ":" + (String)gps.time.second()+ ":"+(String)gps.time.centisecond();
    // String latitude = "LAT: " + (String)gps.location.lat();
    // String longitude  = "LON: "+  (String)gps.location.lng();
    if(gps.location.lat() != 0 && gps.location.lng() != 0)
    {
      gps_state=true;
    }

    if(gps_state==true)
    {
      pinMode(LED ,OUTPUT);
      digitalWrite(LED, HIGH); 
    }
    
      if(gps_get_time<90)
      {
        st7735.setTextColor(ST77XX_GREEN);
      }
      else if(gps_get_time<180)
      {
        st7735.setTextColor(ST77XX_YELLOW);
      }
      else
      {
        st7735.setTextColor(ST77XX_RED);
      }
      st7735.fillScreen(ST77XX_BLACK);
      st7735.setTextSize(1);
      st7735.setCursor(120,66);
      st7735.println(String(gps_get_time));
      st7735.setTextSize(1);


    if( gps.location.age() < 1000 )
    {
      pinMode(LED,OUTPUT);
      digitalWrite(LED, LED_ON_VALUE);
      st7735.setTextColor(ST7735_GREEN);
      st7735.setCursor(150,0);
      st7735.println("A");
      if(first_get_location)
      {
          first_get_location=false;
          gps_get_time=(millis()-gps_start_time)/1000;
      }
    }
    else
    {
      st7735.setCursor(150,0);
      st7735.setTextColor(ST7735_WHITE);
      st7735.println("V");
      pinMode(LED,OUTPUT);
      digitalWrite(LED, !LED_ON_VALUE);
    }


    char str[30];
    int index = sprintf(str,"%02d-%02d-%02d",gps.date.year(),gps.date.day(),gps.date.month());
    str[index] = 0;
    st7735.setCursor(0,0);
    st7735.println(str);

    index = sprintf(str,"%02d:%02d:%02d",gps.time.hour(),gps.time.minute(),gps.time.second(),gps.time.centisecond());
    str[index] = 0;
    st7735.setCursor(80,0);
    st7735.println(str);


    index = sprintf(str,"lat:%d.%d",(int)gps.location.lat(),fracPart(gps.location.lat(),4));
    str[index] = 0;
    st7735.setCursor(0,18);
    st7735.println(str);

    index = sprintf(str,"lon:%d.%d",(int)gps.location.lng(),fracPart(gps.location.lng(),4));
    str[index] = 0;
    st7735.setCursor(0,30);
    st7735.println(str);

    index = sprintf(str,"alt:%d.%d",(int)gps.altitude.meters(),fracPart(gps.altitude.meters(),2));
    str[index] = 0;
    st7735.setCursor(0,42);
    st7735.println(str);

    index = sprintf(str,"hdop:%d.%d",(int)gps.hdop.hdop(),fracPart(gps.hdop.hdop(),2));
    str[index] = 0;
    st7735.setCursor(0,54);
    st7735.println(str);

    index = sprintf(str,"speed:%d.%d km/h",(int)gps.speed.kmph(),fracPart(gps.speed.kmph(),3));
    str[index] = 0;
    st7735.setCursor(0,66);
    st7735.println(str);

    st7735.setCursor(100,18);
    st7735.println("snr GP:"+String(maxSnr_GPS));

    st7735.setCursor(100,30);
    st7735.println("snr BD:"+String(maxSnr_BDS));

    st7735.setCursor(100,42);
    st7735.println("snr GL:"+String(maxSnr_GLO));

    st7735.setCursor(100,54);
    st7735.println("snr GA:"+String(maxSnr_GAL));
}
