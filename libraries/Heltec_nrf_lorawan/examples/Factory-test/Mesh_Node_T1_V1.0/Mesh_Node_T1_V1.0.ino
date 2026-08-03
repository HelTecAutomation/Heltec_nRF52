/*
 * HelTec Automation(TM) WIFI_LoRa_32 factory test code, witch includ
 * follow functions:
 * 
 * - Basic OLED function test;
 * 
 * - Basic serial port test(in baud rate 115200);
 * 
 * - LED blink test;
 * 
 * - WIFI connect and scan test;
 * 
 * - LoRa Ping-Pong test (DIO0 -- GPIO26 interrup check the new incoming messages);
 * 
 * - Timer test and some other Arduino basic functions.
 *
 * by Aaron.Lee from HelTec AutoMation, ChengDu, China
 * �ɶ��������Զ����Ƽ����޹�˾
 * https://heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/

#include <Adafruit_GFX.h>
#include <Adafruit_LittleFS.h>
#include <Adafruit_ST7735.h>
#include <ICM42670P.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>
#include "Arduino.h"
#include "heltec_nrf_lorawan.h"
#include "gps_test.h"
 
typedef struct {
  float acc_g[3];
  float gyro_dps[3];
  float mag_ut[3];
} SensorSample;

typedef struct {
  SensorSample average;
  SensorSample minimum;
  SensorSample maximum;
  SensorSample range;
} SensorStats;
enum CompassReadError {
  COMPASS_READ_OK,
  COMPASS_READ_ACCEL,
  COMPASS_READ_MAG,
  COMPASS_READ_VECTOR,
};
#define SENSOR_SAMPLE_COUNT 20
#define ICM42670_I2C_ADDR_LOW 0x68
#define ICM42670_I2C_ADDR_HIGH 0x69
#define MMC5983MA_I2C_ADDR 0x30
#define ICM_ACCEL_ODR_HZ 100
#define ICM_GYRO_ODR_HZ 100
#define ICM_ACCEL_FSR_G 2
#define ICM_GYRO_FSR_DPS 250
#define ICM_ACCEL_COUNTS_PER_G 16384.0f
#define ICM_GYRO_COUNTS_PER_DPS 131.072f
uint8_t activeIcmAddress = ICM42670_I2C_ADDR_LOW;
ICM42670 icmLow(Wire, false);
ICM42670 icmHigh(Wire, true);
ICM42670 *activeIcm = &icmLow;
SFE_MMC5983MA mag;

/********************************* lora  *********************************************/
#define RF_FREQUENCY_1                                868100000 // Hz
#define RF_FREQUENCY_2                                870900000 // Hz

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

#define USERKEY 42
#define KEY 14
#define SERIAL_BAUDRATE 115200
#define BUTTON_TIMEOUT_MS 15000UL
#define BUZZER_FREQUENCY_HZ 2000
#define BUZZER_BEEP_COUNT 5
#define BUZZER_BEEP_ON_MS 250UL
#define BUZZER_BEEP_OFF_MS 150UL
#define LED_BUZZER_CONFIRM_TIMEOUT_MS 15000UL
#define BLE_Rssi_MIN  -70
#define BLE_Rssi_MAX  -30
#define LORA_RSSI_MIN -50
#define LORA_RSSI_MAX -10
#define LORA_RSSI_DIFF_MAX 15
#define BATTER_MIN 3700
#define BATTER_MAX 4200

#define SENSOR_SAMPLE_COUNT 20
#define ICM42670_I2C_ADDR_LOW 0x68
#define ICM42670_I2C_ADDR_HIGH 0x69
#define MMC5983MA_I2C_ADDR 0x30
#define ICM_ACCEL_ODR_HZ 100
#define ICM_GYRO_ODR_HZ 100
#define ICM_ACCEL_FSR_G 2
#define ICM_GYRO_FSR_DPS 250
#define ICM_ACCEL_COUNTS_PER_G 16384.0f
#define ICM_GYRO_COUNTS_PER_DPS 131.072f

#define ICM42607_REG_ACCEL_DATA_X1 0x0B
#define ICM42607_REG_GYRO_DATA_X1 0x11
#define ICM42607_REG_PWR_MGMT0 0x1F
#define ICM42607_REG_GYRO_CONFIG0 0x20
#define ICM42607_REG_ACCEL_CONFIG0 0x21
#define ICM42607_REG_GYRO_CONFIG1 0x23
#define ICM42607_REG_ACCEL_CONFIG1 0x24
#define ICM42607_REG_WHO_AM_I 0x75
#define ICM42607_WHO_AM_I_VALUE 0x60
#define ICM42607_PWR_MGMT0_GYRO_ACCEL_LN 0x0F
#define ICM42607_ACCEL_CONFIG0_FS_2G_ODR_100HZ 0x69
#define ICM42607_GYRO_CONFIG0_FS_250DPS_ODR_100HZ 0x69
#define ICM42607_FILTER_34HZ 0x05

#define MMC5983_ZERO_OFFSET 131072.0f
#define MMC5983_COUNTS_PER_GAUSS 16384.0f
#define GAUSS_TO_MICROTESLA 100.0f

#define ACC_MAG_MIN_G 0.9f
#define ACC_MAG_MAX_G 1.1f
#define GYRO_STILL_MAX_DPS 15.0f
#define SENSOR_FIXED_ACC_RANGE_G 0.0005f
#define SENSOR_FIXED_GYRO_RANGE_DPS 0.02f
#define ACC_SAT_AXIS_G 1.1f
#define GYRO_SAT_AXIS_DPS 245.0f

#define HARD_VERSION 2

#define LED 16
#define LED_ON_VALUE LOW
extern void ble_slave_start();
extern void ble_center_start();

Adafruit_ST7735 st7735 =
    Adafruit_ST7735(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

uint8_t txpacket[BUFFER_SIZE];
uint8_t rxpacket[BUFFER_SIZE];


static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );


int size_error=0;
bool gps_test_start=false;
uint8_t error_count = 0;

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

States_t state;
int16_t maxTxRssi=-255;
int16_t maxRxRssi=-255;

String packSize = "--";
String packet;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
uint64_t chipId = 0;
char chipIdText[17] = {0};
uint8_t rx_cnt=0;
int test_mode=0;
bool loratimeout=false;
bool intosleep=false;
uint32_t rtc_cnt_1s,rtc_cnt_2s;
uint16_t volt;
uint8_t Buzzer_state = false;
bool useIcm42607Fallback = false;

static void makeChipIdText() {
  chipId = (((uint64_t)NRF_FICR->DEVICEID[1]) << 32) | NRF_FICR->DEVICEID[0];
  snprintf(chipIdText, sizeof(chipIdText), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1],
           (unsigned long)NRF_FICR->DEVICEID[0]);
}

static void initDisplay() {
  pinMode(PIN_TFT_VDD_CTL, OUTPUT);
  pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
  digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);
  digitalWrite(PIN_TFT_LEDA_CTL, TFT_LEDA_ENABLE);
  delay(100);

  st7735.initR(INITR_MINI160x80_PLUGIN);
  st7735.setRotation(3);
  st7735.setSPISpeed(40000000);
  st7735.setTextWrap(false);
  st7735.setTextSize(1);
  st7735.invertDisplay(true);
  st7735.fillScreen(ST7735_BLACK);
  // displayReady = true;
}

static bool ledBuzzerTest() {     //��������
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUZZER_VOLTAGE_MULTIPLIER_1, OUTPUT);
  pinMode(PIN_BUZZER_VOLTAGE_MULTIPLIER_2, OUTPUT);
  digitalWrite(PIN_BUZZER_VOLTAGE_MULTIPLIER_1, HIGH);
  digitalWrite(PIN_BUZZER_VOLTAGE_MULTIPLIER_2, HIGH);

  for (uint8_t i = 0; i < BUZZER_BEEP_COUNT; i++) {
    digitalWrite(PIN_LED1, LOW);
    tone(PIN_BUZZER, BUZZER_FREQUENCY_HZ);
    delay(BUZZER_BEEP_ON_MS);
    noTone(PIN_BUZZER);
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_BUZZER, LOW);
    delay(BUZZER_BEEP_OFF_MS);
  }
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
  return false;
}
#define MAG_STATIC_SAMPLE_COUNT 64
#define MAG_AXIS_FULL_SCALE_FAIL_UT 760.0f
#define MAG_STATIC_B_FAIL_LOW_UT 15.0f
#define MAG_STATIC_B_FAIL_HIGH_UT 90.0f
#define COMPASS_ENTER_HOLD_MS 3000UL
#define COMPASS_UPDATE_INTERVAL_MS 50UL
#define COMPASS_UNAVAILABLE_MS 1500UL
#define COMPASS_HEADING_OFFSET_DEG 180.0f
#define COMPASS_CAL_MIN_AXIS_RADIUS_UT 20.0f

struct CompassCalibration {
  bool valid;
  bool calibrated;
  float hardIronUt[3];
  float softIronScale[3];
};

struct CompassLiveCalibration {
  bool seeded;
  float highestUt[3];
  float lowestUt[3];
};

static CompassCalibration compassCalibration = {};
static CompassLiveCalibration compassLiveCalibration = {};
static bool compassHeadingValid = false;
static float compassHeading = 0.0f;



static CompassReadError lastCompassReadError = COMPASS_READ_OK;


static bool sensor9AxisTest() {
  compassCalibration.valid = false;
  compassCalibration.calibrated = false;
  pinMode(PIN_SENSOR_EN, OUTPUT);
  digitalWrite(PIN_SENSOR_EN, PIN_SENSOR_EN_ACTIVE);
  delay(PERIPHERAL_WARMUP_MS);
  Wire.begin();
  delay(50);

  auto vectorMagnitude = [](float x, float y, float z) -> float {
    return sqrtf((x * x) + (y * y) + (z * z));
  };

  auto maxAxisAbs = [](const float *v) -> float {
    return max(max(fabsf(v[0]), fabsf(v[1])), fabsf(v[2]));
  };

  auto probeAddress = [](uint8_t address) -> bool {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
  };

  auto readRegister = [](uint8_t address, uint8_t reg, uint8_t *buffer,
                         size_t length) -> bool {
    if (buffer == nullptr || length == 0) {
      return false;
    }
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    if (Wire.requestFrom((int)address, (int)length) != length) {
      return false;
    }
    for (size_t i = 0; i < length; i++) {
      if (!Wire.available()) {
        return false;
      }
      buffer[i] = (uint8_t)Wire.read();
    }
    return true;
  };

  auto writeRegister = [](uint8_t address, uint8_t reg,
                          uint8_t value) -> bool {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  };

  if (probeAddress(ICM42670_I2C_ADDR_LOW)) {
    activeIcmAddress = ICM42670_I2C_ADDR_LOW;
  } else if (probeAddress(ICM42670_I2C_ADDR_HIGH)) {
    activeIcmAddress = ICM42670_I2C_ADDR_HIGH;
  } else {
    debug_printf("9AXIS: IMU I2C miss\r\n");
    return false;
  }
  if (!probeAddress(MMC5983MA_I2C_ADDR)) {
    debug_printf("9AXIS: MAG I2C miss\r\n");
    return false;
  }

  activeIcm = (activeIcmAddress == ICM42670_I2C_ADDR_HIGH) ? &icmHigh : &icmLow;
  useIcm42607Fallback = false;
  if (activeIcm->begin() == 0) {
    if (activeIcm->startAccel(ICM_ACCEL_ODR_HZ, ICM_ACCEL_FSR_G) != 0 ||
        activeIcm->startGyro(ICM_GYRO_ODR_HZ, ICM_GYRO_FSR_DPS) != 0) {
      debug_printf("9AXIS: ICM42670 start fail\r\n");
      return false;
    }
    delay(100);
  } else {
    uint8_t who = 0;
    if (!readRegister(activeIcmAddress, ICM42607_REG_WHO_AM_I, &who, 1) ||
        who != ICM42607_WHO_AM_I_VALUE) {
      debug_printf("9AXIS: ICM who fail 0x%02X\r\n", who);
      return false;
    }
    if (!writeRegister(activeIcmAddress, ICM42607_REG_ACCEL_CONFIG0,
                       ICM42607_ACCEL_CONFIG0_FS_2G_ODR_100HZ) ||
        !writeRegister(activeIcmAddress, ICM42607_REG_GYRO_CONFIG0,
                       ICM42607_GYRO_CONFIG0_FS_250DPS_ODR_100HZ) ||
        !writeRegister(activeIcmAddress, ICM42607_REG_ACCEL_CONFIG1,
                       ICM42607_FILTER_34HZ) ||
        !writeRegister(activeIcmAddress, ICM42607_REG_GYRO_CONFIG1,
                       ICM42607_FILTER_34HZ) ||
        !writeRegister(activeIcmAddress, ICM42607_REG_PWR_MGMT0,
                       ICM42607_PWR_MGMT0_GYRO_ACCEL_LN)) {
      debug_printf("9AXIS: ICM42607 fallback init fail\r\n");
      return false;
    }
    delay(50);
    useIcm42607Fallback = true;
  }

  if (!mag.begin(Wire)) {
    debug_printf("9AXIS: MMC5983 begin fail\r\n");
    return false;
  }
  mag.softReset();
  delay(10);
  mag.setFilterBandwidth(100);
  mag.enableAutomaticSetReset();
  mag.performSetOperation();
  delay(10);

  auto readImuSample = [&](SensorSample *sample) -> bool {
    if (sample == nullptr) {
      return false;
    }
    if (useIcm42607Fallback) {
      uint8_t accRaw[6];
      uint8_t gyroRaw[6];
      if (!readRegister(activeIcmAddress, ICM42607_REG_ACCEL_DATA_X1, accRaw,
                        sizeof(accRaw)) ||
          !readRegister(activeIcmAddress, ICM42607_REG_GYRO_DATA_X1, gyroRaw,
                        sizeof(gyroRaw))) {
        return false;
      }
      for (uint8_t i = 0; i < 3; i++) {
        int16_t acc =
            (int16_t)(((uint16_t)accRaw[i * 2] << 8) | accRaw[i * 2 + 1]);
        int16_t gyro =
            (int16_t)(((uint16_t)gyroRaw[i * 2] << 8) | gyroRaw[i * 2 + 1]);
        sample->acc_g[i] = acc / ICM_ACCEL_COUNTS_PER_G;
        sample->gyro_dps[i] = gyro / ICM_GYRO_COUNTS_PER_DPS;
      }
    } else {
      inv_imu_sensor_event_t event;
      if (activeIcm->getDataFromRegisters(event) != 0 ||
          !activeIcm->isAccelDataValid(&event) ||
          !activeIcm->isGyroDataValid(&event)) {
        return false;
      }
      for (uint8_t i = 0; i < 3; i++) {
        sample->acc_g[i] = event.accel[i] / ICM_ACCEL_COUNTS_PER_G;
        sample->gyro_dps[i] = event.gyro[i] / ICM_GYRO_COUNTS_PER_DPS;
      }
    }
    return true;
  };

  auto rawMagToUt = [](uint32_t x, uint32_t y, uint32_t z,
                       float *mag_ut) -> void {
    mag_ut[0] =
        (((float)x - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
        GAUSS_TO_MICROTESLA;
    mag_ut[1] =
        (((float)y - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
        GAUSS_TO_MICROTESLA;
    mag_ut[2] =
        (((float)z - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
        GAUSS_TO_MICROTESLA;
  };

  auto readRawMagUt = [&](float *mag_ut) -> bool {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    if (mag_ut == nullptr || !mag.getMeasurementXYZ(&x, &y, &z)) {
      return false;
    }
    rawMagToUt(x, y, z, mag_ut);
    return true;
  };

  SensorStats stillStats;
  SensorSample imuSample;
  memset(&stillStats, 0, sizeof(stillStats));
  delay(500);

  for (uint8_t i = 0; i < SENSOR_SAMPLE_COUNT; i++) {
    if (!readImuSample(&imuSample)) {
      debug_printf("9AXIS: IMU sample fail\r\n");
      return false;
    }
    if (i == 0) {
      stillStats.minimum = imuSample;
      stillStats.maximum = imuSample;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      stillStats.average.acc_g[axis] += imuSample.acc_g[axis];
      stillStats.average.gyro_dps[axis] += imuSample.gyro_dps[axis];
      stillStats.minimum.acc_g[axis] =
          min(stillStats.minimum.acc_g[axis], imuSample.acc_g[axis]);
      stillStats.minimum.gyro_dps[axis] =
          min(stillStats.minimum.gyro_dps[axis], imuSample.gyro_dps[axis]);
      stillStats.maximum.acc_g[axis] =
          max(stillStats.maximum.acc_g[axis], imuSample.acc_g[axis]);
      stillStats.maximum.gyro_dps[axis] =
          max(stillStats.maximum.gyro_dps[axis], imuSample.gyro_dps[axis]);
    }
    delay(25);
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    stillStats.average.acc_g[axis] /= SENSOR_SAMPLE_COUNT;
    stillStats.average.gyro_dps[axis] /= SENSOR_SAMPLE_COUNT;
    stillStats.range.acc_g[axis] =
        stillStats.maximum.acc_g[axis] - stillStats.minimum.acc_g[axis];
    stillStats.range.gyro_dps[axis] =
        stillStats.maximum.gyro_dps[axis] - stillStats.minimum.gyro_dps[axis];
  }

  float accMag = vectorMagnitude(stillStats.average.acc_g[0],
                                 stillStats.average.acc_g[1],
                                 stillStats.average.acc_g[2]);
  float gyroMax = maxAxisAbs(stillStats.average.gyro_dps);
  bool imuFixed = fabsf(stillStats.range.acc_g[0]) < SENSOR_FIXED_ACC_RANGE_G &&
                  fabsf(stillStats.range.acc_g[1]) < SENSOR_FIXED_ACC_RANGE_G &&
                  fabsf(stillStats.range.acc_g[2]) < SENSOR_FIXED_ACC_RANGE_G &&
                  fabsf(stillStats.range.gyro_dps[0]) < SENSOR_FIXED_GYRO_RANGE_DPS &&
                  fabsf(stillStats.range.gyro_dps[1]) < SENSOR_FIXED_GYRO_RANGE_DPS &&
                  fabsf(stillStats.range.gyro_dps[2]) < SENSOR_FIXED_GYRO_RANGE_DPS;
  if (imuFixed || maxAxisAbs(stillStats.average.acc_g) >= ACC_SAT_AXIS_G ||
      gyroMax >= GYRO_SAT_AXIS_DPS || accMag < ACC_MAG_MIN_G ||
      accMag > ACC_MAG_MAX_G || gyroMax > GYRO_STILL_MAX_DPS) {
    debug_printf("9AXIS: IMU range fail ACC=%.3f GYRO=%.1f\r\n", accMag,
                 gyroMax);
    return false;
  }

  float magStaticSum[3] = {0.0f, 0.0f, 0.0f};
  float magSample[3] = {0.0f, 0.0f, 0.0f};
  for (uint16_t i = 0; i < MAG_STATIC_SAMPLE_COUNT; i++) {
    if (!readRawMagUt(magSample)) {
      debug_printf("9AXIS: MAG static sample fail\r\n");
      return false;
    }
    if (maxAxisAbs(magSample) >= MAG_AXIS_FULL_SCALE_FAIL_UT) {
      debug_printf("9AXIS: MAG static full-scale fail\r\n");
      return false;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      magStaticSum[axis] += magSample[axis];
    }
    delay(10);
  }

  float magStaticAvg[3] = {
      magStaticSum[0] / MAG_STATIC_SAMPLE_COUNT,
      magStaticSum[1] / MAG_STATIC_SAMPLE_COUNT,
      magStaticSum[2] / MAG_STATIC_SAMPLE_COUNT,
  };
  float staticAxisMax = maxAxisAbs(magStaticAvg);
  float staticB =
      vectorMagnitude(magStaticAvg[0], magStaticAvg[1], magStaticAvg[2]);
  debug_printf("MAG static B=%.2f axis=%.2f\r\n", staticB, staticAxisMax);

  if (staticAxisMax >= MAG_AXIS_FULL_SCALE_FAIL_UT ||
      staticB < MAG_STATIC_B_FAIL_LOW_UT ||
      staticB > MAG_STATIC_B_FAIL_HIGH_UT) {
    debug_printf("9AXIS: MAG static fail B=%.1f\r\n", staticB);
    return false;
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    compassCalibration.hardIronUt[axis] = 0.0f;
    compassCalibration.softIronScale[axis] = 1.0f;
  }
  compassCalibration.valid = true;
  compassCalibration.calibrated = false;

  return true;
}

static bool readCompassRegister(uint8_t address, uint8_t reg, uint8_t *buffer,
                                size_t length) {
  if (buffer == nullptr || length == 0) {
    return false;
  }
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)address, (int)length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool readCompassAcceleration(float *accel) {
  if (accel == nullptr || activeIcm == nullptr) {
    return false;
  }

  float raw[3] = {0.0f, 0.0f, 0.0f};
  if (useIcm42607Fallback) {
    uint8_t accRaw[6];
    if (!readCompassRegister(activeIcmAddress, ICM42607_REG_ACCEL_DATA_X1,
                             accRaw, sizeof(accRaw))) {
      return false;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      int16_t value = (int16_t)(((uint16_t)accRaw[axis * 2] << 8) |
                                accRaw[axis * 2 + 1]);
      raw[axis] = value / ICM_ACCEL_COUNTS_PER_G;
    }
  } else {
    inv_imu_sensor_event_t event;
    if (activeIcm->getDataFromRegisters(event) != 0 ||
        !activeIcm->isAccelDataValid(&event)) {
      return false;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      raw[axis] = event.accel[axis] / ICM_ACCEL_COUNTS_PER_G;
    }
  }

  // Match the Mesh Node T1 axis convention used by the firmware compass path.
  accel[0] = raw[0];
  accel[1] = -raw[1];
  accel[2] = -raw[2];
  return true;
}

static void resetCompassLiveCalibration() {
  compassLiveCalibration.seeded = false;
  compassCalibration.calibrated = false;
  for (uint8_t axis = 0; axis < 3; axis++) {
    compassCalibration.hardIronUt[axis] = 0.0f;
    compassCalibration.softIronScale[axis] = 1.0f;
    compassLiveCalibration.highestUt[axis] = 0.0f;
    compassLiveCalibration.lowestUt[axis] = 0.0f;
  }
}

static bool readCompassRawMagnetic(float *rawUt) {
  if (rawUt == nullptr || !compassCalibration.valid) {
    return false;
  }

  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  if (!mag.getMeasurementXYZ(&rawX, &rawY, &rawZ)) {
    return false;
  }

  rawUt[0] = (((float)rawX - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
             GAUSS_TO_MICROTESLA;
  rawUt[1] = (((float)rawY - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
             GAUSS_TO_MICROTESLA;
  rawUt[2] = (((float)rawZ - MMC5983_ZERO_OFFSET) / MMC5983_COUNTS_PER_GAUSS) *
             GAUSS_TO_MICROTESLA;
  return true;
}

static void updateCompassLiveCalibration(const float *rawUt) {
  if (rawUt == nullptr) {
    return;
  }

  if (!compassLiveCalibration.seeded) {
    compassLiveCalibration.seeded = true;
    for (uint8_t axis = 0; axis < 3; axis++) {
      compassLiveCalibration.highestUt[axis] = rawUt[axis];
      compassLiveCalibration.lowestUt[axis] = rawUt[axis];
      compassCalibration.hardIronUt[axis] = 0.0f;
      compassCalibration.softIronScale[axis] = 1.0f;
    }
    compassCalibration.calibrated = false;
    return;
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    compassLiveCalibration.highestUt[axis] =
        max(compassLiveCalibration.highestUt[axis], rawUt[axis]);
    compassLiveCalibration.lowestUt[axis] =
        min(compassLiveCalibration.lowestUt[axis], rawUt[axis]);
  }

  float radius[3];
  bool coverageOk = true;
  for (uint8_t axis = 0; axis < 3; axis++) {
    radius[axis] =
        (compassLiveCalibration.highestUt[axis] -
         compassLiveCalibration.lowestUt[axis]) *
        0.5f;
    if (radius[axis] < COMPASS_CAL_MIN_AXIS_RADIUS_UT) {
      coverageOk = false;
    }
  }

  if (!coverageOk) {
    compassCalibration.calibrated = false;
    return;
  }

  float avgRadius = (radius[0] + radius[1] + radius[2]) / 3.0f;
  if (!(avgRadius > 0.0001f)) {
    compassCalibration.calibrated = false;
    return;
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    compassCalibration.hardIronUt[axis] =
        (compassLiveCalibration.highestUt[axis] +
         compassLiveCalibration.lowestUt[axis]) *
        0.5f;
    compassCalibration.softIronScale[axis] =
        (radius[axis] > 0.0001f) ? (avgRadius / radius[axis]) : 1.0f;
  }
  compassCalibration.calibrated = true;
}

static bool readCompassMagnetic(float *magUt) {
  if (magUt == nullptr || !compassCalibration.valid) {
    return false;
  }

  float rawUt[3];
  if (!readCompassRawMagnetic(rawUt)) {
    return false;
  }

  updateCompassLiveCalibration(rawUt);
  for (uint8_t axis = 0; axis < 3; axis++) {
    float corrected = rawUt[axis] - compassCalibration.hardIronUt[axis];
    magUt[axis] = corrected * compassCalibration.softIronScale[axis];
  }
  return true;
}

static void compassCross(const float *a, const float *b, float *result) {
  result[0] = (a[1] * b[2]) - (a[2] * b[1]);
  result[1] = (a[2] * b[0]) - (a[0] * b[2]);
  result[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static bool compassNormalise(float *v) {
  float magnitude = sqrtf((v[0] * v[0]) + (v[1] * v[1]) +
                          (v[2] * v[2]));
  if (!(magnitude > 0.000001f)) {
    return false;
  }
  v[0] /= magnitude;
  v[1] /= magnitude;
  v[2] /= magnitude;
  return true;
}

static bool computeCompassHeading(const float *accel, const float *magUt,
                                  float *heading) {
  if (accel == nullptr || magUt == nullptr || heading == nullptr) {
    return false;
  }

  float up[3] = {-accel[0], -accel[1], -accel[2]};
  float west[3];
  float north[3];
  if (!compassNormalise(up)) {
    return false;
  }
  compassCross(up, magUt, west);
  if (!compassNormalise(west)) {
    return false;
  }
  compassCross(west, up, north);
  if (!compassNormalise(north)) {
    return false;
  }

  float value = atan2f(west[0], north[0]) * RAD_TO_DEG;
  value += COMPASS_HEADING_OFFSET_DEG;
  while (value >= 360.0f) {
    value -= 360.0f;
  }
  while (value < 0.0f) {
    value += 360.0f;
  }
  value = 360.0f - value;
  if (value >= 360.0f) {
    value -= 360.0f;
  }
  *heading = value;
  return true;
}

static float smoothCompassHeading(float heading) {
  while (heading >= 360.0f) {
    heading -= 360.0f;
  }
  while (heading < 0.0f) {
    heading += 360.0f;
  }

  if (!compassHeadingValid) {
    compassHeading = heading;
    compassHeadingValid = true;
    return compassHeading;
  }

  float delta = heading - compassHeading;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }

  float absDelta = fabsf(delta);
  if (absDelta >= 1.0f) {
    float alpha = 0.35f;
    if (absDelta > 25.0f) {
      alpha = 0.85f;
    } else if (absDelta > 10.0f) {
      alpha = 0.65f;
    }
    float step = delta * alpha;
    step = max(-12.0f, min(12.0f, step));
    compassHeading += step;
    if (compassHeading >= 360.0f) {
      compassHeading -= 360.0f;
    } else if (compassHeading < 0.0f) {
      compassHeading += 360.0f;
    }
  }
  return compassHeading;
}

static const char *compassDirection(float heading) {
  static const char *directions[] = {"N", "NE", "E", "SE",
                                     "S", "SW", "W", "NW"};
  uint8_t index = (uint8_t)((heading + 22.5f) / 45.0f) & 0x07;
  return directions[index];
}

static void drawCompassFrame() {
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextSize(1);
  st7735.setTextWrap(false);
  st7735.drawFastVLine(80, 0, 80, 0x7BEF);
  st7735.setTextColor(ST7735_CYAN);
  st7735.setCursor(88, 3);
  st7735.print("COMPASS");
  st7735.setTextColor(ST7735_WHITE);
  st7735.setCursor(86, 68);
  st7735.print("KEY2 BACK");
}

static void drawCompassDial(float heading) {
  const int16_t centerX = 39;
  const int16_t centerY = 40;
  const int16_t radius = 36;
  st7735.fillRect(0, 0, 80, 80, ST7735_BLACK);
  st7735.drawCircle(centerX, centerY, radius, ST7735_WHITE);

  for (uint8_t tick = 0; tick < 24; tick++) {
    float angle = tick * 15.0f * DEG_TO_RAD;
    int16_t tickLength = ((tick & 1U) == 0U) ? 4 : 2;
    int16_t xOuter = centerX + (int16_t)((radius - 1) * sinf(angle));
    int16_t yOuter = centerY - (int16_t)((radius - 1) * cosf(angle));
    int16_t xInner = centerX + (int16_t)((radius - tickLength) * sinf(angle));
    int16_t yInner = centerY - (int16_t)((radius - tickLength) * cosf(angle));
    st7735.drawLine(xInner, yInner, xOuter, yOuter, ST7735_WHITE);
  }

  float angle = -heading * DEG_TO_RAD;
  float sinHeading = sinf(angle);
  float cosHeading = cosf(angle);
  auto transformPoint = [&](float localX, float localY, int16_t *x,
                            int16_t *y) -> void {
    *x = centerX + (int16_t)((localX * cosHeading) -
                             (localY * sinHeading));
    *y = centerY + (int16_t)((localX * sinHeading) +
                             (localY * cosHeading));
  };

  int16_t northTipX, northTipY, northLeftX, northLeftY, northRightX,
      northRightY;
  int16_t southTipX, southTipY, southLeftX, southLeftY, southRightX,
      southRightY;
  transformPoint(0.0f, -25.0f, &northTipX, &northTipY);
  transformPoint(-4.0f, -2.0f, &northLeftX, &northLeftY);
  transformPoint(4.0f, -2.0f, &northRightX, &northRightY);
  transformPoint(0.0f, 25.0f, &southTipX, &southTipY);
  transformPoint(-4.0f, 2.0f, &southLeftX, &southLeftY);
  transformPoint(4.0f, 2.0f, &southRightX, &southRightY);
  st7735.fillTriangle(northTipX, northTipY, northLeftX, northLeftY,
                      northRightX, northRightY, ST7735_RED);
  st7735.fillTriangle(southTipX, southTipY, southLeftX, southLeftY,
                      southRightX, southRightY, ST7735_BLUE);
  st7735.fillCircle(centerX, centerY, 2, ST7735_WHITE);
}

static void drawCompassValue(float heading) {
  char headingText[16];
  snprintf(headingText, sizeof(headingText), "%.1f deg", heading);
  st7735.fillRect(82, 18, 78, 44, ST7735_BLACK);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setCursor(86, 20);
  st7735.print(headingText);
  st7735.setTextColor(ST7735_YELLOW);
  st7735.setCursor(86, 37);
  st7735.print(compassDirection(heading));
  st7735.setTextColor(compassCalibration.calibrated ? ST7735_GREEN
                                                     : ST7735_YELLOW);
  st7735.setCursor(86, 53);
  st7735.print(compassCalibration.calibrated ? "CAL OK" : "CAL MOVE");
}

static const char *compassReadErrorText(uint8_t error) {
  switch (error) {
  case COMPASS_READ_ACCEL:
    return "ACC ERROR";
  case COMPASS_READ_MAG:
    return "MAG ERROR";
  case COMPASS_READ_VECTOR:
    return "VECTOR ERROR";
  default:
    return "SENSOR ERROR";
  }
}

static void drawCompassReadError(uint8_t error) {
  st7735.fillRect(82, 18, 78, 44, ST7735_BLACK);
  st7735.setTextColor(ST7735_RED);
  st7735.setCursor(86, 27);
  st7735.print(compassReadErrorText(error));
  st7735.setTextColor(ST7735_YELLOW);
  st7735.setCursor(86, 44);
  st7735.print("RETRYING");
}

static void reportCompassReadError(uint8_t error) {
  if (error == lastCompassReadError) {
    return;
  }
  lastCompassReadError = (CompassReadError)error;
  if (error != COMPASS_READ_OK) {
    debug_printf("COMPASS: %s\r\n", compassReadErrorText(error));
  }
}

static void showCompassUnavailable() {
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextSize(1);
  st7735.setTextColor(ST7735_RED);
  st7735.setCursor(18, 24);
  st7735.print("COMPASS UNAVAILABLE");
  st7735.setCursor(43, 43);
  st7735.print("9AXIS FAIL");
  delay(COMPASS_UNAVAILABLE_MS);
}

static void runCompassMode() {
  if (!compassCalibration.valid) {
    showCompassUnavailable();
    return;
  }

  while (digitalRead(KEY) == LOW) {
    delay(5);
  }
  compassHeadingValid = false;
  lastCompassReadError = COMPASS_READ_OK;
  resetCompassLiveCalibration();
  drawCompassFrame();
  uint32_t lastUpdate = 0;

  while (true) {
    if (digitalRead(KEY) == LOW) {
      delay(20);
      if (digitalRead(KEY) == LOW) {
        while (digitalRead(KEY) == LOW) {
          delay(5);
        }
        return;
      }
    }

    uint32_t now = millis();
    if ((uint32_t)(now - lastUpdate) >= COMPASS_UPDATE_INTERVAL_MS) {
      lastUpdate = now;
      float accel[3];
      float magUt[3];
      float heading = 0.0f;
      CompassReadError error = COMPASS_READ_OK;
      if (!readCompassAcceleration(accel)) {
        error = COMPASS_READ_ACCEL;
      } else if (!readCompassMagnetic(magUt)) {
        error = COMPASS_READ_MAG;
      } else if (!computeCompassHeading(accel, magUt, &heading)) {
        error = COMPASS_READ_VECTOR;
      }

      if (error == COMPASS_READ_OK) {
        reportCompassReadError(COMPASS_READ_OK);
        heading = smoothCompassHeading(heading);
        drawCompassDial(heading);
        drawCompassValue(heading);
      } else {
        reportCompassReadError(error);
        drawCompassReadError(error);
      }
    }
    delay(5);
  }
}

void OnTxDone( void )
{
	debug_printf("TX done......\r\n");
	state=STATE_RX;
}

void OnTxTimeout( void )
{
  Radio.Sleep( );
  debug_printf("TX Timeout......\r\n");
	state=STATE_TX;
}


extern int8_t blerssi;
static bool isLoraRssiPass()
{
  int txRssi = abs(maxTxRssi);
  int rxRssi = abs(maxRxRssi);
  bool txInRange = maxTxRssi >= LORA_RSSI_MIN && maxTxRssi <= LORA_RSSI_MAX;
  bool rxInRange = maxRxRssi >= LORA_RSSI_MIN && maxRxRssi <= LORA_RSSI_MAX;

  return txInRange && rxInRange && abs(txRssi - rxRssi) <= LORA_RSSI_DIFF_MAX;
}

static void drawFinalStatusPage(int value, bool sensorResult,
                                uint32_t elapsedSeconds) {
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextSize(1);
  st7735.setTextWrap(false);

  st7735.setTextColor(ST7735_GREEN);
  packet = "t:" + String(elapsedSeconds);
  st7735.setCursor(0, 0);
  st7735.println(packet);
  packet = "bat:" + String(volt);
  st7735.setCursor(90, 0);
  st7735.println(packet);

  st7735.setTextColor(sensorResult ? ST7735_GREEN : ST7735_RED);
  packet = sensorResult ? "Sensor9Axis: Pass" : "Sensor9Axis: Fail";
  st7735.setCursor(0, 14);
  st7735.println(packet);

  if (value) {
    st7735.setTextColor(isLoraRssiPass() ? ST7735_GREEN : ST7735_RED);
    packet = "Lora Rssi:" + String(maxTxRssi) + " " + String(maxRxRssi);
    st7735.setCursor(0, 28);
    st7735.println(packet);

    bool blePass = blerssi <= BLE_Rssi_MAX && blerssi >= BLE_Rssi_MIN;
    st7735.setTextColor(blePass ? ST7735_GREEN : ST7735_RED);
    packet = "BLE Rssi:" + String(blerssi);
    st7735.setCursor(0, 42);
    st7735.println(packet);
  } else {
    st7735.setTextColor(ST7735_RED);
    st7735.setCursor(0, 28);
    st7735.println("LoRa Error");
  }

  st7735.setTextColor(error_count == 0 ? ST7735_GREEN : ST7735_RED);
  packet = error_count == 0 ? "Result:Pass K2:3s COMPASS"
                            : "Result:Fail K2:3s COMPASS";
  st7735.setCursor(0, 64);
  st7735.println(packet);
}

void showStatus(int value)
{
  if(intosleep)
    return;
  st7735.fillScreen(ST7735_BLACK);
  st7735.setTextColor(ST7735_GREEN);
  st7735.setTextSize(1);
   delay(100);
  debug_printf("ShowStatus:\r\n");

  uint32_t t=millis()/1000;
  st7735.setTextColor(ST7735_GREEN);
  packet ="t:"+String(t);
  st7735.setCursor(0,0);
  st7735.println(packet);

  packet ="bat:"+String(volt);
  st7735.setCursor(90,0);
  st7735.println(packet);
  debug_printf("t: %d          bat: %d \r\n",t,volt);
  bool Sensor = sensor9AxisTest();
  error_count = 0;
  if(Sensor == false)
  {
    error_count = 1;
  }
  if(value)
  {
    if(isLoraRssiPass())
    {
      debug_printf("Lora Rssi: %d %d OK\r\n",maxTxRssi,maxRxRssi);
    }
    else
    {
      debug_printf("Lora Rssi: %d %d XX\r\n",maxTxRssi,maxRxRssi);
      error_count = 2;
    }
    debug_printf("BLE Rssi: %d\r\n",blerssi);
    if(blerssi > BLE_Rssi_MAX || blerssi < BLE_Rssi_MIN)
    {
      error_count = 3;
    }
  }
  else
  {
    debug_printf("LoRa Error\r\n");
    error_count = 4;
  }
  delay(200);
  pinMode(KEY, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  if(error_count == 0)
  {
    debug_printf("Test result:OK\r\n");
    debug_printf("Test OK!\r\n");
    digitalWrite(LED, LOW);
  }
  else
  {
    debug_printf("Test result:Failed\r\n");
    debug_printf("Test Error!\r\n");
    digitalWrite(LED, HIGH);
  }
  drawFinalStatusPage(value, Sensor, t);
  bool keyDown = false;
  uint32_t keyDownAt = 0;
  bool failLedHigh = true;
  uint32_t lastLedToggle = millis();

  while (1) {
    uint32_t now = millis();
    if (error_count != 0 && (uint32_t)(now - lastLedToggle) >= 500UL) {
      lastLedToggle = now;
      failLedHigh = !failLedHigh;
      digitalWrite(LED, failLedHigh ? HIGH : LOW);
    }

    bool pressed = digitalRead(KEY) == LOW;
    if (pressed && !keyDown) {
      keyDown = true;
      keyDownAt = now;
    }
    if (!pressed && keyDown) {
      uint32_t heldMs = (uint32_t)(now - keyDownAt);
      if (heldMs >= COMPASS_ENTER_HOLD_MS) {
        if (Sensor && compassCalibration.valid) {
          debug_printf("Compass mode start\r\n");
          runCompassMode();
          debug_printf("Compass mode stop\r\n");
        } else {
          showCompassUnavailable();
        }
        drawFinalStatusPage(value, Sensor, t);
        lastLedToggle = millis();
      } else if (Buzzer_state == false) {
        ledBuzzerTest();
        Buzzer_state = true;
        if (error_count != 0) {
          failLedHigh = true;
          digitalWrite(LED, HIGH);
          lastLedToggle = millis();
        }
      }
      keyDown = false;
    }
    delay(10);
  }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  Radio.Sleep( );
  debug_printf("RX done......");
  debug_printf("Rx size : %d , rssi : %d, snr : %d\r\n",size,rssi,snr);

  if(test_mode==0)
  {
    state=STATE_TX;
    if(size==10)
    {
      uint64_t rxchipid=*((uint64_t *)payload);
      if(rxchipid==chipId)
      {
        int16_t txrssi=*(int16_t *)(payload+8);
        debug_printf("rx own mes,rssi :%d\r\n",txrssi);
        if(maxTxRssi<txrssi)
          maxTxRssi=txrssi;
        if(maxRxRssi<rssi)
          maxRxRssi=rssi;
        rx_cnt++;
        if(rx_cnt>=3)
        {
          showStatus(1);
          state=LOWPOWER;
        }
      }
    }
  }
  else
  {
    if(size==8)
    {
      state=STATE_TX;
      memcpy(txpacket,payload,8);
      memcpy(txpacket+8,(uint8_t *)&rssi,2);
    }
    else
    {
      Radio.Rx(0);
    }
  }
}

void OnRxTimeout()
{
  Radio.Sleep();
  state=STATE_TX;
  debug_printf("RX Timeout......\r\n");
}

void OnRxError()
{
  Radio.Sleep();
  state=STATE_TX;
  debug_printf("RX Error......\r\n");
}

void lora_init(void)
{
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
  Radio.Init( &RadioEvents );
  srand1( Radio.Random( ) );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
	state=STATE_TX;
}



TaskHandle_t checkUserkey1kHandle = NULL;
void intodeepsleep()
{
  debug_printf("into deep sleep\r\n");
  int adcin = 5;
  int adcvalue = 0;
  float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  pinMode(11, OUTPUT);
  digitalWrite(11, 1);
  delay(10);
  for(int i=0;i<50;i++)
  {
    adcvalue += analogRead(adcin);
  }
  digitalWrite(11, 0);
  volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  st7735.fillScreen(ST7735_BLACK);
  if(volt<BATTER_MIN|volt>BATTER_MAX)
  {
    st7735.setTextColor(ST7735_RED);
    turnOnRGB(0x000020,0); //RGB_RED
  }
  else
  {
    st7735.setTextColor(ST7735_GREEN);
  }
  st7735.setTextSize(1);
  packet ="battery:"+String(volt);
  st7735.setCursor(0,30);
  st7735.println(packet);
  delay(1000);
  if(volt<BATTER_MIN|volt>BATTER_MAX)
  {
    
  }
  else
  {
    Radio.Sleep();
    SPI.end();
    SPI1.end();
    Wire.end();
    Serial1.end();

    pinMode(PIN_GPS_EN, OUTPUT);
    digitalWrite(PIN_GPS_EN, !GPS_EN_ACTIVE);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);
    pinMode(PIN_SENSOR_EN, OUTPUT);
    digitalWrite(PIN_SENSOR_EN, !PIN_SENSOR_EN_ACTIVE);
    pinMode(PIN_BAT_ADC_CTL, OUTPUT);
    digitalWrite(PIN_BAT_ADC_CTL, LOW);
    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, HIGH);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    nrf_gpio_cfg_default(PIN_SPI_MISO);
    nrf_gpio_cfg_default(PIN_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_SPI_SCK);
    nrf_gpio_cfg_default(PIN_SPI1_MISO);
    nrf_gpio_cfg_default(PIN_SPI1_MOSI);
    nrf_gpio_cfg_default(PIN_SPI1_SCK);
    nrf_gpio_cfg_default(PIN_WIRE_SDA);
    nrf_gpio_cfg_default(PIN_WIRE_SCL);
    nrf_gpio_cfg_default(PIN_GPS_PPS);
    nrf_gpio_cfg_default(PIN_GPS_RESET);
    nrf_gpio_cfg_default(PIN_GPS_EN);
    nrf_gpio_cfg_default(GPS_TX_PIN);
    nrf_gpio_cfg_default(GPS_RX_PIN);
    nrf_gpio_cfg_default(ST7735_CS);
    nrf_gpio_cfg_default(ST7735_RS);
    nrf_gpio_cfg_default(ST7735_SDA);
    nrf_gpio_cfg_default(ST7735_SCK);
    nrf_gpio_cfg_default(ST7735_RESET);
    nrf_gpio_cfg_default(ST7735_BL);
    nrf_gpio_cfg_default(VTFT_CTRL);
    nrf_gpio_cfg_default(SX126X_CS);
    nrf_gpio_cfg_default(SX126X_DIO1);
    nrf_gpio_cfg_default(SX126X_BUSY);
    nrf_gpio_cfg_default(SX126X_RESET);
    nrf_gpio_cfg_default(PIN_BUZZER_VOLTAGE_MULTIPLIER_1);
    nrf_gpio_cfg_default(PIN_BUZZER_VOLTAGE_MULTIPLIER_2);

    Serial.flush();
    Serial.end();
    sd_power_system_off();
  }
  intosleep=true;
  vTaskSuspend(checkUserkey1kHandle);
}

uint32_t gps_start_time=0;
void checkUserkey(void *pvParameters)
{
  uint32_t keydowntime;
  pinMode(USERKEY,INPUT);
  while(1)
  {
    delay(1);
    if(digitalRead(USERKEY)==0)
    {
      keydowntime=millis();
      debug_printf("key down : %u\r\n",keydowntime);
      delay(10);
      while(digitalRead(USERKEY)==0){
        delay(1);
        if( (millis()-keydowntime)>1000 )
        {
          break;
        }
      }
      if( (millis()-keydowntime)>1000 )
      {
        debug_printf("GPS START");
        st7735.setTextSize(2);
        st7735.fillScreen(ST7735_BLACK);
        st7735.setTextColor(ST7735_WHITE);
        st7735.setCursor(25,30);
        st7735.print("GPS START");
        Bluefruit.Scanner.stop();
        pinMode(35,ANALOG);
        gps_test_start=true;
        pinMode(LED ,OUTPUT);
	      digitalWrite(LED, !LED_ON_VALUE);  
        delay(1500);
        pinMode(LED ,OUTPUT);
        digitalWrite(LED, LED_ON_VALUE); 
        while(digitalRead(USERKEY)==0){delay(1);}
      }
      else
      {
        Bluefruit.Scanner.stop();
        intodeepsleep();
      }
    }
  }
}


void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  InternalFS.begin();
  boardInit(LORA_DEBUG_ENABLE,LORA_DEBUG_SERIAL_NUM,115200);
  makeChipIdText();
  Serial.print(",CHIPID=");
  Serial.println(chipIdText);
  debug_printf("start\r\n");
  xTaskCreate(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle);
  initDisplay();
  lora_init();

  if(!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_RED);
    delay(500);
  }
  if(!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_BLUE);
    delay(500);
  }
  if(!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_GREEN);
    delay(500);
  }
  if(!intosleep && !gps_test_start)
  {
    st7735.setTextSize(2);
    st7735.fillScreen(ST7735_BLACK);
    st7735.setTextColor(ST7735_BLUE);
    st7735.setCursor(5,30);
    st7735.print("FREQUENCY:HF");
    delay(1000);
  }
  
  if(!intosleep && !gps_test_start)
  {
    st7735.fillScreen(ST7735_BLACK);
    st7735.setTextColor(ST7735_GREEN);
    packet ="LORA MODE "+String(test_mode);
    delay(100);
    debug_printf("LORA MODE %d\r\n",test_mode);
    st7735.setCursor(st7735.width()/2-packet.length()/2*12, st7735.height()/2-16/2);
    st7735.println(packet);
    pinMode(LED ,OUTPUT);
    digitalWrite(LED, !LED_ON_VALUE);  
  }

  if(!intosleep && !gps_test_start)
  {
    int adcin = 5;
    int adcvalue = 0;
    float mv_per_lsb = 3000.0F / 4096.0F;  // 10-bit ADC with 3.6V input range
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    pinMode(11, OUTPUT);
    digitalWrite(11, 1);
    delay(10);
    for(int i=0;i<50;i++)
    {
      adcvalue += analogRead(adcin);
    }
    digitalWrite(11, 0);
    volt = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9/50);
  #if (TEST_MODE==1)
    ble_slave_start();
    test_mode=1;
  #else
    ble_center_start();
  #endif
  }
}

bool first_get_location=true;
uint32_t gps_get_time=0;

bool gps_started=false;
void loop()
{
  
  
  delay(1);
  if(gps_test_start)
  {
    if(gps_started==false)
    {
      gps_started=true;
      pinMode(PIN_GPS_EN, OUTPUT);
      digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
      pinMode(PIN_GPS_RESET, OUTPUT);
      digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE);
      delay(50);
      digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);
      delay(PERIPHERAL_WARMUP_MS);
      Serial1.begin(GPS_BAUDRATE);
      initGsvCaptors();
      gps_start_time=millis();
    }
    gps_test();
    return;
  }

  
  if(test_mode)
  {
    switch(state)
    {
      case STATE_TX:
        Radio.SetChannel( RF_FREQUENCY_2 );
        debug_printf("loraMode 1:into TX mode\r\n");
        Radio.Send( txpacket, 10 );
        state=LOWPOWER;
        break;
      case STATE_RX:
        Radio.SetChannel( RF_FREQUENCY_1 );
        debug_printf("loraMode 1:into RX mode\r\n");
        Radio.Rx( 0 );
        state=LOWPOWER;
        break;
      case LOWPOWER:
        TimerLowPowerHandler();
        Radio.IrqProcess();
        break;
      default:
        break;
    }
  }
  else
  {
    if(millis()>60000 && rx_cnt<3)
    {
      if(loratimeout==false)
      {
        loratimeout=true;
        showStatus(0);
      }
      return;
    }
    switch(state)
    {
      case STATE_TX:
        if(!intosleep)
        {
          delay(1000+randr(0,100));
          Radio.SetChannel( RF_FREQUENCY_1 );
          debug_printf("loraMode 0:into TX mode\r\n");
          Radio.Send( (uint8_t *)&chipId, 8 );
        }
        state=LOWPOWER;
        break;
      case STATE_RX:
        if(!intosleep)
        {
          Radio.SetChannel( RF_FREQUENCY_2 );
          debug_printf("loraMode 0:into RX mode\r\n");
          Radio.Rx( 1000 );
        }
        state=LOWPOWER;
        break;
      case LOWPOWER:
        if(intosleep)
        {
          Radio.Sleep();
        }
        TimerLowPowerHandler();
        Radio.IrqProcess();
        break;
      default:
        break;
    }
  }
}
