/* Mesh Pocket V2 MCU board + seven-LED factory fixture.
 * Wiring: MeshPocket_V2.pdf and MeshPocket_V2_Test.pdf.
 * See README.md for the operator sequence and validation limits.
 */
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <bluefruit.h>
#include <nrf_sdm.h>
#include "heltec_nrf_lorawan.h"
#include "gps_test.h"

#if !defined(HELTEC_MESH_POCKET_V2)
#error "Select Mesh Pocket V2 (HT-mesh-pocket-v2)."
#endif

// DUT only, equivalent to T1 TEST_MODE=0. Use a T1 mode-1 test peer.

static const uint32_t RF_FREQUENCY_1 = 868100000;
static const uint32_t RF_FREQUENCY_2 = 870900000;
static const uint32_t RADIO_TIMEOUT_MS = 30000;
static const uint32_t RADIO_RX_WINDOW_MS = 5000;
static const uint32_t RADIO_TX_TIMEOUT_MS = 3000;
static const int16_t LORA_RSSI_MIN = -100;
static const int16_t LORA_RSSI_MAX = -10;
static const int16_t LORA_RSSI_DIFF_MAX = 15;
static const int16_t BLE_RSSI_MIN = -70;
static const int16_t BLE_RSSI_MAX = -30;
static const uint16_t BLACK = 0x0000;
static const uint16_t WHITE = 0xFFFF;
static const uint16_t GREEN = 0x07E0;
static const uint16_t RED = 0xF800;

// LED3/5 names on the fixture describe the external 9006 end of the UART.
// The variant's TX/RX macros describe the MCU end, so they are reversed here.
static const uint8_t fixturePins[] = {
  PIN_9006_IO1, PIN_9006_IO2, PIN_9006_TX, PIN_CW_INT,
  PIN_9006_RX, PIN_WIRE_SDA, PIN_WIRE_SCL
};
// Indices into fixturePins, with the display text upright: LED3,5,7,6,4,2,1.
// Physical positions were confirmed from the previous LED1-to-LED7 test video.
static const uint8_t fixtureLeftToRight[] = {2, 4, 6, 5, 3, 1, 0};

static Arduino_HWSPI displayBus(PIN_TFT_DC, PIN_TFT_CS, &SPI1, true);
static Arduino_NV3001B display(&displayBus, PIN_TFT_RST, NV3001B_ROTATION,
                             NV3001B_IPS, NV3001B_PANEL_WIDTH,
                             NV3001B_PANEL_HEIGHT, 0, 0, 0, 0);

enum TestStage { STARTUP, RADIO_RESTART, RADIO_TEST, RESULT, GPS_TEST, SLEEPING };
enum RadioStep { RADIO_IDLE, RADIO_TX, RADIO_RX };
static volatile TestStage stage = STARTUP;
static TestStage gpsReturnStage = STARTUP;
static volatile RadioStep radioStep = RADIO_IDLE;
static RadioEvents_t radioEvents;
static uint64_t chipId;
static uint8_t request[8];
static uint8_t rxCount = 0;
static int16_t maxTxRssi = -255;
static int16_t maxRxRssi = -255;
static uint32_t radioStartedAt;
static volatile uint32_t nextTxAt;
static uint32_t lastGpsDraw;
static bool displayReady = false;
static bool displayAttempted = false;
static bool radioStarted = false;
static bool radioFinished = false;
static bool bleStarted = false;
static bool bleInitAttempted = false;
static bool bleReady = false;
static bool bleSessionStarted = false;
static bool loraResult = false;
static bool bleResult = false;
static bool autoResult = false;
static bool radioCooling = false;
static uint32_t radioQuietUntil;
static bool button1SleepArmed = false;
// Latch valid B2 releases even during display.begin() or one SPI drawing call.
// ISR only records input; all test/peripheral operations remain in the loop.
static volatile bool button2EdgeDown = false;
static volatile uint32_t button2EdgeAt;
static volatile uint32_t button2ReleaseCount = 0;
static uint32_t button2HandledCount = 0;
static volatile bool bleTesting = false;
static volatile int32_t bleRssi = -128; // No matching BLE advertisement yet.
static BLEClientUart bleClient;

struct Button {
  uint8_t pin;
  uint8_t raw;
  uint8_t stable;
  uint32_t changedAt;

  bool update(uint32_t now)
  {
    uint8_t value = digitalRead(pin);
    if (value != raw) {
      raw = value;
      changedAt = now;
    }
    if (stable == raw || (uint32_t)(now - changedAt) < 20) return false;
    stable = raw;
    return true;
  }
};

static Button button1 = {PIN_BUTTON1, HIGH, HIGH, 0};

static void pollButtons();
static void enterGps();
static void startRadioTest();

static void button2Edge()
{
  uint32_t now = tick2ms(xTaskGetTickCountFromISR());
  if (digitalRead(PIN_BUTTON2) == LOW) {
    button2EdgeDown = true;
    button2EdgeAt = now;
  } else if (button2EdgeDown) {
    button2EdgeDown = false;
    if ((uint32_t)(now - button2EdgeAt) >= 20) {
      ++button2ReleaseCount;
      // The radio wait can suspend loop(). Wake it to handle the latched
      // button event in normal task context, just as a radio IRQ wakes it.
      if (stage == RADIO_TEST && radioStep == RADIO_IDLE) wakefromsleep();
    }
  }
}

static bool canSleep()
{
  return stage == RESULT || (stage == GPS_TEST && gpsReturnStage == RESULT);
}

static bool startupDelay(uint32_t duration)
{
  uint32_t started = millis();
  while ((uint32_t)(millis() - started) < duration) {
    delay(1);
    pollButtons();
    if (stage != STARTUP) return false;
  }
  return true;
}

static void showMessage(const char *title, const char *hint)
{
  if (!displayReady) return;
  display.fillScreen(BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println(title);
  display.setCursor(0, 40);
  display.println(hint);
  display.setCursor(0, 112);
  display.print("B2: GPS / back");
}

static void releaseFixturePins()
{
  for (uint8_t pin : fixturePins) {
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
  }
}

static void initDisplay()
{
  if (!displayAttempted) {
    displayAttempted = true;
    digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL, TFT_VDD_ENABLE);
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    delay(100);
    displayReady = display.begin(1000000);
    if (!displayReady) {
      Serial.println("Display init: FAIL; main result cannot pass");
      return;
    }
    display.setRotation(NV3001B_ROTATION);
    display.setTextWrap(false);
    digitalWrite(PIN_TFT_LEDA_CTL, TFT_LEDA_ENABLE);
  }
  if (!displayReady) return;
  pollButtons();
  if (stage != STARTUP) return;
  const uint16_t colors[] = {RED, 0x001F, GREEN};
  for (uint16_t color : colors) {
    display.fillScreen(color);
    if (!startupDelay(500)) return;
  }
}

static void runFixtureLights()
{
  // The fixture temporarily loads the product's UART/I2C/IO signals with LEDs.
  // Run only with the LED fixture, not with external product modules connected.
  for (uint8_t pin : fixturePins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.println("Fixture LEDs: LEFT-TO-RIGHT ON");
  for (uint8_t led : fixtureLeftToRight) {
    digitalWrite(fixturePins[led], HIGH);
    Serial.print("Fixture LED");
    Serial.print(led + 1);
    Serial.println(" ON");
    // Keep earlier LEDs on; the last step holds all seven on for 300 ms.
    if (!startupDelay(300)) {
      releaseFixturePins();
      return;
    }
  }
  Serial.println("Fixture LEDs: LEFT-TO-RIGHT OFF");
  for (uint8_t led : fixtureLeftToRight) {
    digitalWrite(fixturePins[led], LOW);
    Serial.print("Fixture LED");
    Serial.print(led + 1);
    Serial.println(" OFF");
    if (!startupDelay(300)) {
      releaseFixturePins();
      return;
    }
  }
  // The fixture does not define the product pin modes. Release all seven lines
  // before wireless testing, or immediately when GPS interrupts the sequence.
  releaseFixturePins();
  Serial.println("Fixture LEDs: sequence done; visual inspection remains manual");
}

static void bleConnected(uint16_t handle)
{
  // connect() already stops its scan. A late connection must not stop a new
  // session's scanner after returning from GPS.
  Bluefruit.disconnect(handle);
}

static void bleScan(ble_gap_evt_adv_report_t *report)
{
  if (!bleTesting || stage != RADIO_TEST) return;
  if (Bluefruit.Scanner.checkReportForService(report, bleClient)) {
    // Same advertisement-RSSI criterion as T1; this is not a BLE data loopback.
    bleRssi = report->rssi;
    if (Bluefruit.Central.connect(report)) return;
  }
  Bluefruit.Scanner.resume();
}

static void startBle()
{
  bleSessionStarted = false;
  bleRssi = -128;
  if (!bleInitAttempted) {
    bleInitAttempted = true;
    Bluefruit.autoConnLed(false); // The MCU board has no user LED.
    bleStarted = Bluefruit.begin(0, 1);
    if (!bleStarted) {
      Serial.println("BLE init: FAIL");
      return;
    }
    Bluefruit.setName("MeshPocket V2 DUT");
    Bluefruit.setTxPower(4);
    if (!bleClient.begin()) {
      Serial.println("BLE client init: FAIL");
      return;
    }
    Bluefruit.Central.setConnectCallback(bleConnected);
    Bluefruit.Scanner.setRxCallback(bleScan);
    Bluefruit.Scanner.restartOnDisconnect(false);
    Bluefruit.Scanner.setInterval(160, 80);
    Bluefruit.Scanner.useActiveScan(false);
    bleReady = true;
  }
  if (!bleReady) return;
  bleTesting = true;
  bleSessionStarted = Bluefruit.Scanner.start(0);
  if (!bleSessionStarted) {
    bleTesting = false;
    Serial.println("BLE scan: FAIL");
  }
}

static void stopBle()
{
  bleTesting = false;
  if (!bleStarted) return;
  Bluefruit.Scanner.stop();
  (void)sd_ble_gap_connect_cancel();
  if (Bluefruit.connected()) Bluefruit.disconnect(Bluefruit.connHandle());
}

static void retryRadio()
{
  nextTxAt = millis() + 1000 + randr(0, 100);
  radioStep = RADIO_TX;
}

static void printRadioPacket(const char *direction, uint32_t frequency,
                             const uint8_t *payload, uint16_t size)
{
  Serial.print("LoRa "); Serial.print(direction);
  Serial.print(" freq="); Serial.print(frequency);
  Serial.print(" Hz len="); Serial.print(size);
  Serial.print(" payload=");
  // Expected frames are 8/10 bytes. Bound logs for unrelated oversized frames.
  uint16_t shown = size < 32 ? size : 32;
  for (uint16_t i = 0; i < shown; ++i) {
    if (i) Serial.print(' ');
    if (payload[i] < 0x10) Serial.print('0');
    Serial.print(payload[i], HEX);
  }
  if (shown < size) Serial.print(" ...");
  Serial.println();
}

static void onTxDone()
{
  if (stage != RADIO_TEST) return;
  Serial.println("LoRa TX done");
  radioStep = RADIO_RX;
}

static void onTxTimeout()
{
  if (stage != RADIO_TEST) return;
  Radio.Sleep();
  Serial.println("LoRa TX timeout");
  retryRadio();
}

static void onRxTimeout()
{
  if (stage != RADIO_TEST) return;
  Serial.println("LoRa RX timeout");
  Radio.Sleep();
  retryRadio();
}

static void onRxError()
{
  if (stage != RADIO_TEST) return;
  Serial.println("LoRa RX error");
  onRxTimeout();
}

static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  if (stage != RADIO_TEST) return;
  printRadioPacket("RX", RF_FREQUENCY_2, payload, size);
  Serial.print("LoRa RX RSSI="); Serial.print(rssi);
  Serial.print(" dBm SNR="); Serial.print(snr); Serial.println(" dB");
  Radio.Sleep();
  retryRadio();
  if (size != 10) {
    Serial.println("LoRa RX ignored: expected 10-byte reply");
    return;
  }
  uint64_t receivedId;
  int16_t txRssi;
  // Payload need not be aligned; memcpy preserves T1's 8+2 byte wire format.
  memcpy(&receivedId, payload, 8);
  if (receivedId != chipId) {
    Serial.println("LoRa RX ignored: CHIPID mismatch");
    return;
  }
  memcpy(&txRssi, payload + 8, 2);
  if (txRssi > maxTxRssi) maxTxRssi = txRssi;
  if (rssi > maxRxRssi) maxRxRssi = rssi;
  ++rxCount;
  Serial.print("LoRa RX accepted: peer RSSI="); Serial.print(txRssi); Serial.println(" dBm");
  Serial.print("LoRa matching replies: ");
  Serial.println(rxCount);
  if (rxCount >= 3) {
    radioFinished = true;
    radioStep = RADIO_IDLE;
  }
}

static void startRadioTest()
{
  if (radioCooling && (int32_t)(millis() - radioQuietUntil) < 0) {
    stage = RADIO_RESTART;
    showMessage("Restart LoRa + BLE", "Waiting for old radio timeout");
    return;
  }
  // Driver sleep does not cancel its software RX/TX timers. Do not re-init
  // linked timer objects, or let an old timeout interrupt the next session.
  if (radioStarted) Radio.IrqProcess();
  radioCooling = false;
  rxCount = 0;
  maxTxRssi = maxRxRssi = -255;
  radioFinished = false;
  radioStep = RADIO_IDLE;
  nextTxAt = 0;
  button1SleepArmed = false;
  stage = RADIO_TEST;
  showMessage("LoRa + BLE test", "868.1 / 870.9 MHz");
  // Match T1: finish Radio init, random seed and TX/RX setup before BLE.
  radioEvents.TxDone = onTxDone;
  radioEvents.TxTimeout = onTxTimeout;
  radioEvents.RxDone = onRxDone;
  radioEvents.RxTimeout = onRxTimeout;
  radioEvents.RxError = onRxError;
  if (!radioStarted) {
    Radio.Init(&radioEvents);
    radioStarted = true;
    srand1(Radio.Random());
  }
  Radio.SetTxConfig(MODEM_LORA, 10, 0, 0, 7, 1, 8, false, true, 0, 0, false, RADIO_TX_TIMEOUT_MS);
  Radio.SetRxConfig(MODEM_LORA, 0, 7, 1, 0, 8, 0, false, 0, true, 0, 0, false, true);
  startBle();
  // Peripheral setup and any old-session cooldown do not consume the 30s test.
  radioStartedAt = millis();
  retryRadio();
}

static void showResults()
{
  if (!displayReady) return;
  display.fillScreen(BLACK);
  display.setTextSize(1);
  display.setTextColor(loraResult ? GREEN : RED);
  display.setCursor(0, 0);
  display.print(loraResult ? "LoRa: PASS" : "LoRa: FAIL");
  display.setCursor(0, 14);
  display.print("TX/RX RSSI:");
  display.print(maxTxRssi); display.print('/'); display.print(maxRxRssi);
  display.setTextColor(bleResult ? GREEN : RED);
  display.setCursor(0, 30);
  display.print(bleResult ? "BLE: PASS" : "BLE: FAIL");
  display.setCursor(0, 44);
  display.print("BLE RSSI:");
  if (bleRssi == -128) display.print("NO DATA");
  else display.print((int32_t)bleRssi);
  display.setTextColor(WHITE);
  display.setCursor(0, 60);
  display.print("Display init: OK");
  display.setCursor(0, 76);
  display.setTextColor(autoResult ? GREEN : RED);
  display.print(autoResult ? "Auto result: PASS" : "Auto result: FAIL");
  display.setTextColor(WHITE);
  display.setCursor(0, 94);
  display.print("Screen/LED: manual inspection");
  display.setCursor(0, 112);
  display.print("B1: sleep  B2: GPS");
}

static void finishRadioTest()
{
  stage = RESULT;
  radioStep = RADIO_IDLE;
  Radio.Sleep();
  stopBle();
  int32_t measuredBleRssi = bleRssi;
  bool txInRange = maxTxRssi >= LORA_RSSI_MIN && maxTxRssi <= LORA_RSSI_MAX;
  bool rxInRange = maxRxRssi >= LORA_RSSI_MIN && maxRxRssi <= LORA_RSSI_MAX;
  int rssiDifference = abs(maxTxRssi - maxRxRssi);
  bool differencePass = rssiDifference <= LORA_RSSI_DIFF_MAX;
  loraResult = rxCount >= 3 && txInRange && rxInRange && differencePass;
  bleResult = bleSessionStarted && measuredBleRssi >= BLE_RSSI_MIN && measuredBleRssi <= BLE_RSSI_MAX;
  autoResult = displayReady && loraResult && bleResult;
  Serial.println("Screen/fixture LEDs: manual inspection, excluded from auto result");
  Serial.print("LoRa: ");
  Serial.print(loraResult ? "PASS" : "FAIL");
  Serial.print(" RSSI TX/RX=");
  Serial.print(maxTxRssi);
  Serial.print('/');
  Serial.println(maxRxRssi);
  Serial.print("LoRa absolute RSSI ["); Serial.print(LORA_RSSI_MIN);
  Serial.print(','); Serial.print(LORA_RSSI_MAX); Serial.print("] dBm: TX=");
  Serial.print(txInRange ? "PASS" : "FAIL");
  Serial.print(" RX="); Serial.println(rxInRange ? "PASS" : "FAIL");
  Serial.print("LoRa relative RSSI <="); Serial.print(LORA_RSSI_DIFF_MAX); Serial.print(" dB: ");
  if (rxCount == 0) Serial.println("NO DATA");
  else {
    Serial.print(rssiDifference); Serial.print(" dB ");
    Serial.println(differencePass ? "PASS" : "FAIL");
  }
  Serial.print("LoRa matching replies >=3: "); Serial.println(rxCount);
  Serial.print("BLE: ");
  Serial.print(bleResult ? "PASS" : "FAIL");
  Serial.print(" RSSI=");
  if (measuredBleRssi == -128) Serial.println("NO DATA");
  else Serial.println(measuredBleRssi);
  Serial.print("BLE absolute RSSI range: ["); Serial.print(BLE_RSSI_MIN);
  Serial.print(','); Serial.print(BLE_RSSI_MAX); Serial.println("] dBm");
  Serial.println(autoResult ? "Test result:OK" : "Test result:Failed");
  Serial.println("GPS: separate manual test, not included in main result");
  showResults();
}

static void pollRadio()
{
  if ((uint32_t)(millis() - radioStartedAt) >= RADIO_TIMEOUT_MS) {
    finishRadioTest();
    return;
  }
  if (radioFinished) {
    finishRadioTest();
    return;
  }
  if (radioStep == RADIO_TX) {
    if ((int32_t)(millis() - nextTxAt) < 0) return;
    Radio.SetChannel(RF_FREQUENCY_1);
    printRadioPacket("TX", RF_FREQUENCY_1, request, sizeof(request));
    Radio.Send(request, sizeof(request));
    radioStep = RADIO_IDLE;
  } else if (radioStep == RADIO_RX) {
    Radio.SetChannel(RF_FREQUENCY_2);
    Serial.print("LoRa RX start freq="); Serial.print(RF_FREQUENCY_2);
    Serial.print(" Hz window_ms="); Serial.println(RADIO_RX_WINDOW_MS);
    Radio.Rx(RADIO_RX_WINDOW_MS);
    radioStep = RADIO_IDLE;
  } else {
    // Same order as T1's LOWPOWER branch: start TX/RX first, then wait.
    // Never suspend loop() while a software-scheduled TX is still pending.
    if (button2ReleaseCount == button2HandledCount) {
      TimerLowPowerHandler();
    }
    Radio.IrqProcess();
  }
}

static void enterGps()
{
  if (stage == GPS_TEST) return;
  gpsReturnStage = stage == RADIO_RESTART ? RADIO_TEST : stage;
  button1SleepArmed = false;
  // Ignore late radio timeout callbacks before shutting down that test.
  stage = GPS_TEST;
  stopBle();
  if (radioStarted) {
    Radio.IrqProcess();
    Radio.Sleep();
    radioCooling = true;
    radioQuietUntil = millis() + RADIO_RX_WINDOW_MS + 20;
  }
  radioStep = RADIO_IDLE;
  releaseFixturePins();
  // Clear and draw fixed labels before GNSS starts producing UART data.
  if (displayReady) {
    const char *footer = canSleep() ? "B1 sleep  B2 back" : "B2 back: finish RF first";
    gpsPrepareDisplay(display, footer);
  }
  gpsBegin();
  lastGpsDraw = millis() - 1000;
}

static void leaveGps()
{
  // Stop the UART before any full-page drawing or wireless reconfiguration.
  gpsEnd();
  button1SleepArmed = false;
  if (gpsReturnStage == RESULT) {
    stage = RESULT;
    showResults();
  } else if (gpsReturnStage == STARTUP) {
    stage = STARTUP;
  } else {
    stage = RADIO_RESTART;
    startRadioTest();
  }
}

static void enterSleep()
{
  stage = SLEEPING;
  detachInterrupt(PIN_BUTTON2);
  Serial.println("Entering system-off; reset or power-cycle to restart");
  stopBle();
  if (radioStarted) {
    Radio.Sleep();
    detachInterrupt(SX126X_DIO1);
  }
  if (displayReady) display.fillScreen(BLACK);
  // Follow Mesh_Pocket_V2_V1.0: stop the buses before switching off the panel,
  // then let the power-control outputs settle before releasing signal GPIO.
  SPI.end();
  SPI1.end();
  gpsEnd();
  digitalWrite(PIN_TFT_LEDA_CTL, !TFT_LEDA_ENABLE);
  digitalWrite(PIN_TFT_VDD_CTL, !TFT_VDD_ENABLE);
  delay(100);
  releaseFixturePins();
  for (uint8_t pin : fixturePins) nrf_gpio_cfg_default(pin);
  const uint8_t releasedPins[] = {
    PIN_BUTTON1, PIN_BUTTON2, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCK,
    PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, PIN_TFT_CS, PIN_TFT_DC,
    PIN_TFT_RST, PIN_GPS_RESET, PIN_GPS_PPS, GPS_RX_PIN, GPS_TX_PIN,
    SX126X_CS, SX126X_DIO1, SX126X_BUSY, SX126X_RESET
  };
  for (uint8_t pin : releasedPins) nrf_gpio_cfg_default(pin);
  // Keep the power-control outputs at their inactive levels.
  Serial.flush();
  Serial.end();
  digitalWrite(PIN_IO_CSA, LOW);
  uint8_t softDeviceEnabled = 0;
  sd_softdevice_is_enabled(&softDeviceEnabled);
  if (softDeviceEnabled) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
    __DSB();
  }
  // Also prevents further peripheral access if system-off returns under a debugger.
}

static void pollButtons()
{
  uint32_t now = millis();
  bool button1Changed = button1.update(now);
  uint32_t requests = button2ReleaseCount;
  if (requests != button2HandledCount) {
    button2HandledCount = requests;
    if (stage == GPS_TEST) leaveGps();
    else enterGps();
    return;
  }
  if (button1Changed && button1.stable == LOW) button1SleepArmed = canSleep();
  if (button1Changed && button1.stable == HIGH) {
    bool allowed = button1SleepArmed && canSleep();
    button1SleepArmed = false;
    if (allowed) enterSleep();
    else if (!canSleep()) Serial.println("B1 ignored: finish LoRa/BLE before sleep");
  }
}

static void runStartup()
{
  initDisplay();
  if (stage != STARTUP) return;
  runFixtureLights();
  if (stage == STARTUP) startRadioTest();
}

void setup()
{
  Serial.begin(115200);
  // Force the board helper's debug UART to USB, reserving Serial1 for UC6580.
  boardInit(LORA_DEBUG_ENABLE, 0, 115200);
  chipId = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
  memcpy(request, &chipId, sizeof(request));
  char idText[17];
  snprintf(idText, sizeof(idText), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
  Serial.print("MeshPocket_V2,CHIPID=");
  Serial.println(idText);
  Serial.println("MODE=DUT");
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  button2EdgeDown = digitalRead(PIN_BUTTON2) == LOW;
  button2EdgeAt = millis();
  attachInterrupt(PIN_BUTTON2, button2Edge, CHANGE);
  digitalWrite(PIN_GPS_EN, !GPS_EN_ACTIVE);
  pinMode(PIN_GPS_EN, OUTPUT);
  runStartup();
}

void loop()
{
  delay(1);
  if (stage == SLEEPING) return;
  pollButtons();
  if (stage == STARTUP) runStartup();
  else if (stage == RADIO_RESTART) {
    if (radioStarted) Radio.IrqProcess();
    if (!radioCooling || (int32_t)(millis() - radioQuietUntil) >= 0) startRadioTest();
  } else if (stage == RADIO_TEST) pollRadio();
  else if (stage == GPS_TEST) {
    // Consume old software timeouts while the radio callbacks are gated off.
    if (radioStarted) Radio.IrqProcess();
    gpsPoll();
    if (displayReady && (uint32_t)(millis() - lastGpsDraw) >= 1000) {
      lastGpsDraw = millis();
      gpsDraw(display);
    }
  } else if (stage == RESULT && radioStarted) Radio.IrqProcess();
}
