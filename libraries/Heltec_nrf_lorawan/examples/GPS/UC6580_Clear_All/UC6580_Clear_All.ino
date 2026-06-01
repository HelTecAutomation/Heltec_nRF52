#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <ctype.h>
#include <string.h>

/*
  UC6580 full clear example for Heltec nRF52 boards.

  The command sequence follows the UFirebird protocol:
    $RESET,0,H85   Software reset and clear ephemeris, position/time,
                   and almanac.

  H85 is the cold-start reset mask listed in the protocol. Do not use HFF
  because the protocol marks the other bits as reserved.

  Default fallback pins match Mesh Node T096 (HT-n5262G):
    GNSS power: P0.06, active LOW
    GNSS reset: P1.14, active LOW
    GNSS UART:  Serial2 RX P0.23, TX P0.25, 115200 baud
*/

#ifndef GNSS_BAUD
#define GNSS_BAUD 115200
#endif

#ifndef GNSS_UART
#define GNSS_UART Serial2
#endif

#ifdef GPS_RX_PIN
#define GNSS_RX_PIN GPS_RX_PIN
#else
#define GNSS_RX_PIN (0 + 23)
#endif

#ifdef GPS_TX_PIN
#define GNSS_TX_PIN GPS_TX_PIN
#else
#define GNSS_TX_PIN (0 + 25)
#endif

#ifdef PIN_GPS_EN
#define GNSS_PWR_PIN PIN_GPS_EN
#else
#define GNSS_PWR_PIN (0 + 6)
#endif

#ifdef GPS_EN_ACTIVE
#define GNSS_PWR_ON_LEVEL GPS_EN_ACTIVE
#else
#define GNSS_PWR_ON_LEVEL LOW
#endif

#ifdef PIN_GPS_RESET
#define GNSS_RESET_PIN PIN_GPS_RESET
#else
#define GNSS_RESET_PIN (32 + 14)
#endif

#ifdef GPS_RESET_MODE
#define GNSS_RESET_ACTIVE_LEVEL GPS_RESET_MODE
#else
#define GNSS_RESET_ACTIVE_LEVEL LOW
#endif

#ifndef UC6580_BOOT_WAIT_MS
#define UC6580_BOOT_WAIT_MS 1000UL
#endif

#ifndef UC6580_RESET_PULSE_MS
#define UC6580_RESET_PULSE_MS 120UL
#endif

#ifndef UC6580_RESET_WAIT_MS
#define UC6580_RESET_WAIT_MS 3000UL
#endif

#ifndef UC6580_ACK_TIMEOUT_MS
#define UC6580_ACK_TIMEOUT_MS 3000UL
#endif

#define UC6580_MAX_MESSAGE_LEN 128

static bool clearAlreadySent = false;

enum Uc6580AckResult {
  UC6580_ACK_TIMEOUT,
  UC6580_ACK_OK,
  UC6580_ACK_FAIL
};

static char hexNibble(uint8_t value)
{
  value &= 0x0F;
  return value < 10 ? char('0' + value) : char('A' + value - 10);
}

static uint8_t uc6580Checksum(const char *body)
{
  uint8_t checksum = 0;
  while (*body != '\0') {
    checksum ^= uint8_t(toupper((unsigned char)*body));
    ++body;
  }
  return checksum;
}

static bool makeUc6580Command(const char *body, char *out, size_t outSize)
{
  const size_t bodyLen = strlen(body);
  const size_t totalLen = bodyLen + 6;  // '$' + body + '*' + 2 hex + "\r\n"

  if (outSize <= totalLen || totalLen > UC6580_MAX_MESSAGE_LEN) {
    return false;
  }

  const uint8_t checksum = uc6580Checksum(body);

  out[0] = '$';
  memcpy(out + 1, body, bodyLen);
  out[bodyLen + 1] = '*';
  out[bodyLen + 2] = hexNibble(checksum >> 4);
  out[bodyLen + 3] = hexNibble(checksum);
  out[bodyLen + 4] = '\r';
  out[bodyLen + 5] = '\n';
  out[bodyLen + 6] = '\0';
  return true;
}

static void printGnssConfig()
{
  Serial.println();
  Serial.println("UC6580 full clear");
  Serial.print("GNSS baud: ");
  Serial.println(GNSS_BAUD);
  Serial.print("GNSS RX pin: ");
  Serial.println(GNSS_RX_PIN);
  Serial.print("GNSS TX pin: ");
  Serial.println(GNSS_TX_PIN);
  Serial.print("GNSS power pin: ");
  Serial.println(GNSS_PWR_PIN);
  Serial.print("GNSS reset pin: ");
  Serial.println(GNSS_RESET_PIN);
}

static void enableGnssPower()
{
  pinMode(PIN_VEXT_CTL, OUTPUT);
  digitalWrite(PIN_VEXT_CTL, VEXT_ENABLE);
  if (GNSS_PWR_PIN >= 0) {
    pinMode(GNSS_PWR_PIN, OUTPUT);
    digitalWrite(GNSS_PWR_PIN, GNSS_PWR_ON_LEVEL);
    Serial.println("GNSS power control pin enabled");
  } else {
    Serial.println("GNSS power control pin not configured");
  }
}

static void pulseGnssReset()
{
  if (GNSS_RESET_PIN >= 0) {
    pinMode(GNSS_RESET_PIN, OUTPUT);
    digitalWrite(GNSS_RESET_PIN, GNSS_RESET_ACTIVE_LEVEL);
    delay(UC6580_RESET_PULSE_MS);
    digitalWrite(GNSS_RESET_PIN, !GNSS_RESET_ACTIVE_LEVEL);
    Serial.println("GNSS reset pulse sent");
  } else {
    Serial.println("GNSS reset pin not configured");
  }
}

static void beginGnssSerial()
{
  GNSS_UART.begin(GNSS_BAUD);
}

static void drainGnssInput(unsigned long durationMs)
{
  const unsigned long start = millis();
  while (millis() - start < durationMs) {
    while (GNSS_UART.available() > 0) {
      Serial.write(GNSS_UART.read());
    }
  }
}

static bool sendUc6580Command(const char *body)
{
  char command[UC6580_MAX_MESSAGE_LEN + 1];
  if (!makeUc6580Command(body, command, sizeof(command))) {
    Serial.print("Command too long, not sent: ");
    Serial.println(body);
    return false;
  }

  Serial.print("TX: ");
  Serial.print(command);
  GNSS_UART.print(command);
  GNSS_UART.flush();
  return true;
}

static Uc6580AckResult waitUc6580Ack(const char *body, unsigned long timeoutMs)
{
  char line[UC6580_MAX_MESSAGE_LEN + 1];
  size_t lineLen = 0;
  const unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (GNSS_UART.available() > 0) {
      const char c = char(GNSS_UART.read());
      Serial.write(c);

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        line[lineLen] = '\0';

        if (strstr(line, "$OK") != NULL) {
          Serial.print("ACK OK for ");
          Serial.println(body);
          return UC6580_ACK_OK;
        }

        if (strstr(line, "$FAIL") != NULL) {
          Serial.print("ACK FAIL for ");
          Serial.println(body);
          return UC6580_ACK_FAIL;
        }

        lineLen = 0;
        continue;
      }

      if (lineLen < sizeof(line) - 1) {
        line[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
    delay(1);
  }

  Serial.print("ACK TIMEOUT for ");
  Serial.println(body);
  return UC6580_ACK_TIMEOUT;
}

static void sendFullClearOnce()
{
  if (clearAlreadySent) {
    return;
  }
  clearAlreadySent = true;

  Serial.println("Waiting for UC6580 boot...");
  delay(UC6580_BOOT_WAIT_MS);
  drainGnssInput(200);

  if (sendUc6580Command("RESET,0,H85")) {
    waitUc6580Ack("RESET,0,H85", UC6580_ACK_TIMEOUT_MS);
  }
  Serial.println("Cold-start clear command sequence sent. Listening for receiver output...");
  drainGnssInput(UC6580_RESET_WAIT_MS);
}

void setup()
{
  Serial.begin(115200);
  for(int i = 0; i < 10; ++i) {
    Serial.println("waiting for Serial...");
    delay(100);
  }

  printGnssConfig();
  enableGnssPower();
  pulseGnssReset();
  beginGnssSerial();
  sendFullClearOnce();
}

void loop()
{
  while (GNSS_UART.available() > 0) {
    Serial.write(GNSS_UART.read());
  }

  while (Serial.available() > 0) {
    GNSS_UART.write(Serial.read());
  }
  delay(1);
}
