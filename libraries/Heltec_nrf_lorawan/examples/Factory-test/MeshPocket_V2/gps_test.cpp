#include "gps_test.h"
#include "TinyGPS.h"
#include <stdlib.h>
#include <string.h>

static TinyGPSPlus gps;
// Static zero-initialization also covers staging members omitted by the
// library's constructor; copying an automatic temporary would not do that.
static const TinyGPSPlus freshGps;
static const char *const gsvNames[] = {"GPGSV", "GBGSV", "GLGSV", "GAGSV", "GQGSV"};
static const char *const constellationNames[] = {"GP", "BD", "GL", "GA", "QZ"};
static TinyGPSCustom snr[5][4];
static const char *const rmcNames[] = {"GPRMC", "GNRMC"};
static TinyGPSCustom rmcDate[2];
static TinyGPSCustom rmcTime[2];
static char utcText[20];
static bool utcAvailable = false;
static uint32_t utcAt;
static uint8_t peakSnr[5] = {};
static uint32_t startedAt;
static uint32_t commandAt;
static uint32_t firstFixMs;
static bool running = false;
static bool clearSent = false;
static bool clearThisSession = false;
static bool hasFix = false;
static char ackLine[128];
static size_t ackLength = 0;
static const char *ackStatus = "BOOT";
static bool ackPending = false;
static bool diagnosticActive = false;
static uint32_t receivedBytes;
static uint32_t droppedLogBytes;
static uint32_t passedAtStart;
static uint32_t failedAtStart;
static uint32_t lastPollUs;
static uint32_t maxPollGapUs;
static uint32_t maxCharDrawUs;

static void finishDiagnostic()
{
  if (!diagnosticActive) return;
  diagnosticActive = false;
  Serial.print("\r\nGNSS DIAG END: elapsed_ms=");
  Serial.print(millis() - startedAt);
  Serial.print(" RX_BYTES=");
  Serial.print(receivedBytes);
  Serial.print(" NMEA_PASS=");
  Serial.print(gps.passedChecksum() - passedAtStart);
  Serial.print(" NMEA_FAIL=");
  Serial.print(gps.failedChecksum() - failedAtStart);
  Serial.print(" USB_DROPPED=");
  Serial.print(droppedLogBytes);
  Serial.print(" POLL_GAP_MAX_US=");
  Serial.print(maxPollGapUs);
  Serial.print(" CHAR_DRAW_MAX_US=");
  Serial.println(maxCharDrawUs);
}

static void traceReceivedByte(char c)
{
  if (!diagnosticActive) return;
  ++receivedBytes;
  // Do not wait for USB space while draining the small GNSS UART RX buffer.
  if (Serial && Serial.availableForWrite() > 0) {
    if (Serial.write((uint8_t)c) != 1) ++droppedLogBytes;
  } else {
    ++droppedLogBytes;
  }
}

void gpsBegin()
{
  if (running) return;
  running = true;
  // Reset the parser before registering the same custom fields again. Without
  // this, re-entering GPS inserts those objects into their own existing list.
  gps = freshGps;
  hasFix = false;
  firstFixMs = 0;
  memset(peakSnr, 0, sizeof(peakSnr));
  utcAvailable = false;
  utcAt = 0;
  utcText[0] = '\0';
  ackLength = 0;
  ackPending = false;
  clearThisSession = false;
  for (uint8_t system = 0; system < 5; ++system) {
    for (uint8_t satellite = 0; satellite < 4; ++satellite) {
      snr[system][satellite].begin(gps, gsvNames[system], 7 + 4 * satellite);
    }
  }
  for (uint8_t talker = 0; talker < 2; ++talker) {
    rmcDate[talker].begin(gps, rmcNames[talker], 9);
    rmcTime[talker].begin(gps, rmcNames[talker], 1);
  }

  digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
  pinMode(PIN_GPS_EN, OUTPUT);
  digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE);
  pinMode(PIN_GPS_RESET, OUTPUT);
  delay(50);
  digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);
  Serial1.setPins(PIN_SERIAL1_RX, PIN_SERIAL1_TX);
  Serial1.begin(GPS_BAUDRATE);
  startedAt = millis();
  receivedBytes = droppedLogBytes = 0;
  passedAtStart = gps.passedChecksum();
  failedAtStart = gps.failedChecksum();
  lastPollUs = micros();
  maxPollGapUs = maxCharDrawUs = 0;
  diagnosticActive = true;
  Serial.println(clearSent ? "GPS START: new session; cold clear already attempted this boot"
                           : "GPS START: UC6580 cold start; GPS is outside the main result");
  Serial.println("GNSS DIAG START: raw RX until GPS exit");
}

static void sendFullClearOnce()
{
  if (clearSent) return;
  clearSent = true;
  clearThisSession = true;
  const char body[] = "RESET,0,H85";
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(body) - 1; ++i) checksum ^= body[i];
  char command[24];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", body, checksum);
  Serial.print("GNSS TX: ");
  Serial.print(command);
  Serial1.print(command);
  Serial1.flush();
  commandAt = millis();
  ackStatus = "WAIT";
  ackPending = true;
}

static void readAck(char c)
{
  if (!ackPending || c == '\r') return;
  if (c == '\n') {
    ackLine[ackLength] = '\0';
    if (strncmp(ackLine, "$OK", 3) == 0) {
      ackStatus = "OK";
      ackPending = false;
    } else if (strncmp(ackLine, "$FAIL", 5) == 0) {
      ackStatus = "FAIL";
      ackPending = false;
    }
    if (!ackPending) {
      Serial.print("GNSS reset ACK: ");
      Serial.println(ackStatus);
    }
    ackLength = 0;
  } else if (ackLength < sizeof(ackLine) - 1) {
    ackLine[ackLength++] = c;
  } else {
    ackLength = 0;
  }
}

static uint8_t twoDigits(const char *text)
{
  return (text[0] - '0') * 10 + text[1] - '0';
}

static bool formatUtc(const char *date, const char *time)
{
  if (strlen(date) != 6 || strlen(time) < 6) return false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (date[i] < '0' || date[i] > '9' || time[i] < '0' || time[i] > '9') return false;
  }
  if (time[6]) {
    if (time[6] != '.' || !time[7]) return false;
    for (size_t i = 7; time[i]; ++i) {
      if (time[i] < '0' || time[i] > '9') return false;
    }
  }
  uint8_t day = twoDigits(date);
  uint8_t month = twoDigits(date + 2);
  uint16_t year = 2000 + twoDigits(date + 4);
  uint8_t hour = twoDigits(time);
  uint8_t minute = twoDigits(time + 2);
  uint8_t second = twoDigits(time + 4);
  static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12 || day < 1 || hour > 23 || minute > 59 || second > 60) return false;
  uint8_t days = monthDays[month - 1];
  if (month == 2 && year % 4 == 0) ++days; // RMC years here are 2000..2099.
  if (day > days) return false;
  snprintf(utcText, sizeof(utcText), "%04u-%02u-%02u %02u:%02u:%02u",
           (unsigned)year, (unsigned)month, (unsigned)day,
           (unsigned)hour, (unsigned)minute, (unsigned)second);
  return true;
}

void gpsPoll()
{
  if (!running) return;
  uint32_t nowUs = micros();
  if (diagnosticActive && (uint32_t)(nowUs - lastPollUs) > maxPollGapUs) {
    maxPollGapUs = nowUs - lastPollUs;
  }
  lastPollUs = nowUs;
  if (!clearSent && (uint32_t)(millis() - startedAt) >= PERIPHERAL_WARMUP_MS) {
    // Discard warm-start output so it cannot become this cold start's first fix.
    int pending = Serial1.available();
    while (pending-- > 0) traceReceivedByte((char)Serial1.read());
    sendFullClearOnce();
  }

  // Bound each read pass so Button2 remains responsive even with continuous NMEA.
  for (uint16_t count = 0; count < 256 && Serial1.available(); ++count) {
    char c = (char)Serial1.read();
    traceReceivedByte(c);
    if (!clearSent || (uint32_t)(millis() - startedAt) < PERIPHERAL_WARMUP_MS) continue;
    readAck(c);
    // Do not count buffered pre-reset fixes while waiting for the reset reply
    // and receiver restart. A missing ACK must not stop NMEA forever.
    if (clearThisSession &&
        (ackPending || (uint32_t)(millis() - commandAt) < PERIPHERAL_WARMUP_MS)) continue;
    if (!gps.encode(c)) continue;
    for (uint8_t talker = 0; talker < 2; ++talker) {
      if (rmcDate[talker].isUpdated()) {
        utcAvailable = formatUtc(rmcDate[talker].value(), rmcTime[talker].value());
        utcAt = millis();
      }
    }
    for (uint8_t system = 0; system < 5; ++system) {
      for (uint8_t satellite = 0; satellite < 4; ++satellite) {
        if (snr[system][satellite].isUpdated()) {
          int value = atoi(snr[system][satellite].value());
          if (value > peakSnr[system] && value <= 99) peakSnr[system] = value;
        }
      }
    }
    if (!hasFix && gps.location.isValid() && gps.location.age() < 1000) {
      hasFix = true;
      firstFixMs = millis() - startedAt;
      Serial.print("GNSS first fix ms: ");
      Serial.println(firstFixMs);
    }
  }
  if (ackPending && (uint32_t)(millis() - commandAt) >= 2000UL) {
    ackPending = false;
    ackStatus = "TIMEOUT";
    Serial.println("GNSS reset ACK: TIMEOUT; continuing NMEA reception");
  }
}

enum GpsFieldId { UTC_FIELD, FIX_FIELD, WAIT_FIELD, LAT_FIELD, LON_FIELD,
                  ALT_FIELD, HDOP_FIELD, SPEED_FIELD, ACK_FIELD, SNR_FIELD };

// Fixed character cells keep shorter values from leaving old digits behind.
struct GpsField {
  int16_t x, y;
  uint8_t width;
  char previous[21];
  uint16_t color;
};
static GpsField fields[] = {
  {0, 0, 19, {}, 0}, {204, 0, 1, {}, 0}, {0, 14, 20, {}, 0},
  {24, 28, 16, {}, 0}, {24, 40, 16, {}, 0}, {24, 52, 16, {}, 0},
  {30, 64, 15, {}, 0}, {30, 76, 15, {}, 0}, {60, 96, 7, {}, 0},
  {150, 28, 3, {}, 0}, {150, 40, 3, {}, 0}, {150, 52, 3, {}, 0},
  {150, 64, 3, {}, 0}, {150, 76, 3, {}, 0}
};

static void drawField(Arduino_GFX &display, uint8_t index, const char *text,
                      uint16_t color = 0xFFFF)
{
  GpsField &field = fields[index];
  size_t length = strlen(text);
  gpsPoll();
  for (uint8_t i = 0; i < field.width; ++i) {
    char c = i < length ? text[i] : ' ';
    if (field.previous[i] == c && field.color == color) continue;
    uint32_t before = micros();
    // Opaque 6x8 text uses the TFT's small batched character transfer.
    display.drawChar(field.x + 6 * i, field.y, c, color, 0x0000);
    uint32_t elapsed = micros() - before;
    if (diagnosticActive && elapsed > maxCharDrawUs) maxCharDrawUs = elapsed;
    field.previous[i] = c;
    // A whole line can exceed the 63-byte UART buffer's ~5.5 ms budget.
    gpsPoll();
  }
  field.color = color;
}

void gpsPrepareDisplay(Arduino_GFX &display, const char *footer)
{
  // This full clear and all fixed text must finish before Serial1.begin().
  display.fillScreen(0x0000);
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setTextColor(0xFFFF, 0x0000);
  for (GpsField &field : fields) {
    memset(field.previous, 0, sizeof(field.previous));
    field.color = 0;
  }
  display.setCursor(0, 28); display.print("Lat:");
  display.setCursor(0, 40); display.print("Lon:");
  display.setCursor(0, 52); display.print("Alt:");
  display.setCursor(0, 64); display.print("HDOP:");
  display.setCursor(0, 76); display.print("km/h:");
  display.setCursor(0, 96); display.print("Reset ACK:");
  display.setCursor(132, 14); display.print("Peak SNR");
  for (uint8_t system = 0; system < 5; ++system) {
    display.setCursor(132, 28 + 12 * system);
    display.print(constellationNames[system]);
    display.print(":");
    drawField(display, SNR_FIELD + system, "0");
  }
  display.setCursor(0, 112); display.print(footer);
  drawField(display, UTC_FIELD, "UTC: --");
  drawField(display, FIX_FIELD, "V");
  drawField(display, WAIT_FIELD, "Wait:0s", 0x07E0);
  drawField(display, LAT_FIELD, "--");
  drawField(display, LON_FIELD, "--");
  drawField(display, ALT_FIELD, "--");
  drawField(display, HDOP_FIELD, "--");
  drawField(display, SPEED_FIELD, "--");
  drawField(display, ACK_FIELD, ackStatus);
}

void gpsDraw(Arduino_GFX &display)
{
  // Startup/reset reception takes priority; retain the prepared page meanwhile.
  if (!running || !clearSent || ackPending ||
      (uint32_t)(millis() - startedAt) < PERIPHERAL_WARMUP_MS ||
      (clearThisSession && (uint32_t)(millis() - commandAt) < PERIPHERAL_WARMUP_MS)) return;
  uint32_t seconds = (hasFix ? firstFixMs : millis() - startedAt) / 1000;
  bool currentFix = gps.location.isValid() && gps.location.age() < 1000;
  char text[24];
  drawField(display, UTC_FIELD,
            utcAvailable && (uint32_t)(millis() - utcAt) <= 3000 ? utcText : "UTC: --");
  drawField(display, FIX_FIELD, currentFix ? "A" : "V", currentFix ? 0x07E0 : 0xFFFF);
  uint16_t timeColor = 0xF800;
  if (seconds < 90) timeColor = 0x07E0;
  else if (seconds < 180) timeColor = 0xFFE0;
  snprintf(text, sizeof(text), "%s%lus", hasFix ? "TTFF:" : "Wait:", (unsigned long)seconds);
  drawField(display, WAIT_FIELD, text, timeColor);
  if (gps.location.isValid()) snprintf(text, sizeof(text), "%.5f", gps.location.lat());
  else strcpy(text, "--");
  drawField(display, LAT_FIELD, text);
  if (gps.location.isValid()) snprintf(text, sizeof(text), "%.5f", gps.location.lng());
  else strcpy(text, "--");
  drawField(display, LON_FIELD, text);
  if (gps.altitude.isValid()) snprintf(text, sizeof(text), "%.1f", gps.altitude.meters());
  else strcpy(text, "--");
  drawField(display, ALT_FIELD, text);
  if (gps.hdop.isValid()) snprintf(text, sizeof(text), "%.1f", gps.hdop.hdop());
  else strcpy(text, "--");
  drawField(display, HDOP_FIELD, text);
  if (gps.speed.isValid()) snprintf(text, sizeof(text), "%.1f", gps.speed.kmph());
  else strcpy(text, "--");
  drawField(display, SPEED_FIELD, text);
  for (uint8_t system = 0; system < 5; ++system) {
    snprintf(text, sizeof(text), "%u", (unsigned)peakSnr[system]);
    drawField(display, SNR_FIELD + system, text);
  }
  drawField(display, ACK_FIELD, ackStatus);
}

void gpsEnd()
{
  if (running) {
    if (ackPending) {
      ackPending = false;
      ackStatus = "ABORT";
      ackLength = 0;
      Serial.println("GNSS reset ACK: ABORT; GPS session closed");
    }
    finishDiagnostic();
    Serial1.end();
  }
  running = false;
  digitalWrite(PIN_GPS_EN, !GPS_EN_ACTIVE);
  pinMode(PIN_GPS_EN, OUTPUT);
}
