#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <math.h>

// ============================================================
// Hardware
// ============================================================

static const uint8_t IMU_EN_PIN = PD5;

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

LSM6DS3 myIMU(I2C_MODE, 0x6A);

static bool imuReady = false;

// ============================================================
// Sampling configuration
// ============================================================

static const uint32_t SAMPLE_HZ = 200;
static const uint32_t SAMPLE_INTERVAL_US =
    1000000UL / SAMPLE_HZ;  // 5000 us

static const uint32_t BLE_POLL_INTERVAL_MS = 20;

// Inverted-stop detection
static const float INVERTED_AZ_THRESHOLD_G = -0.55f;
static const float STILL_GYRO_THRESHOLD_DPS = 80.0f;
static const uint32_t INVERTED_STABLE_MS = 700;
static const uint32_t INVERT_DETECT_GUARD_MS = 1500;

// ============================================================
// Status
// ============================================================

static const uint8_t STATUS_ERROR      = 0;
static const uint8_t STATUS_LOGGING    = 1;
static const uint8_t STATUS_DUMP_READY = 2;
static const uint8_t STATUS_DUMPING    = 3;

// ============================================================
// BLE commands
// ============================================================

static const uint8_t CMD_LOG_REQUEST = 1;
static const uint8_t CMD_LOG_GET     = 2;
static const uint8_t CMD_LOG_ABORT   = 3;
static const uint8_t CMD_CLEAR       = 4;
static const uint8_t CMD_RESTART_LOG = 5;

// ============================================================
// IMU log
// ============================================================

// Gyro: dps x 10
// Acceleration: g x 1000
struct ImuLog {
  uint32_t tUs;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

// 16 bytes/sample. 2000 samples = about 32 KB and 10 seconds at 200 Hz.
static const uint16_t IMU_LOG_CAP = 2000;
static ImuLog imuBuf[IMU_LOG_CAP];

static uint16_t imuStart = 0;
static uint16_t imuCount = 0;
static uint16_t imuOverflowCount = 0;

static bool isLogging = false;
static bool dumpActive = false;
static bool dumpMetaPending = false;

static uint32_t logStartUs = 0;
static uint32_t lastSampleUs = 0;
static uint32_t lastBlePollMs = 0;
static uint32_t invertedSinceMs = 0;

static bool bleReady = false;
static bool bleConnected = false;
static uint8_t statusValue = STATUS_ERROR;

// ============================================================
// BLE Service
// ============================================================

BLEService imuService(
  "12345678-1234-5678-1234-56789abcdef0"
);

// PC -> MG24
BLECharacteristic commandChar(
  "12345678-1234-5678-1234-56789abcdef4",
  BLERead | BLEWrite,
  8
);

// MG24 -> PC
BLEByteCharacteristic statusChar(
  "12345678-1234-5678-1234-56789abcdef5",
  BLERead | BLENotify
);

// MG24 -> PC
BLECharacteristic bulkChar(
  "12345678-1234-5678-1234-56789abcdefc",
  BLERead | BLENotify,
  32
);

// ============================================================
// Utilities
// ============================================================

static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] |
         ((uint16_t)p[1] << 8);
}

static void writeU16LE(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)(value >> 8);
}

static void writeU32LE(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
  p[2] = (uint8_t)((value >> 16) & 0xFF);
  p[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void writeI16LE(uint8_t *p, int16_t value) {
  writeU16LE(p, (uint16_t)value);
}

static int16_t floatToI16(float value, float scale) {
  float scaled = value * scale;

  if (!isfinite(scaled)) return 0;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;

  return (int16_t)lroundf(scaled);
}

static void ledSet(bool on) {
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

static void setStatus(uint8_t newStatus) {
  statusValue = newStatus;

  if (bleReady) {
    statusChar.writeValue(statusValue);
  }
}

// ============================================================
// Log control
// ============================================================

static void imuLogReset() {
  imuStart = 0;
  imuCount = 0;
  imuOverflowCount = 0;
  dumpActive = false;
  dumpMetaPending = false;
  invertedSinceMs = 0;
}

static void startLogging() {
  imuLogReset();
  logStartUs = micros();
  lastSampleUs = logStartUs;
  isLogging = true;
  setStatus(STATUS_LOGGING);
}

static void stopLoggingAndPrepareDump() {
  if (!isLogging) return;

  isLogging = false;
  dumpActive = true;
  dumpMetaPending = true;
  setStatus(STATUS_DUMP_READY);
}

// ============================================================
// IMU acquisition
// ============================================================

static void appendImuSample(
    uint32_t nowUs,
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az) {

  if (!isLogging) return;

  if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) ||
      !isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
    return;
  }

  uint16_t index;

  if (imuCount < IMU_LOG_CAP) {
    index = (uint16_t)((imuStart + imuCount) % IMU_LOG_CAP);
    imuCount++;
  } else {
    index = imuStart;
    imuStart = (uint16_t)((imuStart + 1) % IMU_LOG_CAP);

    if (imuOverflowCount < 0xFFFF) {
      imuOverflowCount++;
    }
  }

  ImuLog &row = imuBuf[index];

  row.tUs = nowUs - logStartUs;
  row.gx = floatToI16(gx, 10.0f);
  row.gy = floatToI16(gy, 10.0f);
  row.gz = floatToI16(gz, 10.0f);
  row.ax = floatToI16(ax, 1000.0f);
  row.ay = floatToI16(ay, 1000.0f);
  row.az = floatToI16(az, 1000.0f);
}

static void sampleImu(uint32_t nowUs) {
  if (!imuReady || !isLogging) return;

  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();

  appendImuSample(nowUs, gx, gy, gz, ax, ay, az);

  uint32_t elapsedFromStartUs = nowUs - logStartUs;
  bool guardElapsed =
      elapsedFromStartUs >= (INVERT_DETECT_GUARD_MS * 1000UL);

  float gyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);
  bool inverted = az <= INVERTED_AZ_THRESHOLD_G;
  bool nearlyStill = gyroMagnitude <= STILL_GYRO_THRESHOLD_DPS;
  uint32_t nowMs = millis();

  if (guardElapsed && inverted && nearlyStill) {
    if (invertedSinceMs == 0) {
      invertedSinceMs = nowMs;
    }

    if ((uint32_t)(nowMs - invertedSinceMs) >= INVERTED_STABLE_MS) {
      stopLoggingAndPrepareDump();
    }
  } else {
    invertedSinceMs = 0;
  }
}

// ============================================================
// BLE dump
// ============================================================

static void sendDumpMeta() {
  if (!bleReady || isLogging) return;

  uint8_t packet[32] = {0};

  writeU16LE(&packet[0], 0xFFFE);
  packet[2] = 0;
  writeU16LE(&packet[3], imuCount);
  writeU16LE(&packet[5], imuOverflowCount);
  packet[7] = 1;
  packet[8] = 16;
  packet[9] = 1;
  writeU16LE(&packet[10], IMU_LOG_CAP);
  writeU32LE(&packet[12], SAMPLE_HZ);

  bulkChar.writeValue(packet, 16);
  dumpMetaPending = false;
}

static void sendDumpTerminator() {
  if (!bleReady) return;

  uint8_t packet[7] = {0};

  writeU16LE(&packet[0], 0xFFFF);
  packet[2] = 0;
  writeU16LE(&packet[3], imuCount);
  writeU16LE(&packet[5], imuOverflowCount);

  bulkChar.writeValue(packet, sizeof(packet));

  dumpActive = false;
  setStatus(STATUS_DUMP_READY);
}

static bool getImuLogBySeq(uint16_t sequence, ImuLog &out) {
  if (sequence >= imuCount) return false;

  uint16_t index =
      (uint16_t)((imuStart + sequence) % IMU_LOG_CAP);

  out = imuBuf[index];
  return true;
}

static void sendDumpSequence(uint16_t sequence) {
  if (!dumpActive || !bleReady || isLogging) {
    return;
  }

  if (sequence >= imuCount) {
    sendDumpTerminator();
    return;
  }

  ImuLog row;

  if (!getImuLogBySeq(sequence, row)) {
    sendDumpTerminator();
    return;
  }

  uint8_t packet[19] = {0};

  writeU16LE(&packet[0], sequence);
  packet[2] = 1;
  writeU32LE(&packet[3], row.tUs);
  writeI16LE(&packet[7],  row.gx);
  writeI16LE(&packet[9],  row.gy);
  writeI16LE(&packet[11], row.gz);
  writeI16LE(&packet[13], row.ax);
  writeI16LE(&packet[15], row.ay);
  writeI16LE(&packet[17], row.az);

  bulkChar.writeValue(packet, sizeof(packet));
  setStatus(STATUS_DUMPING);
}

static void dumpAbort() {
  dumpActive = false;
  dumpMetaPending = false;
  setStatus(STATUS_DUMP_READY);
}

// ============================================================
// BLE callbacks
// ============================================================

static void onConnected(BLEDevice central) {
  (void)central;

  bleConnected = true;
  statusChar.writeValue(statusValue);

  if (!isLogging && imuCount > 0) {
    dumpActive = true;
    dumpMetaPending = true;
  }
}

static void onDisconnected(BLEDevice central) {
  (void)central;
  bleConnected = false;
}

static void onCommandWritten(
    BLEDevice central,
    BLECharacteristic characteristic) {

  (void)central;
  (void)characteristic;

  uint8_t buffer[8] = {0};
  int length = commandChar.valueLength();

  if (length < 1) return;

  if (length > (int)sizeof(buffer)) {
    length = sizeof(buffer);
  }

  commandChar.readValue(buffer, length);

  uint8_t command = buffer[0];

  switch (command) {
    case CMD_LOG_REQUEST:
      if (!isLogging && imuCount > 0) {
        dumpActive = true;
        dumpMetaPending = false;
        sendDumpMeta();
      }
      break;

    case CMD_LOG_GET:
      if (!isLogging && length >= 3) {
        uint16_t sequence = readU16LE(&buffer[1]);
        sendDumpSequence(sequence);
      }
      break;

    case CMD_LOG_ABORT:
      dumpAbort();
      break;

    case CMD_CLEAR:
      if (!isLogging) {
        imuLogReset();
        setStatus(STATUS_DUMP_READY);
      }
      break;

    case CMD_RESTART_LOG:
      startLogging();
      break;

    default:
      break;
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledSet(false);

  pinMode(IMU_EN_PIN, OUTPUT);
  digitalWrite(IMU_EN_PIN, HIGH);

  delay(30);

  Wire.begin();

  imuReady = (myIMU.begin() == 0);

  if (!imuReady) {
    setStatus(STATUS_ERROR);
    return;
  }

  delay(80);

  bool bleOk = BLE.begin();

  if (!bleOk) {
    delay(100);
    bleOk = BLE.begin();
  }

  if (!bleOk) {
    setStatus(STATUS_ERROR);
    return;
  }

  bleReady = true;

  BLE.setLocalName("Mini4AI-IMU");
  BLE.setDeviceName("Mini4AI-IMU");

  imuService.addCharacteristic(commandChar);
  imuService.addCharacteristic(statusChar);
  imuService.addCharacteristic(bulkChar);

  BLE.addService(imuService);
  BLE.setAdvertisedService(imuService);

  commandChar.setEventHandler(BLEWritten, onCommandWritten);
  BLE.setEventHandler(BLEConnected, onConnected);
  BLE.setEventHandler(BLEDisconnected, onDisconnected);

  commandChar.writeValue((uint8_t)0);
  statusChar.writeValue(STATUS_LOGGING);

  uint8_t emptyBulk[3] = {0xFF, 0xFF, 0};
  bulkChar.writeValue(emptyBulk, sizeof(emptyBulk));

  BLE.advertise();

  lastBlePollMs = millis();

  // Start recording immediately after power-on.
  startLogging();
}

// ============================================================
// Loop
// ============================================================

void loop() {
  uint32_t nowUs = micros();

  if (isLogging &&
      (uint32_t)(nowUs - lastSampleUs) >= SAMPLE_INTERVAL_US) {

    uint32_t lagUs = (uint32_t)(nowUs - lastSampleUs);

    if (lagUs > SAMPLE_INTERVAL_US * 10UL) {
      lastSampleUs = nowUs;
    } else {
      lastSampleUs += SAMPLE_INTERVAL_US;
    }

    sampleImu(nowUs);
  }

  uint32_t nowMs = millis();

  if (bleReady &&
      (uint32_t)(nowMs - lastBlePollMs) >= BLE_POLL_INTERVAL_MS) {

    lastBlePollMs = nowMs;
    BLE.poll();

    if (bleConnected && dumpMetaPending) {
      sendDumpMeta();
    }
  }

  if (statusValue == STATUS_LOGGING) {
    ledSet((millis() / 100) % 2 == 0);
  } else if (statusValue == STATUS_DUMPING) {
    ledSet((millis() / 250) % 2 == 0);
  } else if (statusValue == STATUS_DUMP_READY) {
    ledSet((millis() / 600) % 2 == 0);
  } else {
    ledSet(true);
  }
}
