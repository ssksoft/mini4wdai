#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <math.h>

// 走行関連
static const uint8_t PWM_PIN    = PC2;
static uint8_t  DUTY_50     = 128;
static uint8_t  DUTY_100     = 255;

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
static const uint32_t SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_HZ;
static const uint32_t BLE_POLL_INTERVAL_MS = 5;

// 40 seconds at 200 Hz.
static const uint16_t IMU_LOG_CAP = 8000;

// Automatic stop: inverted and almost still.
static const float INVERTED_AZ_THRESHOLD_G = -0.55f;
static const float STILL_GYRO_THRESHOLD_DPS = 80.0f;
static const uint32_t INVERTED_STABLE_MS = 700;
static const uint32_t INVERT_DETECT_GUARD_MS = 1500;

// Number of sample notifications attempted in one loop pass.
// Increase cautiously if BLE.poll() responsiveness becomes poor.
static const uint8_t STREAM_BURST = 4;

// ============================================================
// Status
// ============================================================
static const uint8_t STATUS_ERROR = 0;
static const uint8_t STATUS_LOGGING = 1;
static const uint8_t STATUS_DUMP_READY = 2;
static const uint8_t STATUS_DUMPING = 3;

// ============================================================
// BLE commands
// ============================================================
static const uint8_t CMD_LOG_REQUEST = 1;   // Send metadata
static const uint8_t CMD_LOG_GET = 2;       // Legacy: get one sample
static const uint8_t CMD_LOG_ABORT = 3;
static const uint8_t CMD_CLEAR = 4;
static const uint8_t CMD_RESTART_LOG = 5;
static const uint8_t CMD_STREAM_START = 6;  // New: stream all samples

// ============================================================
// IMU log
// ============================================================
// Gyro: dps x 10, acceleration: g x 1000.
struct ImuLog {
  uint32_t tUs;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

static ImuLog imuBuf[IMU_LOG_CAP];
static uint16_t imuStart = 0;
static uint16_t imuCount = 0;
static uint16_t imuOverflowCount = 0;
static bool isLogging = false;
static bool dumpActive = false;
static bool dumpMetaPending = false;
static bool streamActive = false;
static uint16_t streamSequence = 0;
static uint32_t logStartUs = 0;
static uint32_t lastSampleUs = 0;
static uint32_t lastBlePollMs = 0;
static uint32_t invertedSinceMs = 0;
static bool bleReady = false;
static bool bleConnected = false;
static uint8_t statusValue = STATUS_ERROR;

// ============================================================
// BLE service
// ============================================================
// IMUサービスを作成
BLEService imuService("12345678-1234-5678-1234-56789abcdef0");
// コマンド待受用窓口を作成
BLECharacteristic commandChar(
    "12345678-1234-5678-1234-56789abcdef4", BLERead | BLEWrite, 8);
// ステータス通知用窓口を作成
BLEByteCharacteristic statusChar(
    "12345678-1234-5678-1234-56789abcdef5", BLERead | BLENotify);
// 蓄積したIMUデータを送信するための窓口を作成
BLECharacteristic bulkChar(
    "12345678-1234-5678-1234-56789abcdefc", BLERead | BLENotify, 32);

// ============================================================
// Utilities
// ============================================================
static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
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
  if (bleReady) statusChar.writeValue(statusValue);
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
  streamActive = false;
  streamSequence = 0;
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
static void appendImuSample(uint32_t nowUs, float gx, float gy, float gz,
                            float ax, float ay, float az) {
  if (!isLogging) return;
  if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) ||
      !isfinite(ax) || !isfinite(ay) || !isfinite(az)) return;

  uint16_t index;
  if (imuCount < IMU_LOG_CAP) {
    index = (uint16_t)((imuStart + imuCount) % IMU_LOG_CAP);
    imuCount++;
  } else {
    index = imuStart;
    imuStart = (uint16_t)((imuStart + 1) % IMU_LOG_CAP);
    if (imuOverflowCount < 0xFFFF) imuOverflowCount++;
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

  bool guardElapsed = (nowUs - logStartUs) >= INVERT_DETECT_GUARD_MS * 1000UL;
  float gyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);
  bool inverted = az <= INVERTED_AZ_THRESHOLD_G;
  bool nearlyStill = gyroMagnitude <= STILL_GYRO_THRESHOLD_DPS;
  uint32_t nowMs = millis();

  if (guardElapsed && inverted && nearlyStill) {
    if (invertedSinceMs == 0) invertedSinceMs = nowMs;
    if ((uint32_t)(nowMs - invertedSinceMs) >= INVERTED_STABLE_MS)
      stopLoggingAndPrepareDump();
  } else {
    invertedSinceMs = 0;
  }
}

// ============================================================
// BLE dump
// ============================================================
static bool getImuLogBySeq(uint16_t sequence, ImuLog &out) {
  if (sequence >= imuCount) return false;
  uint16_t index = (uint16_t)((imuStart + sequence) % IMU_LOG_CAP);
  out = imuBuf[index];
  return true;
}

static void sendDumpMeta() {
  if (!bleReady || isLogging) return;
  uint8_t packet[16] = {0};
  writeU16LE(&packet[0], 0xFFFE);
  packet[2] = 0;
  writeU16LE(&packet[3], imuCount);
  writeU16LE(&packet[5], imuOverflowCount);
  packet[7] = 1;
  packet[8] = 16;
  packet[9] = 2; // Protocol version 2 supports streaming.
  writeU16LE(&packet[10], IMU_LOG_CAP);
  writeU32LE(&packet[12], SAMPLE_HZ);
  bulkChar.writeValue(packet, sizeof(packet));
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
  streamActive = false;
  setStatus(STATUS_DUMP_READY);
}

static bool sendDumpSequence(uint16_t sequence) {
  if (!dumpActive || !bleReady || isLogging || sequence >= imuCount) return false;
  ImuLog row;
  if (!getImuLogBySeq(sequence, row)) return false;

  uint8_t packet[19] = {0};
  writeU16LE(&packet[0], sequence);
  packet[2] = 1;
  writeU32LE(&packet[3], row.tUs);
  writeI16LE(&packet[7], row.gx);
  writeI16LE(&packet[9], row.gy);
  writeI16LE(&packet[11], row.gz);
  writeI16LE(&packet[13], row.ax);
  writeI16LE(&packet[15], row.ay);
  writeI16LE(&packet[17], row.az);
  bulkChar.writeValue(packet, sizeof(packet));
  setStatus(STATUS_DUMPING);
  return true;
}

static void serviceStream() {
  if (!streamActive || !bleConnected || isLogging) return;
  for (uint8_t i = 0; i < STREAM_BURST; i++) {
    if (streamSequence >= imuCount) {
      sendDumpTerminator();
      return;
    }
    if (!sendDumpSequence(streamSequence)) {
      streamActive = false;
      return;
    }
    streamSequence++;
    BLE.poll();
  }
}

static void dumpAbort() {
  dumpActive = false;
  dumpMetaPending = false;
  streamActive = false;
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
  streamActive = false;
}

static void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  (void)characteristic;
  uint8_t buffer[8] = {0};
  int length = commandChar.valueLength();
  if (length < 1) return;
  if (length > (int)sizeof(buffer)) length = sizeof(buffer);
  commandChar.readValue(buffer, length);

  switch (buffer[0]) {
    case CMD_LOG_REQUEST:
      if (!isLogging && imuCount > 0) {
        dumpActive = true;
        dumpMetaPending = false;
        sendDumpMeta();
      }
      break;
    case CMD_LOG_GET: { // Keeps compatibility with the old MATLAB script.
      if (!isLogging && length >= 3) {
        uint16_t sequence = readU16LE(&buffer[1]);
        if (!sendDumpSequence(sequence)) sendDumpTerminator();
      }
      break;
    }
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
    case CMD_STREAM_START:
      if (!isLogging && imuCount > 0) {
        // Optional bytes 1..2 specify the first sequence; default is zero.
        streamSequence = (length >= 3) ? readU16LE(&buffer[1]) : 0;
        dumpActive = true;      // 確実にtrueにする
        streamActive = true;    // 確実にtrueにする
        setStatus(STATUS_DUMPING);
      }
      break;
    default:
      break;
  }
}

// ============================================================
// Initialization
// ============================================================
static void initLed() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledSet(false);
}

// IMU電源供給ON → I2C通信開始
static bool initImu() {
  pinMode(IMU_EN_PIN, OUTPUT);
  digitalWrite(IMU_EN_PIN, HIGH);
  delay(30);

  Wire.begin();
  imuReady = (myIMU.begin() == 0);
  if (!imuReady) return false;
  delay(80);
  return true;
}

// BLE使用開始（失敗時は1回だけリトライ）
static bool beginBle() {
  if (BLE.begin()) return true;
  delay(100);
  return BLE.begin();
}

// IMUサービスと各窓口（コマンド受信・状態通知・bulk転送）を登録
static void registerImuService() {
  BLE.setLocalName("Mini4AI-IMU");
  BLE.setDeviceName("Mini4AI-IMU");

  imuService.addCharacteristic(commandChar);
  imuService.addCharacteristic(statusChar);
  imuService.addCharacteristic(bulkChar);
  BLE.addService(imuService);
  BLE.setAdvertisedService(imuService);
}

// コマンド受信・接続・切断時のコールバックを登録
static void registerBleHandlers() {
  commandChar.setEventHandler(BLEWritten, onCommandWritten);
  BLE.setEventHandler(BLEConnected, onConnected);
  BLE.setEventHandler(BLEDisconnected, onDisconnected);
}

// コマンド送受信用領域を初期化
static void initCharacteristicValues() {
  commandChar.writeValue((uint8_t)0);
  statusChar.writeValue(STATUS_LOGGING);
  uint8_t emptyBulk[3] = {0xFF, 0xFF, 0};
  bulkChar.writeValue(emptyBulk, sizeof(emptyBulk));
}

static bool initBle() {
  if (!beginBle()) return false;
  bleReady = true;

  registerImuService();
  registerBleHandlers();
  initCharacteristicValues();

  // BLE電波発信開始
  BLE.advertise();
  lastBlePollMs = millis();
  return true;
}

static void initMotor() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 0);
}

// ============================================================
// Periodic tasks
// ============================================================
// 非同期IMUロギング（周期：SAMPLE_INTERVAL_US[μs]）
static void serviceImuSampling() {
  uint32_t nowUs = micros();
  if (!isLogging || (uint32_t)(nowUs - lastSampleUs) < SAMPLE_INTERVAL_US) return;

  // 大きく遅延した場合は追いつこうとせず基準時刻をリセット
  uint32_t lagUs = (uint32_t)(nowUs - lastSampleUs);
  lastSampleUs = (lagUs > SAMPLE_INTERVAL_US * 10UL)
                     ? nowUs
                     : lastSampleUs + SAMPLE_INTERVAL_US;
  sampleImu(nowUs);
}

// 非同期BLE通信（周期：BLE_POLL_INTERVAL_MS[ms]）
static void serviceBle() {
  uint32_t nowMs = millis();
  if (!bleReady || (uint32_t)(nowMs - lastBlePollMs) < BLE_POLL_INTERVAL_MS) return;
  lastBlePollMs = nowMs;

  BLE.poll();
  // BLE接続中かつ未送信のダンプメタデータがあればsendDumpMetaで送信
  if (bleConnected && dumpMetaPending) sendDumpMeta();
  serviceStream();
}

static void updateStatusLed() {
  uint32_t nowMs = millis();
  switch (statusValue) {
    case STATUS_LOGGING:    ledSet((nowMs / 100) % 2 == 0); break; // ロギング中：100ms周期
    case STATUS_DUMPING:    ledSet((nowMs / 250) % 2 == 0); break; // データ転送中：250ms周期
    case STATUS_DUMP_READY: ledSet((nowMs / 600) % 2 == 0); break; // 転送準備完了：600ms周期
    default:                ledSet(true); break;
  }
}

// ロギング中のみ走行、ログ終了時は走行停止
static void updateMotor() {
  analogWrite(PWM_PIN, isLogging ? DUTY_50 : 0);
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  initLed();
  if (!initImu() || !initBle()) {
    setStatus(STATUS_ERROR);
    return;
  }
  startLogging();
  initMotor();

  // 電源投入から3秒後に走行開始
  delay(3000);
}

void loop() {
  serviceImuSampling();
  serviceBle();
  updateStatusLed();
  updateMotor();
}
