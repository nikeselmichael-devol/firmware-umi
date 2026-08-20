/*
 * ESP32-C3 Super Mini — Dual AS5047P Encoder + ICM42688P IMU
 * Reads sensors via shared SPI bus, streams JSON over TCP.
 *
 * Wiring (from schematic):
 *   Shared SPI bus:
 *     SCLK  -> GPIO4
 *     MOSI  -> GPIO6
 *     MISO  -> GPIO5
 *   Chip selects:
 *     CS1   -> GPIO10   -> ICM42688P (IMU)
 *     CS2   -> GPIO2    -> AS5047P #1 (Encoder A)
 *     CS3   -> GPIO7    -> AS5047P #2 (Encoder B)
 *
 * Behavior:
 *   - Sensors are polled at POLL_RATE_HZ (default 100 Hz, valid range 30-100 Hz)
 *   - Latest sample is published to all connected TCP clients at PUBLISH_RATE_HZ (100 Hz)
 *   - One JSON object per line (newline-delimited), e.g.:
 *     {"t":123456,"enc1_deg":123.45,"enc2_deg":67.89,
 *      "ax":0.01,"ay":0.02,"az":9.81,"gx":0.1,"gy":-0.2,"gz":0.05}
 *
 * Board: "ESP32C3 Dev Module" (arduino-esp32 core)
 */

#include <WiFi.h>
#include <SPI.h>

// ---------------- USER CONFIG ----------------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const uint16_t TCP_PORT   = 5000;

#define POLL_RATE_HZ     100     // 30-100 Hz valid. Sensor sampling rate.
#define PUBLISH_RATE_HZ  100     // TCP publish rate.
// -----------------------------------------------

// ---------------- PIN DEFINITIONS ----------------
#define PIN_SCLK   4
#define PIN_MOSI   6
#define PIN_MISO   5
#define CS_IMU     10   // CS1 -> ICM42688P
#define CS_ENC1    2    // CS2 -> AS5047P #1
#define CS_ENC2    7    // CS3 -> AS5047P #2
// ---------------------------------------------------

// ---------------- SPI SETTINGS ----------------
// AS5047P: mode 1, MSB first, up to 10 MHz (use 1 MHz for reliability over wires)
SPISettings spiEnc(1000000, MSBFIRST, SPI_MODE1);
// ICM42688P: mode 0 or 3, MSB first, up to 24 MHz (use 4 MHz to be safe)
SPISettings spiImu(4000000, MSBFIRST, SPI_MODE0);

// ---------------- ICM42688P REGISTERS ----------------
#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_ACCEL_DATA_X1   0x1F  // 6 bytes accel, then 6 bytes gyro follow (0x1F..0x2A)
#define ICM_WHOAMI_EXPECTED     0x47

// Sensitivity for default full-scale ranges we configure below:
//   Accel: +/-8g  -> 4096 LSB/g
//   Gyro:  +/-2000 dps -> 16.4 LSB/(deg/s)
const float ACCEL_SENS_LSB_PER_G   = 4096.0f;
const float GYRO_SENS_LSB_PER_DPS  = 16.4f;
const float GRAVITY_MPS2           = 9.80665f;

// ---------------- Shared sample struct ----------------
struct SensorSample {
  uint32_t t_ms;
  float enc1_deg;
  float enc2_deg;
  bool  enc1_err;
  bool  enc2_err;
  float ax, ay, az;   // m/s^2
  float gx, gy, gz;   // deg/s
};

volatile SensorSample g_latest;
portMUX_TYPE g_sampleMux = portMUX_INITIALIZER_UNLOCKED;

WiFiServer server(TCP_PORT);
WiFiClient clients[4];

// ============================================================
// AS5047P helpers
// ============================================================

// Even parity over lower 15 bits, placed into bit15
uint16_t as5047_addParity(uint16_t frame) {
  uint16_t p = frame & 0x7FFF;
  p ^= p >> 8;
  p ^= p >> 4;
  p ^= p >> 2;
  p ^= p >> 1;
  bool parityBit = p & 0x1; // odd number of 1s -> 1
  if (parityBit) frame |= 0x8000;
  return frame;
}

uint16_t as5047_transfer(uint8_t csPin, uint16_t command) {
  SPI.beginTransaction(spiEnc);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  uint16_t response = SPI.transfer16(command);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return response;
}

// Reads ANGLECOM register (0x3FFF), returns raw 14-bit angle.
// err flag set if the EF (error flag, bit14) of the response is set.
uint16_t as5047_readAngle(uint8_t csPin, bool &err) {
  uint16_t cmd = as5047_addParity(0x4000 | 0x3FFF); // read bit + address
  as5047_transfer(csPin, cmd);           // send command frame
  uint16_t resp = as5047_transfer(csPin, as5047_addParity(0x4000 | 0x3FFF)); // clock out data (send NOP/read again)
  err = (resp & 0x4000) != 0;            // EF bit
  return resp & 0x3FFF;                  // 14-bit angle data
}

float as5047_angleToDeg(uint16_t raw14) {
  return (raw14 * 360.0f) / 16384.0f;
}

// ============================================================
// ICM42688P helpers
// ============================================================

uint8_t icm_readReg(uint8_t reg) {
  SPI.beginTransaction(spiImu);
  digitalWrite(CS_IMU, LOW);
  SPI.transfer(reg | 0x80); // read bit
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_IMU, HIGH);
  SPI.endTransaction();
  return val;
}

void icm_writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(spiImu);
  digitalWrite(CS_IMU, LOW);
  SPI.transfer(reg & 0x7F); // write bit
  SPI.transfer(val);
  digitalWrite(CS_IMU, HIGH);
  SPI.endTransaction();
}

void icm_readBurst(uint8_t startReg, uint8_t* buf, uint8_t len) {
  SPI.beginTransaction(spiImu);
  digitalWrite(CS_IMU, LOW);
  SPI.transfer(startReg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_IMU, HIGH);
  SPI.endTransaction();
}

bool icm_init() {
  delay(50);
  uint8_t who = icm_readReg(ICM_REG_WHO_AM_I);
  if (who != ICM_WHOAMI_EXPECTED) {
    Serial.printf("ICM42688P WHO_AM_I mismatch: got 0x%02X, expected 0x%02X\n", who, ICM_WHOAMI_EXPECTED);
    return false;
  }

  // Enable accel + gyro in low-noise mode
  icm_writeReg(ICM_REG_PWR_MGMT0, 0x0F); // GYRO_MODE=LN(11), ACCEL_MODE=LN(11)
  delay(1);

  // ACCEL_CONFIG0: FS_SEL=8g (001), ODR=1kHz (0110 -> value bits) -- see datasheet table
  // Bits: [7:5] ACCEL_FS_SEL, [3:0] ACCEL_ODR
  // FS_SEL=001 (8g), ODR=0110 (1kHz)
  icm_writeReg(ICM_REG_ACCEL_CONFIG0, (0b001 << 5) | 0b0110);

  // GYRO_CONFIG0: FS_SEL=2000dps (000), ODR=1kHz (0110)
  icm_writeReg(ICM_REG_GYRO_CONFIG0, (0b000 << 5) | 0b0110);

  delay(10);
  return true;
}

void icm_readAccelGyro(float &ax, float &ay, float &az,
                        float &gx, float &gy, float &gz) {
  uint8_t raw[12];
  icm_readBurst(ICM_REG_ACCEL_DATA_X1, raw, 12);

  int16_t axr = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ayr = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t azr = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t gxr = (int16_t)((raw[6] << 8) | raw[7]);
  int16_t gyr = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gzr = (int16_t)((raw[10] << 8) | raw[11]);

  ax = (axr / ACCEL_SENS_LSB_PER_G) * GRAVITY_MPS2;
  ay = (ayr / ACCEL_SENS_LSB_PER_G) * GRAVITY_MPS2;
  az = (azr / ACCEL_SENS_LSB_PER_G) * GRAVITY_MPS2;

  gx = gxr / GYRO_SENS_LSB_PER_DPS;
  gy = gyr / GYRO_SENS_LSB_PER_DPS;
  gz = gzr / GYRO_SENS_LSB_PER_DPS;
}

// ============================================================
// WiFi / TCP
// ============================================================

void connectWiFi() {
  Serial.printf("Connecting to WiFi SSID '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nWiFi connect timed out, retrying...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      start = millis();
    }
  }
  Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

void acceptNewClients() {
  if (server.hasClient()) {
    bool placed = false;
    for (int i = 0; i < 4; i++) {
      if (!clients[i] || !clients[i].connected()) {
        if (clients[i]) clients[i].stop();
        clients[i] = server.available();
        Serial.printf("Client connected on slot %d: %s\n", i, clients[i].remoteIP().toString().c_str());
        placed = true;
        break;
      }
    }
    if (!placed) {
      WiFiClient rejected = server.available();
      rejected.stop(); // no free slots
    }
  }
}

void broadcastSample(const SensorSample &s) {
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"t\":%lu,"
    "\"enc1_deg\":%.3f,\"enc1_err\":%s,"
    "\"enc2_deg\":%.3f,\"enc2_err\":%s,"
    "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
    "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f}\n",
    (unsigned long)s.t_ms,
    s.enc1_deg, s.enc1_err ? "true" : "false",
    s.enc2_deg, s.enc2_err ? "true" : "false",
    s.ax, s.ay, s.az,
    s.gx, s.gy, s.gz
  );
  if (n <= 0) return;

  for (int i = 0; i < 4; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].write((const uint8_t*)buf, n);
    }
  }
}

// ============================================================
// Timing
// ============================================================
const uint32_t POLL_PERIOD_MS    = 1000 / POLL_RATE_HZ;
const uint32_t PUBLISH_PERIOD_MS = 1000 / PUBLISH_RATE_HZ;

uint32_t lastPollMs    = 0;
uint32_t lastPublishMs = 0;

void pollSensors() {
  bool e1err, e2err;
  uint16_t raw1 = as5047_readAngle(CS_ENC1, e1err);
  uint16_t raw2 = as5047_readAngle(CS_ENC2, e2err);

  float ax, ay, az, gx, gy, gz;
  icm_readAccelGyro(ax, ay, az, gx, gy, gz);

  SensorSample s;
  s.t_ms     = millis();
  s.enc1_deg = as5047_angleToDeg(raw1);
  s.enc1_err = e1err;
  s.enc2_deg = as5047_angleToDeg(raw2);
  s.enc2_err = e2err;
  s.ax = ax; s.ay = ay; s.az = az;
  s.gx = gx; s.gy = gy; s.gz = gz;

  portENTER_CRITICAL(&g_sampleMux);
  g_latest = s;
  portEXIT_CRITICAL(&g_sampleMux);
}

SensorSample getLatestSample() {
  SensorSample copy;
  portENTER_CRITICAL(&g_sampleMux);
  copy = g_latest;
  portEXIT_CRITICAL(&g_sampleMux);
  return copy;
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(CS_IMU, OUTPUT);
  pinMode(CS_ENC1, OUTPUT);
  pinMode(CS_ENC2, OUTPUT);
  digitalWrite(CS_IMU, HIGH);
  digitalWrite(CS_ENC1, HIGH);
  digitalWrite(CS_ENC2, HIGH);

  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1); // no default CS, we drive manually

  if (!icm_init()) {
    Serial.println("WARNING: ICM42688P init failed. Check wiring/CS1=GPIO10.");
  } else {
    Serial.println("ICM42688P initialized OK.");
  }

  connectWiFi();
  server.begin();
  server.setNoDelay(true);
  Serial.printf("TCP server listening on port %u\n", TCP_PORT);

  // Prime the first sample immediately
  pollSensors();
  lastPollMs = lastPublishMs = millis();
}

void loop() {
  uint32_t now = millis();

  acceptNewClients();

  if (now - lastPollMs >= POLL_PERIOD_MS) {
    lastPollMs = now;
    pollSensors();
  }

  if (now - lastPublishMs >= PUBLISH_PERIOD_MS) {
    lastPublishMs = now;
    SensorSample s = getLatestSample();
    broadcastSample(s);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped, reconnecting...");
    connectWiFi();
  }
}
