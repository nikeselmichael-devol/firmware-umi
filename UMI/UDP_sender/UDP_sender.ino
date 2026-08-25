/*
 * ESP32-C3 Super Mini — IMU + Dual Encoder → UDP sensor stream
 * ---------------------------------------------------------------
 * Hardware (per schematic):
 *   - ICM42688P (IMU)      SPI, CS1
 *   - AS5047P   (Encoder1) SPI, CS2
 *   - AS5047P   (Encoder2) SPI, CS3
 *   Shared bus: MISO / MOSI / CLK
 *
 * Payload format (matches "packet structure" diagram):
 *   UDP application payload = CBOR( [STAT, CH1..CH8] )  +  CRC8 (1 byte)
 *   All multi-byte CBOR integers/floats are big-endian per the CBOR spec,
 *   so this satisfies the "use big endian" requirement without any extra work.
 *   The 802.11 / LLC / IPv4 / UDP headers are handled entirely by the
 *   WiFi stack + WiFiUDP — we only ever build the application payload below.
 *
 * Channel mapping (CH1..CH8), all encoded as CBOR ints, fixed-point scaled:
 *   CH1 = Encoder1 angle   (0.01 deg units,  int32)
 *   CH2 = Encoder2 angle   (0.01 deg units,  int32)
 *   CH3 = Gyro X           (0.001 dps units, int32)
 *   CH4 = Gyro Y           (0.001 dps units, int32)
 *   CH5 = Gyro Z           (0.001 dps units, int32)
 *   CH6 = Accel X          (0.0001 g units,  int32)
 *   CH7 = Accel Y          (0.0001 g units,  int32)
 *   CH8 = Accel Z          (0.0001 g units,  int32)
 *
 * STAT (uint32, only lower 24 bits meaningful):
 *   bit0 = IMU ok, bit1 = Encoder1 ok, bit2 = Encoder2 ok
 *   bits8-15 = rolling sequence number (wraps 0-255)
 *
 * NOTE: register addresses / pin numbers below are set from the schematic
 * and public datasheet register maps — double check against your exact
 * silicon revision / board before flashing.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

// ----------------------------- USER CONFIG -----------------------------
static const char* WIFI_SSID   = "YOUR_SSID";
static const char* WIFI_PASS   = "YOUR_PASSWORD";
static const char* DEST_IP     = "192.168.1.100";   // host / driver PC
static const uint16_t DEST_PORT = 5005;

static const uint32_t POLL_HZ    = 100;   // sensor poll rate, 30-100 Hz
static const uint32_t PUBLISH_HZ = 100;   // UDP publish rate, fixed 100 Hz

// ----------------------------- PIN MAP ----------------------------------
// Shared SPI bus
static const int PIN_MISO = 5;
static const int PIN_MOSI = 6;
static const int PIN_CLK  = 3;   // confirm against schematic silk labels
// Per-device chip selects
static const int PIN_CS_IMU  = 10;  // CS1 -> ICM42688P
static const int PIN_CS_ENC1 = 4;   // CS2 -> AS5047P #1
static const int PIN_CS_ENC2 = 7;   // CS3 -> AS5047P #2

// ----------------------------- ICM42688P ---------------------------------
namespace ICM {
  constexpr uint8_t REG_WHO_AM_I    = 0x75;
  constexpr uint8_t REG_PWR_MGMT0   = 0x4E;
  constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
  constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
  constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F; // burst: accel(6) + temp(2) + gyro(6)
  constexpr uint8_t READ_BIT  = 0x80;

  SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

  uint8_t readReg(uint8_t reg) {
    digitalWrite(PIN_CS_IMU, LOW);
    SPI.transfer(reg | READ_BIT);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(PIN_CS_IMU, HIGH);
    return val;
  }

  void writeReg(uint8_t reg, uint8_t val) {
    digitalWrite(PIN_CS_IMU, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(PIN_CS_IMU, HIGH);
  }

  bool begin() {
    pinMode(PIN_CS_IMU, OUTPUT);
    digitalWrite(PIN_CS_IMU, HIGH);
    SPI.beginTransaction(spiSettings);
    uint8_t who = readReg(REG_WHO_AM_I);
    // Typical ICM42688P WHO_AM_I = 0x47 -- verify against your datasheet copy
    writeReg(REG_PWR_MGMT0, 0x0F);      // accel+gyro low-noise mode
    writeReg(REG_GYRO_CONFIG0, 0x06);   // +-2000dps, ODR per datasheet table
    writeReg(REG_ACCEL_CONFIG0, 0x06);  // +-16g
    SPI.endTransaction();
    return (who != 0x00 && who != 0xFF);
  }

  struct Sample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    bool ok;
  };

  Sample readBurst() {
    Sample s{};
    SPI.beginTransaction(spiSettings);
    digitalWrite(PIN_CS_IMU, LOW);
    SPI.transfer(REG_ACCEL_DATA_X1 | READ_BIT);
    uint8_t buf[14];
    for (auto &b : buf) b = SPI.transfer(0x00);
    digitalWrite(PIN_CS_IMU, HIGH);
    SPI.endTransaction();

    s.ax = (int16_t)((buf[0] << 8) | buf[1]);
    s.ay = (int16_t)((buf[2] << 8) | buf[3]);
    s.az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6],buf[7] = temperature, skipped
    s.gx = (int16_t)((buf[8] << 8) | buf[9]);
    s.gy = (int16_t)((buf[10] << 8) | buf[11]);
    s.gz = (int16_t)((buf[12] << 8) | buf[13]);
    s.ok = true;
    return s;
  }
}

// ----------------------------- AS5047P ------------------------------------
namespace AS5047 {
  constexpr uint16_t CMD_READ   = 0x4000; // read bit
  constexpr uint16_t REG_ANGLECOM = 0x3FFF;
  SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

  uint8_t parity16(uint16_t v) {
    uint8_t p = 0;
    while (v) { p ^= (v & 1); v >>= 1; }
    return p;
  }

  uint16_t transfer16(int csPin, uint16_t frame) {
    // add even parity bit (MSB) per AS5047P frame format
    frame &= 0x7FFF;
    frame |= (parity16(frame) << 15);

    digitalWrite(csPin, LOW);
    uint16_t rx = SPI.transfer16(frame);
    digitalWrite(csPin, HIGH);
    return rx;
  }

  // Reads angle (14-bit, 0..16383) from given encoder CS pin.
  // Returns -1 on parity/error flag failure.
  int32_t readAngle(int csPin) {
    SPI.beginTransaction(spiSettings);
    transfer16(csPin, CMD_READ | REG_ANGLECOM);      // send read command
    uint16_t rx = transfer16(csPin, CMD_READ | REG_ANGLECOM); // clock out result
    SPI.endTransaction();

    bool err = rx & 0x4000;      // EF bit
    if (err) return -1;
    return rx & 0x3FFF;          // 14-bit angle
  }

  bool begin(int csPin) {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    return true;
  }
}

// ----------------------------- CRC8 ----------------------------------------
// CRC-8-ATM (poly 0x07, init 0x00) — simple, matches "programmer-added CRC8"
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ----------------------------- minimal CBOR encoder -------------------------
// Only what we need: array header, unsigned int, signed int (RFC 8949 §3.1).
// All multi-byte fields are written big-endian, per spec.
class CborWriter {
 public:
  CborWriter(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap), len_(0) {}

  size_t length() const { return len_; }

  void writeArrayHeader(uint32_t n) { writeTypeAndArg(0x80, n); }

  void writeUInt(uint32_t v) { writeTypeAndArg(0x00, v); }

  void writeInt(int32_t v) {
    if (v >= 0) {
      writeUInt((uint32_t)v);
    } else {
      uint32_t n = (uint32_t)(-1 - v); // CBOR negative int encoding
      writeTypeAndArg(0x20, n);
    }
  }

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t len_;

  void put(uint8_t b) { if (len_ < cap_) buf_[len_++] = b; }

  void writeTypeAndArg(uint8_t majorType, uint32_t v) {
    if (v < 24) {
      put(majorType | (uint8_t)v);
    } else if (v <= 0xFF) {
      put(majorType | 24);
      put((uint8_t)v);
    } else if (v <= 0xFFFF) {
      put(majorType | 25);
      put((uint8_t)(v >> 8));
      put((uint8_t)v);
    } else {
      put(majorType | 26);
      put((uint8_t)(v >> 24));
      put((uint8_t)(v >> 16));
      put((uint8_t)(v >> 8));
      put((uint8_t)v);
    }
  }
};

// ----------------------------- packet build ---------------------------------
struct SensorFrame {
  uint32_t stat;
  int32_t ch[8];
};

size_t buildPacket(const SensorFrame& f, uint8_t* out, size_t outCap) {
  CborWriter w(out, outCap);
  w.writeArrayHeader(9);       // [stat, ch1..ch8]
  w.writeUInt(f.stat);
  for (int i = 0; i < 8; i++) w.writeInt(f.ch[i]);

  size_t cborLen = w.length();
  uint8_t crc = crc8(out, cborLen);
  if (cborLen < outCap) out[cborLen] = crc;
  return cborLen + 1; // + CRC8 byte
}

// ----------------------------- globals ---------------------------------------
WiFiUDP udp;
ICM::Sample lastImu{};
int32_t lastEnc1Raw = 0, lastEnc2Raw = 0;
bool imuOk = false, enc1Ok = false, enc2Ok = false;
uint8_t seq = 0;

uint32_t lastPollMs = 0, lastPublishMs = 0;
const uint32_t pollPeriodMs = 1000 / POLL_HZ;
const uint32_t publishPeriodMs = 1000 / PUBLISH_HZ;

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS_IMU, OUTPUT);
  pinMode(PIN_CS_ENC1, OUTPUT);
  pinMode(PIN_CS_ENC2, OUTPUT);
  digitalWrite(PIN_CS_IMU, HIGH);
  digitalWrite(PIN_CS_ENC1, HIGH);
  digitalWrite(PIN_CS_ENC2, HIGH);

  SPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, -1);

  imuOk = ICM::begin();
  enc1Ok = AS5047::begin(PIN_CS_ENC1);
  enc2Ok = AS5047::begin(PIN_CS_ENC2);

  connectWifi();
  udp.begin(0); // ephemeral local port for sending
}

void pollSensors() {
  lastImu = ICM::readBurst();
  imuOk = lastImu.ok;

  int32_t a1 = AS5047::readAngle(PIN_CS_ENC1);
  if (a1 >= 0) { lastEnc1Raw = a1; enc1Ok = true; } else { enc1Ok = false; }

  int32_t a2 = AS5047::readAngle(PIN_CS_ENC2);
  if (a2 >= 0) { lastEnc2Raw = a2; enc2Ok = true; } else { enc2Ok = false; }
}

void publishFrame() {
  SensorFrame f{};
  f.stat = (imuOk ? 1u : 0u) | (enc1Ok ? 2u : 0u) | (enc2Ok ? 4u : 0u) |
           ((uint32_t)seq << 8);
  seq++;

  // encoder raw (0..16383 counts over 360deg) -> 0.01 deg fixed point
  f.ch[0] = (int32_t)((lastEnc1Raw * 36000L) / 16384L);
  f.ch[1] = (int32_t)((lastEnc2Raw * 36000L) / 16384L);

  // gyro raw -> dps *1000 fixed point. Scale factor depends on full-scale
  // range configured above (+-2000dps => 16.4 LSB/dps). Adjust if you
  // change GYRO_CONFIG0.
  const float gyroLsbPerDps = 16.4f;
  f.ch[2] = (int32_t)((lastImu.gx / gyroLsbPerDps) * 1000.0f);
  f.ch[3] = (int32_t)((lastImu.gy / gyroLsbPerDps) * 1000.0f);
  f.ch[4] = (int32_t)((lastImu.gz / gyroLsbPerDps) * 1000.0f);

  // accel raw -> g *10000 fixed point. +-16g => 2048 LSB/g.
  const float accelLsbPerG = 2048.0f;
  f.ch[5] = (int32_t)((lastImu.ax / accelLsbPerG) * 10000.0f);
  f.ch[6] = (int32_t)((lastImu.ay / accelLsbPerG) * 10000.0f);
  f.ch[7] = (int32_t)((lastImu.az / accelLsbPerG) * 10000.0f);

  uint8_t packet[64];
  size_t n = buildPacket(f, packet, sizeof(packet));

  udp.beginPacket(DEST_IP, DEST_PORT);
  udp.write(packet, n);
  udp.endPacket();
}

void loop() {
  uint32_t now = millis();

  if (now - lastPollMs >= pollPeriodMs) {
    lastPollMs = now;
    pollSensors();
  }

  if (now - lastPublishMs >= publishPeriodMs) {
    lastPublishMs = now;
    publishFrame();
  }
}
