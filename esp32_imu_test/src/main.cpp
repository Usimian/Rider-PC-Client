// Milestone 2: ICM-42670 IMU bring-up for XGO Rider.
// I2C on GPIO 18 (SDA) / 19 (SCL), per Rider-Pi_SCH.pdf. Address 0x69 (AD0 high),
// fallback 0x68. Streams accel/gyro + computed roll/pitch over USB at 115200.
//
// Build:  ~/.xgo-cal/bin/pio run -d esp32_imu_test
// Flash:  flash .pio/build/esp32/firmware.bin to 0x10000 (preserves SPIFFS cal)

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 18
#define SCL_PIN 19

// ICM-42670 bank-0 registers
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT0  0x1F
#define REG_ACCEL_DATA 0x0B   // ACCEL_X1..GYRO_Z0 = 12 bytes, big-endian
#define WHOAMI_EXPECT  0x67

static uint8_t imu_addr = 0x69;

static uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void readBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, n);
  for (uint8_t i = 0; i < n && Wire.available(); i++) buf[i] = Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Serial.println("\n=== ICM-42670 IMU bring-up ===");

  Serial.print("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.print(" 0x"); Serial.print(a, HEX); }
  }
  Serial.println();

  uint8_t w = readReg(0x69, REG_WHO_AM_I);
  if (w == WHOAMI_EXPECT) imu_addr = 0x69;
  else {
    w = readReg(0x68, REG_WHO_AM_I);
    if (w == WHOAMI_EXPECT) imu_addr = 0x68;
  }
  Serial.print("Using addr 0x"); Serial.print(imu_addr, HEX);
  Serial.print("  WHO_AM_I=0x"); Serial.print(readReg(imu_addr, REG_WHO_AM_I), HEX);
  Serial.println(WHOAMI_EXPECT == readReg(imu_addr, REG_WHO_AM_I) ? "  (OK)" : "  (UNEXPECTED!)");

  // Enable accel + gyro in Low-Noise mode
  writeReg(imu_addr, REG_PWR_MGMT0, 0x0F);
  delay(100);
  Serial.println("PWR_MGMT0=0x0F, streaming (move the robot to see roll/pitch change)...");
}

void loop() {
  uint8_t b[12];
  readBytes(imu_addr, REG_ACCEL_DATA, b, 12);
  int16_t ax = (b[0] << 8) | b[1], ay = (b[2] << 8) | b[3], az = (b[4] << 8) | b[5];
  int16_t gx = (b[6] << 8) | b[7], gy = (b[8] << 8) | b[9], gz = (b[10] << 8) | b[11];

  // scale-independent tilt from accel
  float roll  = atan2f((float)ay, (float)az) * 57.2958f;
  float pitch = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * 57.2958f;

  Serial.printf("ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d | roll=%7.2f pitch=%7.2f\n",
                ax, ay, az, gx, gy, gz, roll, pitch);
  delay(50);
}
