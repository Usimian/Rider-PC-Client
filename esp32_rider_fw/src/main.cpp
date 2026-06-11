// Tier 3 firmware base (Milestone 3): ICM-42670 IMU + FOC wheels in one image.
//   IMU  : I2C  GPIO 18 (SDA) / 19 (SCL), addr 0x69
//   Wheels: UART2 servo bus GPIO 13 (RX) / 14 (TX) @ 1 Mbps, IDs 11 (L) / 21 (R)
//   Output: USB Serial (UART0) @ 115200
//
// At boot: gentle torque wiggle on the LEFT wheel (proves drive from firmware),
// then continuous stream of IMU pitch/roll/gyro + both wheel odometry.
//
// Build: ~/.xgo-cal/bin/pio run -d esp32_rider_fw
// Flash: firmware.bin -> 0x10000 (preserves SPIFFS cal)

#include <Arduino.h>
#include <Wire.h>

// ---------------- IMU (ICM-42670) ----------------
#define SDA_PIN 18
#define SCL_PIN 19
#define IMU_ADDR 0x69
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT0  0x1F
#define REG_ACCEL_DATA 0x0B

static uint8_t imuRead(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static void imuReadData(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz){
  uint8_t b[12];
  Wire.beginTransmission(IMU_ADDR); Wire.write(REG_ACCEL_DATA); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)12);
  for (uint8_t i=0;i<12 && Wire.available();i++) b[i]=Wire.read();
  *ax=(b[0]<<8)|b[1]; *ay=(b[2]<<8)|b[3]; *az=(b[4]<<8)|b[5];
  *gx=(b[6]<<8)|b[7]; *gy=(b[8]<<8)|b[9]; *gz=(b[10]<<8)|b[11];
}

// ---------------- Wheels (FOC servo bus) ----------------
#define LEFT_ID  11
#define RIGHT_ID 21
static const int WRAP = 1024, WRAP_THRESH = 800;
static long odomL = 0, odomR = 0;
static int lastL = -1, lastR = -1;

static void sendReadReq(uint8_t id) {
  uint8_t ck = ~(id + 0x04 + 0x02 + 0x24 + 0x06);
  uint8_t buf[8] = {0xFF,0xFF,id,0x04,0x02,0x24,0x06,ck};
  Serial2.write(buf, 8);
}

// Returns true and fills pos/vel if a valid response frame for `id` is found.
static bool readWheel(uint8_t id, int* pos, int* vel) {
  while (Serial2.available()) Serial2.read();   // flush stale
  sendReadReq(id);
  delay(3);
  uint8_t buf[40]; int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 6 && n < (int)sizeof(buf)) {
    if (Serial2.available()) buf[n++] = Serial2.read();
  }
  // scan for FF FF <id> <LEN==0x0B> ... (skips half-duplex echo whose LEN==0x04)
  for (int i = 0; i + 4 < n; i++) {
    if (buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id) {
      uint8_t LEN = buf[i+3];
      if (LEN==0x0B && i+3+LEN < n+1 && i+3+(LEN) <= n) {
        // pos/vel at the frame tail (firmware convention)
        uint8_t* f = &buf[i];
        *pos = f[LEN-3] | (f[LEN-2] << 8);
        int v = f[LEN-1] | (f[LEN] << 8);
        if (v >= 0x8000) v -= 0x10000;
        *vel = v;
        return true;
      }
    }
  }
  return false;
}

// Standard FeeTech 1-byte register write (for leg servos)
static void writeServoReg(uint8_t id, uint8_t reg, uint8_t val) {
  uint8_t ck = ~(id + 0x04 + 0x03 + reg + val);
  uint8_t buf[8] = {0xFF,0xFF,id,0x04,0x03,reg,val,ck};
  Serial2.write(buf, 8);
}

// Leg servos (IDs 12/22) are standard position servos: read 2 bytes @ 0x24,
// reply = FF FF id 04 ERR posL posH CK (no bus echo on this hardware).
static int readLeg(uint8_t id) {
  while (Serial2.available()) Serial2.read();
  uint8_t ck = ~(id + 0x04 + 0x02 + 0x24 + 0x02);
  uint8_t req[8] = {0xFF,0xFF,id,0x04,0x02,0x24,0x02,ck};
  Serial2.write(req, 8);
  delay(3);
  uint8_t buf[20]; int n=0; unsigned long t0=millis();
  while (millis()-t0 < 6 && n < (int)sizeof(buf)) if (Serial2.available()) buf[n++]=Serial2.read();
  for (int i=0; i+6 < n; i++)
    if (buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id && buf[i+3]==0x04)
      return buf[i+5] | (buf[i+6] << 8);
  return -1;
}

// Leg goal position: these (XGO) servos use a NON-STANDARD Goal_Position at
// reg 0x06 (2 bytes) — verified empirically (0x2A/0x35 do nothing). Torque
// enable is reg 0x18. The slow per-cycle ramp (caller) controls speed.
static void writeLegGoal(uint8_t id, int pos) {
  uint8_t lo = pos & 0xFF, hi = (pos >> 8) & 0xFF;
  uint8_t ck = ~(id + 0x05 + 0x03 + 0x06 + lo + hi);
  uint8_t buf[9] = {0xFF,0xFF,id,0x05,0x03,0x06,lo,hi,ck};
  Serial2.write(buf, 9);
}

// Leg stance (from measured limits): mid = safe standing target; clamp to range.
#define LEG_MID_L 918
#define LEG_MID_R 82
#define LEG_L_MIN 863
#define LEG_L_MAX 974
#define LEG_R_MIN 30
#define LEG_R_MAX 135
#define LEG_VEL   200          // low/gentle move speed
static bool  legHold = false;
static float legTgtL = LEG_MID_L, legTgtR = LEG_MID_R;
static int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }

static void accumOdom(long* odom, int* last, int pos) {
  if (*last >= 0) {
    int d = pos - *last;
    if (d < -WRAP_THRESH) d += WRAP;
    else if (d > WRAP_THRESH) d -= WRAP;
    *odom += d;
  }
  *last = pos;
}

// SYNC_WRITE torque to reg 0x1E, both wheels (pos field = 0)
static void sendWheelTorque(int16_t tl, int16_t tr) {
  uint8_t buf[18];
  buf[0]=0xFF; buf[1]=0xFF; buf[2]=0xFE; buf[3]=4+5*2; buf[4]=0x83; buf[5]=0x1E; buf[6]=0x04;
  buf[7]=LEFT_ID;  buf[8]=0; buf[9]=0;  buf[10]=tl&0xFF; buf[11]=(tl>>8)&0xFF;
  buf[12]=RIGHT_ID;buf[13]=0;buf[14]=0; buf[15]=tr&0xFF; buf[16]=(tr>>8)&0xFF;
  uint8_t chk=0; for(int i=2;i<17;i++) chk+=buf[i]; buf[17]=~chk;
  Serial2.write(buf, 18);
}

static void pollBoth(int* pl,int* vl,int* pr,int* vr){
  int p,v;
  if (readWheel(LEFT_ID,&p,&v))  { accumOdom(&odomL,&lastL,p); *pl=p; *vl=v; }
  if (readWheel(RIGHT_ID,&p,&v)) { accumOdom(&odomR,&lastR,p); *pr=p; *vr=v; }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(300);
  Serial.println("\n=== Rider FW (IMU + wheels) ===");
  Serial.print("IMU WHO_AM_I=0x"); Serial.println(imuRead(REG_WHO_AM_I), HEX);
  imuWrite(REG_PWR_MGMT0, 0x0F);
  delay(100);

  // leg servos limp at boot (reg 0x18 = torque enable). Commands control hold.
  writeServoReg(12, 0x18, 0); delay(5);
  writeServoReg(22, 0x18, 0); delay(5);
  Serial.println("Legs LIMP. Commands:  'h' = hold mid stance (gentle ramp)   'o' = limp");
}

void loop() {
  // ---- command interface ----
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'h') {
      int cl = readLeg(12), cr = readLeg(22);
      if (cl > 0 && cr > 0) {
        legTgtL = cl; legTgtR = cr;                 // start ramp from current (no snap)
        writeLegGoal(12, cl);
        writeLegGoal(22, cr);
        writeServoReg(12, 0x18, 1);                 // enable torque holding current pos
        writeServoReg(22, 0x18, 1);
        legHold = true;
        Serial.printf("# HOLD on: from L=%d R=%d -> mid %d/%d\n", cl, cr, LEG_MID_L, LEG_MID_R);
      } else Serial.println("# leg read failed, not enabling");
    } else if (c == 'o') {
      legHold = false;
      writeServoReg(12, 0x18, 0);
      writeServoReg(22, 0x18, 0);
      Serial.println("# limp");
    }
  }

  int16_t ax,ay,az,gx,gy,gz;
  imuReadData(&ax,&ay,&az,&gx,&gy,&gz);
  float roll = atan2f((float)ay,(float)az)*57.2958f;

  int legL = readLeg(12), legR = readLeg(22);

  if (legHold) {
    // ramp targets toward mid at 1 count/cycle (~25/s -> ~4s full range), clamped — gentle
    if (legTgtL < LEG_MID_L) legTgtL += 1; else if (legTgtL > LEG_MID_L) legTgtL -= 1;
    if (legTgtR < LEG_MID_R) legTgtR += 1; else if (legTgtR > LEG_MID_R) legTgtR -= 1;
    writeLegGoal(12, clampi((int)legTgtL, LEG_L_MIN, LEG_L_MAX));
    writeLegGoal(22, clampi((int)legTgtR, LEG_R_MIN, LEG_R_MAX));
  }

  Serial.printf("roll=%6.1f | legL=%5d legR=%5d | hold=%d tgtL=%d tgtR=%d\n",
                roll, legL, legR, legHold, (int)legTgtL, (int)legTgtR);
  delay(40);
}
