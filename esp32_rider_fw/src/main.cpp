// XGO Rider — self-balance firmware (Milestone 4d), built on the characterized
// signals. Balances on the WHEELS (inverted pendulum); legs are held rigid by a
// mechanical brace and kept torque-OFF the entire time.
//
//   IMU   : ICM-42670, I2C GPIO18(SDA)/19(SCL) addr 0x69
//   Wheels: UART2 servo bus GPIO13(RX)/14(TX) @1Mbps, IDs 11(L)/21(R), torque via
//           SYNC_WRITE reg 0x1E; body-forward = L:+u / R:-u (mirrored)
//   Legs  : IDs 12/22 — torque FORCED OFF, never enabled (bad right encoder)
//   Out   : USB Serial 115200, live tuning + telemetry
//
// Balance axis (characterized): tilt = roll = atan2(ay,az); rate = gx.
// Control: u = POL * ( Kp*(theta - setpoint) + Kd*thetadot ), clamped, applied
//          L:+u / R:-u. DISABLED by default — send 'e' only on a safety rig.
//
// Build: ~/.xgo-cal/bin/pio run -d esp32_rider_fw ; flash firmware.bin @0x10000

#include <Arduino.h>
#include <Wire.h>

// ---------------- pins / IMU regs ----------------
#define SDA_PIN 18
#define SCL_PIN 19
#define IMU_ADDR 0x69
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT0  0x1F
#define REG_ACCEL_DATA 0x0B

#define LEFT_W  11
#define RIGHT_W 21
#define LEFT_LEG  12
#define RIGHT_LEG 22

// ---------------- shared state (tuned live; scalar volatiles) ----------------
volatile bool  gEnabled   = false;   // balance OFF until 'e'
volatile float gKp        = 18.0f;   // torque per degree of tilt error (RIG-Omni ref=32)
volatile float gKd        = 1.0f;    // torque per (deg/s) of tilt rate (RIG-Omni ref=1.6)
volatile float gSetpoint  = 0.0f;    // balance tilt (deg); capture/tune live
volatile float gPolarity  = -1.0f;   // -1 = CORRECT (verified balancing 2026-06-11; +1 ran away)
volatile int   gUMax      = 110;     // torque clamp (wheels break loose ~60 under load)
volatile float gFallDeg   = 25.0f;   // cut torque past this tilt error
// telemetry (written by task, read by loop)
volatile float tTheta=0, tThetaDot=0, tU=0;

// gyro scale (ICM-42670 default ~+/-2000 dps => 16.4 LSB/dps); folds into Kd/filter
static const float GYRO_LSB_PER_DPS = 16.4f;
static const float CF_ALPHA = 0.98f;     // complementary filter weight on gyro
static const float DT = 0.005f;          // 200 Hz

// ---------------- IMU ----------------
static uint8_t imuRead(uint8_t reg){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)1); return Wire.available()?Wire.read():0xFF;
}
static void imuWrite(uint8_t reg,uint8_t val){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static void imuData(int16_t*ax,int16_t*ay,int16_t*az,int16_t*gx,int16_t*gy,int16_t*gz){
  uint8_t b[12];
  Wire.beginTransmission(IMU_ADDR); Wire.write(REG_ACCEL_DATA); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)12);
  for(uint8_t i=0;i<12&&Wire.available();i++) b[i]=Wire.read();
  *ax=(b[0]<<8)|b[1]; *ay=(b[2]<<8)|b[3]; *az=(b[4]<<8)|b[5];
  *gx=(b[6]<<8)|b[7]; *gy=(b[8]<<8)|b[9]; *gz=(b[10]<<8)|b[11];
}

// ---------------- servo bus (all Serial2 access is in the balance task) ------
static void legTorqueOff(uint8_t id){   // standard 1-byte write reg 0x18 = 0
  uint8_t ck = ~(id + 0x04 + 0x03 + 0x18 + 0x00);
  uint8_t buf[8] = {0xFF,0xFF,id,0x04,0x03,0x18,0x00,ck};
  Serial2.write(buf,8);
}
static void wheelTorque(int16_t tl,int16_t tr){  // SYNC_WRITE reg 0x1E both wheels
  uint8_t buf[18];
  buf[0]=0xFF;buf[1]=0xFF;buf[2]=0xFE;buf[3]=4+5*2;buf[4]=0x83;buf[5]=0x1E;buf[6]=0x04;
  buf[7]=LEFT_W;  buf[8]=0;buf[9]=0;  buf[10]=tl&0xFF; buf[11]=(tl>>8)&0xFF;
  buf[12]=RIGHT_W;buf[13]=0;buf[14]=0;buf[15]=tr&0xFF; buf[16]=(tr>>8)&0xFF;
  uint8_t chk=0; for(int i=2;i<17;i++) chk+=buf[i]; buf[17]=~chk;
  Serial2.write(buf,18);
}
static inline float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}

// ---------------- balance task (core 1, 200 Hz) ----------------
static void balanceTask(void*){
  // hard guarantee: legs limp, wheels zero, before the loop
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); vTaskDelay(2); }
  float theta = 0.0f; bool primed=false;
  TickType_t last = xTaskGetTickCount();
  uint32_t n=0;
  for(;;){
    int16_t ax,ay,az,gx,gy,gz; imuData(&ax,&ay,&az,&gx,&gy,&gz);
    float accelRoll = atan2f((float)ay,(float)az)*57.2958f;
    float rate = (float)gx / GYRO_LSB_PER_DPS;            // deg/s
    if(!primed){ theta=accelRoll; primed=true; }
    theta = CF_ALPHA*(theta + rate*DT) + (1.0f-CF_ALPHA)*accelRoll;

    float err = theta - gSetpoint;
    float u = 0.0f;
    if(gEnabled && fabsf(err) <= gFallDeg){
      u = gPolarity * (gKp*err + gKd*rate);
      // stiction boost (RIG-Omni style): punch past the wheels' deadzone
      if(u > 20.0f) u += 10.0f; else if(u < -20.0f) u -= 10.0f;
      u = clampf(u, -(float)gUMax, (float)gUMax);
    }
    wheelTorque((int16_t)u, (int16_t)(-u));   // mirrored: L:+u, R:-u

    tTheta=theta; tThetaDot=rate; tU=u;

    if((++n % 200)==0){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); } // ~1s re-assert
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

void setup(){
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, 13, 14);
  // *** earliest possible: legs limp, wheels zero (before any delay/IMU) ***
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); delay(3); }

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(200);
  imuWrite(REG_PWR_MGMT0, 0x0F);    // accel+gyro low-noise
  delay(100);

  Serial.printf("\n=== Rider balance FW ===  IMU WHO_AM_I=0x%02X\n", imuRead(REG_WHO_AM_I));
  Serial.println("Legs FORCED torque-OFF (never enabled). Balance DISABLED.");
  Serial.println("Cmds: e/d enable/disable | f flip polarity | c capture setpoint");
  Serial.println("      q/a Kp -/+ | w/s Kd -/+ | z/x setpoint -/+ | [ /] Umax -/+ | ? status");

  xTaskCreatePinnedToCore(balanceTask, "balance", 4096, NULL, 12, NULL, 1);
}

void loop(){
  static uint32_t tp=0;
  // --- live commands (USB) ---
  while(Serial.available()){
    char c=Serial.read();
    switch(c){
      case 'e': gEnabled=true;  Serial.println("# BALANCE ENABLED"); break;
      case 'd': gEnabled=false; Serial.println("# disabled"); break;
      case 'f': gPolarity=-gPolarity; Serial.printf("# polarity=%+.0f\n",gPolarity); break;
      case 'c': gSetpoint=tTheta; Serial.printf("# setpoint captured=%.2f\n",gSetpoint); break;
      case 'q': gKp=fmaxf(0,gKp-0.5f); break;
      case 'a': gKp+=0.5f; break;
      case 'w': gKd=fmaxf(0,gKd-0.05f); break;
      case 's': gKd+=0.05f; break;
      case 'z': gSetpoint-=0.5f; break;
      case 'x': gSetpoint+=0.5f; break;
      case '[': gUMax=max(0,gUMax-10); break;
      case ']': gUMax+=10; break;
      case '?': Serial.printf("# en=%d Kp=%.1f Kd=%.2f set=%.2f pol=%+.0f Umax=%d\n",
                  gEnabled,gKp,gKd,gSetpoint,gPolarity,gUMax); break;
    }
  }
  // --- telemetry @ ~20 Hz ---
  if(millis()-tp >= 50){
    tp=millis();
    Serial.printf("th=%6.2f rate=%7.1f u=%5.0f | en=%d Kp=%.1f Kd=%.2f set=%.2f pol=%+.0f Umax=%d\n",
      tTheta, tThetaDot, tU, gEnabled, gKp, gKd, gSetpoint, gPolarity, gUMax);
  }
}
