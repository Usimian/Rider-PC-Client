// XGO Rider — self-balance firmware (Milestone 5: 4-state LQR).
//
// Replaces the tilt-only PID with the same FULL-STATE feedback the factory /
// RIG-Omni firmware uses, which is what let it "home to a position" and move
// fast enough to catch itself:
//
//   u = POL * ( K0*(wheel_x - stable_pos)   // wheel POSITION error (m)  <-- new
//             + K1*(wheel_vx - vx)          // wheel VELOCITY error      <-- new
//             + K2*(tilt - setpoint)        // tilt angle (deg)
//             + K3*tilt_rate )              // tilt rate (deg/s)
//   + stiction boost (+/-10 past +/-20), clamped to +/-Umax, applied L:+u / R:-u.
//
//   IMU   : ICM-42670, I2C GPIO18(SDA)/19(SCL) addr 0x69. tilt=atan2(ay,az).
//   Wheels: UART2 servo bus GPIO13(RX)/14(TX) @1Mbps, IDs 11(L)/21(R).
//           torque  = SYNC_WRITE reg 0x1E ; read pos/vel = reg 0x24 (6 bytes).
//           odom: 10-bit, wrap thresh 800; RIGHT wheel mirrored (matches L:+u/R:-u).
//   Legs  : IDs 12/22 — torque FORCED OFF, never enabled (bad right encoder).
//   Hosts : UART0 (USB-C) and UART1 (Pi, RX=IO4/TX=IO5) both carry cmd+telemetry.
//
// Gains default to the factory values (1400 / 6.8 / 32 / 1.6); all live-tunable.
// Build: ~/.xgo-cal/bin/pio run -d esp32_rider_fw ; flash firmware.bin @0x10000

#include <Arduino.h>
#include <Wire.h>
#include "rider_policy.h"   // auto-generated PPO policy weights (sim/export_policy.py)

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
// CASCADED PID (factory R-1.1.3 architecture, NOT parallel LQR — see memory).
//   pos(P) -> velocity target ; vel(PI) -> lean target ; pitch(P) -> torque ; + rate damping
volatile bool  gEnabled  = false;   // balance OFF until 'en 1'
// --- DEFAULTS = first config that actually balanced (2026-06-12, two good wheels) ---
// Pure pitch loop (outer cascade zeroed); factory cascade gains fought our torque-direct
// actuation and oscillated. kppit/kdpit empirically tuned. set=measured balance angle.
volatile float gKpPos = 0.0f;       // position P  (0 = pure pitch; outer loop off)
volatile float gKpVel = 0.01f;      // velocity P  (gentle drift-damping; holds its spot)
volatile float gKiVel = 0.0f;       // velocity I  (0 = pure pitch)
volatile float gKpPit = 70.0f;      // pitch P     (lean error -> torque)
volatile float gKdPit = 13.0f;      // pitch-rate damping (-Kd*dq)
volatile float gImuZero = 0.0f;     // base lean trim (deg); setpoint carries the offset
volatile float gSetpoint = 4.18f;   // measured true balance tilt (our frame); pitch = theta-set
volatile float gVx       = 0.0f;    // commanded forward velocity (0 = hold)
volatile float gPolarity = 1.0f;    // +1 for cascade (pitDes-pitch order) = our verified tilt/rate sign
volatile float gPosSign  = -1.0f;   // wheel vel/pos cascade sign (-1: our pitch axis is inverted vs factory)
volatile float gPosErrSign = 1.0f;  // position-loop correction sign (empirically found; flip with 'pesign')
// ---- neural-policy control (ADDITIVE; PID stays the default). 'pol 1' to enable. ----
// SIM-TRAINED, NOT YET HW-VERIFIED. Obs/action signs almost certainly need an
// on-robot check (flip with 'polsign', and watch the obs match the sim's SI units).
volatile bool  gPolMode  = false;   // 'pol 1' -> run the trained policy instead of the PID
volatile float gPolSign  = 1.0f;    // action->wheel-command sign (verify on robot)
volatile float gPolULP   = 0.0f;    // low-pass on the POLICY output cmd (0=off; adds command lag, can destabilize)
volatile float gPolVelLP = 0.7f;    // low-pass on the velocity FED TO THE POLICY (kills quantization shimmy at the input)
// Code-based position hold: policy stays a PURE balancer (its position obs are
// neutralized); a simple outer loop biases the balance SETPOINT toward a target
// tick so the balancer drives there. 'poshold <deg/m>' (0=off), 'posmax <deg>'.
volatile float gPosHoldK   = 5.0f;  // setpoint-bias P gain (deg per meter of position error); 0 = pure balance
volatile float gPosHoldKi  = 2.5f;  // setpoint-bias I gain (deg per m*s); kills steady-state offset (auto-tuned)
volatile float gPosHoldKd  = 28.0f; // setpoint-bias D gain (deg per m/s); damps the approach (auto-tuned)
volatile float gPosHoldMax = 5.0f;  // max setpoint bias / braking authority (deg) (auto-tuned)
volatile float gPosIntBand = 0.15f; // only integrate within this dist of target (m) -> no windup during moves
volatile float gPosVmax    = 0.35f; // move speed cap (m/s): full-stick drive speed. 0.35 = tuned clean-stop ceiling (0.6 overshot/toppled). live-tune via 'posvmax'
volatile float gPosAmax    = 0.8f;  // accel cap (m/s^2) on the drive-velocity ramp: gentle accel kills the lurch/oscillation. live-tune via 'posamax'
volatile float gTurn       = 0.0f;  // wheel differential (raw) ADDED to both wheel cmds -> yaw. Computed by the yaw-rate controller each loop.
volatile float gTurnMax    = 250.0f;// clamp on |gTurn| (raw)
volatile float gYawRateCmd = 0.0f;  // commanded yaw rate (rad/s) from the stick; full stick = +/-2 rad/s. set via 'turnrate'
volatile float gYawKp      = 40.0f; // wheel-differential (raw) per rad/s of yaw-rate error (FB trim; 120 oscillated)
volatile float gYawFF      = 75.0f; // FEEDFORWARD differential (raw) per rad/s commanded -> firm turn open-loop, low FB gain stays stable
volatile float gTurnLP     = 0.5f;  // low-pass on the turn output (0=off..0.95) -> smooths residual jitter
volatile float gYawFb      = -1.0f; // yaw-rate FEEDBACK sign (-1 = correct: gz reads opposite to the turn dir; +1 ran away). live-tune via 'yawfb'
volatile float gPosVelLP   = 0.8f;  // low-pass on the velocity feeding the D term (0=off..→1 heavy); kills D-noise shimmy
volatile float gPosTarget  = 0.0f;  // position setpoint (m, wheel_x frame); anchored at enable, moved via 'ptgt'
static const float POL_DEG2RAD     = 0.017453292f;
static const float POL_WHEEL_R     = 0.03f;      // wheel radius (m) -> x_vel = wheel_omega*r
static const float POL_VELMAX_RADS = 30.0f;      // action=1 -> 30 rad/s (sim vel_max)
static const float POL_RAW_PER_RADS= 7.764f;     // raw wheel-cmd per rad/s (1/0.1288, measured)
volatile int   gUMax     = 1000;    // output clamp = factory pid_pit.UMax class (was throttled to 110)
volatile float gFF       = 10.0f;   // stiction boost beyond +/-FFband (factory: +10 past 20)
volatile float gFFband   = 20.0f;   // boost threshold
volatile float gDqUMax   = 60.0f;   // clamp on rate-damping term
volatile float gVelIClamp= 8.0f;    // velocity integral anti-windup clamp (deg of lean)
volatile int   gDither   = 0;       // +/- buzz added to wheel cmd each cycle (stiction fluidizer); 0=off
volatile float gFallDeg  = 25.0f;   // cut torque past this tilt error
volatile float gMaxPosErr = 0.50f;  // cut torque past this position runaway (m)
// --- factory R-1.1.3 position-goal wheel command (toggleable A/B vs torque-only) ---
volatile int   gPosMode  = 0;       // 0 = torque-write (current); 1 = position-goal accumulate
volatile int   gTorLim   = 1000;    // torque-limit field used in position-goal mode
volatile int   gPosClamp = 120;     // per-cycle goal delta clamp (factory +/-120 counts)
// --- control-loop period: factory runs ~167Hz (3x vTaskDelay(2ms)=6ms). Our per-sample
// filter/integral constants are tuned for THAT rate, so pace the loop to match. ---
volatile int   gLoopUs   = 3000;    // target loop period (us); ~253Hz measured (balancing rate)
volatile int   gWheelFault = 0;     // 0=ok; else ID of a wheel that dropped off the bus (auto-disabled)

// telemetry (written by task, read by loop)
volatile float tTheta=0, tRate=0, tU=0, tWheelX=0, tWheelVx=0;
volatile float tRoll=0;                  // lateral tilt (deg), accel-only readout for leg leveling
volatile float tVbat=0;                  // smoothed pack voltage (V), read from GPIO33 divider
volatile float tYaw=0;                    // integrated heading (deg), gyro-Z only -> drifts slowly
volatile float tTgtEff=0, tVdes=0;        // slewed pos target + profile velocity (driving diagnostics)
volatile float gYawBias=0.0f;             // gyro-Z zero-rate bias (deg/s), from calibrateGyro
volatile float gVbatK=3.0f;              // GPIO33 divider ratio: pack = pin*K. Schematic R8=20K(top)/R7=10K(bot) => x3.0
// 2S pack (net VCC8.4V): 8.4V full, 6.0V empty -> linear % for display.
static const float VBAT_FULL=8.4f, VBAT_EMPTY=6.0f;
volatile float tLoopHz=0, tReadFail=0;   // measured control-loop rate & read-fail %
volatile float tDtMaxMs=0;               // worst-case cycle period (ms) since last report (ground-truth stall detector)
volatile bool  gDumpRead=false;     // 'rd' -> task dumps next raw read frame
volatile int   gTestTor=0;          // 'wt <v>' -> direct wheel torque when DISABLED (stand test)
volatile int   gPollLock=0;         // 'lock <id>' -> poll only that wheel (0 = alternate)
volatile int   gCfgDumpId=0;        // 'cfgdump <id>' -> dump servo config regs 0x00-0x2F
volatile float gGyroBias=0.0f;      // measured gyro zero-rate offset (deg/s), subtracted from rate
volatile float gCfAlpha=0.996f;     // compl-filter gyro weight PER SAMPLE @~333Hz (~1s accel trim).
// --- factory control-input filter (R-1.1.3 FUN_400d529c, Ghidra-confirmed) ---
// Between IMU fusion and the PID, the factory heavily low-passes pitch & rate
// (0.95/0.05) and slew-limits the smoothed pitch to 5 deg/cycle. We lacked this.
volatile float gCtrlLP  = 0.40f;    // control-input low-pass: used = LP*used + (1-LP)*raw
volatile float gSlewDeg = 5.0f;     // per-cycle slew clamp on the smoothed pitch (deg)
                                    // 0.98 was WAY too accel-trusting (~0.15s): forward accel tips the
                                    // apparent gravity vector -> estimator reads "level" while still
                                    // leaning -> runaway in the lean direction. Accel must trim SLOWLY.
volatile bool  gDoGcal=false;       // 'gcal' -> recalibrate gyro bias (robot must be still + disabled)
volatile bool  gStepCap=false;      // 'stepcap' -> capture wheel step response (stand only, disabled)
volatile int   gStepCmd=200;        // step magnitude (wheel torque field) for the capture
static void emit(const char* b, int k);   // fwd decl (defined below)

// IMU / filter constants
static const float GYRO_LSB_PER_DPS = 16.4f;
// wheel encoder constants (match RIG-Omni xgo.cc exactly)
static const float WHEEL_PI = 3.14159265f;
static const float K_V = 60.0f * WHEEL_PI / 1024.0f;  // raw-vel -> scaled vel
static const float LP_VEL = 0.7f;                     // velocity low-pass
static const float WHEEL_CIRC_M = WHEEL_PI * 0.06f;   // 6 cm wheels

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
// Drain the echo + status reply after a 1-byte write so a following write can't
// transmit into the previous servo's reply (half-duplex collision). Back-to-back
// un-drained writes here corrupted right-wheel ID21 (the 2nd write collided with
// ID11's reply) -> silent until power-cycle. Reproduced + fixed 2026-06-12.
static void busSettle(){
  Serial2.flush();                                  // wait for our TX to finish
  uint32_t t0=micros();
  while(micros()-t0 < 700) if(Serial2.available()) Serial2.read();  // consume echo+reply, leave bus idle
}
static void legTorqueOff(uint8_t id){   // standard 1-byte write reg 0x18 = 0
  while(Serial2.available()) Serial2.read();
  uint8_t ck = ~(id + 0x04 + 0x03 + 0x18 + 0x00);
  uint8_t buf[8] = {0xFF,0xFF,id,0x04,0x03,0x18,0x00,ck};
  Serial2.write(buf,8); busSettle();
}
static void wheelSetReg(uint8_t id, uint8_t reg, uint8_t val){  // 1-byte reg write
  while(Serial2.available()) Serial2.read();
  uint8_t ck = ~(id + 0x04 + 0x03 + reg + val);
  uint8_t buf[8] = {0xFF,0xFF,id,0x04,0x03,reg,val,ck};
  Serial2.write(buf,8); busSettle();
}
static void wheelTorque(int16_t tl,int16_t tr){  // SYNC_WRITE reg 0x1E both wheels
  uint8_t buf[18];
  buf[0]=0xFF;buf[1]=0xFF;buf[2]=0xFE;buf[3]=4+5*2;buf[4]=0x83;buf[5]=0x1E;buf[6]=0x04;
  buf[7]=LEFT_W;  buf[8]=0;buf[9]=0;  buf[10]=tl&0xFF; buf[11]=(tl>>8)&0xFF;
  buf[12]=RIGHT_W;buf[13]=0;buf[14]=0;buf[15]=tr&0xFF; buf[16]=(tr>>8)&0xFF;
  uint8_t chk=0; for(int i=2;i<17;i++) chk+=buf[i]; buf[17]=~chk;
  Serial2.write(buf,18); Serial2.flush();
}
// Torque enable/disable BOTH wheels via one SYNC_WRITE to reg 0x18 (broadcast 0xFE ->
// NO status reply per FeeTech protocol). Replaces back-to-back addressed wheelSetReg
// writes whose replies collided on the half-duplex bus and bricked a wheel until
// power-cycle. Broadcast = no reply = no possible collision.
static void wheelTorqueEnAll(uint8_t on){
  uint8_t buf[12];
  buf[0]=0xFF;buf[1]=0xFF;buf[2]=0xFE;buf[3]=8;buf[4]=0x83;buf[5]=0x18;buf[6]=0x01;
  buf[7]=LEFT_W; buf[8]=on;
  buf[9]=RIGHT_W;buf[10]=on;
  uint8_t chk=0; for(int i=2;i<11;i++) chk+=buf[i]; buf[11]=~chk;
  Serial2.write(buf,12); Serial2.flush();
}
// Factory R-1.1.3 wheel command (WritePos_Sync_kp): SYNC_WRITE reg 0x1E carrying
// per-servo [position(2), torque(2)] — the servo's internal position loop drives
// toward the goal under the torque limit. (vs wheelTorque which sends position=0.)
static void wheelPosTor(int16_t pl,int16_t pr,int16_t tl,int16_t tr){
  uint8_t buf[18];
  buf[0]=0xFF;buf[1]=0xFF;buf[2]=0xFE;buf[3]=4+5*2;buf[4]=0x83;buf[5]=0x1E;buf[6]=0x04;
  buf[7]=LEFT_W;  buf[8]=pl&0xFF;buf[9]=(pl>>8)&0xFF;  buf[10]=tl&0xFF; buf[11]=(tl>>8)&0xFF;
  buf[12]=RIGHT_W;buf[13]=pr&0xFF;buf[14]=(pr>>8)&0xFF; buf[15]=tr&0xFF; buf[16]=(tr>>8)&0xFF;
  uint8_t chk=0; for(int i=2;i<17;i++) chk+=buf[i]; buf[17]=~chk;
  Serial2.write(buf,18); Serial2.flush();
}
// Read present pos (10-bit) + vel (signed) from reg 0x24. Half-duplex: flush RX
// first to drop the TX echo; reply frame is FF FF id 0x0B ... (LEN filter).
static uint8_t gRaw[48]; static int gRawN=0;   // last raw read frame (for 'rd' dump)
static bool wheelRead(uint8_t id, int* pos, int* vel){
  uint8_t ck = ~(id + 0x04 + 0x02 + 0x24 + 0x06);
  uint8_t req[8] = {0xFF,0xFF,id,0x04,0x02,0x24,0x06, ck};
  while(Serial2.available()) Serial2.read();
  Serial2.write(req,8); Serial2.flush();
  uint8_t buf[48]; int n=0; uint32_t t0=micros();
  while(micros()-t0 < 2000 && n < 48){            // enough for a good reply; fail fast on a miss
    while(Serial2.available() && n < 48) buf[n++]=Serial2.read();
    if(n >= 15) break;                            // got a full frame, stop early
  }
  memcpy(gRaw,buf,n); gRawN=n;   // stash for debug dump
  // Skip the half-duplex echo of our 8-byte request, then parse the reply
  // frame: FF FF id LEN err <params> chk. pos/vel sit at the frame tail.
  for(int i=0;i+5 < n;i++){
    if(buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id){
      int LEN = buf[i+3];
      if(LEN==0x04) continue;                 // this is the echoed request — skip
      if(i+LEN >= n) continue;                // frame incomplete
      int p = buf[i+LEN-3] | (buf[i+LEN-2]<<8);
      int v = buf[i+LEN-1] | (buf[i+LEN]  <<8);
      if(v >= 0x8000) v -= 0x10000;
      *pos = p & 0x3FF; *vel = v; return true;
    }
  }
  return false;
}
static inline float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}

// Read 'len' bytes from register 'reg' of a servo (for config dump). Returns true + fills out[].
static bool servoReadN(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out){
  uint8_t ck = ~(id + 0x04 + 0x02 + reg + len);
  uint8_t req[8] = {0xFF,0xFF,id,0x04,0x02,reg,len,ck};
  while(Serial2.available()) Serial2.read();
  Serial2.write(req,8); Serial2.flush();
  uint8_t buf[48]; int n=0; uint32_t t0=micros();
  while(micros()-t0 < 3000 && n < 48){
    while(Serial2.available() && n<48) buf[n++]=Serial2.read();
    if(n >= len+6) break;
  }
  for(int i=0;i+4<n;i++){
    if(buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id){
      if(buf[i+3]==0x04) continue;                  // echoed request, skip
      int dstart = i+5;                             // standard: data right after ERR
      if(i+6<n && buf[i+6]==0x02) dstart = i+8;      // wheel servos prepend [len 02 reg] echo
      if(dstart+len > n) continue;
      for(int k=0;k<len;k++) out[k]=buf[dstart+k];
      return true;
    }
  }
  return false;
}

// Average the gyro while the robot is held still -> zero-rate bias.
// gx = balance/pitch axis; gz = yaw/heading axis (both biased for drift-free integration).
static void calibrateGyro(){
  const int N=200; float sum=0, sumz=0;
  for(int i=0;i<N;i++){
    int16_t ax,ay,az,gx,gy,gz; imuData(&ax,&ay,&az,&gx,&gy,&gz);
    sum  += (float)gx / GYRO_LSB_PER_DPS;
    sumz += (float)gz / GYRO_LSB_PER_DPS;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  gGyroBias = sum / N;
  gYawBias  = sumz / N;
  tYaw = 0.0f;                            // zero heading at each (re)calibration
}

// ---------------- wheel odometry state ----------------
static float wheel1_x=0, wheel2_x=0;       // accumulated counts (L, R-mirrored)
static float wheel1_vel=0, wheel2_vel=0;   // filtered scaled velocity
static int   last11=-1, last21=-1;
static float wheel_x=0, wheel_vx=0;        // fused: meters, scaled vel
static float stable_pos=0;                 // home position (m), captured on enable
// Faithful copy of factory CalIWeakenPID (RIG-Omni xgo.cc:715) + struct (xgo.h).
struct PID { float Des,FB,Kp,Ki,Kd,E,PreE,SumE,U,UMax,EMax,EMin,Up,PMax,Ui,IMax,Ud,DMax; };
static PID pidPos = {0,0, 60.0f, 0.0f,    0,0,0,0,0, 1000,100, 0.0001f, 0,1000, 0,1000, 0,1000};
static PID pidVel = {0,0, 0.15f, 0.01f,   0,0,0,0,0,  100,100, 0.01f,   0, 100, 0, 100, 0, 100};
static PID pidPit = {0,0, 21.0f, 0.0f,    0,0,0,0,0, 1000,  0, 0.01f,   0,1000, 0,1000, 0, 100};
static void calPID(PID* p){
  p->E = p->Des - p->FB;
  if(p->E * p->PreE < 0){ p->Ui = 0; p->SumE = 0; }       // reset I on error sign-flip
  p->SumE += p->E;
  if(p->EMax > 0) p->SumE = clampf(p->SumE, -p->EMax, p->EMax);
  p->Ui = clampf(p->Ki * p->SumE, -p->IMax, p->IMax);
  p->Ud = clampf(p->Kd * (p->E - p->PreE), -p->DMax, p->DMax);
  if(p->E < p->EMin && p->E > -p->EMin) p->SumE *= 0.95f;   // I-weaken near setpoint (factory 0.95)
  p->Up = clampf(p->Kp * p->E, -p->PMax, p->PMax);
  p->PreE = p->E;
  p->U = clampf(p->Up + p->Ui + p->Ud, -p->UMax, p->UMax);
}
static void pidReset(PID* p){ p->SumE=0; p->PreE=0; p->Ui=0; p->U=0; }

static void odomUpdate(uint8_t id, int pos, int vel){
  if(id==LEFT_W){
    if(last11>=0){
      int d = pos - last11;
      if(d < -800) wheel1_x += 1023 + d; else if(d > 800) wheel1_x += -1023 + d; else wheel1_x += d;
    }
    last11 = pos;
    wheel1_vel = LP_VEL*wheel1_vel + (1.0f-LP_VEL)*(4.5f*(float)vel*K_V);
  } else { // RIGHT_W — mirrored (matches L:+u / R:-u)
    if(last21>=0){
      int d = pos - last21;
      if(d < -800) wheel2_x -= 1023 + d; else if(d > 800) wheel2_x -= -1023 + d; else wheel2_x -= d;
    }
    last21 = pos;
    wheel2_vel = LP_VEL*wheel2_vel + (1.0f-LP_VEL)*(4.5f*(float)(-vel)*K_V);
  }
  wheel_x  = (wheel1_x + wheel2_x) / 2.0f / 1024.0f * WHEEL_CIRC_M;
  wheel_vx = (wheel1_vel + wheel2_vel) / 2.0f;
}

// ---- neural-policy inference: 14-dim obs -> action in [-1,1] (deterministic).
//      z = clip((obs-mean)/sqrt(var+eps)); a = clip(W2 tanh(W1 tanh(W0 z+b0)+b1)+b2).
//      Validated against SB3 to 1.9e-7 in sim/export_policy.py. ~5k MACs, microseconds. ----
static float policyInfer(const float* obs){
  float z[POL_OBS], h0[64], h1[64];
  for(int i=0;i<POL_OBS;i++){
    float v=(obs[i]-POL_MEAN[i])/sqrtf(POL_VAR[i]+POL_EPS);
    z[i]= v>POL_CLIP?POL_CLIP:(v<-POL_CLIP?-POL_CLIP:v);
  }
  for(int j=0;j<64;j++){ float s=POL_B0[j]; for(int i=0;i<POL_OBS;i++) s+=POL_W0[j][i]*z[i];  h0[j]=tanhf(s); }
  for(int j=0;j<64;j++){ float s=POL_B1[j]; for(int i=0;i<64;i++)     s+=POL_W1[j][i]*h0[i]; h1[j]=tanhf(s); }
  float a=POL_B2[0]; for(int i=0;i<64;i++) a+=POL_W2[0][i]*h1[i];
  return a>1.0f?1.0f:(a<-1.0f?-1.0f:a);
}

// ---------------- balance task (core 1) ----------------
static void balanceTask(void*){
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); vTaskDelay(2); }
  calibrateGyro();                  // boot cal (robot should be held still); redo live with 'gcal'
  float theta=0, dqf=0; bool primed=false;
  float pitchF=0, rateF=0; bool fpInit=false;   // factory control-input filter state
  uint8_t pollId = LEFT_W;            // alternate which wheel we read each cycle
  uint32_t lastUs = micros(); uint32_t n=0;
  uint32_t hzN=0, hzFail=0, hzT=millis();   // loop-rate / read-fail measurement
  float dtMaxAcc=0;                          // worst-case raw cycle period this report window
  for(;;){
    // --- on-demand gyro recal (only while disabled + still) ---
    if(gDoGcal && !gEnabled){ wheelTorque(0,0); calibrateGyro(); gDoGcal=false; primed=false; fpInit=false; }
    // --- config dump (read-only): regs 0x00-0x7F in 8-byte chunks ---
    // (extended past 0x2F to hunt the wheel-servo present-voltage register for battery telemetry)
    if(gCfgDumpId){ uint8_t id=gCfgDumpId; gCfgDumpId=0;
      for(uint8_t reg=0; reg<0x80; reg+=8){
        uint8_t d[8]={0}; bool ok=servoReadN(id,reg,8,d);
        char b[80]; int k=snprintf(b,sizeof(b),"# cfg id=%d r0x%02X:",id,reg);
        if(ok) for(int j=0;j<8;j++) k+=snprintf(b+k,sizeof(b)-k," %02X",d[j]);
        else k+=snprintf(b+k,sizeof(b)-k," (noresp)");
        k+=snprintf(b+k,sizeof(b)-k,"\n"); emit(b,k); vTaskDelay(6);
      }
    }

    // --- step-response capture (stand only, disabled): apply a torque step to the
    //     wheels and log velocity at full read rate, to measure actuator latency /
    //     lag / gain and whether the command behaves as torque (vel ramps) or
    //     velocity (vel settles). Dumps "# cap i t_us cmd vel" then "# cap done". ---
    if(gStepCap && !gEnabled){
      gStepCap=false;
      static uint32_t capT[128]; static int16_t capV[128]; static int16_t capU[128];
      uint8_t wid = gPollLock ? (uint8_t)gPollLock : (uint8_t)LEFT_W;
      wheelTorqueEnAll(1);
      uint32_t t0=micros();
      for(int i=0;i<128;i++){
        int cmd = (i<16) ? 0 : gStepCmd;            // 16 baseline samples, then step
        wheelTorque((int16_t)cmd, (int16_t)(-cmd)); // mirrored, as in balance
        int pp=0,vv=0; bool ok=wheelRead(wid,&pp,&vv);
        capT[i]=micros()-t0; capU[i]=(int16_t)cmd; capV[i]= ok ? (int16_t)vv : (int16_t)-32768;
      }
      wheelTorque(0,0); wheelTorqueEnAll(0);
      for(int i=0;i<128;i++){
        char b[56]; int k=snprintf(b,sizeof(b),"# cap %d %lu %d %d\n",
                                   i,(unsigned long)capT[i],capU[i],capV[i]);
        emit(b,k); vTaskDelay(2);
      }
      emit("# cap done\n",11);
      continue;
    }

    // --- dt ---
    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f; lastUs = now;
    if(dt > dtMaxAcc) dtMaxAcc = dt;            // capture RAW period BEFORE the safety clamp below
    if(dt <= 0 || dt > 0.05f) dt = 0.006f;
    hzN++;
    if(millis()-hzT >= 250){ uint32_t el=millis()-hzT; tLoopHz=hzN*1000.0f/el; tReadFail=hzFail*100.0f/hzN; tDtMaxMs=dtMaxAcc*1000.0f; dtMaxAcc=0; hzN=0; hzFail=0; hzT=millis(); }

    // --- IMU -> tilt + rate ---
    int16_t ax,ay,az,gx,gy,gz; imuData(&ax,&ay,&az,&gx,&gy,&gz);
    // ----- factory IMU pitch fusion (RIG-Omni imu.cc:240-246, verbatim) -----
    float pit_acc  = atan2f((float)ay,(float)az)*57.2958f;       // forward/vertical -> deg
    // roll readout (side-to-side tilt, orthogonal axis). Accel-only + gentle LP --
    // not controlled, just a display value for when the servo legs level the body.
    float roll_acc = atan2f((float)ax,(float)az)*57.2958f;
    static float rollF=0; static bool rollInit=false;
    if(!rollInit){ rollF=roll_acc; rollInit=true; }
    rollF = 0.9f*rollF + 0.1f*roll_acc; tRoll = rollF;
    // yaw heading: integrate bias-corrected gyro-Z (drifts slowly; no magnetometer on ICM-42670)
    float yaw_rate = (float)gz / GYRO_LSB_PER_DPS - gYawBias;
    if(fabsf(yaw_rate) > 0.5f){                                  // deadband suppresses idle bias creep
      float y = tYaw + yaw_rate*dt;
      while(y > 180.0f) y -= 360.0f; while(y < -180.0f) y += 360.0f;
      tYaw = y;
    }
    // yaw-RATE control: drive the wheel differential (gTurn) to track the commanded rate
    // gYawRateCmd (rad/s). Closed loop on the gyro => a hard rate cap (full stick = +/-2 rad/s)
    // instead of an open-loop torque that spins ever faster. Zeroed when disabled.
    static float gTurnF = 0.0f;
    if(gEnabled){
      float yawErr = gYawRateCmd - gYawFb * yaw_rate * POL_DEG2RAD;  // rad/s error (gYawFb fixes gz sign)
      float rawTurn = clampf(gYawFF*gYawRateCmd + gYawKp*yawErr, -gTurnMax, gTurnMax); // FF (firm) + FB (trim)
      gTurnF = gTurnLP*gTurnF + (1.0f-gTurnLP)*rawTurn;             // light low-pass smooths residual jitter
      gTurn = gTurnF;
    } else { gTurn = 0.0f; gTurnF = 0.0f; }
    float pit_gyro = (float)gx / GYRO_LSB_PER_DPS - gGyroBias;   // lateral rate, deg/s
    if(!primed){ theta=pit_acc; primed=true; }
    dqf = 0.6f*dqf + 0.4f*pit_gyro;                              // dq for control rate-damping
    float pit_freq = 1.0f/(fabsf(pit_gyro) + 200.0f);           // rate-adaptive accel weight
    if(fabsf(pit_acc) < 100.0f && fabsf(pit_gyro) < 1000.0f)
      theta = pit_freq*pit_acc + (1.0f-pit_freq)*(theta + pit_gyro*dt);
    float rate = pit_gyro;

    // --- read ONE wheel this cycle, update odometry ---
    static int failL=0, failR=0;            // consecutive per-wheel read failures
    int p=0,v=0;
    bool ok = wheelRead(pollId,&p,&v);
    if(ok){ odomUpdate(pollId,p,v); if(pollId==LEFT_W) failL=0; else failR=0; }
    else { hzFail++; if(pollId==LEFT_W) failL++; else failR++; }
    // --- wheel-dropout guard: a wheel that stops answering (e.g. ID21 falling off
    //     the bus) leaves the robot on one wheel -> pivots and falls. Auto-disable
    //     + flag the offending ID instead of running away on contaminated data. ---
    if(gEnabled && (failL > 12 || failR > 12)){
      gEnabled = false; gWheelFault = (failL>12)? LEFT_W : RIGHT_W;
      char wb[72]; int wk=snprintf(wb,sizeof(wb),"# WHEEL FAULT id=%d off bus -> DISABLED\n",gWheelFault);
      emit(wb,wk);
    }
    if(gDumpRead){
      char b[200]; int k=snprintf(b,sizeof(b),"# rd id=%d ok=%d n=%d p=%d v=%d raw=",pollId,ok,gRawN,p,v);
      for(int i=0;i<gRawN && k<(int)sizeof(b)-4;i++) k+=snprintf(b+k,sizeof(b)-k,"%02X ",gRaw[i]);
      k+=snprintf(b+k,sizeof(b)-k,"\n"); emit(b,k); gDumpRead=false;
    }
    pollId = gPollLock ? (uint8_t)gPollLock : ((pollId==LEFT_W) ? RIGHT_W : LEFT_W);

    // while disabled, keep home = current position so 'enable' locks here
    if(!gEnabled) stable_pos = wheel_x;

    // --- CASCADED PID (factory architecture) ---
    // pos/vel errors RESHAPE the target lean; the pitch loop produces torque.
    float pitch = theta - gSetpoint;                 // our tilt, offset to factory's frame
    // --- factory control-input filter (R-1.1.3 FUN_400d529c): 0.95/0.05 LP + slew ---
    if(!fpInit){ pitchF = pitch; rateF = dqf; fpInit = true; }
    float pn = gCtrlLP*pitchF + (1.0f-gCtrlLP)*pitch;        // heavy low-pass on pitch
    if(fabsf(pn - pitchF) < gSlewDeg) pitchF = pn;           // slew clamp: <=gSlewDeg per cycle
    else pitchF += (pn > pitchF ? gSlewDeg : -gSlewDeg);
    rateF = gCtrlLP*rateF + (1.0f-gCtrlLP)*dqf;              // same LP on rate
    float qErr  = pitch - gImuZero;                  // for fall/telemetry (lean error from base)
    static bool  polInit=false;          // neural-policy episode state (reset on disable)
    static float polIpitch=0,polPrevX=0,polPrevFrame[7]={0},posErrInt=0,xvelF=0,xvelP=0,uF=0,gPosTargetEff=0,vdesS=0;
    // position-runaway cutout: in policy mode measure from the SLEWED target gPosTargetEff
    // (what the wheels actually track) -- a joystick that jumps gPosTarget far ahead slews
    // in gradually at posvmax, so driving never trips the guard; only a genuine track loss
    // (wheel_x lags the slewed target by >gMaxPosErr) does. Before policy init (fresh enable)
    // reference current position so the in-branch homing isn't locked out. Non-policy: enable home.
    float posRef = gPolMode ? (polInit ? gPosTargetEff : wheel_x) : stable_pos;
    bool fallen = (fabsf(qErr) > gFallDeg) || (fabsf(wheel_x - posRef) > gMaxPosErr);
    float u = 0.0f;
    if(gEnabled && !fallen && gPolMode){
      // ---- neural-policy control: PURE balancer + code-based position hold ----
      // Pitch sign flipped to the sim frame (MEASURED: forward lean -> th DECREASES,
      // but sim wants forward=+pitch). Position handled in CODE: bias the setpoint
      // toward gPosTarget so the balancer drives there; the policy's position obs
      // (x_err, x_int) are zeroed so it doesn't fight. Velocity obs kept for damping.
      float xvel=0.0f, wvel=0.0f, vdes=0.0f;
      if(!polInit){ polIpitch=0; polPrevX=wheel_x; gPosTarget=wheel_x; gPosTargetEff=wheel_x;
                    posErrInt=0; xvelF=0; xvelP=0; uF=0; vdesS=0; }
      else {
        float sdt = (dt>1e-4f?dt:0.004f);
        xvel = (wheel_x - polPrevX) / sdt;                   // m/s from position (exact units)
        wvel = xvel / POL_WHEEL_R;                            // wheel rad/s
        polPrevX = wheel_x;
        // raw differentiated-encoder vel is quantization-noisy (1024 cnt/rev). Two LP
        // filters: xvelF for the position-D term, xvelP for the policy's velocity obs.
        xvelF = gPosVelLP*xvelF + (1.0f-gPosVelLP)*xvel;
        xvelP = gPolVelLP*xvelP + (1.0f-gPolVelLP)*xvel;
        // TRAPEZOIDAL MOTION PROFILE on the effective target: accel-limited (gPosAmax) ramp
        // toward a velocity capped by BOTH gPosVmax and the braking-distance limit
        // sqrt(2*a*dist) -- so it decelerates in time to STOP exactly on gPosTarget instead of
        // coasting past and ringing. vdes (=vdesS) is the feedforward wheel velocity.
        float dist = gPosTarget - gPosTargetEff;
        float vstop = sqrtf(2.0f * gPosAmax * fabsf(dist));   // fastest speed we can still brake to rest by target
        float vcap = fminf(gPosVmax, vstop);
        float vcmd = (dist >= 0.0f) ? vcap : -vcap;
        float damax = gPosAmax * sdt;
        if(vcmd > vdesS + damax) vdesS += damax; else if(vcmd < vdesS - damax) vdesS -= damax; else vdesS = vcmd;
        gPosTargetEff += vdesS * sdt;
        if((dist >= 0.0f && gPosTargetEff > gPosTarget) ||    // never coast past the goal
           (dist <  0.0f && gPosTargetEff < gPosTarget)){ gPosTargetEff = gPosTarget; vdesS = 0.0f; }
        vdes = vdesS;                                          // profile feedforward velocity (m/s)
        tTgtEff = gPosTargetEff; tVdes = vdes;                 // telemetry (driving diagnostics)
      }
      float posErr = wheel_x - gPosTargetEff;                // track the SLEWED target
      // integrate only once the slew has SETTLED at the final target and we're near it ->
      // trims the steady-state offset at rest, never winds up during a move.
      if(polInit && gPosTargetEff == gPosTarget && fabsf(posErr) < gPosIntBand){
        float kiLim = (gPosHoldKi>0.01f) ? (gPosHoldMax/gPosHoldKi) : 1e6f;
        posErrInt = clampf(posErrInt + posErr*dt, -kiLim, kiLim);
      }
      // DAMPED PI+D position loop. Sign VERIFIED on robot: +gPosHoldK drives back
      // TOWARD gPosTarget (opposite sign ran away). I-term kills the steady-state
      // offset (a P-only loop parks short of target against a constant bias); D (on the
      // LOW-PASSED velocity) damps the approach without the encoder-noise shimmy.
      float biasDeg = clampf(gPosHoldK*posErr + gPosHoldKi*posErrInt + gPosHoldKd*(xvelF - vdes),
                             -gPosHoldMax, gPosHoldMax);
      float pr_pitch = (biasDeg - pitch) * POL_DEG2RAD;   // = -(theta-(setpoint+bias)) in rad
      float pr_rate  = -dqf * POL_DEG2RAD;                // pitch rate (rad/s), sim frame
      if(polInit) polIpitch = clampf(polIpitch + pr_pitch*dt, -1.0f, 1.0f);
      // feed the FILTERED velocity to the policy: the raw differentiated-encoder vel is
      // quantization-noisy (1024 cnt/rev), but the policy trained on a clean signal, so
      // smoothing it to ~clean kills the velocity-driven shimmy without retraining.
      (void)wvel;
      float frame[7]={pr_pitch,pr_rate,0.0f,xvelP,xvelP/POL_WHEEL_R,polIpitch,0.0f}; // x_err=0,x_int=0 (pos in code)
      if(!polInit){ for(int k=0;k<7;k++) polPrevFrame[k]=frame[k]; polInit=true; }
      float obs[POL_OBS];                            // frame_stack=2: [prev_frame, curr_frame]
      for(int k=0;k<7;k++){ obs[k]=polPrevFrame[k]; obs[7+k]=frame[k]; }
      for(int k=0;k<7;k++) polPrevFrame[k]=frame[k];
      float uraw = gPolSign * policyInfer(obs) * POL_VELMAX_RADS * POL_RAW_PER_RADS; // raw wheel velocity cmd
      uF = gPolULP*uF + (1.0f-gPolULP)*uraw;   // low-pass the policy command -> kills the cycle-to-cycle shimmy
      u = clampf(uF, -(float)gUMax, (float)gUMax);
    } else if(gEnabled && !fallen){
      // live gains -> structs
      pidPos.Kp=gKpPos; pidVel.Kp=gKpVel; pidVel.Ki=gKiVel; pidPit.Kp=gKpPit;
      // --- factory cascade: pos(P) -> vel target ; vel(PI) -> lean ; pitch(P) -> torque ---
      // position error in the home frame: ZERO at enable (stable_pos==wheel_x), so no
      // start kick regardless of the absolute odometer offset. Sign found empirically.
      pidPos.Des = gPosErrSign * (wheel_x - stable_pos);  pidPos.FB = 0.0f;  calPID(&pidPos);
      pidVel.Des = gVx + pidPos.U;      pidVel.FB = gPosSign * wheel_vx;  calPID(&pidVel);
      pidPit.Des = gImuZero + pidVel.U; pidPit.FB = pitchF;               calPID(&pidPit);
      float pitU = pidPit.U;
      float dqU  = clampf(-gKdPit * rateF, -gDqUMax, gDqUMax);   // pitch-rate damping (filtered rate)
      // factory anti-deadzone min-magnitude (±5) + stiction boost (±10 past ±20)
      if(pitU > 0.0f && pitU < 5.0f) pitU = 5.0f; else if(pitU < 0.0f && pitU > -5.0f) pitU = -5.0f;
      if(pitU > gFFband) pitU += gFF; else if(pitU < -gFFband) pitU -= gFF;
      u = gPolarity * (pitU + dqU);
      u = clampf(u, -(float)gUMax, (float)gUMax);
    } else {
      pidReset(&pidPos); pidReset(&pidVel); pidReset(&pidPit);   // clear PID state when idle
      polInit = false;                                           // re-init policy episode on next enable
      if(!gEnabled && gTestTor!=0) u = (float)gTestTor;          // stand-only debug
    }
    // Output: drive wheels only when actively balancing (or stand-test). When idle,
    // DISABLE wheel torque (reg 0x18=0) so the wheels go limp instead of holding pos-0.
    static int8_t ditherSign = 1;
    static bool wasDriving = false;
    static int torqAcc=0;                   // factory-style rate-limited torque accumulator
    bool driving = (gEnabled && !fallen) || (!gEnabled && gTestTor!=0);
    if(driving){
      if(!wasDriving){ wheelTorqueEnAll(1);                                      // enable both (broadcast, no collision)
                       torqAcc=0; }                                             // seed accumulator at 0
      float uout = u;
      if(gEnabled && !fallen && gDither>0){ uout += (float)(gDither*ditherSign); ditherSign = -ditherSign; }
      if(gPosMode){
        // factory-style: rate-limit + ACCUMULATE the wheel-driving (torque) field.
        // While the lean persists, the command ramps up (+/-gPosClamp/cycle) to a
        // large sustained push, ceilinged at +/-gTorLim. Seeded at 0 => cannot kick.
        int d = (int)(uout>=0 ? uout+0.5f : uout-0.5f);          // round(control)
        if(d>gPosClamp) d=gPosClamp; else if(d<-gPosClamp) d=-gPosClamp;        // +/-120/cycle ramp limit
        torqAcc += d;
        if(torqAcc>gTorLim) torqAcc=gTorLim; else if(torqAcc<-gTorLim) torqAcc=-gTorLim;
        float tn = clampf(gTurn, -gTurnMax, gTurnMax);
        wheelTorque((int16_t)(torqAcc + tn), (int16_t)(-torqAcc + tn)); // + yaw differential
      } else {
        float tn = clampf(gTurn, -gTurnMax, gTurnMax);
        wheelTorque((int16_t)(uout + tn), (int16_t)(-uout + tn));   // mirrored L:+u/R:-u + yaw differential
      }
    } else if(wasDriving){
      // Falling edge (disable / fall / Ctrl-C): the servos latch the last velocity
      // command and would COAST forever. Actively command velocity 0 (burst, in
      // case a frame is dropped) BEFORE releasing torque, so the wheels stop dead.
      for(int i=0;i<5;i++){ wheelTorque(0,0); }
      wheelTorqueEnAll(0);                                  // disable both (broadcast, no collision)
    }
    wasDriving = driving;

    tTheta=theta; tRate=dqf; tU=u; tWheelX=wheel_x; tWheelVx=wheel_vx;

    n++;
    // NOTE: legs are untorqued ONCE at startup (setup); we no longer write to them
    // mid-control. Repeated writes to the dead-encoder right leg (ID22) were a prime
    // suspect for knocking its bus-neighbor right wheel (ID21) offline under load.
    // pace the loop to the factory period so per-sample constants match in time
    uint32_t spent = micros() - now;                 // 'now' = cycle start (dt block)
    if(spent < (uint32_t)gLoopUs){
      uint32_t rem = (uint32_t)gLoopUs - spent;
      vTaskDelay(rem/1000 + 1);                       // tick = 1ms; +1 to cover remainder
    } else vTaskDelay(1);
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 4, 5);   // UART1 -> Raspberry Pi (RX=IO4, TX=IO5)
  Serial2.begin(1000000, SERIAL_8N1, 13, 14);
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); delay(3); }

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(200);
  imuWrite(REG_PWR_MGMT0, 0x0F);    // accel+gyro low-noise
  delay(100);

  Serial.printf("\n=== Rider balance FW (LQR) ===  IMU WHO_AM_I=0x%02X\n", imuRead(REG_WHO_AM_I));
  Serial.println("CASCADE PID: pos->vel->pitch + rate damp (factory R-1.1.3). Legs torque-OFF. DISABLED.");
  Serial.println("Cmds: en 1|0 | kppos kpvel kivel kppit kdpit izero <v> | kp(=kppit) kd(=kdpit) |");
  Serial.println("      umax dqmax iclamp ff ffband fall <v> | set cap vx psign pol home gcal | d get");

  xTaskCreatePinnedToCore(balanceTask, "balance", 4096, NULL, 12, NULL, 1);
}

// ---------------- command protocol (UART0 + UART1) ----------------
static void emit(const char* b, int k){ Serial.write((const uint8_t*)b,k); Serial1.write((const uint8_t*)b,k); }
static void ackState(){
  char b[210];
  int k = snprintf(b, sizeof(b),
    "# en=%d kppos=%.1f kpvel=%.3f kivel=%.3f kppit=%.1f kdpit=%.2f izero=%.1f set=%.2f vx=%.2f pol=%+.0f psign=%+.0f Umax=%d ff=%.0f ffb=%.0f clp=%.2f slew=%.1f posmode=%d torlim=%d fault=%d gbias=%.2f\n",
    gEnabled,gKpPos,gKpVel,gKiVel,gKpPit,gKdPit,gImuZero,gSetpoint,gVx,gPolarity,gPosSign,gUMax,gFF,gFFband,gCtrlLP,gSlewDeg,gPosMode,gTorLim,gWheelFault,gGyroBias);
  emit(b,k);
}
static void applyCmd(char* s){
  while(*s==' ') s++;
  char* sp=s; while(*sp && *sp!=' ') sp++;
  float v=0;
  if(*sp){ *sp=0; v=atof(sp+1); }
  if      (!strcmp(s,"en"))  { gEnabled=(v!=0.0f); gTurn=0.0f; gYawRateCmd=0.0f; if(gEnabled){ gWheelFault=0; gPosTarget=tWheelX; } }  // enable: clear fault + HOME position target to current spot (else a stale ptgt trips the pos fall-guard -> u=0); zero any turn
  else if (!strcmp(s,"d"))     gEnabled=false;
  else if (!strcmp(s,"kppos")) gKpPos=v;               // cascade gains
  else if (!strcmp(s,"kpvel")) gKpVel=v;
  else if (!strcmp(s,"kivel")) gKiVel=v;
  else if (!strcmp(s,"kppit")||!strcmp(s,"kp")) gKpPit=v;
  else if (!strcmp(s,"kdpit")||!strcmp(s,"kd")) gKdPit=v;
  else if (!strcmp(s,"izero")) gImuZero=v;
  else if (!strcmp(s,"dqmax")) gDqUMax=v;
  else if (!strcmp(s,"posmode")) gPosMode=(v!=0.0f);            // factory position-goal wheel cmd on/off
  else if (!strcmp(s,"torlim"))  gTorLim=(int)v;               // torque-limit field (position mode)
  else if (!strcmp(s,"posclamp"))gPosClamp=(int)v;             // per-cycle goal delta clamp
  else if (!strcmp(s,"loopus"))  gLoopUs=(int)v;               // control-loop period (us); 6000=~167Hz
  else if (!strcmp(s,"iclamp")) gVelIClamp=v;
  else if (!strcmp(s,"umax"))  gUMax=(int)v;
  else if (!strcmp(s,"set"))   gSetpoint=v;
  else if (!strcmp(s,"cap"))   gSetpoint=tTheta;
  else if (!strcmp(s,"vx"))    gVx=v;
  else if (!strcmp(s,"pol"))   gPolarity=(v<0.0f?-1.0f:1.0f);
  else if (!strcmp(s,"psign")) gPosSign=(v<0.0f?-1.0f:1.0f);
  else if (!strcmp(s,"pesign")) gPosErrSign=(v<0.0f?-1.0f:1.0f);  // flip position-loop correction sign
  else if (!strcmp(s,"home"))  stable_pos=tWheelX;     // re-home here
  else if (!strcmp(s,"rd")){ gDumpRead=true; return; } // dump next raw read frame
  else if (!strcmp(s,"wt")){ gTestTor=(int)v; }        // stand-only wheel torque test
  else if (!strcmp(s,"wmode")){ wheelSetReg(LEFT_W,0x11,(uint8_t)v); wheelSetReg(RIGHT_W,0x11,(uint8_t)v); } // wheel mode reg 0x11
  else if (!strcmp(s,"wten")){ wheelTorqueEnAll((uint8_t)v); }  // wheel torque enable reg 0x18 (broadcast)
  else if (!strcmp(s,"lock")){ gPollLock=(int)v; }     // poll only this wheel ID (0=alternate)
  else if (!strcmp(s,"vbatk"))  gVbatK=v;                        // trim battery divider ratio (default 3.0 per schematic)
  else if (!strcmp(s,"cfgdump")){ gCfgDumpId=(int)v; return; }  // dump servo config regs
  else if (!strcmp(s,"adcscan")){                               // probe ADC1 pins to find battery-sense divider
    const int pins[]={32,33,34,35,36,39};
    for(unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);i++){
      int raw=analogRead(pins[i]); float vp=raw/4095.0f*3.3f;
      char b[64]; int k=snprintf(b,sizeof(b),"# adc gpio%d raw=%d vpin=%.3f\n",pins[i],raw,vp);
      emit(b,k); delay(3);
    }
    return;
  }
  else if (!strcmp(s,"gcal")){ gDoGcal=true; return; } // recalibrate gyro bias (hold still)
  else if (!strcmp(s,"stepcap")){ if(v!=0.0f) gStepCmd=(int)v; gStepCap=true; return; } // wheel step-response capture
  else if (!strcmp(s,"polrun")) gPolMode=(v!=0.0f);   // neural-policy control on/off (additive; PID is default; 'pol' is gPolarity!)
  else if (!strcmp(s,"polsign")) gPolSign=(v<0.0f?-1.0f:1.0f);  // flip action->wheel sign (verify on robot)
  else if (!strcmp(s,"polulp"))  gPolULP=clampf(v,0.0f,0.95f);  // policy-output low-pass (0=off..0.95 heavy)
  else if (!strcmp(s,"polvlp"))  gPolVelLP=clampf(v,0.0f,0.97f);// policy velocity-OBS low-pass (kills quantization shimmy)
  else if (!strcmp(s,"poshold")) gPosHoldK=v;          // code position-hold P gain (deg/m); 0 = pure balance
  else if (!strcmp(s,"poshi"))   gPosHoldKi=v;         // code position-hold I gain (deg per m*s); kills offset
  else if (!strcmp(s,"poshd"))   gPosHoldKd=v;         // code position-hold D gain (deg per m/s)
  else if (!strcmp(s,"posmax"))  gPosHoldMax=v;        // max setpoint bias (deg)
  else if (!strcmp(s,"posiband")) gPosIntBand=v;       // integrate only within this dist of target (m)
  else if (!strcmp(s,"posvmax"))  gPosVmax=v;          // move speed cap (m/s) for the target slew
  else if (!strcmp(s,"posamax"))  gPosAmax=v;          // accel cap (m/s^2) on the drive-velocity ramp
  else if (!strcmp(s,"turnrate")) gYawRateCmd=v;       // commanded yaw rate (rad/s); 0=straight
  else if (!strcmp(s,"yawkp"))    gYawKp=v;            // yaw-rate FB trim gain (raw per rad/s)
  else if (!strcmp(s,"yawff"))    gYawFF=v;            // yaw-rate FEEDFORWARD (raw per rad/s cmd) -> turn firmness
  else if (!strcmp(s,"turnlp"))   gTurnLP=clampf(v,0.0f,0.95f); // low-pass on turn output
  else if (!strcmp(s,"yawfb"))    gYawFb=(v<0?-1.0f:1.0f);  // yaw-rate feedback sign (-1/+1)
  else if (!strcmp(s,"turnmax"))  gTurnMax=v;          // clamp on |turn differential| (raw)
  else if (!strcmp(s,"posvlp"))  gPosVelLP=clampf(v,0.0f,0.99f); // D-term velocity low-pass (0=off..0.99 heavy)
  else if (!strcmp(s,"ptgt"))    gPosTarget=v;         // position SETPOINT (m, wheel_x frame)
  else if (!strcmp(s,"ff"))    gFF=v;                  // friction feed-forward magnitude
  else if (!strcmp(s,"ffband")) gFFband=v;             // friction FF deadband
  else if (!strcmp(s,"cfa"))   gCfAlpha=clampf(v,0.9f,0.9999f); // compl-filter gyro weight
  else if (!strcmp(s,"clp"))   gCtrlLP=clampf(v,0.0f,0.999f);   // control-input low-pass (factory 0.95)
  else if (!strcmp(s,"slew"))  gSlewDeg=clampf(v,0.5f,90.0f);   // per-cycle pitch slew clamp (factory 5)
  else if (!strcmp(s,"fall"))  gFallDeg=clampf(v,10.0f,70.0f);  // tilt-error torque cutoff (deg)
  else if (!strcmp(s,"dither")) gDither=(int)v;                 // stiction dither amplitude (0=off)
  else if (!strcmp(s,"get")||!strcmp(s,"?")) {}
  else { char b[48]; int k=snprintf(b,sizeof(b),"# ? '%s'\n", s); emit(b,k); return; }
  ackState();
}
static void pump(Stream& in, char* buf, int& n){
  while(in.available()){
    char c=in.read();
    if(c=='\n'||c=='\r'){ if(n>0){ buf[n]=0; applyCmd(buf); n=0; } }
    else if(n<39) buf[n++]=c;
  }
}

void loop(){
  static uint32_t tp=0;
  static char b0[40]; static int n0=0;
  static char b1[40]; static int n1=0;
  pump(Serial,  b0, n0);
  pump(Serial1, b1, n1);
  if(millis()-tp >= 50){   // telemetry @ ~20 Hz
    tp=millis();
    static float vbatF=0; static bool vbInit=false;
    float vb = (analogReadMilliVolts(33)/1000.0f) * gVbatK;   // factory-calibrated pin mV * divider ratio
    if(!vbInit){ vbatF=vb; vbInit=true; }
    vbatF = 0.95f*vbatF + 0.05f*vb; tVbat=vbatF;              // heavy LP (battery is slow)
    int bpct = (int)clampf((tVbat-VBAT_EMPTY)/(VBAT_FULL-VBAT_EMPTY)*100.0f, 0.0f, 100.0f);
    char b[320];
    int k=snprintf(b,sizeof(b),
      "th=%.2f roll=%.2f yaw=%.1f rate=%.1f wx=%.3f ptgt=%.3f tgteff=%.3f vdes=%.2f wv=%.2f w1=%.1f w2=%.1f u=%.0f en=%d vbat=%.2f batt=%d kppit=%.0f kdpit=%.2f kppos=%.0f kpvel=%.3f set=%.2f izero=%.1f psign=%+.0f Umax=%d gbias=%.2f lhz=%.0f dtmax=%.1f rfail=%.0f polrun=%d poshold=%.1f\n",
      tTheta, tRoll, tYaw, tRate, tWheelX, gPosTarget, tTgtEff, tVdes, tWheelVx, wheel1_vel, wheel2_vel, tU, gEnabled, tVbat, bpct, gKpPit, gKdPit, gKpPos, gKpVel, gSetpoint, gImuZero, gPosSign, gUMax, gGyroBias, tLoopHz, tDtMaxMs, tReadFail, gPolMode, gPosHoldK);
    if(k > (int)sizeof(b)-1) k = sizeof(b)-1;   // clamp: snprintf returns intended len, not written
    emit(b,k);
  }
}
