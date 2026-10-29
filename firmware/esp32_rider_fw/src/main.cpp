// XGO Rider — self-balance firmware: the RL policy (ctrl 0) or the LQR (ctrl 1), chosen at runtime.
//
// The LQR is the Hover's (~/hover: robot.cc Tightest, plan_step, friction; imu.cc; hover_board.cc imu_task), with the
// Rider's measured numbers (sim/lqr/): one IMU loop (imuTask), the axle from the encoders corrected for the body's
// rocking, a driving plan, friction compensation; u is applied L:+u / R:-u, clamped to +/-gUMax.
//
//   IMU   : ICM-42670, I2C GPIO18(SDA)/19(SCL) addr 0x69, read only by imuTask.
//   Wheels: UART2 servo bus GPIO13(RX)/14(TX) @1Mbps, IDs 11(L)/21(R).
//           torque  = SYNC_WRITE reg 0x1E ; read pos/vel = reg 0x24 (6 bytes).
//           odom: 10-bit, wrap thresh 800; RIGHT wheel mirrored (matches L:+u/R:-u).
//   Legs  : IDs 12/22, held at the chosen height while balancing (the leg state machine below).
//   Hosts : UART0 (USB-C) and UART1 (Pi, RX=IO4/TX=IO5) both carry cmd+telemetry.
//
// Build: ~/.xgo-cal/bin/pio run -d esp32_rider_fw ; flash firmware.bin @0x10000

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>       // NVS key/value store -- persists the controller choice (gCtrl)
#include "rider_policy.h"   // auto-generated PPO policy weights (sim/export_policy.py)
#ifndef POL_FRAME
#define POL_FRAME 7                 // inputs per frame: 7, or 8 with the turn input (headers from 2026-10-20 define it)
#endif
static_assert(POL_OBS == 2 * POL_FRAME && (POL_FRAME == 7 || POL_FRAME == 8), "policy frame: 2 frames of 7 or 8");
#include <Adafruit_NeoPixel.h>
// status LEDs: 4x WS2812B daisy-chained on ESP32 IO27 (5V) -- see Rider-Pi_SCH.pdf / xgo-cm4-pinout.md
#define LED_PIN 27
#define LED_N   4
// discrete single-color status LEDs (schematic: 3.3V -> R -> LED -> GPIO, i.e. active-low)
#define LED_RED  22   // D7 single red  -- on while fault / fallen
#define LED_BLUE 23   // D6 single blue -- on while balancing (enabled)
#define LED_ON   LOW  // active-low; flip these two if the bench shows it inverted
#define LED_OFF  HIGH
static Adafruit_NeoPixel gLeds(LED_N, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile bool gFallen = false;     // set by balanceTask: robot tipped past the fall angle
volatile int      gLedOvr   = -1;  // LED test override: -1=auto; 1=blue 2=green 3=amber 4=red 5=white ('led <n>')
volatile uint32_t gLedOvrMs = 0;   // when override was set (auto-clears after 20s so it can't mask status)
volatile int      gLedBright = 20; // global LED brightness 0..255 (scales status colors). low-battery amber overrides to full. live-tune 'ledbright'

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
volatile bool  gEnabled  = false;   // balance OFF until 'en 1'
volatile float gImuZero = 0.0f;     // base lean trim (deg); setpoint carries the offset
volatile float gSetpoint = 3.55f;   // balance tilt (deg); pitch = theta - set. 3.64->3.55 (2026-06-19, FLOOR):
                                    // bare-balance crept backward at 3.64; 3.55 = true no-creep balance point (0.0 mm/s
                                    // drift over 24s on the floor). Higher = rearward drive. 'set' to tune.
// ---- LQR (ctrl 1): the Hover's balance law (2026-10-28; ~/hover robot.cc), designed on a model fitted to the Rider's
// drops and floor sweeps (sim/lqr/). It replaced the hand-tuned June LQR (-500 / -6 / 32 / 3 on the raw encoder; git tag
// lqr-and-lqr2-2026-10-28 has both). The wheel servos read the wheel's turn against the body, so the axle is
// wheel_x - r*pitch and its speed the register + r*(forward lean rate) (MEASURED, tools/balance/imu_position_fit.py),
// smoothed over gLqrVTau. Friction compensation (the Hover's kFric*): the measured friction (sim/lqr/model.py: 32.5
// units) in the direction the law pushes, tanh-blended through zero, read from the command smoothed over 30 ms. Gains:
// sim/lqr/lqr_design.py, Bryson weights 15 mm, 0.03 m/s, 1 deg, 20 deg/s, 100 units, stable with the motors x1/x2 and a
// cycle more delay; the Hover's Tightest is 1936 / 8.26 / 53.8 / 1.25. Floor 2026-10-28 (2026-10-28_lqr2_hover_exact_
// watch.csv): standing pitch sd 0.02-0.09 deg, drives to 0.44 m/s, no falls. Its pitch comes from the Hover's IMU path
// (lqrImuStep). The first version (8 ms smoothing, -1844 / -6.89 / 46.0 / 3.67) shook at 14 Hz on the floor.
volatile float gLqrX    = -1916.0f; // position gain, per m ('lqrx')
volatile float gLqrVx   = -7.46f;   // speed gain, per scaled speed unit ('lqrvx')
volatile float gLqrQ    = 59.4f;    // pitch gain, per deg ('lqrq')
volatile float gLqrDq   = 1.39f;    // pitch-rate gain, per deg/s ('lqrdq')
volatile float gLqrUp   = 2.8f;     // the pitch reference, deg on its IMU path: the measured balance point (the Hover's
                                    // measured upright, imu_zero), drop tests + standing 10-28 ('lqrup')
volatile float gLqrVTau = 0.100f;   // speed smoothing time constant, s ('lqrvtau'; the Hover's 0.1)
volatile float gLqrFric = 32.5f;    // friction compensation, command units ('lqrfric'; 0 = off)
static const float LQR_FRIC_WIDTH = 20.0f;   // command units: the tanh blend through zero
static const float LQR_FRIC_TAU   = 0.030f;  // s: smoothing of the command whose sign sets the friction's direction
static const float LQR_WHEEL_R    = 0.030f;  // m
static const float LQR_SPEED_PER_MPS = 327.6f * 4.5f * (60.0f * 3.14159265f / 1024.0f);   // scaled units per m/s (271)
// The LQR drives by the Hover's plan (robot.cc plan_step): the planned speed closes on the stick's at up to gLqrAMax,
// its acceleration building over LQR_PLAN_TJERK; its position (the law's home) never gets more than LQR_PLAN_LEAD
// from the axle; the law tracks the plan's position, smoothed speed and lean (LQR_PLAN_LEAN_PER_ACC per m/s^2), the
// plan's command is fed forward (per m/s and per m/s^2), and the plan's own integral always runs. The lean and command
// per speed / acceleration are the Rider model's steady values (sim/lqr/drive_design.py feedforward()); the rest are
// the Hover's numbers. 'ptgt' distance moves do not drive the LQR (the stick does).
// ONE CHANGE FROM THE HOVER (Marc, 2026-10-28): the home is held. When the stick is released, the robot's position at
// that moment becomes the home; while released, the plan's target speed is LQR_HOME_K x (home - plan), at most
// LQR_HOME_VMAX: with the plan's own closing (gain 8) that is a critically damped approach, no overshoot, no step in
// the plan's speed, lean or command (a first version braked at the full rate and stopped the plan dead on arrival:
// its lean reference jumped ~10 deg in a cycle -- twitching, floor 10-28). The 15 cm lead then only keeps the plan
// near the robot; it no longer moves the home (the Hover's plan, at rest, sat wherever a push of > 15 cm left it).
static const float LQR_HOME_K    = 2.0f;      // 1/s: target speed per m from home (8 x 2 = 16 = (2 x 4)^2: critical)
static const float LQR_HOME_VMAX = 0.15f;     // m/s: the most the plan heads home at
volatile float gLqrAMax = 0.6f;               // plan acceleration limit, m/s^2 ('lqramax'). The Hover's 1.0 threw the
                                               // body ~14 deg at ~140 deg/s on a full-stick start (2026-10-28); 0.6: ~8.5 deg
static const float LQR_PLAN_TJERK = 0.12f;    // s for the acceleration to build up (Hover kPlanTJerk)
static const float LQR_PLAN_GAIN  = 8.0f;     // 1/s, how the plan's speed closes on the stick's (Hover kPlanGain)
static const float LQR_PLAN_LEAD  = 0.15f;    // m (Hover kPlanMaxLeadM)
static const float LQR_PLAN_KI    = 100.0f;   // command units per m s (Hover kPlanKi)
static const float LQR_PLAN_IMAX  = 100.0f;   // command units (Hover kPlanIMax)
static const float LQR_PLAN_LEAN_PER_ACC = 10.68f;  // deg per m/s^2 (Rider model; the Hover's 14.0)
static const float LQR_PLAN_U_PER_ACC    = 58.0f;   // command per m/s^2 (Rider model; the Hover's 47.2)
static const float LQR_PLAN_U_PER_SPEED  = 333.0f;  // command per m/s (Rider model; the Hover's 333.3)
// ---- neural-policy control. 'pol 1' to enable. ----
// SIM-TRAINED, NOT YET HW-VERIFIED. Obs/action signs almost certainly need an
// on-robot check (flip with 'polsign', and watch the obs match the sim's SI units).
// ---- controller selection: RUNTIME, one binary carries both (2026-10-18) ----
// Was a compile-time #ifdef split (env esp32 = policy, env esp32_lqr = LQR) so the wrong one
// could never be armed by accident. The runtime switch keeps that guarantee a different way:
// 'ctrl 0|1' is ACCEPTED ONLY WHILE DISABLED (en=0), and the active controller is reported in
// every telemetry line (ctrl=POL|LQR -> LCD + GUI).
// PERSISTED (2026-10-19): the choice is saved in the ESP32's NVS (namespace "rider", key "ctrl") on
// every accepted 'ctrl' command and read back in setup(), so it survives power cycles and flashes.
// A blank / unreadable NVS boots as CTRL_POL; a stored 2 (LQR2, the LQR's name 2026-10-28) is the LQR.
// (The NVS write briefly stalls the other core -- fine, 'ctrl' is only accepted while balancing is off.)
enum { CTRL_POL = 0, CTRL_LQR = 1 };
volatile int gCtrl = CTRL_POL;
static Preferences gPrefs;
static const char* ctrlName(){ return gCtrl == CTRL_LQR ? "LQR" : "POL"; }
static int ctrlFrom(int c){ return (c == 1 || c == 2) ? CTRL_LQR : CTRL_POL; }
volatile float gPolSign  = 1.0f;    // action->wheel-command sign (verify on robot)
volatile float gPolULP   = 0.0f;    // low-pass on the POLICY output cmd (0=off; adds command lag, can destabilize)
volatile float gPolFF    = 0.0f;    // stiction feed-forward on the POLICY output (raw counts added past +/-gFFband;
                                    // 0=off). The factory boost's mechanism: keeps the
                                    // wheel out of the loaded dead zone (~35 raw) that drives the ~1 Hz hold
                                    // see-saw (ppo_fw_s9, floor 2026-09-16). Live-tune 'polff' (try 20..35).
volatile float gPolVelLP = 0.7f;    // low-pass on the velocity FED TO THE POLICY (kills quantization shimmy at the input)
// Code-based position hold: policy stays a PURE balancer (its position obs are
// neutralized); a simple outer loop biases the balance SETPOINT toward a target
// tick so the balancer drives there. 'poshold <deg/m>' (0=off), 'posmax <deg>'.
volatile float gPosHoldK   = 20.0f; // setpoint-bias P gain (deg per meter of position error); 0 = pure balance.
                                    // 5->20 cut hold drift ~4.5x (was -68mm/5s, now ~-15mm/5s); 30 oscillates.
                                    // Eliminated the post-drive overshoot (settles within 1-2mm). Robot 2026-06-16.
volatile float gPosHoldKi  = 1.0f;  // setpoint-bias I gain (deg per m*s); kills steady-state offset. 1.0 = the knee:
                                    // 2.5 wound up -> 0.18Hz hunt; 3.0 (tried 2026-06-16) also hunts the stopped hold.
                                    // 1.0 is stable and the residual drift is deadband-floored anyway. Robot 2026-06-17.
volatile float gPosHoldKd  = 10.0f; // setpoint-bias D gain (deg per m/s) -- HOLD/close-in damping ONLY.
                                    // 20 buzzed at 3.7Hz AFTER the loop-reorder staled the wheel-velocity it differentiates
                                    // (reorder freshened pitch at velocity's expense); 10 = clean hold (std 0.3deg). Robot 2026-06-17.
volatile float gDriveKd    = 10.0f; // velocity-DRIVE gain (deg per m/s): used while moving (stick drive or ptgt slew).
                                    // 10 has the phase margin for the ~50ms wheel-vel delay (28/20 rang while driving).
volatile float gPosHoldMax = 8.0f;  // max setpoint bias / braking authority (deg). 5->8 (2026-10-19) with the LQR-like
                                    // stick drive below: at 5 it pinned in reverse with the legs lowered (a 3 deg balance-
                                    // point shift + the reverse lean) and held 0.25 instead of 0.30 m/s (sim, drive_sim).
volatile float gPolLeash   = 0.25f; // POLICY stick drive (2026-10-19), 'polleash': >0 = LQR-LIKE -- the target ADVANCES at
                                    // the commanded speed and the position P keeps acting, the target kept within this
                                    // many m of the robot. Copes with a moved balance point (low legs) and the turn-time
                                    // pitch drift that made the old drive creep/stall/speed up (floor 2026-10-19).
                                    // 0 = the old drive: target latched to wheel_x every cycle, velocity-D only.
volatile float gPosIntBand = 0.15f; // only integrate within this dist of target (m) -> no windup during moves
volatile float gPosDeadband= 0.012f;// hold dead-band (m): when settled within this of target, STOP chasing the
                                    // exact center (P off, I frozen, D kept) so the loop can't limit-cycle against
                                    // wheel stiction -> robot sits STILL instead of hunting. 0=off. live-tune 'posdb'
volatile float gPosVmax    = 0.45f; // move speed cap (m/s), the most the stick may ask: ->0.45 (2026-10-28 later: at 0.60, starts
                                    // fell on a 34 % battery; sim/lqr/drive_design.py); 0.60 earlier that day (as the Hover).
                                    // The DS4 service starts full stick at 0.30. ->0.30 (2026-10-19, Marc; matches the
                                    // joystick's MAX_SPEED). ->0.25 (2026-06-19, LQR): allow
                                    // real drive speeds. The cap
                                    // only bounds the target ramp; no motion without a command. live-tune via 'posvmax'.
volatile float gPosVmaxBack= 0.45f; // BACKWARD speed cap (m/s). matched to forward (0.45 since 2026-10-28, 0.30 since 2026-10-19).
                                    // (the +setpoint lean + ppo_v_pure's forward cruise make backward the "against"
                                    // direction -> the balance policy saturates ~23% at full speed = the shimmy).
                                    // Matching the demand to the headroom (lower cap backward) keeps it out of the
                                    // rail. Drive-shaping fix for the cruise-bias asymmetry. live-tune via 'posvmaxb'
volatile float gPosAmax    = 0.4f;  // accel cap (m/s^2) on the drive-velocity ramp: gentle accel kills the lurch/oscillation.
volatile float gPosDmax    = 0.8f;  // BRAKING cap (m/s^2), 'posdmax' (2026-10-26): the ramp's rate while the speed falls
                                    // (stick release, flip, end of a ptgt move). It shared gPosAmax's 0.4 before, so a
                                    // release from 0.30 m/s took 0.75 s and ~11 cm of target travel (LQR stop 7.5 cm,
                                    // policy 18-22 on the floor). Sim: LQR 0.8 -> ~2-3 cm, swings 9-11 deg.
                                    // 0.8 -> 0.4 (2026-10-19): with the matched LQR speed reference, 0.8 made starts lean
                                    // 9-10 deg; 0.4 cut that by ~1/3 (floor). Applies to the policy's drive ramp too. 'posamax'
volatile float gTurn       = 0.0f;  // wheel differential (raw) ADDED to both wheel cmds -> yaw. Computed by the yaw-rate controller each loop.
volatile float gTurnMax    = 250.0f;// clamp on |gTurn| (raw)
volatile float gWheelDZ    = 5.0f;  // per-wheel anti-stiction floor (raw), applied DURING TURNS only: keeps
                                    // neither wheel in the stiction band so a turn can't go one-sided. live-tune 'wheeldz'
volatile float gYawRateCmd = 0.0f;  // commanded yaw rate (rad/s) from the stick; full stick = +/-2 rad/s. set via 'turnrate'
// DRIVE WATCHDOG (2026-10-20): 'dv' and 'turnrate' are heartbeat-fed like 'legrate' -- the DS4 service re-sends them
// every 0.5 s while it drives. Not refreshed for DRIVE_CMD_TIMEOUT_MS -> zeroed (the robot stops and holds). Before
// this, a controller that died mid-drive left 'dv 0.30' latched and the robot drove on into a wall.
#define DRIVE_CMD_TIMEOUT_MS 1000
volatile uint32_t gDriveMs = 0, gTurnMs = 0;   // when 'dv' / 'turnrate' last arrived
volatile uint32_t gDriveWdTrips = 0;            // times the watchdog zeroed a stale command (telemetry 'dwd')
volatile float gYawKp      = 40.0f; // wheel-differential (raw) per rad/s of yaw-rate error (FB trim; 120 oscillated)
volatile float gYawFF      = 75.0f; // FEEDFORWARD differential (raw) per rad/s commanded -> firm turn open-loop, low FB gain stays stable
volatile float gTurnLP     = 0.5f;  // low-pass on the turn output (0=off..0.95) -> smooths residual jitter
volatile float gYawFb      = -1.0f; // yaw-rate FEEDBACK sign (-1 = correct: gz reads opposite to the turn dir; +1 ran away). live-tune via 'yawfb'
// --- encoder heading-lock: while NOT turning, hold (wheel1_x - wheel2_x) at a target so the
// robot keeps its heading (a bump that rotates it moves the wheels off their individual targets
// -> corrected). Pure encoder-based -> no gyro yaw-bias dependence. 'hdghold 0' falls back to the
// old yaw-rate anti-spin. Gains need on-robot tuning (start conservative). ---
volatile bool  gHdgHold = true;     // heading-lock on/off ('hdghold'). ON: verified on-robot 2026-06-15 (holds heading; sign correct). gains gHdgKp/Kd may want tuning if it oscillates.
volatile float gHdgKp   = 1.0f;     // P: raw wheel-differential per count of (w1-w2) error ('hdgkp')
volatile float gHdgKd   = 2.0f;     // D: raw per (vel1-vel2) differential velocity ('hdgkd')
volatile float gHdgSign = 1.0f;     // correction sign -- flip if heading diverges ('hdgsign')
volatile float gPosVelLP   = 0.9f;  // low-pass on the velocity feeding the D term (0=off..→1 heavy); kills D-noise shimmy.
                                    // 0.8->0.9 (2026-06-17): smooths the coarse low-speed encoder velocity the drive loop
                                    // chases -> cut forward accel-zone roughness ~40% (1.77->1.04deg); 0.95 over-lagged.
volatile float gPosTarget  = 0.0f;  // position setpoint (m, wheel_x frame); anchored at enable, moved via 'ptgt'
volatile float gDriveVel   = 0.0f;  // velocity-drive command (m/s) from the stick; 0 = release. While !=0 the
                                    // position target is IGNORED, home tracks current pos (no windup), and the
                                    // gPosHoldKd loop tracks this speed. On release the home latches at the spot
                                    // you let go (-> stop quick + hold). 'ptgt' position moves still honored when 0.
static const float POL_DEG2RAD     = 0.017453292f;
static const float POL_WHEEL_R     = 0.03f;      // wheel radius (m) -> x_vel = wheel_omega*r
static const float POL_VELMAX_RADS = 30.0f;      // action=1 -> 30 rad/s (sim vel_max)
static const float POL_RAW_PER_RADS= 7.764f;     // raw wheel-cmd per rad/s (1/0.1288, measured)
volatile int   gUMax     = 400;     // output clamp (current limit). 1000 browned out ID21 (shared 8.4V
                                    // rail sags under high wheel current -> right wheel drops off the bus
                                    // -> latched runaway). 400 (Marc, 2026-06-18) = below the free-spin hold
                                    // (500), runaway guard for the first armed balance test. 'umax' to tune.
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
volatile float tPacc=0, tPraw=0;    // accel-pitch: comp-corrected vs raw (lever-arm comp verify)
// drivecap extras (2026-10-19, turning-shimmy hunt): what the POLICY is fed this cycle (0 under LQR), both
// wheels' FINAL commands (after the turn differential), and the right wheel's reported speed.
static float tPolPP=0, tPolPR=0, tPolXV=0, tPolPI=0, tPolBias=0, tCmdL=0, tCmdR=0; static int tVR=0;
static float tPlanA=0, tPlanLean=0;  // (2026-10-28) the LQR plan's acceleration (m/s^2) and lean (deg) -- 0 under the policy
volatile float tVest=0;             // velocity-observer estimate (vs raw wv) -- observer verify
volatile float tLqrInQ=0;           // pitch as the LQR sees it after the factory input filter ('lqrin')
volatile float tRoll=0;                  // lateral tilt (deg), GYRO-FUSED (low-lag) -- feeds leg leveling
volatile float tRollAcc=0;               // raw accel-only roll (deg), for verifying the fusion vs tRoll
volatile float tVbat=0;                  // smoothed pack voltage (V), read from GPIO33 divider
volatile float tYaw=0;                    // integrated heading (deg), gyro-Z only -> drifts slowly
volatile float tTgtEff=0, tVdes=0;        // slewed pos target + profile velocity (driving diagnostics)
volatile float tPoseX=0, tPoseY=0;        // world-frame dead-reckoned pose (m): fwd distance projected through heading (tYaw)
volatile bool  gOdoReset=true;            // 'odoreset' + each gyro-cal -> loop zeros pose & resyncs prev wheel_x (init true: clean first-loop sync)
volatile bool  gPosZeroReq=false;         // 'poszero' (controller button) -> loop re-zeros the distance frame WITHOUT dropping balance
// ---- leg height control (controller Triangle=extend / Cross=retract via 'legstep') ----
// Mirrored body raise/lower, clamped to the per-leg servo limits, serviced in-loop on an idle bus.
// Legs energize lazily on first 'legstep' (SAFE-ENERGIZE: seed goal 0x1E = present pos BEFORE torque
// 0x18=1, so it HOLDS, no snap) and go LIMP on a fall/disable. Feasible now that the dead-encoder
// ID22 (the old mid-control leg-write -> ID21-dropout cause) is replaced. See docs/servo_registers.md.
#define LEG_L_MIN 865
#define LEG_L_MAX 960              // lowered 975->960: lowest-body floor so the wheels can't rub the frame (bench 2026-06-25)
#define LEG_R_MIN 440              // raised 425->440: same floor, mirror leg (body down = R toward MIN, L toward MAX)
#define LEG_R_MAX 535
#define LEG_SPEED 100                      // leg move speed (reg 0x20); ~slow/gentle
#define LEG_NONE  (-32768)                 // sentinel: no target / leg inactive
#define LEG_SETTLE 3                        // control cycles to wait after torque-on before writing the goal:
                                            // the servo DROPS a goal write fired right after 0x18=1, so defer it.
volatile int  gLegStepReq=0;              // pending mirrored step (BOTH legs, body raise/lower); 'legstep'
volatile float    gLegRate   = 0.0f;      // 'legrate' body-height RATE (leg counts/s, + = up): the DS4 right stick.
volatile uint32_t gLegRateMs = 0;         //   Integrated every control cycle -> 1-count goal steps = smooth (a 10 Hz
#define LEG_RATE_TIMEOUT_MS 300           //   'legstep' of ~5 counts made a staircase). Heartbeat-fed: no 'legrate'
                                          //   for LEG_RATE_TIMEOUT_MS -> rate 0, so a lost link can't keep legs moving.
volatile int  gLegAbsL=LEG_NONE, gLegAbsR=LEG_NONE; // pending PER-LEG absolute target; 'legL'/'legR' (leg-leveling foundation)
static int    gLegTgtL=LEG_NONE, gLegTgtR=LEG_NONE; // target goal per leg (LEG_NONE = inactive/limp)
static bool   gLegEnerL=false,   gLegEnerR=false;   // leg torque on?
static int    gLegWroteL=0,      gLegWroteR=0;      // last goal written to the servo (skip redundant writes)
static int    gLegSetlL=0,       gLegSetlR=0;       // post-torque-on settle countdown
// VERIFIED ENERGIZE (2026-10-21): the three energize writes go out on separate visits (never back-to-back: a
// write sent into the previous servo's reply is lost -- the limp path spaces + verifies for the same reason),
// then torque, goal and speed are read back; a mismatch starts over. Seen: the left leg stayed limp after a
// balance-on. The speed is checked too: it reads 0 at servo power-up, and a lost speed write could leave a
// leg powered but not moving -- a later balance-on works because the servo keeps the value.
enum { LEG_EN_GOAL=0, LEG_EN_SPEED, LEG_EN_TORQUE, LEG_EN_VERIFY, LEG_EN_FAILED };
#define LEG_EN_TRIES 5                               // failed verifies before the leg is left alone (until next balance-on)
struct LegEn { uint8_t stage, tries; int sent; };    // energize progress; sent = goal written in LEG_EN_GOAL
static LegEn  gLegEnL={LEG_EN_GOAL,0,0}, gLegEnR={LEG_EN_GOAL,0,0};
volatile uint32_t tLegRetryL=0, tLegRetryR=0;        // energize retries since boot (telemetry legretry=L/R)
volatile uint32_t tLegFailL=0,  tLegFailR=0;         // energize give-ups since boot (telemetry legfail=L/R)
// ARRIVAL CHECK (2026-10-21): 1 s after a slow leg move (ride height / equalize) ends, each powered leg's
// position is read back. A leg more than LEG_ARRIVE_TOL counts off its target counts as stuck (telemetry
// legstkL/R) and its registers 0x18-0x27 go out as '# leg' (recorded in riderlog). Seen: the first
// balance-on after boot rolled the body 10 deg -- one leg rose, the other didn't -- with energize verified.
#define LEG_ARRIVE_TOL 25                  // on the stand a settled leg sits ~8-9 counts short (gravity on a P-only servo);
                                          // the 10 deg roll of the incident is ~50 counts of leg difference
#define LEG_ARRIVE_MS 2000                 // at speed 100 a leg was still ~12 counts short 1 s after a 55-count move
volatile uint32_t tLegStuckL=0, tLegStuckR=0;
static int    gLegStepL=0,       gLegStepR=0;       // per-leg accumulated pending step (shared 'legstep' distributed here)
static uint8_t gLegTurn=0;                           // alternate which leg gets the bus each cycle (NO same-cycle contention)
// ---- leg LEVELING: keep the body level laterally by a SAME-SIGN differential on the legs ----
// Height (legstep) is the common-mode (R+/L-); leveling is the orthogonal mode (R+c/L+c) -> tilts the
// body without changing average height. Negative feedback on the IMU roll (tRoll): c = -sign*Kp*(roll-set).
// 'level 1' captures the current leg height as the anchor and holds the body level around it; legstep then
// moves that anchor. Edge case (Marc): when one leg hits its servo limit, the leftover correction SPILLS to
// the other leg, so the body keeps leveling (at the cost of some height) instead of giving up. Legs have hard
// limits (0x06/0x08) -> safe on the stand, no wheels involved.
// Factory-matched law (decompiled XGO R-1.1.x roll-balance, FUN_400d4514): the factory ACCUMULATES the
// controller output into the body-roll command each cycle (out_c += output), clamps it, and drives the
// legs differentially. That accumulation is integral action -> it drives roll to ZERO. Our first try was
// proportional, which stalls at a steady-state offset (the "doesn't reach level" we saw). So: integrate
// the roll error into the leg differential, rate-limit the per-cycle step (keeps the leg loop below the
// body's ~2Hz lateral resonance so it can't self-excite), and hard-clamp the accumulator to the authority.
volatile bool  gLevelOn  = false;            // 'level 1/0' -- IMU-roll body leveling via differential legs
#define LEV_REF_HZ 165.0f                    // loop rate levki/levslew were tuned at; the step scales by dt*this
volatile float gLevelKi  = 0.14f;            // integral gain: leg-counts accumulated per (deg of roll error) per cycle ('levki'). 0.14 bench-tuned 2026-06-25 (gyro-fused roll): holds level, reaches zero, no ring; 0.18/slew0.4 showed slight oscillation
volatile float gLevelSet  = 0.0f;            // target body roll (deg); 0 = level ('levset')
volatile float gLevelSlew = 0.30f;           // per-cycle rate limit on the accumulator (counts/cycle) ('levslew'); keeps the leg loop below the body's ~2Hz roll resonance. 0.30 bench-tuned (with gyro-fused roll)
volatile int   gLevelMax = 50;               // max |differential| authority (counts) ('levmax')
volatile int   gLevelSign= -1;               // correction sign ('levsign'); -1 per geometry, flip if it tilts the wrong way
static float   gLevCmd   = 0.0f;             // ACCUMULATOR: leg differential command (counts); integrates roll error to reach level
static int     gLevAnchR=LEG_NONE, gLevAnchL=LEG_NONE; // leg height anchor, captured when the roll split starts
// ---- TURN LEAN (2026-10-29). Roll level ON: the body stays FLAT -- turning, or one wheel higher than the other.
// Roll level OFF: the body leans ALONG THE PULL, gravity plus the turn's centripetal pull: driving round a curve the
// pull tilts by atan(v w / g) (tPull, from the measured wheel speed and gyro turn rate, IMU section) and the body's
// roll is set to minus that -- leaning in -- as a feed-forward split on the legs (gLeanCpd counts per deg); no slope
// correction. First try (10-29, gain 1, cpd 5, cap 8 deg): the circles turned 1.9 rad/s, not the stick's 1, so the
// lean reached 7-8 deg, and the body rolled ~2.5x the lean asked (cpd nearer 2); with the body rolled the single-gyro
// pitch ran away (tiltRates) and it fell. Held OFF (gain 0) until cpd is measured.
volatile float gLeanGain = 0.0f;             // share of the full lean ('lean'; 0 = none, 1 = along the pull)
volatile float gLeanCpd  = 5.0f;             // leg differential counts per deg of body roll ('leancpd'). ESTIMATED:
                                             // the 10-21 incident's 10 deg of roll was ~50 counts; 10-29 says ~2
#define LEAN_TAU_S     0.10f                 // smoothing of the pull (s)
#define LEAN_SLEW_DPS  8.0f                  // the lean's rate limit (deg/s)
#define LEAN_MAX_DEG   4.0f                  // the lean's limit (deg); full stick at 0.45 m/s is ~2.5-4.5
volatile float tPull = 0.0f;                 // the pull's tilt (deg, roll's sign), telemetry 'pull'
volatile float tLean = 0.0f;                 // the lean the legs are given (deg of roll), telemetry 'lean'
// ---- leg state machine: balance-ON -> slew both legs to MID (slow); balance-OFF -> roll-level off +
// equalize legs (remove the roll differential, keep height) and HOLD torqued; leveling-OFF while balancing
// -> same equalize (return to neutral). 'Slowly' = a per-cycle slew on the leg command.
#define LEG_R_MID 480                              // ride height PINNED (was midpoint of MIN/MAX) so tightening the
#define LEG_L_MID 920                              // low-height floor doesn't shift the default balancing height
#define LEG_EQUAL_SUM (LEG_R_MID+LEG_L_MID)        // R+L when the two legs are equal length (differential = 0)
volatile float gLegHomeSlew = 0.3f;          // home/equalize slew (counts/cycle) = 'slowly' ('leghomeslew')
volatile bool  gLegLimpReq = false;          // 'leglimp' (STOP button) -> torque-OFF both legs and stay limp
volatile bool  gLegHomeReq = false;          // 'leghome' (DS4 R3) -> slew both legs back to the ride height (MID),
                                             // the same slow move balance-on does; works balancing or not
static bool    gLegLimped  = false;          // legs held limp (suppresses the balance-off equalize) until next balance-on
static bool    gLegHoming = false;           // slewing legs to a home/equalized destination, then hold
static float   gLegHomeR=0, gLegHomeL=0;     // slewing command (float for fractional slew)
static int     gLegDestR=0, gLegDestL=0;     // destination (mid, or equalized), counts
// --- drive-capture: high-rate commanded-vs-actual wheel velocity + pitch, WHILE balancing/driving ---
// (the sim can't model the real servo under load; this catches commanded(u) vs actual(encoder v) divergence)
#define DCAP_N 600                        // ~3.7s @ ~163Hz: enough for a full enable->balance->fall episode
volatile bool gDcArm=false;               // true = capturing (auto-armed on 'en 1', stopped on 'en 0' or full)
volatile bool gDcReady=false;             // capture stopped/full -> data HELD until 'logdump'
volatile bool gLogDump=false;             // 'logdump' -> loop() emits the held balance log over USB
static uint32_t dcT[DCAP_N]; static int16_t dcCmd[DCAP_N]; static int16_t dcV[DCAP_N];
static uint8_t  dcWid[DCAP_N]; static int16_t dcP[DCAP_N]; static int16_t dcR[DCAP_N]; static int dcIdx=0;
// extras (2026-10-19): final wheel cmds L/R, right-wheel speed, and the policy's inputs (0 under LQR)
static int16_t dcL[DCAP_N], dcRc[DCAP_N], dcVR[DCAP_N], dcPP[DCAP_N], dcPR[DCAP_N], dcXV[DCAP_N], dcPI[DCAP_N], dcB[DCAP_N];
// (2026-10-20) the accelerometer pitch and the yaw rate: the fused pitch's turn drift vs its accel reference
static int16_t dcPA[DCAP_N], dcYR[DCAP_N], dcYC[DCAP_N];   // + the turnrate command (the turn input), x1000
// (2026-10-28) the stick's speed command (mm/s) and the LQR plan's acceleration (x1000) and lean (deg x100):
// what a drive start was asked for (the start twitch: a full-stick start vs a gentle one looked alike without it)
static int16_t dcSt[DCAP_N], dcPlA[DCAP_N], dcPlL[DCAP_N];
// --- ring-down capture (2026-10-29, 'ringcap' / 'ringdump'): every cycle for RCAP_N cycles (~16 s), balancing or
// not -- a whole tap session without the drive capture's ~5 s download gaps. The lean, the gyro and both wheels'
// speed registers (the wheels against the legs): tools/capture/ring_capture.py, tools/balance/ring_down.py. 48 KB.
#define RCAP_N 4000
volatile bool gRcArm=false, gRcDump=false;
static uint32_t rcT[RCAP_N]; static int16_t rcP[RCAP_N], rcR[RCAP_N], rcV[RCAP_N], rcVR[RCAP_N]; static int rcIdx=0;

// ---- wheel FREQUENCY SWEEP (2026-10-19): identify the loaded wheel's true response on the floor ----
// While balancing (either controller), 'chirp 1' adds a known log-swept sine to the balance output (the forward
// wheel command), gChirpA raw cmd, gChirpF0 -> gChirpF1 Hz over gChirpDur s, faded in/out over 0.5 s. Because
// the injected signal is known, the wheel's response can be separated from the controller's (closed-loop
// recordings alone mix the two). Its own capture, every 2nd cycle, streamed with 'chirpdump' as '# dcapc' lines.
// Stops itself at the end, on balance-off or a fall, or with 'chirp 0'.
volatile float gChirpA = 60.0f, gChirpF0 = 1.0f, gChirpF1 = 10.0f, gChirpDur = 20.0f;
volatile bool  gChirpOn = false, gChirpDump = false;
static float   tChirpInj = 0.0f;                  // this cycle's injected value (raw cmd)
#define CHIRP_N 2400                               // >= 25 s at ~95 Hz (every 2nd cycle)
struct ChirpSample { uint32_t t; int16_t inj, cmdL, cmdR, vL, vR, wx, th, rate; };
static ChirpSample* ci = nullptr;                  // 48 KB on the HEAP, allocated by the first 'chirp 1' (static DRAM is full)
static int ciIdx = 0; static float ciWx0 = 0.0f;
volatile float gYawBias=0.0f;             // gyro-Z zero-rate bias (deg/s), from calibrateGyro
volatile float gVbatK=3.0f;              // GPIO33 divider ratio: pack = pin*K. Schematic R8=20K(top)/R7=10K(bot) => x3.0
// 2S pack (net VCC8.4V): 8.4V full, 6.0V empty -> linear % for display.
static const float VBAT_FULL=8.4f, VBAT_EMPTY=6.0f;
volatile float tLoopHz=0;                // measured control-loop rate
volatile uint32_t tReadFails=0;          // failed wheel reads since start-up (an event count, not a rate)
volatile uint32_t tRfL=0, tRfR=0, tRfFirstMs=0, tRfLastMs=0;   // per wheel + when (ms since boot) the first/last counted
#define READ_FAIL_GRACE_MS 5000           // a wheel counts once it has answered, or from this long after boot:
                                          // at power-up the wheels answer ~1 s late (~500 failed reads every boot)
volatile float tDtMaxMs=0;               // worst-case cycle period (ms) since last report (ground-truth stall detector)
volatile float tReadMaxMs=0, tWorkMaxMs=0; // INSTRUMENT: worst wheel-read time + total cycle work (ms) per report window
volatile float tPostMaxMs=0;             // INSTRUMENT: post-read work (control/policy + output) ms per window
volatile float tInferMaxMs=0;            // INSTRUMENT: policyInfer() time ms per window
volatile int   tRetDL=-1, tRetDR=-1;     // INSTRUMENT: wheel return-delay reg 0x07 (raw), read once at boot
volatile bool  gDumpRead=false;     // 'rd' -> task dumps next raw read frame
volatile int   gTestTor=0;          // 'wt <v>' -> direct wheel torque when DISABLED (stand test)
volatile int   gPollLock=0;         // 'lock <id>' -> poll only that wheel (0 = alternate)
volatile int   gCfgDumpId=0;        // 'cfgdump <id>' -> dump servo config regs 0x00-0x2F
volatile float gGyroBias=-1.19f;    // pitch gyro zero-rate bias (deg/s) -- BAKED CONSTANT (avg of measured boots -1.15/-1.16/-1.27). NOT re-measured at boot (boot cal sometimes sampled a moving robot -> bad bias -> wouldn't balance). 'gcal' still re-measures.
volatile float gCfAlpha=0.996f;     // compl-filter gyro weight PER SAMPLE @~333Hz (~1s accel trim).
// --- IMU lever-arm comp: the IMU sits ~120mm ABOVE the axle, so body rotation about the axle
// injects tangential (alpha*r) + centripetal (omega^2*r) accel that isn't gravity and corrupts
// the accel-tilt. Subtract it so atan2 sees only gravity -> the body's TRUE lean (IMU reading
// normalized to the axle/CoM). KIMU converts the m/s^2 lever terms to accel counts: (LSB/g)/g * r.
static const float KIMU = (2048.0f/9.81f)*0.12f;   // ICM-42670 +-16g => 2048 LSB/g; r=0.12m (120mm up)
volatile float gImuComp = -0.5f;    // lever-arm comp scale+sign (0=off; -0.5 verified on robot). 'imucomp'
// --- factory control-input filter (R-1.1.3 FUN_400d529c, Ghidra-confirmed) ---
// Between IMU fusion and the PID, the factory heavily low-passes pitch & rate
// (0.95/0.05) and slew-limits the smoothed pitch to 5 deg/cycle. We lacked this.
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
volatile float LP_VEL = 0.85f;                        // wheel-velocity feedback low-pass. 0.7->0.85 (2026-06-19, FLOOR):
                                                      // smoother wheel_vx pushed the lqrvx damping cliff higher (-6 vs -5)
                                                      // -> hold ~4.7mm. RUNTIME-TUNABLE via 'vlp'
                                                      // (higher = smoother wheel_vx -> less velocity-loop buzz, more lag)
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

// THE IMU LOOP (2026-10-28), like the Hover's imu_task (hover_board.cc: imu_read_once(); vTaskDelay(pdMS_TO_TICKS(2));
// repeat): the only code that reads the IMU. Each sample is stored (imuLatest) and run through the LQR's IMU path, the
// Hover's boards/common/imu.cc (lqrImuStep): the plain accelerometer angle (no lever correction), the rate
// smoothed 0.6/0.4 per sample, the complementary filter with weight 1/(|rate|+200) per sample -- its one change, the
// pitch rate from all three gyros (tiltRates, 2026-10-29: the Hover cannot roll, the Rider's legs can). Everything else
// (the balance task's shared estimate theta/dqf, the gyro calibration) takes the latest sample. Pinned to core 0
// (the Hover's is on its control core): on core 1 it would cut into the Rider's 2 ms-timeout servo reads.
// THE TILT RATES (2026-10-29). Pitch and roll are the accelerometer's two tilt angles, pitch = atan(ay/az) and
// roll = atan(ax/az) in the IMU chip's axes, so "down" in the chip's frame is g = (tan roll, tan pitch, 1). A body
// turning at w (the three gyros, the chip's axes: right-handed, as the accelerometer's) sees a fixed "down" turn
// as dg/dt = g x w; the angles' rates follow from that. Turning about the true vertical changes neither angle --
// the turn's cross-talk drops out. A single gyro per angle (pitch from gx, roll from -gy: the same signs, the
// roll's bench-found in June) is this with the body upright and not turning; turning, it was wrong by
// tan(pitch) x turn rate in roll and ~tan(roll) x turn rate in pitch -- MEASURED 10-29: turning in place at 1 rad/s
// the fused roll walked 3.5 deg/s; in a 1.9 rad/s circle with the body rolled ~5-9 deg (the turn lean) both pitch
// estimates climbed 3.5 -> 17 deg in 2 s and the LQR, balancing on the wrong pitch, fell.
static void tiltRates(float wx, float wy, float wz, float pitchDeg, float rollDeg, float* pitchRate, float* rollRate){
  float X = tanf(fmaxf(-60.0f, fminf(60.0f, rollDeg)) * 0.0174533f);     // capped: lying down, tan would explode
  float Y = tanf(fmaxf(-60.0f, fminf(60.0f, pitchDeg)) * 0.0174533f);
  float dX = Y * wz - wy, dY = wx - X * wz, dZ = X * wy - Y * wx;         // dg/dt = g x w, g = (X, Y, 1)
  *pitchRate = (dY - Y * dZ) / (1.0f + Y * Y);                            // d atan(Y)
  *rollRate  = (dX - X * dZ) / (1.0f + X * X);                            // d atan(X)
}

static float lqrPitch = 0.0f, lqrRate = 0.0f;   // the LQR's pitch (deg) and smoothed rate (deg/s), firmware frame
static uint32_t lqrImuUs = 0;
static void lqrImuStep(int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
  uint32_t now = micros();
  float dts = (now - lqrImuUs) * 1e-6f;
  float pit_gyro, roll_rate;                     // the pitch rate from all three gyros (roll: the balance task's)
  tiltRates((float)gx / GYRO_LSB_PER_DPS - gGyroBias, (float)gy / GYRO_LSB_PER_DPS,
            (float)gz / GYRO_LSB_PER_DPS - gYawBias, lqrPitch, tRoll, &pit_gyro, &roll_rate);
  float pit_acc = atanf((float)ay / (float)az) * 57.2958f;
  if(lqrImuUs == 0){ lqrPitch = pit_acc; lqrRate = pit_gyro; dts = 0.0f; }   // the first sample
  lqrImuUs = now;
  lqrRate = 0.6f * lqrRate + 0.4f * pit_gyro;
  float pit_freq = 1.0f / (fabsf(pit_gyro) + 200.0f);
  if(fabsf(pit_acc) < 100.0f && fabsf(pit_gyro) < 1000.0f)
    lqrPitch = pit_freq * pit_acc + (1.0f - pit_freq) * (lqrPitch + pit_gyro * dts);
}
static portMUX_TYPE gImuMux = portMUX_INITIALIZER_UNLOCKED;
static int16_t gImuLast[6];                 // the IMU loop's latest raw sample (ax ay az gx gy gz)
static volatile uint32_t gImuN = 0;         // samples taken so far
static void imuTask(void*){
  for(;;){
    int16_t a[6]; imuData(&a[0],&a[1],&a[2],&a[3],&a[4],&a[5]);
    portENTER_CRITICAL(&gImuMux); memcpy(gImuLast, a, sizeof(a)); gImuN++; portEXIT_CRITICAL(&gImuMux);
    lqrImuStep(a[1], a[2], a[3], a[4], a[5]);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
static void imuLatest(int16_t*ax,int16_t*ay,int16_t*az,int16_t*gx,int16_t*gy,int16_t*gz){
  while(gImuN == 0) vTaskDelay(1);          // until the IMU loop's first sample
  int16_t a[6]; portENTER_CRITICAL(&gImuMux); memcpy(a, gImuLast, sizeof(a)); portEXIT_CRITICAL(&gImuMux);
  *ax=a[0]; *ay=a[1]; *az=a[2]; *gx=a[3]; *gy=a[4]; *gz=a[5];
}

// ---------------- servo bus (all Serial2 access is in the balance task) ------
// Drain the echo + status reply after a 1-byte write so a following write can't
// transmit into the previous servo's reply (half-duplex collision). Back-to-back
// un-drained writes here corrupted right-wheel ID21 (the 2nd write collided with
// ID11's reply) -> silent until power-cycle. Reproduced + fixed 2026-06-12.
static void busSettle(){
  Serial2.flush();                                  // wait for our TX to finish
  uint32_t t0=micros(), last=t0;
  while(micros()-t0 < 700){                          // hard cap 700us (startup/config writes)
    if(Serial2.available()){ Serial2.read(); last=micros(); }      // consume echo (+ any reply)
    else if(micros()-last > 80) break;               // bus idle >80us -> drained; leave early so the
  }                                                  // per-loop wheelTorque settle stays cheap (~0.3ms).
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
  Serial2.write(buf,18); busSettle();   // drain TX echo + leave bus idle -- WITHOUT this the next
                                        // loop's RIGHT_W (ID21) read collided with stragglers under
                                        // drive load -> ID21 dropped off the bus (failR>12 -> fault).
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
  Serial2.write(buf,12); busSettle();   // drain echo + bus idle (same ID21-dropout fix)
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
  Serial2.write(buf,18); busSettle();   // drain TX echo + leave bus idle -- WITHOUT this the next
                                        // loop's RIGHT_W (ID21) read collided with stragglers under
                                        // drive load -> ID21 dropped off the bus (failR>12 -> fault).
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
    // Stop as soon as a COMPLETE reply frame is in the buffer (echo 8 + reply LEN+4 bytes).
    // The wheel reply is LEN=0x0B -> 15 bytes, so echo+reply = 19; the old fixed `n >= 20`
    // threshold could never be met and EVERY read ran to the 2 ms timeout (2 reads/cycle =
    // 4.4 ms, measured rdms 4.36) -- the loop ran 165 Hz instead of the coded 333 Hz.
    // Found 2026-09-16 in the 34 ms loop-delay budget.
    bool done=false;
    for(int i=0;i+3 < n;i++){
      if(buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id && buf[i+3]!=0x04 && n >= i+buf[i+3]+4){ done=true; break; }
    }
    if(done) break;
  }
  memcpy(gRaw,buf,n); gRawN=n;   // stash for debug dump
  // Skip the half-duplex echo of our 8-byte request, then parse the reply
  // frame: FF FF id LEN err <params> chk. pos/vel sit at the frame tail.
  for(int i=0;i+5 < n;i++){
    if(buf[i]==0xFF && buf[i+1]==0xFF && buf[i+2]==id){
      int LEN = buf[i+3];
      if(LEN==0x04) continue;                 // this is the echoed request — skip
      if(i+LEN >= n) continue;                // frame incomplete
      int cks = i+LEN+3;                       // checksum = last byte of frame (id..last-param then ~sum)
      if(cks >= n) continue;                   // checksum byte not received yet
      uint8_t sum=0; for(int j=i+2;j<cks;j++) sum+=buf[j];   // sum id+LEN+err+params
      if((uint8_t)~sum != buf[cks]) continue;  // bad checksum -> ignore this frame (don't feed garbage to control)
      int p = buf[i+LEN-3] | (buf[i+LEN-2]<<8);
      int v = buf[i+LEN-1] | (buf[i+LEN]  <<8);
      if(v >= 0x8000) v -= 0x10000;
      *pos = p & 0x3FF; *vel = v; return true;
    }
  }
  return false;
}
// ---- leg servos (RC08P, IDs 12/22): 2-byte reg write + present-position read ----
// Legs use the verified RC08P map: goal 0x1E, speed 0x20, torque 0x18, hard limits 0x06/0x08.
static void legSetReg2(uint8_t id, uint8_t reg, uint16_t val){   // addressed 2-byte write (e.g. goal 0x1E)
  while(Serial2.available()) Serial2.read();
  uint8_t lo=val&0xFF, hi=(val>>8)&0xFF;
  uint8_t ck = ~(id + 0x05 + 0x03 + reg + lo + hi);
  uint8_t buf[9] = {0xFF,0xFF,id,0x05,0x03,reg,lo,hi,ck};
  Serial2.write(buf,9); busSettle();
}
// SYNC_WRITE the goal (reg 0x1E, 2 bytes) to BOTH leg servos in ONE broadcast packet, so they move
// SIMULTANEOUSLY -- no alternating-cycle stagger (~6ms apart) that made roll leveling jerky. Broadcast
// (0xFE) gets NO reply, so unlike two addressed writes there's no half-duplex collision to dodge.
static void legGoalSync(int16_t goalR, int16_t goalL){
  uint8_t buf[14];
  buf[0]=0xFF;buf[1]=0xFF;buf[2]=0xFE;buf[3]=4+3*2;buf[4]=0x83;buf[5]=0x1E;buf[6]=0x02;
  buf[7]=RIGHT_LEG; buf[8]=goalR&0xFF; buf[9]=(goalR>>8)&0xFF;
  buf[10]=LEFT_LEG; buf[11]=goalL&0xFF; buf[12]=(goalL>>8)&0xFF;
  uint8_t chk=0; for(int i=2;i<13;i++) chk+=buf[i]; buf[13]=~chk;
  while(Serial2.available()) Serial2.read();
  Serial2.write(buf,14); busSettle();
}
// Read 'len' bytes from leg register 'reg'. Legs reply with a STANDARD frame (unlike the wheels'
// non-standard one), so data sits right after ERR. Skips the half-duplex echo (LEN==0x04 = our request)
// and keeps reading until a COMPLETE, checksummed reply is in (or 2 ms): the reply trails the echo, and
// servoReadN does not fit legs: its byte-count exit can stop on the echo alone, and its wheel-frame guess
// (2nd data byte == 0x02) misreads a leg goal like 0x0217. Both failed the first verified-energize test.
static bool legRead(uint8_t id, uint8_t reg, uint8_t len, uint8_t* out){
  uint8_t ck = ~(id + 0x04 + 0x02 + reg + len);
  uint8_t req[8] = {0xFF,0xFF,id,0x04,0x02,reg,len, ck};
  while(Serial2.available()) Serial2.read();
  Serial2.write(req,8); Serial2.flush();
  uint8_t buf[48]; int n=0; uint32_t t0=micros();
  while(micros()-t0 < 2000 && n < 48){
    if(!Serial2.available()) continue;
    while(Serial2.available() && n < 48) buf[n++]=Serial2.read();
    for(int i=0;i+3 < n;i++){
      if(buf[i]!=0xFF || buf[i+1]!=0xFF || buf[i+2]!=id) continue;
      if(i+8 <= n && memcmp(buf+i, req, 8) == 0){ i += 7; continue; }   // our own echo (a 2-byte reply has its LEN 0x04 too)
      int LEN = buf[i+3];
      if(LEN != len+2) continue;               // not our reply
      int cks = i+LEN+3;
      if(cks >= n) break;                      // our reply, not all in yet -> read more
      uint8_t sum=0; for(int j=i+2;j<cks;j++) sum+=buf[j];
      if((uint8_t)~sum != buf[cks]) continue;  // bad checksum -> ignore
      for(int k=0;k<len;k++) out[k]=buf[i+5+k];
      return true;
    }
  }
  return false;
}
static bool legReadPos(uint8_t id, int* pos){   // present position (reg 0x24, 10-bit)
  uint8_t d[6];
  if(!legRead(id,0x24,6,d)) return false;
  *pos = (d[0] | (d[1]<<8)) & 0x3FF;
  return true;
}
// Drive ONE leg toward its target with energize + post-torque-on settle. Energize seeds the goal to the
// TARGET, then torque on -- the servo slews there at LEG_SPEED (gentle, no snap) -- one write per visit,
// then reads back torque + goal (VERIFIED ENERGIZE above). It does NOT read present position first: a
// dropped read on the busy bus used to leave the leg permanently limp (energize was gated on the read).
// reqAbs=LEG_NONE/reqStep=0 -> no new request (continue any deferred write). Pointers = this leg's state.
static void legService(uint8_t id, int* tgt, bool* ener, int* wrote, int* setl, LegEn* en, volatile uint32_t* retries,
                       volatile uint32_t* fails, int reqAbs, int reqStep, int lo, int hi){
  int nt = *tgt;
  if(reqAbs != LEG_NONE) nt = reqAbs;                              // absolute request (legL/legR)
  else if(reqStep != 0){                                          // relative request (legstep)
    int base = LEG_NONE;
    if(*ener) base = *tgt; else { int p; if(legReadPos(id,&p)) base = p; }
    if(base != LEG_NONE) nt = base + reqStep;
  }
  if(nt != LEG_NONE){ if(nt<lo) nt=lo; else if(nt>hi) nt=hi; }     // clamp to this leg's limits
  *tgt = nt;
  if(*tgt == LEG_NONE) return;                                    // leg inactive -> nothing on the bus
  if(!*ener){                                                     // ENERGIZE: one write per visit, then verify
    switch(en->stage){
      case LEG_EN_GOAL:   en->sent = *tgt; legSetReg2(id,0x1E,en->sent); en->stage = LEG_EN_SPEED;  break;  // goal = TARGET
      case LEG_EN_SPEED:  legSetReg2(id,0x20,LEG_SPEED);                 en->stage = LEG_EN_TORQUE; break;
      case LEG_EN_TORQUE: wheelSetReg(id,0x18,1);                        en->stage = LEG_EN_VERIFY; break;  // -> servo slews
      case LEG_EN_VERIFY: {
        uint8_t tq = 0, g[4] = {0,0,0,0};                          // goal 0x1E + speed 0x20 (power-up speed is 0)
        bool ok = legRead(id,0x18,1,&tq) && tq==1 && legRead(id,0x1E,4,g)
                  && (g[0] | (g[1]<<8)) == en->sent && (g[2] | (g[3]<<8)) == LEG_SPEED;
        if(ok){ *ener=true; *wrote=en->sent; *setl=LEG_SETTLE; en->stage=LEG_EN_GOAL; en->tries=0; }  // legGoalWriter
        else if(++en->tries >= LEG_EN_TRIES){ en->stage=LEG_EN_FAILED; (*fails)++; }                 // sends any newer *tgt
        else { (*retries)++; en->stage=LEG_EN_GOAL; }
      } break;
      default: break;                                             // LEG_EN_FAILED: off the bus until the next balance-on
    }
  } else if(*setl > 0){ (*setl)--; }
  // Goal UPDATES are no longer written here: legGoalWriter() sends both legs' targets in one rate-capped
  // broadcast (2026-10-18). Per-cycle addressed writes each drew a reply that collided on the shared bus
  // (wheel rfail rose during height moves) and left a leg servo deaf + holding until a power cycle.
}
// Send pending leg targets: ONE broadcast SYNC_WRITE for both legs (no reply -> nothing to collide), at most
// every LEG_WRITE_MS. A leg that is energized+settled but whose partner isn't falls back to one addressed write,
// under the same cap. The servo slews to each goal at LEG_SPEED, so a 25 Hz goal stream still moves smoothly.
#define LEG_WRITE_MS 40
// ARRIVAL CHECK for one leg (see LEG_ARRIVE_TOL): read registers 0x18-0x27 (torque, goal 0x1E, speed 0x20,
// present position 0x24 ...), report '# leg <id> ok|STUCK tgt= pos= regs=<hex>'; stuck -> count it.
static void legArrivalCheck(uint8_t id, bool ener, int tgt, volatile uint32_t* stuck){
  if(!ener || tgt == LEG_NONE) return;
  uint8_t r[16];
  char b[120]; int k;
  if(!legRead(id, 0x18, 16, r)){ k = snprintf(b, sizeof(b), "# leg %d noread tgt=%d\n", id, tgt); emit(b, k); return; }
  int pos = (r[0x24-0x18] | (r[0x25-0x18] << 8)) & 0x3FF;
  bool bad = abs(pos - tgt) > LEG_ARRIVE_TOL;
  if(bad) (*stuck)++;
  k = snprintf(b, sizeof(b), "# leg %d %s tgt=%d pos=%d regs=", id, bad ? "STUCK" : "ok", tgt, pos);
  for(int i = 0; i < 16; i++) k += snprintf(b + k, sizeof(b) - k, "%02X", r[i]);
  k += snprintf(b + k, sizeof(b) - k, "\n");
  emit(b, k);
}
static void legGoalWriter(){
  static uint32_t lastMs = 0;
  bool readyR = gLegEnerR && gLegSetlR==0 && gLegTgtR!=LEG_NONE;
  bool readyL = gLegEnerL && gLegSetlL==0 && gLegTgtL!=LEG_NONE;
  bool pendR = readyR && gLegTgtR != gLegWroteR;
  bool pendL = readyL && gLegTgtL != gLegWroteL;
  if(!(pendR || pendL) || millis() - lastMs < LEG_WRITE_MS) return;
  if(readyR && readyL){ legGoalSync((int16_t)gLegTgtR, (int16_t)gLegTgtL); gLegWroteR = gLegTgtR; gLegWroteL = gLegTgtL; }
  else if(pendR){ legSetReg2(RIGHT_LEG, 0x1E, gLegTgtR); gLegWroteR = gLegTgtR; }
  else          { legSetReg2(LEFT_LEG,  0x1E, gLegTgtL); gLegWroteL = gLegTgtL; }
  lastMs = millis();
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
    int16_t ax,ay,az,gx,gy,gz; imuLatest(&ax,&ay,&az,&gx,&gy,&gz);   // a new sample every ~2.4 ms, taken every 5
    sum  += (float)gx / GYRO_LSB_PER_DPS;
    sumz += (float)gz / GYRO_LSB_PER_DPS;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  gGyroBias = sum / N;
  gYawBias  = sumz / N;
  tYaw = 0.0f;                            // zero heading at each (re)calibration
  gOdoReset = true;                       // pose origin follows the heading origin
}

// ---------------- wheel odometry state ----------------
static float wheel1_x=0, wheel2_x=0;       // accumulated counts (L, R-mirrored)
static float wheel1_vel=0, wheel2_vel=0;   // filtered scaled velocity
static float wheel1_vr=0, wheel2_vr=0;     // the same, unfiltered (this reply's speed register) -- the LQR filters its own
static int   last11=-1, last21=-1;
static float wheel_x=0, wheel_vx=0;        // fused: meters, scaled vel
static float wheel_vr=0;                   // fused unfiltered scaled vel
static float gHdgTarget=0;                 // heading-lock target = (wheel1_x - wheel2_x) to hold

static void odomUpdate(uint8_t id, int pos, int vel){
  if(id==LEFT_W){
    if(last11>=0){
      int d = pos - last11;
      if(d < -800) wheel1_x += 1023 + d; else if(d > 800) wheel1_x += -1023 + d; else wheel1_x += d;
    }
    last11 = pos;
    wheel1_vr = 4.5f*(float)vel*K_V;
    wheel1_vel = LP_VEL*wheel1_vel + (1.0f-LP_VEL)*wheel1_vr;
  } else { // RIGHT_W — mirrored (matches L:+u / R:-u)
    if(last21>=0){
      int d = pos - last21;
      if(d < -800) wheel2_x -= 1023 + d; else if(d > 800) wheel2_x -= -1023 + d; else wheel2_x -= d;
    }
    last21 = pos;
    wheel2_vr = 4.5f*(float)(-vel)*K_V;
    wheel2_vel = LP_VEL*wheel2_vel + (1.0f-LP_VEL)*wheel2_vr;
  }
  wheel_x  = (wheel1_x + wheel2_x) / 2.0f / 1024.0f * WHEEL_CIRC_M;
  wheel_vx = (wheel1_vel + wheel2_vel) / 2.0f;
  wheel_vr = (wheel1_vr + wheel2_vr) / 2.0f;
}

// ---- neural-policy inference: 14-dim obs -> action in [-1,1] (deterministic).
//      z = clip((obs-mean)/sqrt(var+eps)); a = clip(W2 tanh(W1 tanh(W0 z+b0)+b1)+b2).
//      Validated against SB3 to 1.9e-7 in sim/export_policy.py. ~5k MACs, microseconds. ----
// policy weights copied to internal RAM at boot. The tables are read-only so the toolchain
// keeps them in flash (.rodata); the matmul then pays flash-access latency 5000x/cycle.
// Copying to .bss SRAM (written by memcpy, so the compiler can't fold it back to flash)
// makes the matmul read single-cycle SRAM. policyToRam() is called once in setup().
static float rW0[64][POL_OBS], rW1[64][64], rW2[1][64], rB0[64], rB1[64], rMEAN[POL_OBS], rVAR[POL_OBS];
static void policyToRam(){
  memcpy(rW0,POL_W0,sizeof rW0); memcpy(rW1,POL_W1,sizeof rW1); memcpy(rW2,POL_W2,sizeof rW2);
  memcpy(rB0,POL_B0,sizeof rB0); memcpy(rB1,POL_B1,sizeof rB1);
  memcpy(rMEAN,POL_MEAN,sizeof rMEAN); memcpy(rVAR,POL_VAR,sizeof rVAR);
}
// fast tanh: Pade[7/8] rational (~1e-7 vs libm tanhf; validated to 5e-5 on the policy action).
// libm tanhf measured ~20us/call x 128 = ~2.5ms/cycle on this ESP32 -- the control loop's #1 cost.
static inline float ftanh(float x){
  if(x > 4.97f) return 1.0f; else if(x < -4.97f) return -1.0f;
  float x2=x*x;
  float a=x*(135135.0f+x2*(17325.0f+x2*(378.0f+x2)));
  float b=135135.0f+x2*(62370.0f+x2*(3150.0f+x2*28.0f));
  return a/b;
}
static float policyInfer(const float* obs){
  float z[POL_OBS], h0[64], h1[64];
  for(int i=0;i<POL_OBS;i++){
    float v=(obs[i]-rMEAN[i])/sqrtf(rVAR[i]+POL_EPS);
    z[i]= v>POL_CLIP?POL_CLIP:(v<-POL_CLIP?-POL_CLIP:v);
  }
  for(int j=0;j<64;j++){ float s=rB0[j]; for(int i=0;i<POL_OBS;i++) s+=rW0[j][i]*z[i];  h0[j]=ftanh(s); }
  for(int j=0;j<64;j++){ float s=rB1[j]; for(int i=0;i<64;i++)     s+=rW1[j][i]*h0[i]; h1[j]=ftanh(s); }
  float a=POL_B2[0]; for(int i=0;i<64;i++) a+=rW2[0][i]*h1[i];
  return a>1.0f?1.0f:(a<-1.0f?-1.0f:a);
}

// Stick RELEASE (both controllers, 2026-10-21): keep the current drive speed and set the home where the
// braking ramp (gPosDmax) brings it to rest; the hold trapezoid then decelerates from that speed. It used to zero
// the speed and latch the home at the release spot -- a full-strength emergency stop, and a stick FLIP
// passes through the dead zone, so every full reversal began with one (floor: a 12.8 deg backward lurch
// at ride height, a fall at full leg height). Now a flip just ramps through zero into reverse.
static float releaseStop(float tgtEff, float v){   // -> the new home
  return tgtEff + ((v >= 0.0f) ? 1.0f : -1.0f) * v * v / (2.0f * gPosDmax);
}
// One step of the drive-speed ramp toward tgt: gPosDmax while the speed falls (braking), gPosAmax while it rises.
// A braking step never carries through zero: the speed stops at 0 for a cycle, then ramps up the other way.
static float driveRamp(float v, float tgt, float dt){
  bool braking = (v > 0.0f && tgt < v) || (v < 0.0f && tgt > v);
  float d = (braking ? gPosDmax : gPosAmax) * dt;
  float n = (tgt > v + d) ? v + d : (tgt < v - d) ? v - d : tgt;
  if(braking && ((v > 0.0f && n < 0.0f) || (v < 0.0f && n > 0.0f))) n = 0.0f;
  return n;
}

// ---------------- balance task (core 1) ----------------
static void balanceTask(void*){
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); vTaskDelay(2); }
  wheelTorqueEnAll(0);   // LIMP the wheels at boot (reg 0x18=0 after the coast-kill): FREE until 'en 1'
  // NOTE: wheel velocity-open-loop mode (reg 0x11=0) is "set and forget" -- it lives in the servo's
  // persistent EEPROM and nothing in normal operation touches it. Set it ONCE with 'wmode 0' (done
  // 2026-06-18) and it stays across reboots. We do NOT re-assert it at boot: that only guards against
  // someone deliberately re-flipping it (i.e. us during servo work), and a boot-time servo write before
  // the bus is settled is unreliable anyway (a flip-test proved it didn't take). See docs/servo_registers.md.
  calibrateGyro();                  // boot cal -> sets yaw bias + zeroes heading/pose (robot should be still)
  gGyroBias = -1.19f;               // ...but FORCE the pitch (balance) bias to the baked constant, so a
                                    // boot measurement on a moving robot can never break balance startup. 'gcal' re-measures both.
  float theta=0, dqf=0; bool primed=false;
  uint32_t lastUs = micros(); uint32_t n=0;
  uint32_t hzN=0, hzT=millis();             // loop-rate measurement
  float dtMaxAcc=0;                          // worst-case raw cycle period this report window
  float rdMaxAcc=0, workMaxAcc=0, postMaxAcc=0, inferMaxAcc=0; // INSTRUMENT: read + work + post-read + inference time this window
  for(;;){
    // --- on-demand gyro recal (only while disabled + still) ---
    if(gDoGcal && !gEnabled){ wheelTorque(0,0); calibrateGyro(); gDoGcal=false; primed=false; }
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
    if(millis()-hzT >= 250){ uint32_t el=millis()-hzT; tLoopHz=hzN*1000.0f/el; tDtMaxMs=dtMaxAcc*1000.0f; tReadMaxMs=rdMaxAcc; tWorkMaxMs=workMaxAcc; tPostMaxMs=postMaxAcc; tInferMaxMs=inferMaxAcc; dtMaxAcc=0; rdMaxAcc=0; workMaxAcc=0; postMaxAcc=0; inferMaxAcc=0; hzN=0; hzT=millis(); }

    // --- The IMU is read by the IMU loop alone (imuTask, every ~2.4 ms, as the Hover's); below, after the wheel
    //     reads, the shared estimate takes its latest sample (0-2.4 ms old). (2026-06-16 to 10-28 this task read the
    //     IMU itself just there, ~0.5 ms before the compute.) ---

    // --- read wheels + update odometry. Poll BOTH every cycle (was alternating, which
    //     half-staled the fused wheel_x and forced a heavy velocity LP -> ~tens of ms of
    //     phase lag in the balance loop = the shimmy). gPollLock still narrows to one
    //     wheel for bench diagnostics (stepcap). ---
    static int failL=0, failR=0;            // consecutive per-wheel read failures
    static bool upL=false, upR=false;       // wheel has answered at least once (start of its read-fail count)
    bool countAll = millis() > READ_FAIL_GRACE_MS;
    int p=0,v=0; uint8_t rdWid=LEFT_W; bool ok=false;
    bool pollBoth = (gPollLock==0);
    uint32_t trd0=micros();                 // INSTRUMENT: time the wheel-read block (both reads)
    if(pollBoth || gPollLock==LEFT_W){
      int pL=0,vL=0; bool okL=wheelRead(LEFT_W,&pL,&vL);
      if(okL){ odomUpdate(LEFT_W,pL,vL); failL=0; upL=true; } else { failL++; if(upL || countAll){ tReadFails++; tRfL++; tRfLastMs=millis(); if(!tRfFirstMs) tRfFirstMs=tRfLastMs; } }
      rdWid=LEFT_W; p=pL; v=vL; ok=okL;     // drivecap / dumpread sample = left wheel
    }
    if(pollBoth || gPollLock==RIGHT_W){
      int pR=0,vR=0; bool okR=wheelRead(RIGHT_W,&pR,&vR);
      if(okR){ odomUpdate(RIGHT_W,pR,vR); failR=0; tVR=vR; upR=true; } else { failR++; if(upR || countAll){ tReadFails++; tRfR++; tRfLastMs=millis(); if(!tRfFirstMs) tRfFirstMs=tRfLastMs; } }
      if(gPollLock==RIGHT_W){ rdWid=RIGHT_W; p=pR; v=vR; ok=okR; }
    }
    { float rdms=(micros()-trd0)*1e-3f; if(rdms>rdMaxAcc) rdMaxAcc=rdms; }   // INSTRUMENT: read-block ms

    // --- IMU -> tilt + rate (the IMU loop's latest sample -- see note above) ---
    int16_t ax,ay,az,gx,gy,gz; imuLatest(&ax,&ay,&az,&gx,&gy,&gz);   // the IMU loop's latest sample
    // ----- factory IMU pitch fusion + LEVER-ARM COMP (IMU ~120mm above axle) -----
    // Remove the IMU's own rotation-induced accel before atan2, so the accel-tilt reflects the
    // body's true lean (normalized to the axle/CoM), not the IMU swinging on its 120mm lever.
    float pcOmega = ((float)gx/GYRO_LSB_PER_DPS - gGyroBias) * 0.0174533f;  // pitch rate, rad/s
    static float pcOmPrev=0, pcAlphaF=0;
    float pcAlpha = (dt>1e-4f) ? (pcOmega - pcOmPrev)/dt : 0.0f;            // angular accel, rad/s^2
    pcAlphaF = 0.8f*pcAlphaF + 0.2f*pcAlpha;                               // filter the noisy derivative
    pcOmPrev = pcOmega;
    float ayC = (float)ay - gImuComp*KIMU*pcAlphaF;            // remove tangential (alpha*r)
    float azC = (float)az + gImuComp*KIMU*pcOmega*pcOmega;     // remove centripetal (omega^2*r)
    float pit_acc  = atan2f(ayC, azC)*57.2958f;                // forward/vertical -> deg (lever-arm corrected)
    float pit_acc_raw = atan2f((float)ay,(float)az)*57.2958f;  // uncorrected, for verify-against-raw
    tPacc = pit_acc; tPraw = pit_acc_raw;                      // expose both for the lever-arm-comp test
    // roll (side-to-side tilt): GYRO-FUSED, mirroring the pitch complementary filter below. Accel-only
    // + heavy LP lagged ~45ms (~30deg phase at 2Hz), which drove the leg-leveling loop into a ~2Hz
    // limit cycle. Both angles' rates come from all three gyros (tiltRates: turning, a single gyro per angle
    // reads the turn as tilt). The accelerometer's half: in a turn it feels the centripetal pull v x w on top of
    // gravity, so racc tilts by atan(v w / g) (MEASURED sign 2026-10-29, tools/balance/turn_pull.py: + wheel speed
    // x + turn rate -> racc +); removed before the fusion, so tRoll is the body's true roll in a turn too.
    // racc (tRollAcc) stays the raw accelerometer roll.
    float yaw_rate = (float)gz / GYRO_LSB_PER_DPS - gYawBias;       // deg/s, gz's sign (the heading below)
    float turnDps = gYawFb * yaw_rate;                               // deg/s, + = a + 'turnrate' command
    {
      static float pullF = 0.0f;
      float pull = atanf((wheel_vx / LQR_SPEED_PER_MPS) * turnDps * POL_DEG2RAD / 9.81f) * 57.2958f;
      pullF += (pull - pullF) * fminf(1.0f, dt / LEAN_TAU_S);
      tPull = pullF;
    }
    float roll_acc  = atan2f((float)ax,(float)az)*57.2958f;          // absolute roll ref (accel, laggy/noisy)
    static float rollC=0; static bool rollInit=false;
    if(!rollInit){ rollC=roll_acc; rollInit=true; }
    float pit_gyro, roll_gyro;                                       // the tilt rates, deg/s (the previous angles)
    tiltRates((float)gx/GYRO_LSB_PER_DPS - gGyroBias, (float)gy/GYRO_LSB_PER_DPS, yaw_rate, theta, rollC,
              &pit_gyro, &roll_gyro);
    float roll_freq = 1.0f/(fabsf(roll_gyro) + 200.0f);             // rate-adaptive accel weight (mirror pitch)
    if(fabsf(roll_acc) < 100.0f && fabsf(roll_gyro) < 1000.0f)
      rollC = roll_freq*(roll_acc - tPull) + (1.0f-roll_freq)*(rollC + roll_gyro*dt);
    tRollAcc = roll_acc; tRoll = rollC;
    // yaw heading: integrate bias-corrected gyro-Z (drifts slowly; no magnetometer on ICM-42670)
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
      float rawTurn;
      bool turning = (fabsf(gYawRateCmd) > 0.001f);
      if(turning || !gHdgHold){
        // active turn (stick) OR heading-lock off: yaw-RATE control (FF + gyro FB).
        float yawErr = gYawRateCmd - gYawFb * yaw_rate * POL_DEG2RAD; // rad/s error (gYawFb fixes gz sign)
        rawTurn = gYawFF*gYawRateCmd + gYawKp*yawErr;                 // FF (firm) + FB (trim)
        gHdgTarget = wheel1_x - wheel2_x;     // track heading while turning -> latches the NEW heading on release
      } else {
        // HEADING LOCK: PD on the wheel-encoder differential (w1-w2) -> hold heading. Pure encoder-based
        // (no gyro yaw-bias dependence). Verified on-robot 2026-06-15.
        float d     = wheel1_x   - wheel2_x;
        float dRate = wheel1_vel - wheel2_vel;
        rawTurn = gHdgSign * (gHdgKp*(gHdgTarget - d) - gHdgKd*dRate);
      }
      rawTurn = clampf(rawTurn, -gTurnMax, gTurnMax);
      gTurnF = gTurnLP*gTurnF + (1.0f-gTurnLP)*rawTurn;              // light low-pass smooths residual jitter
      gTurn = gTurnF;
    } else { gTurn = 0.0f; gTurnF = 0.0f; gHdgTarget = wheel1_x - wheel2_x; }  // disabled: keep target = current heading
    if(!primed){ theta=pit_acc; primed=true; }
    dqf = 0.6f*dqf + 0.4f*pit_gyro;                              // dq for control rate-damping
    float pit_freq = 1.0f/(fabsf(pit_gyro) + 200.0f);           // rate-adaptive accel weight
    if(fabsf(pit_acc) < 100.0f && fabsf(pit_gyro) < 1000.0f)
      theta = pit_freq*pit_acc + (1.0f-pit_freq)*(theta + pit_gyro*dt);
    float rate = pit_gyro;

    uint32_t tpost0=micros();               // INSTRUMENT: time post-read work (control/policy + output)
    // --- wheel-dropout guard: a wheel that stops answering (e.g. ID21 falling off
    //     the bus) leaves the robot on one wheel -> pivots and falls. Auto-disable
    //     + flag the offending ID instead of running away on contaminated data. ---
    if(gEnabled && (failL > 12 || failR > 12)){
      gEnabled = false; gWheelFault = (failL>12)? LEFT_W : RIGHT_W;
      char wb[72]; int wk=snprintf(wb,sizeof(wb),"# WHEEL FAULT id=%d off bus -> DISABLED\n",gWheelFault);
      emit(wb,wk);
    }
    if(gDumpRead){
      char b[200]; int k=snprintf(b,sizeof(b),"# rd id=%d ok=%d n=%d p=%d v=%d raw=",rdWid,ok,gRawN,p,v);
      for(int i=0;i<gRawN && k<(int)sizeof(b)-4;i++) k+=snprintf(b+k,sizeof(b)-k,"%02X ",gRaw[i]);
      k+=snprintf(b+k,sizeof(b)-k,"\n"); emit(b,k); gDumpRead=false;
    }

    float pitch = theta - gSetpoint;                 // tilt error fed to the controllers (deg)
    float qErr  = pitch - gImuZero;                  // for fall/telemetry (lean error from base)
    static bool  polInit=false;          // neural-policy episode state (reset on disable)
    static bool  lqrInit=false;          // LQR drive-state init on enable; declared here (not in the LQR block) so the runaway cutout's posRef can read it
    static float polIpitch=0,polPrevX=0,polPrevFrame[8]={0},posErrInt=0,xvelF=0,xvelP=0,uF=0,gPosTargetEff=0,vdesS=0;
    static bool  wasVelDrive=false;     // edge-detect velocity-drive -> release, to latch the home cleanly
    static bool  wasTurning=false;      // edge-detect turn start -> re-anchor fore/aft home to the spin center
    // position-runaway cutout: in policy mode measure from the SLEWED target gPosTargetEff
    // (what the wheels actually track) -- a joystick that jumps gPosTarget far ahead slews
    // in gradually at posvmax, so driving never trips the guard; only a genuine track loss
    // (wheel_x lags the slewed target by >gMaxPosErr) does. Before policy/LQR init (fresh enable)
    // reference current position so the in-branch homing isn't locked out.
    // Both controllers: runaway is measured from the slewed target while balancing; when NOT
    // balancing (fresh enable OR post-fall) reference wheel_x so the position term contributes 0 --
    // else a fall AFTER driving >gMaxPosErr from home latches 'fallen' forever (wheels are off
    // here anyway, so runaway protection is unaffected).
    bool ctrlInit = (gCtrl == CTRL_POL) ? polInit : lqrInit;
    float posRef = ctrlInit ? gPosTargetEff : wheel_x;
    bool fallen = (fabsf(qErr) > gFallDeg) || (fabsf(wheel_x - posRef) > gMaxPosErr);
    gFallen = fallen;                          // expose to the LED status (loop() reads it)
    if(gPosZeroReq){                           // re-zero the distance frame mid-balance: odometer + target + slew
      wheel1_x = 0; wheel2_x = 0; wheel_x = 0; //   are zeroed TOGETHER so posErr stays 0 -> no jolt, balance holds
      gHdgTarget = 0;
      gPosTarget = 0; gPosTargetEff = 0; posErrInt = 0; vdesS = 0; polPrevX = 0;
      lqrInit = false;                         // the LQR: the Hover's Reset home -- plan_start where the robot is, at its
                                               // speed (its plan kept the old position: the robot drove to it)
      gOdoReset = true;                        // dead-reckoned pose origin too
      gPosZeroReq = false;
    }
    // drive watchdog (see DRIVE_CMD_TIMEOUT_MS): a stale stick command is zeroed -> release -> the robot holds
    { uint32_t ms = millis();
      if(gDriveVel != 0.0f && ms - gDriveMs > DRIVE_CMD_TIMEOUT_MS){ gDriveVel = 0.0f; gDriveWdTrips++; }
      if(gYawRateCmd != 0.0f && ms - gTurnMs > DRIVE_CMD_TIMEOUT_MS){ gYawRateCmd = 0.0f; gDriveWdTrips++; } }
    float u = 0.0f;
    if(gCtrl == CTRL_POL){
    // ===== POLICY controller =====
    if(gEnabled && !fallen){
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
        // TURN-START re-anchor: when a turn BEGINS, set the fore/aft home to the current
        // position (the spin center). The position loop stays ACTIVE through the turn (below),
        // so it holds that center -> bounds wheel path length -> robot spins ~in place (drift
        // canceled, the textbook nested balance+yaw+position architecture). And because the
        // target is the spin center (not a stale far point), there's no wrong-way homing after
        // the turn -- which was the 2.5-turn "homes opposite" bug.
        bool turning = (fabsf(gYawRateCmd) > 0.001f);
        // Only for a turn from a HOLD (stick drive idle): mid-drive the re-anchor is pointless and, in
        // the LQR, wipes the target lead that is driving the robot (stop-then-turn, 2026-10-18). The
        // hold integral is KEPT: it carries the policy's bias cancel (see the stick-drive branch).
        if(turning && !wasTurning && fabsf(gDriveVel) <= 0.001f){ gPosTargetEff = wheel_x; gPosTarget = releaseStop(wheel_x, vdesS); }   // still braking from a release -> keep braking
        wasTurning = turning;
        if(fabsf(gDriveVel) > 0.001f){
          // ---- VELOCITY DRIVE (stick held) ----------------------------------------
          // vdes ramps (accel-limited) to the stick's commanded speed. gPolLeash > 0 (default, 2026-10-19):
          // the target ADVANCES at vdes like the LQR's and the position P keeps pulling the robot to it (at
          // most gPolLeash behind/ahead), so speed errors and pitch offsets are corrected. gPolLeash 0 (the
          // old drive): the home tracks the current position, posErr ~0, only the velocity-D term drives.
          // Either way, the instant the stick releases the home latches at the current spot.
          float vtgt = (gDriveVel < -gPosVmaxBack) ? -gPosVmaxBack : gDriveVel;   // asymmetric: limit backward
          vdesS = driveRamp(vdesS, vtgt, sdt);
          // posErrInt is FROZEN here, not zeroed (2026-09-16): at rest the integrator tilts the
          // setpoint to cancel the policy's constant command bias (ppo_fw_s2: ~-50 raw -> hold
          // pitch 5.0 vs setpoint 3.55). Zeroing it on stick touch brought the bias back and ate
          // one direction's drive authority (s2: forward 'non-existent'; mc_l4a: weak reverse).
          // Held constant, the cancel carries into the drive. No windup: it only integrates with the
          // stick released (gDriveVel 0), so the drive's lead error never reaches it.
          if(gPolLeash > 0.0f){                              // LQR-like: advance the target at the commanded speed
            gPosTargetEff = clampf(gPosTargetEff + vdesS*sdt, wheel_x - gPolLeash, wheel_x + gPolLeash);
            gPosTarget = gPosTargetEff;
          } else { gPosTarget = wheel_x; gPosTargetEff = wheel_x; }
          wasVelDrive = true;
        } else {
          // ---- POSITION HOLD / COMMANDED MOVE (stick released, or 'ptgt' distance) -----
          // Trapezoidal profile to gPosTarget (= the release stop point, OR a commanded
          // ptgt move -- specific-distance moves are honored here). On the release edge the
          // speed is KEPT and the home set where the ramp brings it to rest (releaseStop).
          if(wasVelDrive){ gPosTarget = releaseStop(gPosTargetEff, vdesS); wasVelDrive = false; }
          float dist = gPosTarget - gPosTargetEff;
          float vstop = sqrtf(2.0f * gPosDmax * fabsf(dist));  // brake-distance speed cap -> stop ON target
          float vcap = fminf((dist >= 0.0f) ? gPosVmax : gPosVmaxBack, vstop);  // asymmetric: limit backward moves
          float vcmd = (dist >= 0.0f) ? vcap : -vcap;
          vdesS = driveRamp(vdesS, vcmd, sdt);
          gPosTargetEff += vdesS * sdt;
          if((dist >= 0.0f && gPosTargetEff > gPosTarget) ||   // never coast past the goal
             (dist <  0.0f && gPosTargetEff < gPosTarget)){ gPosTargetEff = gPosTarget; vdesS = 0.0f; }
        }
        vdes = vdesS;                                          // feedforward velocity (m/s)
        tTgtEff = gPosTargetEff; tVdes = vdes;                 // telemetry (driving diagnostics)
      }
      float posErr = wheel_x - gPosTargetEff;                // track the SLEWED target
      bool moving = (fabsf(gDriveVel) > 0.001f) || (gPosTargetEff != gPosTarget);
      // DEAD-BAND: when SETTLED (not moving) and within gPosDeadband of target, stop chasing
      // the exact center. A loop that always hunts dead-center limit-cycles against wheel
      // stiction (-> the ~3cm back-and-forth); dropping P inside the band lets it sit STILL.
      bool inBand = (!moving && gPosDeadband > 0.0f && fabsf(posErr) < gPosDeadband);
      // integrate only once the slew has SETTLED at the final target and we're near it -- but
      // NOT inside the dead-band, so the integral trim holds steady instead of slowly creeping.
      if(polInit && fabsf(gDriveVel) <= 0.001f && gPosTargetEff == gPosTarget && fabsf(posErr) < gPosIntBand && !inBand){
        float kiLim = (gPosHoldKi>0.01f) ? (gPosHoldMax/gPosHoldKi) : 1e6f;
        posErrInt = clampf(posErrInt + posErr*dt, -kiLim, kiLim);
      }
      // DAMPED PI+D position loop. Sign VERIFIED on robot: +gPosHoldK drives back TOWARD
      // gPosTarget. REGIME-SCHEDULED D gain: MOVING (stick drive / ptgt slew) uses the gentle
      // gDriveKd (velocity-tracking loop -- high rings); CLOSE-IN HOLD uses the high gPosHoldKd.
      // Inside the dead-band P is dropped (pErr=0) so it stops hunting; velocity-D stays (anti-drift).
      float kd = moving ? gDriveKd : gPosHoldKd;
      float pErr = inBand ? 0.0f : posErr;
      float biasDeg = clampf(gPosHoldK*pErr + gPosHoldKi*posErrInt + kd*(xvelF - vdes), -gPosHoldMax, gPosHoldMax);
      float pr_pitch = (biasDeg - pitch) * POL_DEG2RAD;   // = -(theta-(setpoint+bias)) in rad
      float pr_rate  = -dqf * POL_DEG2RAD;                // pitch rate (rad/s), sim frame
      if(polInit) polIpitch = clampf(polIpitch + pr_pitch*dt, -1.0f, 1.0f);
      tPolPP = pr_pitch; tPolPR = pr_rate; tPolXV = xvelP; tPolPI = polIpitch; tPolBias = biasDeg;   // drivecap
      // feed the FILTERED velocity to the policy: the raw differentiated-encoder vel is
      // quantization-noisy (1024 cnt/rev), but the policy trained on a clean signal, so
      // smoothing it to ~clean kills the velocity-driven shimmy without retraining.
      (void)wvel;
      uint32_t ti0=micros();                         // INSTRUMENT: time policy inference
      float uraw;
      {
        // the pure balancer feeds x_err=0, x_int=0 (slots 2, 6): position is handled in code above.
        // 8th input (POL_FRAME 8, sim --turn_obs, 2026-10-20): |turnrate command| -- lets the balancer tell a spin
        // from holding still.
        float frame[8]={pr_pitch,pr_rate,0.0f,xvelP,xvelP/POL_WHEEL_R,polIpitch,0.0f, fminf(2.0f, fabsf(gYawRateCmd))};
        const int nf = POL_FRAME;
        if(!polInit){ for(int k=0;k<8;k++) polPrevFrame[k]=frame[k]; polInit=true; }
        float obs[2*8];                                // frame_stack=2: [prev_frame, curr_frame]
        for(int k=0;k<nf;k++){ obs[k]=polPrevFrame[k]; obs[nf+k]=frame[k]; }
        for(int k=0;k<8;k++) polPrevFrame[k]=frame[k];
        uraw = gPolSign * policyInfer(obs) * POL_VELMAX_RADS * POL_RAW_PER_RADS; // raw wheel velocity cmd
      }
      { float ims=(micros()-ti0)*1e-3f; if(ims>inferMaxAcc) inferMaxAcc=ims; }   // INSTRUMENT
      uF = gPolULP*uF + (1.0f-gPolULP)*uraw;   // low-pass the policy command -> kills the cycle-to-cycle shimmy
      float uOut = uF;
      if(gPolFF > 0.0f){                        // stiction feed-forward: jump past the loaded dead zone ('polff')
        if(uOut > gFFband) uOut += gPolFF; else if(uOut < -gFFband) uOut -= gPolFF;
      }
      u = clampf(uOut, -(float)gUMax, (float)gUMax);
    } else {
      polInit = false;                                          // re-init policy episode on next enable
      if(!gEnabled && gTestTor!=0) u = (float)gTestTor;         // stand-only debug
    }
    } else {
    // ===== LQR (see gLqrX): the Hover's law (robot.cc: Tightest's form, plan_start / plan_step, friction), on the
    // Hover's IMU path (lqrPitch, lqrRate: lqrImuStep), in its order. Firmware frame: forward lean makes pitch and rate
    // NEGATIVE, so the axle is wheel_x - r*q and its speed the register - r*dq (sim/lqr/closed_loop.py, its copy).
    // (lqrInit is declared up top alongside polInit so the position-runaway cutout's posRef can read it.)
    static float lqrV=0, lqrULp=0; static bool lqrVInit=false;  // the axle's smoothed speed; the smoothed command (friction sign)
    static float planX=0, planV=0, planA=0, planVF=0, planI=0;  // the plan: position (axle), speed, acceleration, smoothed speed, integral
    static float lqrHome=0; static bool lqrStick=false;          // the held home (axle) and whether the stick was held
    if(gEnabled && !fallen){
      float odt = (dt>1e-4f ? dt : 0.004f);
      float q  = lqrPitch - gLqrUp;                                 // lqr_q: the pitch against the measured upright
      float dq = lqrRate;                                           // lqr_dq
      float vax = wheel_vr - LQR_WHEEL_R * dq * 0.0174533f * LQR_SPEED_PER_MPS;   // wheek_vx + v_corr (one smoothing of
      float av = (gLqrVTau > 0.0f) ? expf(-odt / gLqrVTau) : 0.0f;  // the sum = the Hover's two of the same time constant)
      if(!lqrVInit){ lqrV = vax; lqrULp = 0.0f; lqrVInit = true; }
      lqrV = av*lqrV + (1.0f-av)*vax;                               // v_axle, scaled units
      float xax = wheel_x - LQR_WHEEL_R * q * 0.0174533f;           // x_axle
      if(!lqrInit){                                                 // plan_start: where the robot is, at its speed
        planX = xax; planV = planVF = lqrV / LQR_SPEED_PER_MPS; planA = 0.0f; planI = 0.0f; lqrInit = true;
        lqrHome = xax; lqrStick = false;
      }
      float lqr_x = xax - planX;                                    // before the plan's step
      {                                                             // plan_step(stick, dt, x_axle, lqr_x)
        bool held = fabsf(gDriveVel) > 0.001f;
        if(lqrStick && !held) lqrHome = xax;                        // the stick released: hold this spot
        lqrStick = held;
        float stick = clampf(gDriveVel, -gPosVmaxBack, gPosVmax);
        if(!held) stick = clampf(LQR_HOME_K * (lqrHome - planX), -LQR_HOME_VMAX, LQR_HOME_VMAX);   // back home, gently
        float ades = clampf(LQR_PLAN_GAIN * (stick - planV), -gLqrAMax, gLqrAMax);
        float jerk = gLqrAMax / LQR_PLAN_TJERK * odt;
        planA += clampf(ades - planA, -jerk, jerk);
        planV += planA * odt;
        planX = clampf(planX + planV * odt, xax - LQR_PLAN_LEAD, xax + LQR_PLAN_LEAD);
        planVF = av * planVF + (1.0f - av) * planV;
        planI = clampf(planI + LQR_PLAN_KI * lqr_x * odt, -LQR_PLAN_IMAX, LQR_PLAN_IMAX);
      }
      float lean = -LQR_PLAN_LEAN_PER_ACC * planA;                  // plan_lean_deg, firmware frame
      float uPlan = LQR_PLAN_U_PER_SPEED * planV + LQR_PLAN_U_PER_ACC * planA;
      gPosTarget = gPosTargetEff = planX; vdesS = planV;            // the home, for the runaway cutout
      tTgtEff = planX; tVdes = planV; tVest = lqrV; tLqrInQ = q;    // telemetry
      tPlanA = planA; tPlanLean = lean;                             // drivecap
      float uraw = -(gLqrX*lqr_x + gLqrVx*(lqrV - planVF*LQR_SPEED_PER_MPS) + gLqrQ*(q - lean) + gLqrDq*dq) + planI + uPlan;
      float af = expf(-odt / LQR_FRIC_TAU);                         // friction, its direction from the smoothed command
      lqrULp = af*lqrULp + (1.0f-af)*uraw;
      uraw += gLqrFric * tanhf(lqrULp / LQR_FRIC_WIDTH);
      u = clampf(uraw, -(float)gUMax, (float)gUMax);
    } else {
      lqrInit = false; lqrVInit = false;                            // restart from where the robot is on the next enable
      tPlanA = tPlanLean = 0.0f;
      if(!gEnabled && gTestTor!=0) u = (float)gTestTor;          // stand-only debug
    }
    }  // end controller select (gCtrl)
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
      // frequency sweep injection (see gChirpA): log sweep, 0.5 s fade in/out, only while balancing
      {
        static float chT = 0.0f, chPh = 0.0f;
        float inj = 0.0f;
        if(gChirpOn){
          float sdt = (dt > 1e-4f) ? dt : 0.005f;
          chT += sdt;
          if(chT >= gChirpDur){ gChirpOn = false; }
          else {
            float f = gChirpF0 * powf(gChirpF1 / gChirpF0, chT / gChirpDur);
            chPh += 6.2831853f * f * sdt; if(chPh > 6.2831853f) chPh -= 6.2831853f;
            float env = fminf(1.0f, fminf(chT / 0.5f, (gChirpDur - chT) / 0.5f));
            inj = gChirpA * env * sinf(chPh);
          }
        }
        if(!gChirpOn){ chT = 0.0f; chPh = 0.0f; }
        tChirpInj = inj;
        uout += inj;
      }
      if(driving && gDither>0){ uout += (float)(gDither*ditherSign); ditherSign = -ditherSign; }  // dither in balance OR stand-test
      float tn = clampf(gTurn, -gTurnMax, gTurnMax);              // turn differential (the turn controller's output)
      if(gPosMode){
        // factory-style: rate-limit + ACCUMULATE the wheel-driving (torque) field.
        // While the lean persists, the command ramps up (+/-gPosClamp/cycle) to a
        // large sustained push, ceilinged at +/-gTorLim. Seeded at 0 => cannot kick.
        int d = (int)(uout>=0 ? uout+0.5f : uout-0.5f);          // round(control)
        if(d>gPosClamp) d=gPosClamp; else if(d<-gPosClamp) d=-gPosClamp;        // +/-120/cycle ramp limit
        torqAcc += d;
        if(torqAcc>gTorLim) torqAcc=gTorLim; else if(torqAcc<-gTorLim) torqAcc=-gTorLim;
        wheelTorque((int16_t)(torqAcc + tn), (int16_t)(-torqAcc + tn)); // + yaw differential
        tCmdL = torqAcc + tn; tCmdR = -torqAcc + tn;
      } else {
        float L = uout + tn, R = -uout + tn;                        // mirrored L:+u/R:-u + yaw differential
        // during a commanded turn, keep neither wheel in the stiction band -> both wheels
        // share the turn (no one-sided pivot). Gated on the turn so the tuned HOLD is untouched.
        if(fabsf(gYawRateCmd) > 0.001f && gWheelDZ > 0.0f){
          if(L > 0.0f && L < gWheelDZ) L = gWheelDZ; else if(L < 0.0f && L > -gWheelDZ) L = -gWheelDZ;
          if(R > 0.0f && R < gWheelDZ) R = gWheelDZ; else if(R < 0.0f && R > -gWheelDZ) R = -gWheelDZ;
        }
        wheelTorque((int16_t)L, (int16_t)R);
        tCmdL = L; tCmdR = R;
      }
    } else if(wasDriving){
      // Falling edge (disable / fall / Ctrl-C): the servos latch the last velocity
      // command and would COAST forever. Actively command velocity 0 (burst, in
      // case a frame is dropped) BEFORE releasing torque, so the wheels stop dead.
      for(int i=0;i<5;i++){ wheelTorque(0,0); }
      wheelTorqueEnAll(0);                                  // disable both (broadcast, no collision)
      // legs are NOT limped on balance-off -- the leg state machine below equalizes them (remove the roll
      // differential, keep height) and HOLDS torqued, so the robot sits even instead of keeping its last tilt.
    }
    wasDriving = driving;

    // ---- leg control (serviced HERE, on the IDLE bus after the wheel output stage -> avoids the
    // wheel-read collision that once dropped ID21; safe now that ID22 is replaced). Called every cycle
    // (cheap -- only touches the bus when a leg is active) so the deferred post-energize goal write lands.
    //   'legstep n'  -> BOTH legs mirrored (body raise/lower; extend = R+n / L-n)
    //   'legR pos' / 'legL pos' -> ONE leg to an absolute position (per-leg; leg-leveling foundation)
    {
      // Distribute a shared 'legstep' to per-leg pending (mirrored), then service exactly ONE leg
      // this cycle (alternating). Servicing BOTH in one cycle collided on the bus -> one leg stayed
      // unenergized (which one was timing-dependent). One-leg-per-cycle = ~83Hz/leg, plenty fast.
      // ===== leg state machine =====
      static bool drvPrev=false, levPrev=false;
      int hp;
      // STOP-button limp ('leglimp'): torque-OFF both legs and stay limp until the next balance-on.
      if(gLegLimpReq){
        // robust torque-OFF: SPACE the writes (the busy bus drops back-to-back writes -- the startup loops
        // space them too) and VERIFY via a reg 0x18 read-back; re-send until each leg confirms limp. Balance
        // is off when this fires (STOP sends 'en 0' first), so the brief per-leg stall is harmless.
        for(int leg=0; leg<2; leg++){
          uint8_t id = leg ? RIGHT_LEG : LEFT_LEG;
          for(int t=0; t<4; t++){
            legTorqueOff(id); vTaskDelay(2);                     // write, then let the bus settle
            uint8_t tq; if(legRead(id,0x18,1,&tq) && tq==0) break;      // confirmed limp -> next leg
          }
        }
        gLegTgtR=gLegTgtL=LEG_NONE; gLegEnerR=gLegEnerL=false;    // legs forget they were energized
        gLegEnR=gLegEnL=LegEn{LEG_EN_GOAL,0,0};                  // ...and any half-done energize
        gLegAbsR=gLegAbsL=LEG_NONE; gLegStepR=gLegStepL=0;        // clear PENDING per-leg targets/steps: a stale gLegAbs
                                                                 // left by the balance-off equalize would otherwise
                                                                 // re-energize that leg in the dispatch below (the
                                                                 // one-leg-not-limp bug -- only one leg serviced/cycle)
        gLegHoming=false; gLegStepReq=0; gLegLimped=true; gLegLimpReq=false;
      }
      if(driving && !drvPrev){ gLegLimped=false;                 // re-enabling balance un-limps the legs
        if(gLegEnR.stage==LEG_EN_FAILED) gLegEnR=LegEn{LEG_EN_GOAL,0,0};   // ...and gives a given-up leg fresh tries
        if(gLegEnL.stage==LEG_EN_FAILED) gLegEnL=LegEn{LEG_EN_GOAL,0,0}; }
      if(gLegLimped){ drvPrev=driving; levPrev=gLevelOn; gLegHomeReq=false; }  // stay limp: skip the machine (a 'leghome' while limp is dropped)
      else {
      // balance ON, or 'leghome' (DS4 R3) -> slew both legs to the ride height (MID), slowly. A balance-on
      // with both legs still powered (a pick-up / put-down, a fall) KEEPS the chosen height (2026-10-21):
      // the legs never went limp, so the height is known. Limp legs (power-up, STOP) still go to MID.
      bool keepHeight = gLegEnerR && gLegEnerL && gLegTgtR != LEG_NONE && gLegTgtL != LEG_NONE;
      if((driving && !drvPrev && !keepHeight) || gLegHomeReq){
        gLegHomeReq = false; gLegStepReq = 0;                     // a reset discards any height nudge still pending
        gLegHomeR = (gLegEnerR&&gLegTgtR!=LEG_NONE)?gLegTgtR:(legReadPos(RIGHT_LEG,&hp)?hp:LEG_R_MID);
        gLegHomeL = (gLegEnerL&&gLegTgtL!=LEG_NONE)?gLegTgtL:(legReadPos(LEFT_LEG,&hp)?hp:LEG_L_MID);
        gLegDestR = LEG_R_MID; gLegDestL = LEG_L_MID; gLegHoming = true;
      }
      // balance OFF -> roll-level off, then equalize (remove differential, keep height) + hold;
      // leveling OFF while still balancing -> same equalize (legs return to neutral, body un-tilts).
      bool equalize = (!driving && drvPrev) || (!gLevelOn && levPrev && driving);
      if(!driving && drvPrev) gLevelOn = false;
      if(equalize){
        int cr=(gLegTgtR!=LEG_NONE)?gLegTgtR:LEG_R_MID, cl=(gLegTgtL!=LEG_NONE)?gLegTgtL:LEG_L_MID;
        int diff=(cr+cl-LEG_EQUAL_SUM)/2;                         // the roll-differential component
        gLegHomeR=cr; gLegHomeL=cl; gLegDestR=cr-diff; gLegDestL=cl-diff; gLegHoming=true;
      }
      drvPrev=driving; levPrev=gLevelOn;

      // the roll split (the turn lean, + roll balancing's accumulator when on) runs whenever balancing, once homing
      // is done; it starts from the legs as they stand
      bool levRun = (!gLegHoming && driving);
      static bool levRunPrev=false;
      if(levRun && !levRunPrev){                                  // the split just started -> anchor at the (homed) pos
        if(gLegEnerR && gLegTgtR!=LEG_NONE) gLevAnchR=gLegTgtR; else if(legReadPos(RIGHT_LEG,&hp)) gLevAnchR=hp; else gLevAnchR=LEG_R_MID;
        if(gLegEnerL && gLegTgtL!=LEG_NONE) gLevAnchL=gLegTgtL; else if(legReadPos(LEFT_LEG,&hp)) gLevAnchL=hp; else gLevAnchL=LEG_L_MID;
        gLevCmd = 0.0f; tLean = 0.0f;
      }
      levRunPrev=levRun;
      {                                                           // smooth height: integrate 'legrate' into 1-count steps
        static float legRateAcc = 0.0f;
        if(millis() - gLegRateMs > LEG_RATE_TIMEOUT_MS) gLegRate = 0.0f;   // heartbeat lost -> stop
        if(gLegRate != 0.0f && !gLegHoming){
          legRateAcc += gLegRate * dt;
          int n = (int)legRateAcc;                                 // whole counts, sign kept
          if(n){ legRateAcc -= n; gLegStepReq += n; }              // same path as 'legstep' (honours leveling + limits)
        } else legRateAcc = 0.0f;
      }
      if(gLegStepReq){
        if(levRun){ gLevAnchR += gLegStepReq; gLevAnchL -= gLegStepReq; gLegStepReq=0; }      // move the level anchor (mirror)
        else if(!gLegHoming){ gLegStepR += gLegStepReq; gLegStepL -= gLegStepReq; gLegStepReq = 0; }  // (ignored mid-home)
      }
      if(gLegHoming){
        // slew the command toward the destination (mid / equalized), slowly; HOLD when arrived
        if(gLegHomeR<gLegDestR){ gLegHomeR+=gLegHomeSlew; if(gLegHomeR>gLegDestR)gLegHomeR=(float)gLegDestR; }
        else if(gLegHomeR>gLegDestR){ gLegHomeR-=gLegHomeSlew; if(gLegHomeR<gLegDestR)gLegHomeR=(float)gLegDestR; }
        if(gLegHomeL<gLegDestL){ gLegHomeL+=gLegHomeSlew; if(gLegHomeL>gLegDestL)gLegHomeL=(float)gLegDestL; }
        else if(gLegHomeL>gLegDestL){ gLegHomeL-=gLegHomeSlew; if(gLegHomeL<gLegDestL)gLegHomeL=(float)gLegDestL; }
        int tR=(int)lroundf(gLegHomeR); if(tR<LEG_R_MIN)tR=LEG_R_MIN; else if(tR>LEG_R_MAX)tR=LEG_R_MAX;
        int tL=(int)lroundf(gLegHomeL); if(tL<LEG_L_MIN)tL=LEG_L_MIN; else if(tL>LEG_L_MAX)tL=LEG_L_MAX;
        gLegAbsR=tR; gLegAbsL=tL;
        if((int)gLegHomeR==gLegDestR && (int)gLegHomeL==gLegDestL) gLegHoming=false;          // arrived -> legService holds
      } else if(levRun){
        // ACCUMULATOR (factory FUN_400d4514): integrate the roll error into the leg differential so roll
        // reaches ZERO (proportional stalls at an offset). Per-cycle step rate-limited (stays below ~2Hz
        // body resonance); accumulator hard-clamped to the authority. As roll->0 the step shrinks -> settles.
        // RATE-INDEPENDENT (2026-10-19): levki/levslew were tuned PER CYCLE at the ~165 Hz loop (June-Aug:
        // roll std ~0.06-0.5 deg). The Sept wheel-read fix raised the loop to ~234 Hz, so the same per-cycle
        // step integrated 42 % harder per second and leveling began rocking at ~1.2-1.5 Hz (std ~0.9 deg).
        // Scale the step by elapsed time so levki/levslew keep their 165 Hz meaning at ANY loop rate.
        // TURN LEAN (roll level off): the body's target roll is minus the pull, rate-limited; the legs get it as
        // feed-forward counts. The accumulator raises roll (-sign) per count, so L deg needs -sign * cpd * L counts.
        // Roll level on: the lean slews back to 0 and the accumulator holds the body flat.
        float leanT = gLevelOn ? 0.0f : clampf(-gLeanGain * tPull, -LEAN_MAX_DEG, LEAN_MAX_DEG);
        tLean += clampf(leanT - tLean, -LEAN_SLEW_DPS * dt, LEAN_SLEW_DPS * dt);
        float leanC = -(float)gLevelSign * gLeanCpd * tLean;
        if(gLevelOn){                                               // roll level: integrate toward levset (flat)
          float levK = dt * LEV_REF_HZ;                             // = 1 at the tuned 165 Hz
          float inc = gLevelSign * gLevelKi * (tRoll - gLevelSet) * levK;   // integration step this cycle
          float slewC = gLevelSlew * levK;
          if(inc >  slewC) inc =  slewC; else if(inc < -slewC) inc = -slewC;
          gLevCmd += inc;
          if(gLevCmd >  (float)gLevelMax) gLevCmd =  (float)gLevelMax; else if(gLevCmd < -(float)gLevelMax) gLevCmd = -(float)gLevelMax;
        } else gLevCmd = 0.0f;
        float cmd = clampf(gLevCmd + leanC, -(float)gLevelMax, (float)gLevelMax);
        // RESOLUTION: split the tilt between the legs at single-leg-count granularity. The old code put
        // the same +c on BOTH legs -> roll could only step in 2s. t = total tilt (counts); even t splits
        // evenly, odd t puts the extra count on ONE leg -> a tiny roll correction moves a single leg.
        int t = (int)lroundf(2.0f * cmd);
        int cR = t / 2, cL = t - cR;                             // |t|==1 -> one leg moves, the other holds
        int desR=gLevAnchR+cR, desL=gLevAnchL+cL;                // desired (pre-clamp) per-leg goals
        int tR=desR; if(tR<LEG_R_MIN) tR=LEG_R_MIN; else if(tR>LEG_R_MAX) tR=LEG_R_MAX;
        int tL=desL; if(tL<LEG_L_MIN) tL=LEG_L_MIN; else if(tL>LEG_L_MAX) tL=LEG_L_MAX;
        int spillR=desR-tR, spillL=desL-tL;                       // a clamped leg's leftover -> spill to the other
        tL+=spillR; if(tL<LEG_L_MIN) tL=LEG_L_MIN; else if(tL>LEG_L_MAX) tL=LEG_L_MAX;
        tR+=spillL; if(tR<LEG_R_MIN) tR=LEG_R_MIN; else if(tR>LEG_R_MAX) tR=LEG_R_MAX;
        // write BOTH legs in ONE sync packet (simultaneous = smooth) once both are energized+settled;
        // until then fall back to the per-leg dispatch (gLegAbs) for the lazy safe-energize.
        if(gLegEnerR && gLegEnerL && gLegSetlR==0 && gLegSetlL==0){
          gLegTgtR = tR; gLegTgtL = tL;                           // legGoalWriter() sends both in one capped broadcast
        } else {
          if(gLegTgtR==LEG_NONE || tR!=gLegTgtR) gLegAbsR=tR;
          if(gLegTgtL==LEG_NONE || tL!=gLegTgtL) gLegAbsL=tL;
        }
      }
      }  // end !gLegLimped
      gLegTurn ^= 1;
      if(gLegTurn){
        legService(RIGHT_LEG, &gLegTgtR, &gLegEnerR, &gLegWroteR, &gLegSetlR, &gLegEnR, &tLegRetryR, &tLegFailR, gLegAbsR, gLegStepR, LEG_R_MIN, LEG_R_MAX);
        gLegAbsR = LEG_NONE; gLegStepR = 0;
      } else {
        legService(LEFT_LEG,  &gLegTgtL, &gLegEnerL, &gLegWroteL, &gLegSetlL, &gLegEnL, &tLegRetryL, &tLegFailL, gLegAbsL, gLegStepL, LEG_L_MIN, LEG_L_MAX);
        gLegAbsL = LEG_NONE; gLegStepL = 0;
      }
      if(!gLegLimped) legGoalWriter();                            // the ONLY place leg goal updates go on the bus
      {                                                           // ARRIVAL CHECK: LEG_ARRIVE_MS after a slow move ends
        static bool homPrev = false; static uint32_t arriveMs = 0;
        if(homPrev && !gLegHoming && !gLegLimped) arriveMs = millis() | 1u;
        homPrev = gLegHoming;
        if(arriveMs && !gLegHoming && millis() - arriveMs > LEG_ARRIVE_MS){
          arriveMs = 0;
          legArrivalCheck(RIGHT_LEG, gLegEnerR, gLegTgtR, &tLegStuckR);
          legArrivalCheck(LEFT_LEG,  gLegEnerL, gLegTgtL, &tLegStuckL);
        }
      }
    }

    // dead-reckoned pose: project the change in fused forward distance through the
    // current heading (tYaw). gyro gives rotation, wheels give translation -- the right
    // split for a balancer (wheels slip during balance, but net forward travel is sound).
    static float odoPrevWx=0;
    if(gOdoReset){ tPoseX=0; tPoseY=0; odoPrevWx=wheel_x; gOdoReset=false; }
    float dWx=wheel_x-odoPrevWx; odoPrevWx=wheel_x;
    float yawRad=tYaw*POL_DEG2RAD;
    tPoseX+=dWx*cosf(yawRad); tPoseY+=dWx*sinf(yawRad);

    tTheta=theta; tRate=dqf; tU=u; tWheelX=wheel_x; tWheelVx=wheel_vx;

    // drive-capture sample (commanded u vs actual encoder v, per wheel, + pitch) at loop rate
    if(gDcArm && dcIdx<DCAP_N){
      dcT[dcIdx]=micros(); dcCmd[dcIdx]=(int16_t)u; dcV[dcIdx]=(int16_t)v;
      dcWid[dcIdx]=rdWid; dcP[dcIdx]=(int16_t)(theta*100.0f); dcR[dcIdx]=(int16_t)(rate*10.0f);
      bool pol = (gCtrl == CTRL_POL);
      dcL[dcIdx]=(int16_t)tCmdL; dcRc[dcIdx]=(int16_t)tCmdR; dcVR[dcIdx]=(int16_t)tVR;
      dcPP[dcIdx]=pol ? (int16_t)(tPolPP*57.2958f*100.0f) : 0;   // fed pitch, deg x100 (sim frame, incl. bias)
      dcPR[dcIdx]=pol ? (int16_t)(tPolPR*57.2958f*10.0f)  : 0;   // fed pitch rate, deg/s x10 (sim frame)
      dcXV[dcIdx]=pol ? (int16_t)(tPolXV*1000.0f)         : 0;   // fed velocity, mm/s
      dcPI[dcIdx]=pol ? (int16_t)(tPolPI*1000.0f)         : 0;   // pitch integral, mrad*s
      dcB[dcIdx] =pol ? (int16_t)(tPolBias*100.0f)        : 0;   // drive-loop bias, deg x100
      dcPA[dcIdx]=(int16_t)(pit_acc*100.0f);                     // accel pitch (lever-arm corrected), deg x100
      dcYR[dcIdx]=(int16_t)(yaw_rate*10.0f);                     // yaw rate, deg/s x10
      dcYC[dcIdx]=(int16_t)(gYawRateCmd*1000.0f);                // turnrate command (stick), x1000
      dcSt[dcIdx]=(int16_t)(gDriveVel*1000.0f);                  // stick speed command, mm/s
      dcPlA[dcIdx]=pol ? 0 : (int16_t)(tPlanA*1000.0f);          // LQR plan acceleration, x1000
      dcPlL[dcIdx]=pol ? 0 : (int16_t)(tPlanLean*100.0f);        // LQR plan lean, deg x100 (firmware frame)
      dcIdx++;
      if(dcIdx>=DCAP_N){ gDcArm=false; gDcReady=true; }
    }
    if(gRcArm && rcIdx<RCAP_N){                                    // ring-down capture, every cycle
      rcT[rcIdx]=micros(); rcP[rcIdx]=(int16_t)(theta*100.0f); rcR[rcIdx]=(int16_t)(rate*10.0f);
      rcV[rcIdx]=(int16_t)v; rcVR[rcIdx]=(int16_t)tVR;
      if(++rcIdx>=RCAP_N) gRcArm=false;
    }
    // frequency-sweep capture: every 2nd cycle while the sweep runs
    if(gChirpOn && !(gEnabled && !fallen)) gChirpOn = false;             // balance off / fall -> stop the sweep
    if(gChirpOn && ci && (n & 1) == 0 && ciIdx < CHIRP_N){
      ChirpSample& c = ci[ciIdx++];
      c.t = micros(); c.inj = (int16_t)tChirpInj; c.cmdL = (int16_t)tCmdL; c.cmdR = (int16_t)tCmdR;
      c.vL = (int16_t)v; c.vR = (int16_t)tVR;
      c.wx = (int16_t)((wheel_x - ciWx0) * 10000.0f);                          // 0.1 mm since the sweep began
      c.th = (int16_t)(theta * 100.0f); c.rate = (int16_t)(rate * 10.0f);
    }

    n++;
    // NOTE: legs are untorqued at startup and stay limp until a 'legstep' (Triangle/Cross)
    // energizes them; leg writes happen ONLY on a pending step, on the idle bus AFTER the wheel
    // output stage (above). The old "never write legs mid-control" rule existed because writing
    // the DEAD ID22 encoder knocked ID21 offline -- that servo is replaced now, so it's safe.
    // pace the loop to the factory period so per-sample constants match in time
    { float pms=(micros()-tpost0)*1e-3f; if(pms>postMaxAcc) postMaxAcc=pms; }   // INSTRUMENT: post-read work ms
    uint32_t spent = micros() - now;                 // 'now' = cycle start (dt block)
    { float wkms=spent*1e-3f; if(wkms>workMaxAcc) workMaxAcc=wkms; }   // INSTRUMENT: total cycle work ms
    if(spent < (uint32_t)gLoopUs){
      uint32_t rem = (uint32_t)gLoopUs - spent;
      vTaskDelay(rem/1000 + 1);                       // tick = 1ms; +1 to cover remainder
    } else vTaskDelay(1);
  }
}

void setup(){
  Serial.begin(115200);
  // restore the persisted controller choice before the control task starts (blank NVS -> POL)
  if(gPrefs.begin("rider", false)){
    uint8_t c = gPrefs.getUChar("ctrl", CTRL_POL);
    gCtrl = ctrlFrom(c);
  }
  Serial1.begin(115200, SERIAL_8N1, 4, 5);   // UART1 -> Raspberry Pi (RX=IO4, TX=IO5)
  Serial2.begin(1000000, SERIAL_8N1, 13, 14);
  for(int i=0;i<3;i++){ legTorqueOff(LEFT_LEG); legTorqueOff(RIGHT_LEG); wheelTorque(0,0); delay(3); }
  wheelTorqueEnAll(0);   // LIMP the wheels at boot (reg 0x18=0 after the coast-kill): FREE until 'en 1'

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(200);
  imuWrite(REG_PWR_MGMT0, 0x0F);    // accel+gyro low-noise
  delay(100);

  { uint8_t rb[2];                                   // INSTRUMENT: wheel return-delay reg 0x07
    tRetDL = servoReadN(LEFT_W,0x07,1,rb)  ? rb[0] : -1;
    tRetDR = servoReadN(RIGHT_W,0x07,1,rb) ? rb[0] : -1; }
  Serial.printf("\n=== Rider balance FW (LQR) ===  IMU WHO_AM_I=0x%02X  retDelay L=%d R=%d\n", imuRead(REG_WHO_AM_I), tRetDL, tRetDR);
  Serial.println("LQR full-state: u = -(lqrx*x + lqrvx*vx + lqrq*q + lqrdq*dq). Legs torque-OFF. DISABLED.");
  Serial.println("Cmds: en 1|0 | lqrx lqrvx lqrq lqrdq izero set <v> | umax ff ffband fall vlp <v> |");
  Serial.println("      driveff obsa obsb cap home gcal | d get");

  gLeds.begin(); gLeds.clear(); gLeds.show();       // status LEDs (IO27): off until first state update
  pinMode(LED_RED, OUTPUT);  digitalWrite(LED_RED,  LED_OFF);   // discrete red (fault) -- off
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, LED_OFF);   // discrete blue (heartbeat) -- off
  policyToRam();                                    // copy NN weights flash -> SRAM (fast matmul)
  xTaskCreatePinnedToCore(imuTask, "imu", 2048, NULL, 12, NULL, 0);   // the IMU loop (the Hover's imu_task)
  xTaskCreatePinnedToCore(balanceTask, "balance", 4096, NULL, 12, NULL, 1);
}

// ---------------- command protocol (UART0 + UART1) ----------------
static void emit(const char* b, int k){ Serial.write((const uint8_t*)b,k); Serial1.write((const uint8_t*)b,k); }
static void ackState(){
  char b[210];
  int k = snprintf(b, sizeof(b),
    "# en=%d lqrx=%.0f lqrvx=%.2f lqrq=%.1f lqrdq=%.2f lqrup=%.2f izero=%.1f set=%.2f Umax=%d ffb=%.0f posmode=%d torlim=%d fault=%d gbias=%.2f\n",
    gEnabled,gLqrX,gLqrVx,gLqrQ,gLqrDq,gLqrUp,gImuZero,gSetpoint,gUMax,gFFband,gPosMode,gTorLim,gWheelFault,gGyroBias);
  emit(b,k);
}
static void applyCmd(char* s){
  while(*s==' ') s++;
  char* sp=s; while(*sp && *sp!=' ') sp++;
  float v=0;
  if(*sp){ *sp=0; v=atof(sp+1); }
  if      (!strcmp(s,"en"))  { gEnabled=(v!=0.0f); gTurn=0.0f; gYawRateCmd=0.0f; gDriveVel=0.0f; if(gEnabled){ gWheelFault=0; wheel1_x=0; wheel2_x=0; wheel_x=0; gPosTarget=0.0f; gHdgTarget=0.0f; gOdoReset=true; dcIdx=0; gDcReady=false; gDcArm=true; } else { gDcArm=false; gDcReady=true; } }  // enable: clear fault + ZERO the position frame (wheel odometer + target + dead-reckoned pose) so current pos and target both start at 0; zero any turn
  else if (!strcmp(s,"d"))     gEnabled=false;
  else if (!strcmp(s,"izero")) gImuZero=v;
  else if (!strcmp(s,"dqmax")) gDqUMax=v;
  else if (!strcmp(s,"ctrl")){                                   // controller select: 0=POL (policy), 1=LQR
    if(gEnabled){ emit("# ctrl REFUSED: disable first (en 0)\n", 37); return; }   // never swap mid-balance
    gCtrl = ctrlFrom((int)v);
    if(gPrefs.getUChar("ctrl", 255) != (uint8_t)gCtrl) gPrefs.putUChar("ctrl", (uint8_t)gCtrl);  // persist (only on change)
    char cb[40]; int ck=snprintf(cb,sizeof(cb),"# ctrl=%s\n",ctrlName()); emit(cb,ck); return;
  }
  else if (!strcmp(s,"posmode")) gPosMode=(v!=0.0f);            // factory position-goal wheel cmd on/off
  else if (!strcmp(s,"torlim"))  gTorLim=(int)v;               // torque-limit field (position mode)
  else if (!strcmp(s,"posclamp"))gPosClamp=(int)v;             // per-cycle goal delta clamp
  else if (!strcmp(s,"loopus"))  gLoopUs=(int)v;               // control-loop period (us); 6000=~167Hz
  else if (!strcmp(s,"iclamp")) gVelIClamp=v;
  else if (!strcmp(s,"umax"))  gUMax=(int)v;
  else if (!strcmp(s,"set"))   gSetpoint=v;
  else if (!strcmp(s,"lqrx"))    gLqrX=v;                 // LQR gains (sim/lqr/lqr_design.py) ...
  else if (!strcmp(s,"lqrvx"))   gLqrVx=v;
  else if (!strcmp(s,"lqrq"))    gLqrQ=v;
  else if (!strcmp(s,"lqrdq"))   gLqrDq=v;
  else if (!strcmp(s,"lqrvtau")) gLqrVTau=clampf(v,0.0f,0.5f);   // ... its speed smoothing (s; 0 = none)
  else if (!strcmp(s,"lqrfric")) gLqrFric=v;               // ... friction compensation (units; 0 = off)
  else if (!strcmp(s,"lqramax")) gLqrAMax=clampf(v,0.1f,3.0f);   // ... its driving plan's acceleration (m/s^2)
  else if (!strcmp(s,"lqrup"))   gLqrUp=v;                 // ... and its pitch reference (deg)
  else if (!strcmp(s,"cap"))   gSetpoint=tTheta;
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
  else if (!strcmp(s,"drivecap")){ dcIdx=0; gDcReady=false; gDcArm=true; return; }       // arm high-rate cmd-vs-actual capture (run WHILE balancing/driving)
  else if (!strcmp(s,"chirp")){                                  // wheel frequency sweep: 1 = start (balancing only), 0 = stop
    if(v == 0.0f){ gChirpOn = false; return; }
    if(!gEnabled){ emit("# dcapc REFUSED: balance first\n", 31); return; }
    if(!ci) ci = (ChirpSample*)malloc(sizeof(ChirpSample) * CHIRP_N);
    if(!ci){ emit("# dcapc REFUSED: no memory\n", 27); return; }
    ciIdx = 0; ciWx0 = wheel_x; gChirpOn = true; return;
  }
  else if (!strcmp(s,"chirpa"))   gChirpA = clampf(v, 0.0f, 80.0f);      // sweep amplitude, raw wheel cmd
  else if (!strcmp(s,"chirpf0"))  gChirpF0 = clampf(v, 0.2f, 20.0f);     // sweep start frequency, Hz
  else if (!strcmp(s,"chirpf1"))  gChirpF1 = clampf(v, 0.2f, 20.0f);     // sweep end frequency, Hz
  else if (!strcmp(s,"chirpt"))   gChirpDur = clampf(v, 2.0f, 25.0f);    // sweep duration, s
  else if (!strcmp(s,"chirpdump")) gChirpDump = true;                    // stream the sweep capture ('# dcapc' lines)
  else if (!strcmp(s,"ringcap")){ rcIdx=0; gRcArm=true; return; }                         // ring-down capture: arm
  else if (!strcmp(s,"ringdump")){ gRcArm=false; gRcDump=true; return; }                   // ...stop and stream it
  else if (!strcmp(s,"logdump")){ gLogDump=true; return; }                                // dump the held enable->fall balance log over USB
  // ('polrun' removed: controller is selected at BUILD time now -- see gPolMode at top. No runtime toggle.)
  else if (!strcmp(s,"polsign")) gPolSign=(v<0.0f?-1.0f:1.0f);  // flip action->wheel sign (verify on robot)
  else if (!strcmp(s,"polff"))   gPolFF=clampf(v,0.0f,100.0f);  // policy-output stiction feed-forward (raw counts past +/-ffband; 0=off)
  else if (!strcmp(s,"polulp"))  gPolULP=clampf(v,0.0f,0.95f);  // policy-output low-pass (0=off..0.95 heavy)
  else if (!strcmp(s,"polvlp"))  gPolVelLP=clampf(v,0.0f,0.97f);// policy velocity-OBS low-pass (kills quantization shimmy)
  else if (!strcmp(s,"vlp"))     LP_VEL=clampf(v,0.0f,0.98f);   // wheel-velocity feedback low-pass (wheel buzz)
  else if (!strcmp(s,"poshold")) gPosHoldK=v;          // code position-hold P gain (deg/m); 0 = pure balance
  else if (!strcmp(s,"poshi"))   gPosHoldKi=v;         // code position-hold I gain (deg per m*s); kills offset
  else if (!strcmp(s,"poshd"))   gPosHoldKd=v;         // HOLD/close-in D gain (deg per m/s) -- high = crisp hold
  else if (!strcmp(s,"drivekd")) gDriveKd=v;           // DRIVE/moving D gain (deg per m/s) -- low = smooth drive
  else if (!strcmp(s,"posmax"))  gPosHoldMax=v;        // max setpoint bias (deg)
  else if (!strcmp(s,"polleash")) gPolLeash=clampf(v,0.0f,1.0f);   // policy stick drive: >0 LQR-like target lead (m), 0 = old
  else if (!strcmp(s,"posiband")) gPosIntBand=v;       // integrate only within this dist of target (m)
  else if (!strcmp(s,"posdb"))    gPosDeadband=v;      // hold dead-band (m): stop position-chasing within this -> no hunt
  else if (!strcmp(s,"posvmax"))  gPosVmax=v;          // move speed cap (m/s) for the target slew
  else if (!strcmp(s,"posvmaxb")) gPosVmaxBack=v;      // BACKWARD speed cap (m/s); lower = keep policy out of saturation
  else if (!strcmp(s,"posamax"))  gPosAmax=v;          // accel cap (m/s^2) on the drive-velocity ramp
  else if (!strcmp(s,"posdmax"))  gPosDmax=clampf(v,0.1f,3.0f);  // BRAKING cap (m/s^2) on the drive-velocity ramp
  else if (!strcmp(s,"turnrate")) { gYawRateCmd=v; gTurnMs=millis(); }  // commanded yaw rate (rad/s); 0=straight; heartbeat (watchdog)
  else if (!strcmp(s,"yawkp"))    gYawKp=v;            // yaw-rate FB trim gain (raw per rad/s)
  else if (!strcmp(s,"yawff"))    gYawFF=v;            // yaw-rate FEEDFORWARD (raw per rad/s cmd) -> turn firmness
  else if (!strcmp(s,"turnlp"))   gTurnLP=clampf(v,0.0f,0.95f); // low-pass on turn output
  else if (!strcmp(s,"yawfb"))    gYawFb=(v<0?-1.0f:1.0f);  // yaw-rate feedback sign (-1/+1)
  else if (!strcmp(s,"turnmax"))  gTurnMax=v;          // clamp on |turn differential| (raw)
  else if (!strcmp(s,"wheeldz"))  gWheelDZ=v;          // per-wheel anti-stiction floor during turns (raw)
  else if (!strcmp(s,"hdghold"))  gHdgHold=(v!=0.0f);  // encoder heading-lock on/off
  else if (!strcmp(s,"hdgkp"))    gHdgKp=v;            // heading-lock P (raw per count of w1-w2 error)
  else if (!strcmp(s,"hdgkd"))    gHdgKd=v;            // heading-lock D (raw per vel1-vel2)
  else if (!strcmp(s,"hdgsign"))  gHdgSign=(v<0?-1.0f:1.0f);  // heading-lock correction sign
  else if (!strcmp(s,"led"))     { gLedOvr=(v<0.5f)?-1:(int)v; gLedOvrMs=millis(); }  // LED: 0=auto 1=blu 2=grn 3=amb 4=red 5=wht 6=OFF
  else if (!strcmp(s,"ledbright")) gLedBright=(int)clampf(v,0.0f,255.0f);  // global LED brightness 0..255 (scales the status colors)
  else if (!strcmp(s,"posvlp"))  gPosVelLP=clampf(v,0.0f,0.99f); // D-term velocity low-pass (0=off..0.99 heavy)
  else if (!strcmp(s,"ptgt"))    { gPosTarget=v; gDriveVel=0.0f; }   // position MOVE to v (m); cancels velocity drive
  else if (!strcmp(s,"dv"))      { gDriveVel=v; gDriveMs=millis(); }   // velocity-drive command (m/s); 0 = release -> latch home + hold; heartbeat (watchdog)
  else if (!strcmp(s,"odoreset")) gOdoReset=true;      // zero dead-reckoned pose (px,py); heading zero is via gyro-cal
  else if (!strcmp(s,"poszero"))  gPosZeroReq=true;    // re-zero the distance frame (odometer+target) mid-balance, no drop
  else if (!strcmp(s,"legstep"))  gLegStepReq += (int)v; // BOTH legs mirrored (body raise/lower); +extend / -retract; clamped, in-loop
  else if (!strcmp(s,"legR"))     gLegAbsR = (int)v;     // RIGHT leg -> absolute position (clamped); per-leg, for test + leveling
  else if (!strcmp(s,"legL"))     gLegAbsL = (int)v;     // LEFT  leg -> absolute position (clamped); per-leg, for test + leveling
  else if (!strcmp(s,"level"))    gLevelOn = (v!=0);     // IMU-roll body leveling on/off (differential legs)
  else if (!strcmp(s,"levki"))    gLevelKi = v;          // leveling integral gain: leg counts accumulated per deg roll error per cycle
  else if (!strcmp(s,"levslew"))  gLevelSlew = v;        // leveling slew limit: max leg-counts change per control cycle
  else if (!strcmp(s,"levset"))   gLevelSet = v;         // leveling roll setpoint (deg); 0 = level
  else if (!strcmp(s,"levmax"))   gLevelMax = (int)v;    // leveling max |differential| authority (counts)
  else if (!strcmp(s,"levsign"))  gLevelSign = (v<0)?-1:1; // leveling correction sign (flip if it tilts the wrong way)
  else if (!strcmp(s,"lean"))     gLeanGain = v;         // turn lean: share of the lean along the pull (0 = none, 1 = full)
  else if (!strcmp(s,"leancpd"))  gLeanCpd = v;          // turn lean: leg differential counts per deg of body roll
  else if (!strcmp(s,"leghomeslew")) gLegHomeSlew = v;   // leg home/equalize slew (counts/cycle) = how 'slowly' the legs move on balance on/off
  else if (!strcmp(s,"legrate"))  { gLegRate = clampf(v,-200.0f,200.0f); gLegRateMs = millis(); }  // height rate (counts/s, + up); heartbeat
  else if (!strcmp(s,"leghome"))  gLegHomeReq = true;    // legs -> ride height (MID), slow slew; ignored while limp
  else if (!strcmp(s,"leglimp"))  gLegLimpReq = true;    // STOP button: torque-OFF both legs, stay limp until next balance-on
  else if (!strcmp(s,"ffband")) gFFband=v;             // friction FF deadband
  else if (!strcmp(s,"cfa"))   gCfAlpha=clampf(v,0.9f,0.9999f); // compl-filter gyro weight
  else if (!strcmp(s,"imucomp")) gImuComp=v;                    // IMU lever-arm comp scale/sign (0=off, -0.5 verified)
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
    else if(c < 0x20 || c > 0x7E) n = 0;   // line noise (a power-up fills the Pi link with 0xFF-like bytes, seen
                                           // 2026-10-21): drop the partial line so it can't swallow the next command
    else if(n<39) buf[n++]=c;
  }
}

// status LEDs (4x WS2812B on IO27): red=fault/fallen, amber=low batt, green=balancing, blue=idle.
// Called from loop() (low priority, off the balance core); only re-shows on a color change.
static void updateLEDs(int bpct){
  static uint32_t last = 0xFF000000;                 // sentinel so the first call always shows
  if(gLedOvr >= 1 && gLedOvr <= 5 && millis() - gLedOvrMs > 20000) gLedOvr = -1;  // test colors time out -> auto
  uint32_t c;
  int bright = gLedBright;          // user-set global brightness; the low-battery branch forces full
  if(gLedOvr == 6)                  c = 0;                        // OFF (persistent)
  else if(gLedOvr == 1)             c = gLeds.Color(0, 0, 40);    // test: blue
  else if(gLedOvr == 2)             c = gLeds.Color(0, 60, 0);    // test: green
  else if(gLedOvr == 3)             c = gLeds.Color(60, 28, 0);   // test: amber
  else if(gLedOvr == 4)             c = gLeds.Color(60, 0, 0);    // test: red
  else if(gLedOvr == 5)             c = gLeds.Color(40, 40, 40);  // test: white
  else if(gWheelFault != 0 || gFallen) { c = gLeds.Color(60, 0, 0); bright = 255; }   // RED:   tilt error / fault -> FULL brightness
  else if(bpct > 0 && bpct < 15)       { c = gLeds.Color(60, 28, 0); bright = 255; }  // AMBER: low battery      -> FULL brightness
  else                                   c = 0;                                        // otherwise OFF -- alerts only (no run-flag / idle color)
  static int lastBright = -1;
  if(c != last || bright != lastBright){          // re-show on color OR brightness change
    gLeds.setBrightness(bright);                  // global scale; low-battery branch forces full
    for(int i=0;i<LED_N;i++) gLeds.setPixelColor(i, c);
    gLeds.show(); last = c; lastBright = bright;
  }
  // discrete single-color status LEDs:
  //   blue = on while balancing (enabled) ; red = on while fault / fallen
  digitalWrite(LED_BLUE, gEnabled ? LED_ON : LED_OFF);
  digitalWrite(LED_RED,  (gWheelFault != 0 || gFallen) ? LED_ON : LED_OFF);
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
    updateLEDs(bpct);                                        // status LEDs (only redraws on change)
    char b[704];
    int k=snprintf(b,sizeof(b),
      "th=%.2f roll=%.2f yaw=%.1f px=%.3f py=%.3f rate=%.1f wx=%.3f wp1=%.3f wp2=%.3f ptgt=%.3f tgteff=%.3f vdes=%.2f wv=%.2f w1=%.1f w2=%.1f u=%.0f en=%d vbat=%.2f batt=%d lqrx=%.0f lqrvx=%.1f lqrq=%.1f lqrdq=%.2f amax=%.2f set=%.2f izero=%.1f Umax=%d gbias=%.2f lhz=%.0f dtmax=%.1f rfails=%u ctrl=%s poshold=%.1f rdms=%.2f wkms=%.2f post=%.2f inf=%.2f rd07L=%d rd07R=%d pacc=%.2f praw=%.2f vest=%.2f lqrin=%.2f lev=%d racc=%.2f pull=%.2f lean=%.2f gturn=%.1f htg=%.3f yrc=%.3f legR=%d legL=%d dwd=%u legrtL=%u legrtR=%u legfL=%u legfR=%u legstkL=%u legstkR=%u legst=%u rfL=%u rfR=%u rft0=%u rft1=%u ms=%u\n",
      tTheta, tRoll, tYaw, tPoseX, tPoseY, tRate, tWheelX, wheel1_x/1024.0f*WHEEL_CIRC_M, wheel2_x/1024.0f*WHEEL_CIRC_M, gPosTarget, tTgtEff, tVdes, tWheelVx, wheel1_vel, wheel2_vel, tU, gEnabled, tVbat, bpct, gLqrX, gLqrVx, gLqrQ, gLqrDq, gLqrAMax, gSetpoint, gImuZero, gUMax, gGyroBias, tLoopHz, tDtMaxMs, (unsigned)tReadFails, ctrlName(), gPosHoldK, tReadMaxMs, tWorkMaxMs, tPostMaxMs, tInferMaxMs, tRetDL, tRetDR, tPacc, tPraw, tVest, tLqrInQ, gLevelOn, tRollAcc, tPull, tLean, gTurn, gHdgTarget, gYawRateCmd,
      (gLegTgtR!=LEG_NONE?gLegTgtR:LEG_R_MID), (gLegTgtL!=LEG_NONE?gLegTgtL:LEG_L_MID), (unsigned)gDriveWdTrips, (unsigned)tLegRetryL, (unsigned)tLegRetryR, (unsigned)tLegFailL, (unsigned)tLegFailR, (unsigned)tLegStuckL, (unsigned)tLegStuckR, (unsigned)((gLegLimped?1:0)|(gLegHoming?2:0)|(gLegEnerR?4:0)|(gLegEnerL?8:0)), (unsigned)tRfL, (unsigned)tRfR, (unsigned)tRfFirstMs, (unsigned)tRfLastMs, (unsigned)millis());
    if(k > (int)sizeof(b)-1) k = sizeof(b)-1;   // clamp: snprintf returns intended len, not written
    emit(b,k);
  }
  // drive-capture dump: buffer filled by the control task -> emit here (loop() task), so the
  // control task keeps balancing while we stream ~256 lines. Yield periodically (WDT + control).
  if(gLogDump){
    int n = (dcIdx>DCAP_N)?DCAP_N:dcIdx;                 // captured sample count, held since 'en 1'
    { char h[240]; int hk=snprintf(h,sizeof(h),"# dcap n=%d cols: i t_us u wid v th_x100 rate_x10 cmdL cmdR vR "
                                   "pol_pitch_x100 pol_rate_x10 pol_xvel_mm pol_int_m pol_bias_x100 pacc_x100 yaw_x10 yrc_x1000 "
                                   "stick_mms plan_acc_x1000 plan_lean_x100\n",n);
      emit(h,hk); }
    for(int i=0;i<n;i++){
      char b[200]; int k=snprintf(b,sizeof(b),"# dcap %d %lu %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                                 i,(unsigned long)dcT[i],(int)dcCmd[i],(int)dcWid[i],(int)dcV[i],(int)dcP[i],(int)dcR[i],
                                 (int)dcL[i],(int)dcRc[i],(int)dcVR[i],(int)dcPP[i],(int)dcPR[i],(int)dcXV[i],(int)dcPI[i],
                                 (int)dcB[i],(int)dcPA[i],(int)dcYR[i],(int)dcYC[i],(int)dcSt[i],(int)dcPlA[i],(int)dcPlL[i]);
      emit(b,k);
      if((i & 15)==0){
        delay(1);
        // keep reading commands while the ~4 s dump streams: before 2026-10-19 they waited until it ended, so
        // a stick release (or a stop) arrived seconds late -- a floor spin kept turning ~half a turn
        pump(Serial, b0, n0); pump(Serial1, b1, n1);
      }
    }
    { char d[16]; int dk=snprintf(d,sizeof(d),"# dcap done\n"); emit(d,dk); }
    gLogDump=false;
  }
  // ring-down dump: '# dcapr' lines (the bridge forwards '# dcap...' to rider/debug/dcap), commands keep flowing
  if(gRcDump){
    int n = rcIdx;
    { char h[96]; int hk=snprintf(h,sizeof(h),"# dcapr n=%d cols: i t_us pitch_cdeg rate_x10 v vR\n",n); emit(h,hk); }
    for(int i=0;i<n;i++){
      char b[64]; int k=snprintf(b,sizeof(b),"# dcapr %d %lu %d %d %d %d\n", i,(unsigned long)rcT[i],
                                 (int)rcP[i],(int)rcR[i],(int)rcV[i],(int)rcVR[i]);
      emit(b,k);
      if((i & 15)==0){ delay(1); pump(Serial, b0, n0); pump(Serial1, b1, n1); }
    }
    { char d[20]; int dk=snprintf(d,sizeof(d),"# dcapr done\n"); emit(d,dk); }
    gRcDump=false;
  }
  // frequency-sweep dump (refused while the sweep still runs): same pattern as above, commands keep flowing
  if(gChirpDump){
    gChirpDump = false;
    if(gChirpOn){ emit("# dcapc REFUSED: sweep running\n", 30); }
    else {
      int n = ciIdx;
      { char h[128]; int hk = snprintf(h, sizeof(h), "# dcapc n=%d cols: i t_us inj cmdL cmdR vL vR wx_01mm th_x100 rate_x10\n", n); emit(h, hk); }
      for(int i = 0; i < n; i++){
        const ChirpSample& c = ci[i];
        char b[112]; int k = snprintf(b, sizeof(b), "# dcapc %d %lu %d %d %d %d %d %d %d %d\n", i, (unsigned long)c.t,
                                      (int)c.inj, (int)c.cmdL, (int)c.cmdR, (int)c.vL, (int)c.vR, (int)c.wx,
                                      (int)c.th, (int)c.rate);
        emit(b, k);
        if((i & 15) == 0){ delay(1); pump(Serial, b0, n0); pump(Serial1, b1, n1); }
      }
      { char d[16]; int dk = snprintf(d, sizeof(d), "# dcapc done\n"); emit(d, dk); }
    }
  }
}
