/* ============================================================================
   PYRO ROTATOR  -  ESP32 Az/El antenna rotator firmware
   PJ4  ·  PyroLabs

   Hardware : ESP32-WROOM-32 (30-pin) on an Arduino-Uno carrier + CNC Shield V3
              2x NEMA 17 via TMC2209 in STANDALONE mode, open-loop, limit-switch homing.
              StealthChop2 + MicroPlyer (1/256 interpolation) are on by default in
              standalone -> silent, smooth tracking with no UART wiring.
   Pin map  : matches the wiring sheet (Az = X axis, El = Y axis).

   TMC2209 standalone setup (no UART):
     - Microstepping is set by the MS1/MS2 jumpers on the CNC shield. Set them to
       match MICROSTEP below. Default (both jumpers OFF) = 1/8.  Jumper table:
         MS2 MS1 -> microsteps:  off off = 8 | off on = 32 | on off = 64 | on on = 16
     - Motor current is set by each driver's Vref trim pot (not in software).
     - Leave the EN jumpers OFF; the ESP drives EN_PIN (D13) directly.

   Control surfaces (NO GRBL):
     1) Web app          ->  http://rotator.local/      (port 80, manual control)
     2) rotctld / SuperRot TCP -> <esp-ip>:4533         (toggle in web app)
   3) SuperRot / EasyComm II -> USB serial @ 115200

   Libraries: AccelStepper (Library Manager).  Everything else is ESP32 core.
   Board    : "ESP32 Dev Module".

   >>> EDIT THE CONFIG BLOCK BELOW: WiFi creds, gear ratios, microstepping. <<<
   ============================================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>   // over-the-air firmware update (Arduino IDE / espota over WiFi)
#include <AccelStepper.h>
#include <Preferences.h>
#include "index_html.h"   // web page lives here (keeps the raw-string HTML out
                          // of the .ino, which breaks Arduino's preprocessor)
#if __has_include("secrets.h")
#include "secrets.h"
#else
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
#endif

// ----------------------------- CONFIG --------------------------------------
// --- WiFi ---
const char* HOSTNAME  = "rotator";          // -> http://rotator.local/

// --- Mechanics (set these to YOUR build) ---
const float MOTOR_STEPS = 200.0;   // 1.8deg NEMA 17 = 200 full steps/rev
const float MICROSTEP   = 8.0;     // MUST match the CNC shield MS jumpers (see header table)
const float GEAR_AZ     = 7.5;     // 20t -> 150t
const float GEAR_EL     = 4.0;     // 20t -> 80t

// steps the motor takes per 1 degree of ANTENNA movement
const float SPD_AZ = (MOTOR_STEPS * MICROSTEP * GEAR_AZ) / 360.0;
const float SPD_EL = (MOTOR_STEPS * MICROSTEP * GEAR_EL) / 360.0;

// --- Travel / soft limits (degrees) ---
const float AZ_MIN = -3600.0, AZ_MAX = 3600.0; // az is free (shortest-path); wide bound for velocity-jog only
const float EL_MIN = 0.0;
// EL_MAX is runtime-togglable via /api/elmode (90 = normal, 180 = flip-over passes)
float g_elMax = 90.0;

// --- Speeds (steps/sec) ---
const float MAX_SPEED = 800.0;
const float ACCEL     = 400.0;
float g_trackKp = 1.6;
float g_captureErr = 0.50;
float g_trackErr = 0.08;
float g_stateHyst = 0.04;
float g_reverseHyst = 0.06;
uint8_t g_settleSamples = 5;
uint32_t g_commandTimeoutMs = 750;
const float HOME_SPEED      = 300.0;    // fast seek to find the switch
const float HOME_SPEED_SLOW = 80.0;     // slow final approach -> soft, repeatable contact
const float HOME_ACCEL      = 1200.0;   // brisk decel while homing -> tiny overshoot into the switch

// --- Runtime config (set live over SuperRot via `C key=value ...`, e.g. from the
//     SkyPhreak "Push settings to rotator" button). Defaults keep stock behaviour. ---
float g_maxSpeedAz = MAX_SPEED;   // per-axis speed cap (steps/s), from host maxVel °/s
float g_maxSpeedEl = MAX_SPEED;
float g_azOffset = 0, g_elOffset = 0;      // mount-alignment offset (deg) — stored for
                                           // standalone use; host applies its own offset,
                                           // so SuperRot 'A' tracking does NOT re-apply it
float g_backlashAz = 0, g_backlashEl = 0;  // gear backlash (deg) — taken up on gotos
int   g_lastDirAz = 0, g_lastDirEl = 0;    // last commanded travel direction per axis
enum TrackState : uint8_t { TS_SLEW, TS_CAPTURE, TS_TRACK };
TrackState g_trackAz = TS_SLEW, g_trackEl = TS_SLEW;
uint8_t g_settleAz = 0, g_settleEl = 0;
uint32_t g_lastTrackMs = 0;
uint32_t g_lastTrackSeq = 0;
bool g_trackStreamActive = false;

enum MotionMode : uint8_t { MM_IDLE, MM_GOTO, MM_SLEW, MM_CAPTURE, MM_TRACK, MM_HOMING, MM_PARK, MM_FAULT };
enum FaultCode : uint8_t { FAULT_NONE, FAULT_COMMAND_TIMEOUT, FAULT_HOME };
MotionMode g_motionMode = MM_IDLE;
FaultCode g_fault = FAULT_NONE;
Preferences prefs;

// --- Homing ---
// Az has NO limit switch — zero is set by hand with a compass before power-up.
// El has end stops and homes to its lower limit switch.
const int   HOME_DIR_EL = -1;         // direction to drive toward the EL lower end stop
const float HOME_BACKOFF_DEG = 5.0;   // back off this far after the switch trips
const bool  AUTO_HOME_ON_BOOT = false; // true = home el automatically at power-up

// --- Park position ---
const float PARK_AZ = 0.0;
const float PARK_EL = 0.0;

// --- Pins (Az = X, El = Y) ---
#define AZ_STEP 25
#define AZ_DIR  26
#define EL_STEP 27
#define EL_DIR  14
#define EN_PIN  13          // active LOW (LOW = drivers enabled)
#define AZ_LIM  32          // reserved — az zero set by compass, not a switch
#define EL_LIM  33          // INPUT_PULLUP, switch to GND (active LOW)

// --- Serial --- SuperRot uses 115200 8N1. EasyComm remains accepted on the same
// port for legacy clients, which must also be configured for 115200 baud.
const uint32_t SERIAL_BAUD = 115200;
#define DEBUG 0             // 1 = boot/debug prints. Keep 0 when using EasyComm.
// ---------------------------------------------------------------------------

AccelStepper az(AccelStepper::DRIVER, AZ_STEP, AZ_DIR);
AccelStepper el(AccelStepper::DRIVER, EL_STEP, EL_DIR);

WebServer http(80);
WiFiServer rotctld(4533);
WiFiClient rotClient;

bool  g_homed = false;
bool  g_homeErr = false;   // true if the last homing run never found the EL limit switch
float g_targetAz = 0, g_targetEl = 0;

// TCP 4533 protocol mode, toggled from the web app via /api/proto.
//   ROTCTLD  = Hamlib net-rotctl (P/p/S) — go-to-and-stop, lurches on each update
//   SUPERROT = PyroSatTrack continuous-motion (A/V/P/S/K/?) — velocity feedforward,
//              never fully stops between setpoints -> smooth pass tracking
enum Proto { PROTO_ROTCTLD, PROTO_SUPERROT };
Proto g_proto = PROTO_ROTCTLD;

// ----------------------------- HELPERS -------------------------------------
inline void motorsEnable()  { digitalWrite(EN_PIN, LOW);  }
inline void motorsDisable() { digitalWrite(EN_PIN, HIGH); }

float currentAz() { return az.currentPosition() / SPD_AZ; }
float currentEl() { return el.currentPosition() / SPD_EL; }
bool  isMoving()  { return az.distanceToGo() != 0 || el.distanceToGo() != 0; }

// HH:MM:SS since boot (wraps after ~49 days, fine for a status readout)
String uptimeStr() {
  uint32_t s = millis() / 1000UL;
  char u[16];
  snprintf(u, sizeof(u), "%02lu:%02lu:%02lu",
           (unsigned long)(s / 3600), (unsigned long)((s % 3600) / 60),
           (unsigned long)(s % 60));
  return String(u);
}

// IP of the connected rotctld client (SatDump/SkyRoof), else "" = manual
String controlStr() {
  if (rotClient && rotClient.connected()) return rotClient.remoteIP().toString();
  return String();
}

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

const char* motionModeName() {
  switch (g_motionMode) {
    case MM_GOTO: return "goto"; case MM_SLEW: return "slew";
    case MM_CAPTURE: return "capture"; case MM_TRACK: return "track";
    case MM_HOMING: return "homing"; case MM_PARK: return "park";
    case MM_FAULT: return "fault"; default: return "idle";
  }
}

const char* faultName() {
  switch (g_fault) {
    case FAULT_COMMAND_TIMEOUT: return "command_timeout";
    case FAULT_HOME: return "home_failed";
    default: return "none";
  }
}

void clearFault() { g_fault = FAULT_NONE; if (g_motionMode == MM_FAULT) g_motionMode = MM_IDLE; }

// Nearest equivalent of a target heading to the current azimuth — the shortest-path
// move (never more than 180°). Used by one-shot gotos (web/rotctld/park/jog) so e.g.
// 0°→330° goes −30°, not +330°. (Streamed SuperRot 'A' setpoints stay absolute; the
// host already resolves their continuity.)
float shortestAz(float target, float cur) {
  float d = target - cur;
  d -= 360.0f * roundf(d / 360.0f);
  return cur + d;
}

// If a move reverses travel direction, bias the target by the backlash slack (in the
// new direction) so the gear train takes up before the antenna actually moves. With
// backlash 0 this is a no-op, so stock behaviour is unchanged.
long backlashBias(AccelStepper &s, long target, float slackSteps, int &lastDir) {
  long cur = s.currentPosition();
  int dir = (target > cur) ? 1 : (target < cur ? -1 : lastDir);
  if (dir && lastDir && dir != lastDir && slackSteps > 0) target += dir * lround(slackSteps);
  lastDir = dir;
  return target;
}

void gotoAzEl(float a, float e) {
  // Azimuth is FREE (no clamp) and takes the SHORTEST PATH from where it is — so a
  // one-shot goto to any heading turns the short way (may go negative). Only elevation
  // is bounded (0..elMax, elMax up to 180 for flip).
  a = shortestAz(a, currentAz());
  e = clampf(e, EL_MIN, g_elMax);
  g_targetAz = a; g_targetEl = e;
  g_trackStreamActive = false;
  clearFault();
  g_motionMode = MM_GOTO;
  motorsEnable();
  // restore full (configured) speed in case SuperRot velocity-capped it on a previous move
  az.setMaxSpeed(g_maxSpeedAz);
  el.setMaxSpeed(g_maxSpeedEl);
  az.moveTo(backlashBias(az, lround(a * SPD_AZ), g_backlashAz * SPD_AZ, g_lastDirAz));
  el.moveTo(backlashBias(el, lround(e * SPD_EL), g_backlashEl * SPD_EL, g_lastDirEl));
}

// --- SuperRot continuous motion (AccelStepper adaptation) ------------------
// Track: drive toward a position at a velocity-derived speed cap. Because the
// host streams setpoints continuously, the axis keeps moving instead of
// decelerating to a halt on each command -> smooth tracking.
void srAxisTrack(AccelStepper &s, float deg, float rate, float spd, float maxHz,
                 TrackState &state, uint8_t &settle, int &lastDir, float backlashDeg) {
  float err = deg - s.currentPosition() / spd;
  float ae = fabs(err);
  if (state == TS_SLEW) {
    if (ae < g_captureErr) state = TS_CAPTURE;
  } else if (state == TS_CAPTURE) {
    if (ae > g_captureErr + g_stateHyst) { state = TS_SLEW; settle = 0; }
    else if (ae < g_trackErr) { if (++settle >= g_settleSamples) state = TS_TRACK; }
    else settle = 0;
  } else if (ae > g_trackErr + g_stateHyst) {
    state = TS_CAPTURE; settle = 0;
  }

  float commandedRate = rate + g_trackKp * err;
  int dir = commandedRate > 0 ? 1 : (commandedRate < 0 ? -1 : 0);
  float noReverse = fmaxf(g_reverseHyst, backlashDeg);
  if (state == TS_TRACK && dir && lastDir && dir != lastDir && ae < noReverse) {
    commandedRate = 0;
    dir = lastDir;
  }
  if (dir && fabs(commandedRate) > 0.01f) lastDir = dir;

  float hz = fabs(commandedRate) * spd;
  float floorHz = 0.5f * spd / 60.0f;          // tiny floor so position still trims
  if (hz < floorHz) hz = floorHz;
  if (hz > maxHz) hz = maxHz;
  s.setMaxSpeed(hz);
  s.moveTo(lround(deg * spd));
}

// Pure velocity: drive toward the soft limit in the rate's direction at |rate|.
void srAxisVel(AccelStepper &s, float rate, float spd, float lo, float hi, float maxHz) {
  float maxd = maxHz / spd;
  rate = clampf(rate, -maxd, maxd);
  if (fabs(rate) < 1e-4) { s.moveTo(s.currentPosition()); return; }
  s.setMaxSpeed(fabs(rate) * spd);
  s.moveTo(lround((rate > 0 ? hi : lo) * spd));
}

void srTrack(float a, float e, float aRate, float eRate) {
  e = clampf(e, EL_MIN, g_elMax);  // az free (shortest-path); el bounded
  g_targetAz = a; g_targetEl = e;
  g_lastTrackMs = millis();
  g_trackStreamActive = true;
  clearFault();
  motorsEnable();
  srAxisTrack(az, a, aRate, SPD_AZ, g_maxSpeedAz, g_trackAz, g_settleAz, g_lastDirAz, g_backlashAz);
  srAxisTrack(el, e, eRate, SPD_EL, g_maxSpeedEl, g_trackEl, g_settleEl, g_lastDirEl, g_backlashEl);
  g_motionMode = (g_trackAz == TS_SLEW || g_trackEl == TS_SLEW) ? MM_SLEW
    : (g_trackAz == TS_CAPTURE || g_trackEl == TS_CAPTURE) ? MM_CAPTURE : MM_TRACK;
}

const char* trackStateName() {
  if (g_trackAz == TS_SLEW || g_trackEl == TS_SLEW) return "slew";
  if (g_trackAz == TS_CAPTURE || g_trackEl == TS_CAPTURE) return "capture";
  return "track";
}

void srVel(float aRate, float eRate) {
  g_trackStreamActive = false;
  clearFault();
  g_motionMode = MM_SLEW;
  motorsEnable();
  srAxisVel(az, aRate, SPD_AZ, AZ_MIN, AZ_MAX, g_maxSpeedAz);
  srAxisVel(el, eRate, SPD_EL, EL_MIN, g_elMax, g_maxSpeedEl);
  g_targetAz = currentAz(); g_targetEl = currentEl();
}

void stopAll() {
  g_trackStreamActive = false;
  az.stop();          // decelerate to a stop
  el.stop();
  g_targetAz = currentAz();
  g_targetEl = currentEl();
  if (g_fault == FAULT_NONE) g_motionMode = MM_IDLE;
}

// Unwind the azimuth cable: drive to the 0-turn equivalent of the current heading —
// same compass direction, but with accumulated turns removed. This is an ABSOLUTE move
// and deliberately does NOT shortest-path, so it can turn a full revolution to take the
// cable back to a neutral wrap. Only meaningful with the free/continuous azimuth model.
void unwindAz() {
  g_trackStreamActive = false;
  clearFault();
  g_motionMode = MM_GOTO;
  float cur = currentAz();
  float u = fmodf(cur, 360.0f);
  if (u < 0) u += 360.0f;
  g_targetAz = u;
  motorsEnable();
  az.setMaxSpeed(g_maxSpeedAz);
  az.moveTo(lround(u * SPD_AZ));
}

// Drive one axis until its limit switch (active LOW) reads LOW; stop on first
// contact. `slow` selects the gentle final-approach speed. Returns false if the
// switch wasn't found within the travel cap (so a mis-directed/unseen switch can't
// grind into the hard stop). Brisk HOME_ACCEL keeps the decel-into-switch tiny.
static bool seekSwitch(AccelStepper &s, int limitPin, int dir, float spd, bool slow) {
  s.setMaxSpeed(slow ? HOME_SPEED_SLOW : HOME_SPEED);
  s.move(dir * lround(spd * (slow ? (HOME_BACKOFF_DEG * 4 + 10) : 400.0)));
  bool found = false;
  while (s.distanceToGo() != 0) {
    s.run();
    if (digitalRead(limitPin) == LOW) { found = true; break; }
  }
  s.stop();
  while (s.distanceToGo() != 0) s.run();   // finish the (small) decel
  return found;
}

// Back away from the switch until it releases, then a defined offset past release,
// so we end reliably clear of it.
static void backOff(AccelStepper &s, int limitPin, int dir, float spd) {
  s.setMaxSpeed(HOME_SPEED);
  s.move(-dir * lround(spd * 30));
  while (s.distanceToGo() != 0 && digitalRead(limitPin) == LOW) s.run();
  s.stop();
  while (s.distanceToGo() != 0) s.run();
  s.move(-dir * lround(HOME_BACKOFF_DEG * 2 * spd));
  while (s.distanceToGo() != 0) s.run();
}

// Two-stage homing for a soft, repeatable contact: a fast seek to find the switch,
// back off, then a SLOW re-approach that just kisses it (negligible overshoot, no
// hard press / skipped step) — and we zero at that gentle contact. Returns false if
// the switch is never found.
bool homeAxis(AccelStepper &s, int limitPin, int dir, float spd) {
  s.setAcceleration(HOME_ACCEL);

  // If already pressing the switch, release it first.
  if (digitalRead(limitPin) == LOW) backOff(s, limitPin, dir, spd);

  // Stage 1: fast seek.
  if (!seekSwitch(s, limitPin, dir, spd, false)) {
    s.setMaxSpeed(MAX_SPEED); s.setAcceleration(ACCEL); return false;
  }
  backOff(s, limitPin, dir, spd);

  // Stage 2: slow, gentle re-approach -> precise, soft contact.
  if (!seekSwitch(s, limitPin, dir, spd, true)) {
    s.setMaxSpeed(MAX_SPEED); s.setAcceleration(ACCEL); return false;
  }
  s.setCurrentPosition(0);               // precise zero at the gentle contact point

  // Rest just clear of the switch; call that home.
  backOff(s, limitPin, dir, spd);
  s.setCurrentPosition(0);

  s.setMaxSpeed(MAX_SPEED);
  s.setAcceleration(ACCEL);
  return true;
}

void homeAll() {
  g_trackStreamActive = false;
  g_motionMode = MM_HOMING;
  motorsEnable();
  // Az: no limit switch — physical 0° set by compass before power-up.
  az.setCurrentPosition(0);

  // El: drive to lower end stop, back off, zero. Flag if the switch was never seen.
  g_homeErr = !homeAxis(el, EL_LIM, HOME_DIR_EL, SPD_EL);
  g_homed   = !g_homeErr;
  g_fault = g_homeErr ? FAULT_HOME : FAULT_NONE;
  g_motionMode = g_homeErr ? MM_FAULT : MM_IDLE;

  g_targetAz = currentAz();
  g_targetEl = currentEl();
}

// ----------------------------- WEB APP -------------------------------------
// The HTML/CSS/JS page is in index_html.h (declared as INDEX_HTML).
// Keep both files in the same sketch folder.

// ----------------------------- HTTP ROUTES ---------------------------------
void handleRoot()   { http.send_P(200, "text/html", INDEX_HTML); }

void handleStatus() {
  char b[440];
  snprintf(b, sizeof(b),
    "{\"az\":%.2f,\"el\":%.2f,\"taz\":%.2f,\"tel\":%.2f,"
    "\"moving\":%s,\"homed\":%s,\"homeerr\":%s,\"ellim\":%s,\"azlim\":%s,\"ip\":\"%s\","
    "\"rssi\":%d,\"uptime\":\"%s\",\"control\":\"%s\",\"elMax\":%.0f,\"proto\":\"%s\"}",
    currentAz(), currentEl(), g_targetAz, g_targetEl,
    isMoving() ? "true" : "false", g_homed ? "true" : "false", g_homeErr ? "true" : "false",
    digitalRead(EL_LIM) == LOW ? "true" : "false",   // true = EL switch pressed (pulled low)
    digitalRead(AZ_LIM) == LOW ? "true" : "false",
    WiFi.localIP().toString().c_str(),
    (int)WiFi.RSSI(), uptimeStr().c_str(), controlStr().c_str(), g_elMax,
    g_proto == PROTO_SUPERROT ? "superrot" : "rotctld");
  http.send(200, "application/json", b);
}

void handleGoto() {
  if (http.hasArg("az") && http.hasArg("el"))
    gotoAzEl(http.arg("az").toFloat(), http.arg("el").toFloat());
  http.send(200, "text/plain", "OK");
}

void handleJog() {
  float d = http.arg("delta").toFloat();
  String ax = http.arg("axis");
  if (ax == "az") gotoAzEl(g_targetAz + d, g_targetEl);
  else            gotoAzEl(g_targetAz, g_targetEl + d);
  http.send(200, "text/plain", "OK");
}

void handleStop()   { stopAll();              http.send(200, "text/plain", "OK"); }
void handlePark()   { gotoAzEl(PARK_AZ, PARK_EL); http.send(200, "text/plain", "OK"); }
void handleHome()   { http.send(200, "text/plain", "homing"); homeAll(); }
void handleElMode() {
  g_elMax = (g_elMax < 120.0f) ? 180.0f : 90.0f;
  // clamp el if we just dropped back to 90 and antenna is above it
  if (g_targetEl > g_elMax) gotoAzEl(g_targetAz, g_elMax);
  prefs.putFloat("elMax", g_elMax);
  char b[32]; snprintf(b, sizeof(b), "{\"elMax\":%.0f}", g_elMax);
  http.send(200, "application/json", b);
}

void handleProto() {
  g_proto = (g_proto == PROTO_ROTCTLD) ? PROTO_SUPERROT : PROTO_ROTCTLD;
  stopAll();   // clean handoff: don't carry motion across a protocol switch
  http.send(200, "application/json",
    g_proto == PROTO_SUPERROT ? "{\"proto\":\"superrot\"}" : "{\"proto\":\"rotctld\"}");
}

// ----------------------------- TCP 4533 (rotctld / SuperRot) ---------------
// One listener; the active protocol is chosen by g_proto (toggle via /api/proto).

// Minimal Hamlib net-rotctl: P (set), p (get), S (stop).
void parseRotctld(String &line) {
  char k = line.charAt(0);
  if (k == 'P' || line.startsWith("set_pos") || line.startsWith("\\set_pos")) {
    float a, e;
    if (sscanf(line.c_str() + 1, "%f %f", &a, &e) == 2 ||
        sscanf(line.c_str(), "%*s %f %f", &a, &e) == 2) {
      gotoAzEl(a, e);
      rotClient.print("RPRT 0\n");
    } else rotClient.print("RPRT -1\n");
  } else if (k == 'p' || line.startsWith("get_pos")) {
    rotClient.printf("%.6f\n%.6f\n", currentAz(), currentEl());
  } else if (k == 'S' || line.startsWith("stop")) {
    stopAll(); rotClient.print("RPRT 0\n");
  } else if (k == 'q' || k == 'Q') {
    rotClient.stop();
  } else {
    rotClient.print("RPRT 0\n");   // tolerate dump_state, get_info, etc.
  }
}

// Apply one config key=value pair pushed by the host ("Push settings to rotator").
// Unknown keys are ignored. Speeds arrive as °/s and are converted to steps/s, capped
// by the hardware MAX_SPEED. Offsets are stored (host applies its own for SuperRot).
void applyConfigKV(const char* key, float v) {
  if      (!strcmp(key, "maxVelAz")) g_maxSpeedAz = fminf(v * SPD_AZ, MAX_SPEED);
  else if (!strcmp(key, "maxVelEl")) g_maxSpeedEl = fminf(v * SPD_EL, MAX_SPEED);
  else if (!strcmp(key, "elMax"))    g_elMax = clampf(v, 90.0f, 180.0f);
  else if (!strcmp(key, "azOffset")) g_azOffset = v;
  else if (!strcmp(key, "elOffset")) g_elOffset = v;
  else if (!strcmp(key, "backlashAz")) g_backlashAz = fmaxf(0.0f, v);
  else if (!strcmp(key, "backlashEl")) g_backlashEl = fmaxf(0.0f, v);
  else if (!strcmp(key, "trackKp")) g_trackKp = clampf(v, 0.05f, 10.0f);
  else if (!strcmp(key, "captureErr")) g_captureErr = clampf(v, 0.05f, 10.0f);
  else if (!strcmp(key, "trackErr")) g_trackErr = clampf(v, 0.005f, g_captureErr);
  else if (!strcmp(key, "stateHyst")) g_stateHyst = clampf(v, 0.0f, 2.0f);
  else if (!strcmp(key, "reverseHyst")) g_reverseHyst = clampf(v, 0.0f, 2.0f);
  else if (!strcmp(key, "settleSamples")) g_settleSamples = (uint8_t)clampf(v, 1, 50);
  else if (!strcmp(key, "commandTimeoutMs")) g_commandTimeoutMs = (uint32_t)clampf(v, 250, 10000);
}

void saveRuntimeConfig() {
  prefs.putFloat("maxAzHz", g_maxSpeedAz); prefs.putFloat("maxElHz", g_maxSpeedEl);
  prefs.putFloat("elMax", g_elMax); prefs.putFloat("azOff", g_azOffset); prefs.putFloat("elOff", g_elOffset);
  prefs.putFloat("backAz", g_backlashAz); prefs.putFloat("backEl", g_backlashEl);
  prefs.putFloat("trkKp", g_trackKp); prefs.putFloat("capErr", g_captureErr); prefs.putFloat("trkErr", g_trackErr);
  prefs.putFloat("stHyst", g_stateHyst); prefs.putFloat("revHyst", g_reverseHyst);
  prefs.putUChar("settle", g_settleSamples); prefs.putUInt("timeout", g_commandTimeoutMs);
}

void loadRuntimeConfig() {
  g_maxSpeedAz = prefs.getFloat("maxAzHz", MAX_SPEED); g_maxSpeedEl = prefs.getFloat("maxElHz", MAX_SPEED);
  g_elMax = prefs.getFloat("elMax", 90); g_azOffset = prefs.getFloat("azOff", 0); g_elOffset = prefs.getFloat("elOff", 0);
  g_backlashAz = prefs.getFloat("backAz", 0); g_backlashEl = prefs.getFloat("backEl", 0);
  g_trackKp = prefs.getFloat("trkKp", 1.6); g_captureErr = prefs.getFloat("capErr", .5);
  g_trackErr = prefs.getFloat("trkErr", .08); g_stateHyst = prefs.getFloat("stHyst", .04);
  g_reverseHyst = prefs.getFloat("revHyst", .06); g_settleSamples = prefs.getUChar("settle", 5);
  g_commandTimeoutMs = prefs.getUInt("timeout", 750);
}

// Parse a `C key=value key=value ...` config line.
void parseConfig(String &line) {
  char buf[120];
  line.toCharArray(buf, sizeof(buf));
  for (char* t = strtok(buf + 1, " "); t; t = strtok(NULL, " ")) {
    char* eq = strchr(t, '=');
    if (!eq) continue;
    *eq = 0;
    applyConfigKV(t, atof(eq + 1));
  }
  saveRuntimeConfig();
}

// SuperRot continuous-motion: A/V/P/S/K/H/U/C/?  (replies OK / ERR / telemetry).
void parseSuperrot(String &line, Print &reply) {
  char k = line.charAt(0);
  if (k == 'C') { parseConfig(line); reply.print("OK\n"); return; }
  if (k == 'H') { reply.print("OK\n"); homeAll(); return; }  // homing (blocks briefly)
  if (k == 'U') { unwindAz(); reply.print("OK\n"); return; }  // cable unwind
  if (line.startsWith("A2 ")) {
    unsigned long seq; float a, b, c, d;
    if (sscanf(line.c_str() + 2, "%lu %f %f %f %f", &seq, &a, &b, &c, &d) != 5) {
      reply.print("ERR bad_A2\n"); return;
    }
    if (g_lastTrackSeq && (int32_t)((uint32_t)seq - g_lastTrackSeq) <= 0) {
      reply.print("ERR stale_seq\n"); return;
    }
    g_lastTrackSeq = (uint32_t)seq;
    srTrack(a, b, c, d); reply.print("OK\n"); return;
  }
  float a = 0, b = 0, c = 0, d = 0;
  sscanf(line.c_str() + 1, "%f %f %f %f", &a, &b, &c, &d);
  switch (k) {
    case 'A': srTrack(a, b, c, d);          reply.print("OK\n"); break;
    case 'V': srVel(a, b);                  reply.print("OK\n"); break;
    case 'P': gotoAzEl(a, b);               reply.print("OK\n"); break;
    case 'S': stopAll();                    reply.print("OK\n"); break;
    case 'K': gotoAzEl(PARK_AZ, PARK_EL);   reply.print("OK\n"); break;
    case '?': /* telemetry streamed in handleTcp */                  break;
    default:  reply.print("ERR unknown\n");                          break;
  }
}

void handleTcp() {
  if (!rotClient || !rotClient.connected()) {
    if (rotClient) rotClient.stop();   // free the old socket so a reconnect is accepted
    rotClient = rotctld.available();
    if (rotClient) { g_lastTrackSeq = 0; g_trackStreamActive = false; }
    return;
  }
  static String line;
  while (rotClient.available()) {
    char c = rotClient.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        line.trim();
        if (g_proto == PROTO_SUPERROT) parseSuperrot(line, rotClient);
        else                           parseRotctld(line);
      }
      line = "";
    } else if (line.length() < 80) {
      line += c;
    }
  }
  // SuperRot streams telemetry ~10 Hz so the host can close its tracking loop. The
  // trailing key=value fields are diagnostics the host shows when present (endstop
  // states, homed flag, ESP32 core temperature) — older hosts ignore the extras.
  if (g_proto == PROTO_SUPERROT && rotClient.connected()) {
    static uint32_t lastT = 0;
    if (millis() - lastT >= 100) {
      lastT = millis();
      rotClient.printf("T %.6f %.6f %.6f %.6f azSteps=%ld elSteps=%ld state=%s mode=%s fault=%s seq=%lu errAz=%.6f errEl=%.6f dirAz=%d dirEl=%d ageMs=%lu esAz=%d esEl=%d homed=%d tempC=%.1f\n",
        currentAz(), currentEl(), az.speed() / SPD_AZ, el.speed() / SPD_EL,
        az.currentPosition(), el.currentPosition(), trackStateName(), motionModeName(), faultName(), (unsigned long)g_lastTrackSeq,
        g_targetAz - currentAz(), g_targetEl - currentEl(), g_lastDirAz, g_lastDirEl,
        (unsigned long)(g_lastTrackMs ? millis() - g_lastTrackMs : 0),
        digitalRead(AZ_LIM) == LOW ? 1 : 0, digitalRead(EL_LIM) == LOW ? 1 : 0,
        g_homed ? 1 : 0, temperatureRead());
    }
  }
}

// ---------------------- USB serial (SuperRot / EasyComm) -------------------
// SuperRot is auto-detected from its command letter. Legacy EasyComm tokens
// "AZ123.4" / "EL45.6" remain accepted on the same 115200-baud connection.
void handleSerial() {
  static String line;
  static bool superrotSession = false;
  static uint32_t lastT = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        line.trim();
        char k = line.charAt(0);
        bool easycomm = line.startsWith("AZ") || line.startsWith("EL") ||
                        line.startsWith("SA") || line.startsWith("SE");
        bool superrot = !easycomm && strchr("AVPSKHUC?", k) != nullptr;
        if (superrot) {
          superrotSession = true;
          parseSuperrot(line, Serial);
          line = "";
          continue;
        }
        float a = currentAz(), e = currentEl();
        bool setA = false, setE = false, query = false;
        char buf[80]; line.toCharArray(buf, sizeof(buf));
        for (char* t = strtok(buf, " "); t; t = strtok(NULL, " ")) {
          if (!strncmp(t, "AZ", 2)) {
            if (t[2]) { a = atof(t + 2); setA = true; } else query = true;
          } else if (!strncmp(t, "EL", 2)) {
            if (t[2]) { e = atof(t + 2); setE = true; } else query = true;
          } else if (!strcmp(t, "SA") || !strcmp(t, "SE")) {
            stopAll();
          }
        }
        if (setA || setE) gotoAzEl(a, e);
        if (query) Serial.printf("AZ%.1f EL%.1f\n", currentAz(), currentEl());
      }
      line = "";
    } else if (line.length() < 119) {
      line += c;
    }
  }
  if (superrotSession && millis() - lastT >= 100) {
    lastT = millis();
    Serial.printf("T %.6f %.6f %.6f %.6f azSteps=%ld elSteps=%ld state=%s mode=%s fault=%s seq=%lu errAz=%.6f errEl=%.6f dirAz=%d dirEl=%d ageMs=%lu esAz=%d esEl=%d homed=%d tempC=%.1f\n",
      currentAz(), currentEl(), az.speed() / SPD_AZ, el.speed() / SPD_EL,
      az.currentPosition(), el.currentPosition(), trackStateName(), motionModeName(), faultName(), (unsigned long)g_lastTrackSeq,
      g_targetAz - currentAz(), g_targetEl - currentEl(), g_lastDirAz, g_lastDirEl,
      (unsigned long)(g_lastTrackMs ? millis() - g_lastTrackMs : 0),
      digitalRead(AZ_LIM) == LOW ? 1 : 0, digitalRead(EL_LIM) == LOW ? 1 : 0,
      g_homed ? 1 : 0, temperatureRead());
  }
}

// ----------------------------- SETUP / LOOP --------------------------------
void setup() {
  pinMode(EN_PIN, OUTPUT);
  motorsDisable();                 // stay disarmed through boot glitch
  pinMode(AZ_LIM, INPUT_PULLUP);
  pinMode(EL_LIM, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD);
  prefs.begin("pyro-rot", false);
  loadRuntimeConfig();

  az.setMaxSpeed(MAX_SPEED);  az.setAcceleration(ACCEL);
  el.setMaxSpeed(MAX_SPEED);  el.setAcceleration(ACCEL);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  if (WIFI_SSID[0]) WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WIFI_SSID[0] && WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(200);
  if (WiFi.status() != WL_CONNECTED) {        // fallback: own AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("PyroRotator", "pyrolabs");
  }
  MDNS.begin(HOSTNAME);

  // Over-the-air updates: flash new firmware from the Arduino IDE (network port) or
  // espota.py without a USB cable. Motors are disabled before an update starts so the
  // mount can't be left driving if the reboot interrupts a move.
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.onStart([]() { stopAll(); motorsDisable(); });
  ArduinoOTA.begin();

  http.on("/",            handleRoot);
  http.on("/api/status",  handleStatus);
  http.on("/api/goto",    handleGoto);
  http.on("/api/jog",     handleJog);
  http.on("/api/stop",    handleStop);
  http.on("/api/home",    handleHome);
  http.on("/api/park",    handlePark);
  http.on("/api/elmode",  handleElMode);
  http.on("/api/proto",   handleProto);
  http.begin();
  rotctld.begin();

  motorsEnable();
  if (AUTO_HOME_ON_BOOT) homeAll();
  g_targetAz = currentAz();
  g_targetEl = currentEl();

#if DEBUG
  Serial.printf("\nPYRO ROTATOR up. IP=%s  SPD_AZ=%.2f SPD_EL=%.2f\n",
                WiFi.localIP().toString().c_str(), SPD_AZ, SPD_EL);
#endif
}

void loop() {
  az.run();
  el.run();
  ArduinoOTA.handle();
  http.handleClient();
  handleTcp();
  handleSerial();
  if (g_trackStreamActive && millis() - g_lastTrackMs > g_commandTimeoutMs) {
    az.stop(); el.stop();
    g_trackStreamActive = false;
    g_fault = FAULT_COMMAND_TIMEOUT;
    g_motionMode = MM_FAULT;
    g_targetAz = currentAz(); g_targetEl = currentEl();
  } else if (!g_trackStreamActive && g_motionMode == MM_GOTO && !isMoving()) {
    g_motionMode = MM_IDLE;
  }
}
