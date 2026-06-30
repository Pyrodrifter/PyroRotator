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
     3) EasyComm II        ->  USB serial @ 9600         (SkyRoof via rotctld -m 202)

   Libraries: AccelStepper (Library Manager).  Everything else is ESP32 core.
   Board    : "ESP32 Dev Module".

   >>> EDIT THE CONFIG BLOCK BELOW: WiFi creds, gear ratios, microstepping. <<<
   ============================================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <AccelStepper.h>
#include "index_html.h"   // web page lives here (keeps the raw-string HTML out
                          // of the .ino, which breaks Arduino's preprocessor)

// ----------------------------- CONFIG --------------------------------------
// --- WiFi ---
const char* WIFI_SSID = "Pyrolabs";        // TODO
const char* WIFI_PASS = "67306730";    // TODO
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
const float AZ_MIN = 0.0,   AZ_MAX = 450.0;   // 90° overlap past full rotation; unwind manually
const float EL_MIN = 0.0;
// EL_MAX is runtime-togglable via /api/elmode (90 = normal, 180 = flip-over passes)
float g_elMax = 90.0;

// --- Speeds (steps/sec) ---
const float MAX_SPEED = 800.0;
const float ACCEL     = 400.0;
const float HOME_SPEED      = 300.0;    // fast seek to find the switch
const float HOME_SPEED_SLOW = 80.0;     // slow final approach -> soft, repeatable contact
const float HOME_ACCEL      = 1200.0;   // brisk decel while homing -> tiny overshoot into the switch

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

// --- Serial (EasyComm) ---  match rotctld -s in SkyRoof
const uint32_t SERIAL_BAUD = 9600;
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

void gotoAzEl(float a, float e) {
  a = clampf(a, AZ_MIN, AZ_MAX);
  e = clampf(e, EL_MIN, g_elMax);
  g_targetAz = a; g_targetEl = e;
  motorsEnable();
  // restore full speed in case SuperRot velocity-capped it on a previous move
  az.setMaxSpeed(MAX_SPEED);
  el.setMaxSpeed(MAX_SPEED);
  az.moveTo(lround(a * SPD_AZ));
  el.moveTo(lround(e * SPD_EL));
}

// --- SuperRot continuous motion (AccelStepper adaptation) ------------------
// Track: drive toward a position at a velocity-derived speed cap. Because the
// host streams setpoints continuously, the axis keeps moving instead of
// decelerating to a halt on each command -> smooth tracking.
void srAxisTrack(AccelStepper &s, float deg, float rate, float spd) {
  float hz = fabs(rate) * spd;                 // requested speed in steps/s
  float floorHz = 0.5f * spd / 60.0f;          // tiny floor so position still trims
  if (hz < floorHz) hz = floorHz;
  if (hz > MAX_SPEED) hz = MAX_SPEED;
  s.setMaxSpeed(hz);
  s.moveTo(lround(deg * spd));
}

// Pure velocity: drive toward the soft limit in the rate's direction at |rate|.
void srAxisVel(AccelStepper &s, float rate, float spd, float lo, float hi) {
  float maxd = MAX_SPEED / spd;
  rate = clampf(rate, -maxd, maxd);
  if (fabs(rate) < 1e-4) { s.moveTo(s.currentPosition()); return; }
  s.setMaxSpeed(fabs(rate) * spd);
  s.moveTo(lround((rate > 0 ? hi : lo) * spd));
}

void srTrack(float a, float e, float aRate, float eRate) {
  a = clampf(a, AZ_MIN, AZ_MAX);
  e = clampf(e, EL_MIN, g_elMax);
  g_targetAz = a; g_targetEl = e;
  motorsEnable();
  srAxisTrack(az, a, aRate, SPD_AZ);
  srAxisTrack(el, e, eRate, SPD_EL);
}

void srVel(float aRate, float eRate) {
  motorsEnable();
  srAxisVel(az, aRate, SPD_AZ, AZ_MIN, AZ_MAX);
  srAxisVel(el, eRate, SPD_EL, EL_MIN, g_elMax);
  g_targetAz = currentAz(); g_targetEl = currentEl();
}

void stopAll() {
  az.stop();          // decelerate to a stop
  el.stop();
  g_targetAz = currentAz();
  g_targetEl = currentEl();
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
  motorsEnable();
  // Az: no limit switch — physical 0° set by compass before power-up.
  az.setCurrentPosition(0);

  // El: drive to lower end stop, back off, zero. Flag if the switch was never seen.
  g_homeErr = !homeAxis(el, EL_LIM, HOME_DIR_EL, SPD_EL);
  g_homed   = !g_homeErr;

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

// SuperRot continuous-motion: A/V/P/S/K/?  (replies OK / ERR / telemetry).
void parseSuperrot(String &line) {
  char k = line.charAt(0);
  float a = 0, b = 0, c = 0, d = 0;
  sscanf(line.c_str() + 1, "%f %f %f %f", &a, &b, &c, &d);
  switch (k) {
    case 'A': srTrack(a, b, c, d);          rotClient.print("OK\n"); break;
    case 'V': srVel(a, b);                  rotClient.print("OK\n"); break;
    case 'P': gotoAzEl(a, b);               rotClient.print("OK\n"); break;
    case 'S': stopAll();                    rotClient.print("OK\n"); break;
    case 'K': gotoAzEl(PARK_AZ, PARK_EL);   rotClient.print("OK\n"); break;
    case '?': /* telemetry streamed in handleTcp */                  break;
    default:  rotClient.print("ERR unknown\n");                      break;
  }
}

void handleTcp() {
  if (!rotClient || !rotClient.connected()) {
    if (rotClient) rotClient.stop();   // free the old socket so a reconnect is accepted
    rotClient = rotctld.available();
    return;
  }
  static String line;
  while (rotClient.available()) {
    char c = rotClient.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        line.trim();
        if (g_proto == PROTO_SUPERROT) parseSuperrot(line);
        else                           parseRotctld(line);
      }
      line = "";
    } else if (line.length() < 80) {
      line += c;
    }
  }
  // SuperRot streams telemetry ~10 Hz so the host can close its tracking loop.
  if (g_proto == PROTO_SUPERROT && rotClient.connected()) {
    static uint32_t lastT = 0;
    if (millis() - lastT >= 100) {
      lastT = millis();
      rotClient.printf("T %.2f %.2f %.3f %.3f\n",
        currentAz(), currentEl(), az.speed() / SPD_AZ, el.speed() / SPD_EL);
    }
  }
}

// ----------------------------- EasyComm II (serial) ------------------------
// Tokens "AZ123.4" / "EL45.6" set target; bare "AZ EL" queries position.
void handleEasycomm() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
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
    } else if (line.length() < 70) {
      line += c;
    }
  }
}

// ----------------------------- SETUP / LOOP --------------------------------
void setup() {
  pinMode(EN_PIN, OUTPUT);
  motorsDisable();                 // stay disarmed through boot glitch
  pinMode(AZ_LIM, INPUT_PULLUP);
  pinMode(EL_LIM, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD);

  az.setMaxSpeed(MAX_SPEED);  az.setAcceleration(ACCEL);
  el.setMaxSpeed(MAX_SPEED);  el.setAcceleration(ACCEL);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(200);
  if (WiFi.status() != WL_CONNECTED) {        // fallback: own AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("PyroRotator", "pyrolabs");
  }
  MDNS.begin(HOSTNAME);

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
  http.handleClient();
  handleTcp();
  handleEasycomm();
}
