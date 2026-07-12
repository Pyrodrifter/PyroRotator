/*
  SuperRot Universal firmware (experimental).

  This sketch intentionally lives beside, and does not replace, PyroRotator.ino.
  First boot is SAFE SETUP MODE: drivers disabled and no motion accepted. Configure
  over USB serial at 115200 baud, save, then reboot.

  Newline-delimited JSON commands:
    {"cmd":"get"}
    {"cmd":"defaults"}
    {"cmd":"set","azGearRatio":7.5,"elGearRatio":4,"microsteps":8}
    {"cmd":"validate"}
    {"cmd":"save"}
    {"cmd":"reboot"}
    {"cmd":"factoryReset"}

  SuperRot commands remain available after configuration: A2/A/V/P/S/H/K/?
*/

#include <AccelStepper.h>
#include <Preferences.h>
#include <WiFi.h>
#include "RotatorConfig.h"

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 750;
static constexpr float TRACK_KP = 1.6f;
static constexpr int STOCK_ENABLE_PIN = 13;
static constexpr bool STOCK_ENABLE_ACTIVE_LOW = true;

Preferences prefs;
RotatorConfig cfg;
AccelStepper *az = nullptr;
AccelStepper *el = nullptr;
WiFiServer superrotServer(4533);
WiFiClient superrotClient;

bool setupMode = true;
bool configured = false;
bool homed = false;
bool tracking = false;
uint32_t lastTrackMs = 0;
uint32_t lastSequence = 0;
float targetAz = 0, targetEl = 0;

float azStepsPerDeg() { return cfg.motorSteps * cfg.microsteps * cfg.azGearRatio / 360.0f; }
float elStepsPerDeg() { return cfg.motorSteps * cfg.microsteps * cfg.elGearRatio / 360.0f; }
float currentAz() { return az ? az->currentPosition() / azStepsPerDeg() : 0; }
float currentEl() { return el ? el->currentPosition() / elStepsPerDeg() : 0; }

void jsonError(Print &out, const char *code, const String &detail = "") {
  out.print("{\"ok\":false,\"error\":\""); out.print(code); out.print("\"");
  if (detail.length()) { out.print(",\"detail\":\""); out.print(detail); out.print("\""); }
  out.println("}");
}

bool outputCapablePin(int p) {
  if (p < 0 || p > 39) return false;
  if (p >= 34) return false; // input-only
  // UART0 carries the only setup/recovery protocol. Reassigning either pin can
  // cut off the host before a bad configuration can be repaired.
  if (p == 1 || p == 3) return false;
  // ESP32-WROOM flash bus. GPIO 6..11 must never be assigned.
  if (p >= 6 && p <= 11) return false;
  // These affect ROM boot mode. Advanced boards may opt in explicitly.
  if (!cfg.allowStrappingPins && (p == 0 || p == 2 || p == 5 || p == 12 || p == 15)) return false;
  return true;
}

bool inputCapablePin(int p) {
  // GPIO1/3 are reserved even for limits: pinMode() on them would disrupt USB
  // serial setup on common ESP32-WROOM dev boards.
  return p == -1 || (p >= 0 && p <= 39 && p != 1 && p != 3 && !(p >= 6 && p <= 11));
}

void driveEnableInactive(int pin, bool activeLow) {
  if (!outputCapablePin(pin)) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, activeLow ? HIGH : LOW);
}

bool validateConfig(String &why) {
  if (cfg.magic != ROTATOR_CONFIG_MAGIC || cfg.version != ROTATOR_CONFIG_VERSION || cfg.size != sizeof(cfg)) {
    why = "schema"; return false;
  }
  int outputs[] = {cfg.azStepPin, cfg.azDirPin, cfg.elStepPin, cfg.elDirPin, cfg.enablePin};
  for (int p : outputs) if (!outputCapablePin(p)) { why = "unsafe_output_gpio_" + String(p); return false; }
  for (size_t i = 0; i < 5; ++i)
    for (size_t j = i + 1; j < 5; ++j)
      if (outputs[i] == outputs[j]) { why = "duplicate_output_gpio_" + String(outputs[i]); return false; }
  if (!inputCapablePin(cfg.azLimitPin) || !inputCapablePin(cfg.elLimitPin)) { why = "unsafe_limit_gpio"; return false; }
  for (int p : outputs) {
    if ((cfg.azLimitPin >= 0 && p == cfg.azLimitPin) || (cfg.elLimitPin >= 0 && p == cfg.elLimitPin)) {
      why = "gpio_used_as_input_and_output_" + String(p); return false;
    }
  }
  if (cfg.azLimitPin >= 0 && cfg.azLimitPin == cfg.elLimitPin) { why = "duplicate_limit_gpio"; return false; }
  if (!isfinite(cfg.motorSteps) || cfg.motorSteps < 1 || cfg.motorSteps > 10000) { why = "motorSteps_range"; return false; }
  if (!isfinite(cfg.microsteps) || cfg.microsteps < 1 || cfg.microsteps > 256) { why = "microsteps_range"; return false; }
  if (!isfinite(cfg.azGearRatio) || cfg.azGearRatio <= 0 || cfg.azGearRatio > 10000 ||
      !isfinite(cfg.elGearRatio) || cfg.elGearRatio <= 0 || cfg.elGearRatio > 10000) { why = "gearRatio_range"; return false; }
  if (!isfinite(cfg.azMin) || !isfinite(cfg.azMax) || !isfinite(cfg.elMin) || !isfinite(cfg.elMax) ||
      !(cfg.azMin < cfg.azMax) || !(cfg.elMin < cfg.elMax)) { why = "soft_limits"; return false; }
  if (!isfinite(cfg.maxSpeedAz) || !isfinite(cfg.maxSpeedEl) ||
      !isfinite(cfg.accelerationAz) || !isfinite(cfg.accelerationEl) ||
      cfg.maxSpeedAz <= 0 || cfg.maxSpeedEl <= 0 || cfg.maxSpeedAz > 1000 || cfg.maxSpeedEl > 1000 ||
      cfg.accelerationAz <= 0 || cfg.accelerationEl <= 0 || cfg.accelerationAz > 10000 || cfg.accelerationEl > 10000) {
    why = "motion_range"; return false;
  }
  if (!isfinite(cfg.homeSpeed) || !isfinite(cfg.homeSlowSpeed) || !isfinite(cfg.homeBackoffDeg) ||
      cfg.homeSpeed <= 0 || cfg.homeSlowSpeed <= 0 || cfg.homeBackoffDeg <= 0 ||
      cfg.homeSpeed > 1000 || cfg.homeSlowSpeed > cfg.homeSpeed || cfg.homeBackoffDeg > 90) {
    why = "homing_range"; return false;
  }
  if (cfg.homingMode > HOME_DISABLED) { why = "homingMode"; return false; }
  if (cfg.homingMode == HOME_AZ_MANUAL_EL_SWITCH && cfg.elLimitPin < 0) { why = "el_limit_required"; return false; }
  if (cfg.elHomeDirection != -1 && cfg.elHomeDirection != 1) { why = "elHomeDirection"; return false; }
  why = ""; return true;
}

void driverEnable(bool enabled) {
  if (!configured) enabled = false;
  digitalWrite(cfg.enablePin, enabled == cfg.enableActiveLow ? LOW : HIGH);
}

bool limitActive(int pin, bool activeLow) {
  if (pin < 0) return false;
  return digitalRead(pin) == (activeLow ? LOW : HIGH);
}

void printConfig(Print &out) {
  out.printf("{\"ok\":true,\"configured\":%s,\"setupMode\":%s,\"config\":{",
    configured ? "true" : "false", setupMode ? "true" : "false");
  out.printf("\"azStepPin\":%d,\"azDirPin\":%d,\"elStepPin\":%d,\"elDirPin\":%d,\"enablePin\":%d,",
    cfg.azStepPin,cfg.azDirPin,cfg.elStepPin,cfg.elDirPin,cfg.enablePin);
  out.printf("\"azLimitPin\":%d,\"elLimitPin\":%d,\"motorSteps\":%.3f,\"microsteps\":%.3f,",
    cfg.azLimitPin,cfg.elLimitPin,cfg.motorSteps,cfg.microsteps);
  out.printf("\"azGearRatio\":%.6f,\"elGearRatio\":%.6f,\"azMin\":%.3f,\"azMax\":%.3f,\"elMin\":%.3f,\"elMax\":%.3f,",
    cfg.azGearRatio,cfg.elGearRatio,cfg.azMin,cfg.azMax,cfg.elMin,cfg.elMax);
  out.printf("\"maxSpeedAz\":%.3f,\"maxSpeedEl\":%.3f,\"accelerationAz\":%.3f,\"accelerationEl\":%.3f,",
    cfg.maxSpeedAz,cfg.maxSpeedEl,cfg.accelerationAz,cfg.accelerationEl);
  out.printf("\"homeSpeed\":%.3f,\"homeSlowSpeed\":%.3f,\"homeBackoffDeg\":%.3f,",
    cfg.homeSpeed,cfg.homeSlowSpeed,cfg.homeBackoffDeg);
  out.printf("\"azDirectionInvert\":%s,\"elDirectionInvert\":%s,\"enableActiveLow\":%s,",
    cfg.azDirectionInvert?"true":"false",cfg.elDirectionInvert?"true":"false",cfg.enableActiveLow?"true":"false");
  out.printf("\"azLimitActiveLow\":%s,\"elLimitActiveLow\":%s,\"allowStrappingPins\":%s,\"elHomeDirection\":%d,\"homingMode\":%u}}\n",
    cfg.azLimitActiveLow?"true":"false",cfg.elLimitActiveLow?"true":"false",cfg.allowStrappingPins?"true":"false",cfg.elHomeDirection,cfg.homingMode);
}

bool findJsonValue(const String &line, const char *key, String &value) {
  String needle = "\"" + String(key) + "\"";
  int p = line.indexOf(needle); if (p < 0) return false;
  p = line.indexOf(':', p + needle.length()); if (p < 0) return false;
  ++p; while (p < (int)line.length() && isspace((unsigned char)line[p])) ++p;
  if (p >= (int)line.length()) return false;
  if (line[p] == '"') { int e = line.indexOf('"', p + 1); if (e < 0) return false; value = line.substring(p + 1, e); }
  else { int e = p; while (e < (int)line.length() && line[e] != ',' && line[e] != '}') ++e; value = line.substring(p, e); value.trim(); }
  return true;
}

bool setNumber(const String &line, const char *key, float &v) { String s; if (!findJsonValue(line,key,s)) return false; v=s.toFloat(); return true; }
bool setInt8(const String &line, const char *key, int8_t &v) { String s; if (!findJsonValue(line,key,s)) return false; v=(int8_t)s.toInt(); return true; }
bool setU8(const String &line, const char *key, uint8_t &v) { String s; if (!findJsonValue(line,key,s)) return false; v=(uint8_t)s.toInt(); return true; }
bool setBool(const String &line, const char *key, bool &v) { String s; if (!findJsonValue(line,key,s)) return false; v=(s=="true"||s=="1"); return true; }

void applyJsonFields(const String &s) {
  setInt8(s,"azStepPin",cfg.azStepPin); setInt8(s,"azDirPin",cfg.azDirPin);
  setInt8(s,"elStepPin",cfg.elStepPin); setInt8(s,"elDirPin",cfg.elDirPin);
  setInt8(s,"enablePin",cfg.enablePin); setInt8(s,"azLimitPin",cfg.azLimitPin); setInt8(s,"elLimitPin",cfg.elLimitPin);
  setNumber(s,"motorSteps",cfg.motorSteps); setNumber(s,"microsteps",cfg.microsteps);
  setNumber(s,"azGearRatio",cfg.azGearRatio); setNumber(s,"elGearRatio",cfg.elGearRatio);
  setNumber(s,"azMin",cfg.azMin); setNumber(s,"azMax",cfg.azMax); setNumber(s,"elMin",cfg.elMin); setNumber(s,"elMax",cfg.elMax);
  setNumber(s,"maxSpeedAz",cfg.maxSpeedAz); setNumber(s,"maxSpeedEl",cfg.maxSpeedEl);
  setNumber(s,"accelerationAz",cfg.accelerationAz); setNumber(s,"accelerationEl",cfg.accelerationEl);
  setNumber(s,"homeSpeed",cfg.homeSpeed); setNumber(s,"homeSlowSpeed",cfg.homeSlowSpeed); setNumber(s,"homeBackoffDeg",cfg.homeBackoffDeg);
  setBool(s,"azDirectionInvert",cfg.azDirectionInvert); setBool(s,"elDirectionInvert",cfg.elDirectionInvert);
  setBool(s,"enableActiveLow",cfg.enableActiveLow); setBool(s,"azLimitActiveLow",cfg.azLimitActiveLow); setBool(s,"elLimitActiveLow",cfg.elLimitActiveLow);
  setBool(s,"allowStrappingPins",cfg.allowStrappingPins);
  setInt8(s,"elHomeDirection",cfg.elHomeDirection); setU8(s,"homingMode",cfg.homingMode);
}

void stopAll();

void handleJson(const String &line, Print &out) {
  String cmd;
  if (!findJsonValue(line,"cmd",cmd)) { jsonError(out,"missing_cmd"); return; }
  if (cmd == "get") { printConfig(out); return; }
  if (cmd == "enterSetup") {
    if (configured) {
      stopAll();
      driveEnableInactive(cfg.enablePin, cfg.enableActiveLow);
      configured = false;
    }
    setupMode = true;
    out.println("{\"ok\":true,\"setupMode\":true,\"driversDisabled\":true}");
    return;
  }
  if (cmd == "defaults") {
    if (!setupMode) { jsonError(out,"enter_setup_required"); return; }
    cfg=stockConfig();
    driveEnableInactive(cfg.enablePin,cfg.enableActiveLow);
    out.println("{\"ok\":true,\"unsaved\":true,\"driversDisabled\":true}"); return;
  }
  if (cmd == "set") {
    if (!setupMode) { jsonError(out,"enter_setup_required"); return; }
    applyJsonFields(line);
    String why;
    if (!validateConfig(why)) jsonError(out,"invalid_config",why);
    else {
      // Validation proves this is a legal output and not STEP/DIR. Disarm it
      // immediately, but do not construct steppers or emit a single step pulse.
      driveEnableInactive(cfg.enablePin, cfg.enableActiveLow);
      out.println("{\"ok\":true,\"unsaved\":true,\"driversDisabled\":true}");
    }
    return;
  }
  if (cmd == "validate") { String why; if (!validateConfig(why)) jsonError(out,"invalid_config",why); else out.println("{\"ok\":true,\"valid\":true}"); return; }
  if (cmd == "diagnose") {
    if (!setupMode) { jsonError(out,"enter_setup_required"); return; }
    String why; if (!validateConfig(why)) { jsonError(out,"invalid_config",why); return; }
    driveEnableInactive(cfg.enablePin, cfg.enableActiveLow);
    if (cfg.azLimitPin >= 0) pinMode(cfg.azLimitPin, cfg.azLimitActiveLow ? INPUT_PULLUP : INPUT);
    if (cfg.elLimitPin >= 0) pinMode(cfg.elLimitPin, cfg.elLimitActiveLow ? INPUT_PULLUP : INPUT);
    out.printf("{\"ok\":true,\"diagnostics\":true,\"driversDisabled\":true,\"stepPulses\":0,\"azLimit\":%s,\"elLimit\":%s}\n",
      limitActive(cfg.azLimitPin,cfg.azLimitActiveLow)?"true":"false",
      limitActive(cfg.elLimitPin,cfg.elLimitActiveLow)?"true":"false");
    return;
  }
  if (cmd == "save") {
    if (!setupMode) { jsonError(out,"enter_setup_required"); return; }
    String why; if (!validateConfig(why)) { jsonError(out,"invalid_config",why); return; }
    cfg.crc=configCrc(cfg); prefs.putBytes("config",&cfg,sizeof(cfg)); prefs.putBool("valid",true);
    out.println("{\"ok\":true,\"saved\":true,\"rebootRequired\":true}"); return;
  }
  if (cmd == "factoryReset") { driverEnable(false); prefs.clear(); out.println("{\"ok\":true,\"factoryReset\":true,\"rebootRequired\":true}"); return; }
  if (cmd == "reboot") { out.println("{\"ok\":true,\"rebooting\":true}"); delay(100); ESP.restart(); }
  jsonError(out,"unknown_cmd",cmd);
}

void stopAll() { tracking=false; if(az)az->stop(); if(el)el->stop(); }

void gotoPosition(float a, float e) {
  if (setupMode || !az || !el) return;
  e=constrain(e,cfg.elMin,cfg.elMax); a=constrain(a,cfg.azMin,cfg.azMax);
  targetAz=a; targetEl=e; tracking=false; driverEnable(true);
  az->setMaxSpeed(cfg.maxSpeedAz*azStepsPerDeg()); el->setMaxSpeed(cfg.maxSpeedEl*elStepsPerDeg());
  az->moveTo(lroundf(a*azStepsPerDeg())); el->moveTo(lroundf(e*elStepsPerDeg()));
}

void trackPosition(float a,float e,float ar,float er) {
  if (setupMode || !az || !el) return;
  e=constrain(e,cfg.elMin,cfg.elMax); a=constrain(a,cfg.azMin,cfg.azMax);
  targetAz=a; targetEl=e; tracking=true; lastTrackMs=millis(); driverEnable(true);
  float ac=constrain(ar+TRACK_KP*(a-currentAz()),-cfg.maxSpeedAz,cfg.maxSpeedAz);
  float ec=constrain(er+TRACK_KP*(e-currentEl()),-cfg.maxSpeedEl,cfg.maxSpeedEl);
  az->setMaxSpeed(fmaxf(fabsf(ac)*azStepsPerDeg(),1.0f)); el->setMaxSpeed(fmaxf(fabsf(ec)*elStepsPerDeg(),1.0f));
  az->moveTo(lroundf(a*azStepsPerDeg())); el->moveTo(lroundf(e*elStepsPerDeg()));
}

bool seekEl(bool slow) {
  float spd=elStepsPerDeg(); float rate=(slow?cfg.homeSlowSpeed:cfg.homeSpeed)*spd;
  el->setMaxSpeed(rate); el->move(cfg.elHomeDirection*lroundf(spd*(slow?30:400)));
  while(el->distanceToGo()!=0){ el->run(); if(limitActive(cfg.elLimitPin,cfg.elLimitActiveLow)){el->stop();while(el->distanceToGo())el->run();return true;} yield(); }
  return false;
}

void backoffEl() {
  el->setMaxSpeed(cfg.homeSpeed*elStepsPerDeg());
  el->move(-cfg.elHomeDirection*lroundf(cfg.homeBackoffDeg*elStepsPerDeg()));
  while(el->distanceToGo()){el->run();yield();}
}

bool home() {
  if(setupMode||!az||!el||cfg.homingMode==HOME_DISABLED)return false;
  driverEnable(true); az->setCurrentPosition(0);
  if(cfg.homingMode==HOME_MANUAL_ALL){el->setCurrentPosition(0);homed=true;return true;}
  if(limitActive(cfg.elLimitPin,cfg.elLimitActiveLow))backoffEl();
  if(!seekEl(false)){homed=false;return false;} backoffEl();
  if(!seekEl(true)){homed=false;return false;} el->setCurrentPosition(0); backoffEl(); el->setCurrentPosition(0);
  homed=true;return true;
}

void telemetry(Print &out) {
  out.printf("T %.6f %.6f %.6f %.6f azSteps=%ld elSteps=%ld state=%s seq=%lu homed=%d configured=%d\n",
    currentAz(),currentEl(),az?az->speed()/azStepsPerDeg():0,el?el->speed()/elStepsPerDeg():0,
    az?az->currentPosition():0,el?el->currentPosition():0,tracking?"track":"idle",(unsigned long)lastSequence,homed,configured);
}

void handleSuperrot(String line, Print &out) {
  line.trim(); if(!line.length())return;
  if(line[0]=='{'){handleJson(line,out);return;}
  if(setupMode){jsonError(out,"setup_mode","save valid configuration and reboot");return;}
  unsigned long seq; float a,b,c,d;
  if(line.startsWith("A2 ")){if(sscanf(line.c_str()+2,"%lu %f %f %f %f",&seq,&a,&b,&c,&d)==5){if(tracking&&lastSequence&&(int32_t)(seq-lastSequence)<=0){out.print("ERR stale_seq\n");return;}lastSequence=seq;trackPosition(a,b,c,d);out.print("OK\n");}else out.print("ERR bad_A2\n");return;}
  switch(line[0]){
    case 'A': if(sscanf(line.c_str()+1,"%f %f %f %f",&a,&b,&c,&d)==4){trackPosition(a,b,c,d);out.print("OK\n");}else out.print("ERR bad_A\n");break;
    case 'P': if(sscanf(line.c_str()+1,"%f %f",&a,&b)==2){gotoPosition(a,b);out.print("OK\n");}else out.print("ERR bad_P\n");break;
    case 'S': stopAll();out.print("OK\n");break;
    case 'H': out.print(home()?"OK\n":"ERR home_failed\n");break;
    case 'K': gotoPosition(0,0);out.print("OK\n");break;
    case '?': telemetry(out);break;
    default: out.print("ERR unknown\n");
  }
}

void handleStream(Stream &stream, Print &reply, String &buffer) {
  while(stream.available()){
    char c=stream.read();
    if(c=='\n'||c=='\r'){if(buffer.length()){handleSuperrot(buffer,reply);buffer="";}}
    else if(buffer.length()<1023)buffer+=c;
  }
}

void initializeHardware() {
  pinMode(cfg.enablePin,OUTPUT); configured=true; driverEnable(false);
  if(cfg.azLimitPin>=0)pinMode(cfg.azLimitPin,cfg.azLimitActiveLow?INPUT_PULLUP:INPUT);
  if(cfg.elLimitPin>=0)pinMode(cfg.elLimitPin,cfg.elLimitActiveLow?INPUT_PULLUP:INPUT);
  az=new AccelStepper(AccelStepper::DRIVER,cfg.azStepPin,cfg.azDirPin);
  el=new AccelStepper(AccelStepper::DRIVER,cfg.elStepPin,cfg.elDirPin);
  az->setPinsInverted(cfg.azDirectionInvert,false,false); el->setPinsInverted(cfg.elDirectionInvert,false,false);
  az->setMaxSpeed(cfg.maxSpeedAz*azStepsPerDeg()); el->setMaxSpeed(cfg.maxSpeedEl*elStepsPerDeg());
  az->setAcceleration(cfg.accelerationAz*azStepsPerDeg()); el->setAcceleration(cfg.accelerationEl*elStepsPerDeg());
  setupMode=false;
}

void setup() {
  // We cannot know an arbitrary custom EN pin before loading its saved config.
  // We can and do make the documented stock CNC-shield preset safe immediately.
  pinMode(STOCK_ENABLE_PIN, OUTPUT);
  digitalWrite(STOCK_ENABLE_PIN, STOCK_ENABLE_ACTIVE_LOW ? HIGH : LOW);
  Serial.begin(SERIAL_BAUD); delay(150);
  prefs.begin("superrot-u",false); cfg=stockConfig();
  RotatorConfig stored{}; size_t n=prefs.getBytes("config",&stored,sizeof(stored));
  bool marker=prefs.getBool("valid",false);
  if(n==sizeof(stored)&&marker&&stored.crc==configCrc(stored)){cfg=stored;String why;if(validateConfig(why))initializeHardware();else jsonError(Serial,"invalid_saved_config",why);}
  if(setupMode){Serial.println("{\"event\":\"setup_mode\",\"reason\":\"configuration_required\"}");return;}
  WiFi.mode(WIFI_AP); WiFi.softAP("SuperRot-Setup"); superrotServer.begin();
  Serial.println("{\"event\":\"ready\",\"protocol\":\"SuperRotUniversal/1\"}");
}

void loop() {
  static String serialLine,tcpLine; static uint32_t lastT=0;
  handleStream(Serial,Serial,serialLine);
  if(!setupMode){
    if(!superrotClient||!superrotClient.connected()){if(superrotClient)superrotClient.stop();superrotClient=superrotServer.available();}
    else handleStream(superrotClient,superrotClient,tcpLine);
    az->run();el->run();
    if(tracking&&millis()-lastTrackMs>COMMAND_TIMEOUT_MS)stopAll();
    if(millis()-lastT>=100){lastT=millis();telemetry(Serial);}
  }
}
