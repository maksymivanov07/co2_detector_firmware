#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SensirionI2cScd4x.h>
#include <esp32-hal-ledc.h>
#include <math.h>
#include <ArduinoJson.h>
#ifndef CO2_HOST_TEST
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <HomeSpan.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <atomic>
#include <esp_timer.h>
#include <time.h>
#include "web_ui.h"
#endif

// Installed wiring; confirm board model before uploading. Arduino-ESP32 >= 3.3.0.
constexpr uint8_t I2C_SDA = 8, I2C_SCL = 9, OLED_SCL = 3, OLED_SDA = 4;
constexpr uint8_t SCD_ADDRESS = 0x62, LIGHT_SENSOR_PIN = 2;
constexpr uint8_t LED_PINS[3] = {20, 5, 6};
constexpr uint8_t LED_RES = 12;
constexpr uint32_t LED_FREQ = 5000, LED_MAX = 4095, LED_STEP_MS = 20;
constexpr int DARK_ADC = 2219, BRIGHT_ADC = 2224;
constexpr uint32_t SENSOR_RETRY_MS = 10000, LIGHT_POLL_MS = 100;
constexpr uint32_t DISPLAY_MS = 250;
constexpr uint16_t HISTORY_SIZE = 1440;
constexpr const char* FIRMWARE_VERSION = "4.1.2";
constexpr const char* FIRMWARE_MARKER = "CO2_MONITOR_API_V1";

struct ArchiveHeader;
bool inspect_archive(const char* path, ArchiveHeader& header);

struct Settings {
  uint16_t schema = 1;
  int dark_adc = DARK_ADC, bright_adc = BRIGHT_ADC;
  float light_low = 0.3f, light_high = 0.7f, gamma = 1.6f;
  int led_day = 200, led_night = 1, oled_day = 200, oled_night = 60;
  int gains[3] = {160, 180, 255};
  int colors[5][3] = {{0,60,255},{0,0,255},{160,0,255},{255,50,0},{255,0,0}};
  int thresholds[4] = {300,600,1000,1500};
  int co2_hysteresis = 25, alarm_ppm = 1500, alarm_seconds = 60;
  int vent_target = 800;
  bool led_enabled = true, smooth_light = true, oled_sleep = false;
  bool alarm_pulse = false, gesture = false, eco = false, archive = false;
  bool wifi_enabled = true, homekit = true, sensor_override = false, asc = true;
  float wifi_tx_dbm = 13.0f;
  float temperature_offset = 4.0f;
  int altitude = 0, pressure_pa = 0;
  char timezone[64] = "EET-2EEST,M3.5.0/3,M10.5.0/4";
  char ssid[33] = "", wifi_password[65] = "";
  bool mqtt_enabled = false;
  char mqtt_host[65] = "", mqtt_user[65] = "", mqtt_password[65] = "";
  int mqtt_port = 1883;
} cfg;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
SensirionI2cScd4x scd4x;
enum class SensorPhase : uint8_t { retry, stopping, command_wait, measuring };
enum class SensorCommand : uint8_t { none, self_test, recalibrate };
struct SensorState {
  SensorPhase phase = SensorPhase::retry;
  SensorCommand command = SensorCommand::none;
  uint32_t phase_ms = 0, poll_ms = 0, sample_ms = 0, failures = 0, restarts = 0;
  uint16_t co2 = 0, reference = 0;
  float temperature = 0, humidity = 0, offset = 0;
  uint16_t altitude = 0, asc = 0;
  uint64_t serial = 0;
  uint8_t errors = 0;
  bool valid = false;
  char result[96] = "Not run";
} sensor;
struct LightState {
  float filtered = 0, normalized = 0, brightness = 0;
  int raw = 0;
  bool night = true, pending = false, candidate = true;
  uint32_t poll_ms = 0, candidate_ms = 0, gesture_ms = 0;
  bool covered = false;
} light;
struct LedState {
  float current[3] = {0,0,0};
  uint32_t duty[3] = {0,0,0}, step_ms = 0, preview_ms = 0;
  bool attached[3] = {false,false,false};
  int band = -1, preview = -1;
} led;
struct HistoryRecord {
  uint32_t epoch = 0, uptime_minute = 0;
  uint16_t co2 = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t night = 0, valid = 0;
};
HistoryRecord history[HISTORY_SIZE];
uint16_t history_head = 0, history_count = 0;
struct MinuteState {
  uint32_t last_ms = 0, count = 0, co2_sum = 0;
  float temperature_sum = 0, humidity_sum = 0;
} minute_state;
struct PeriodStats {
  uint64_t sum = 0;
  uint32_t seconds = 0, above_seconds = 0;
  uint16_t peak = 0;
} night_stats, last_night;
struct AnalysisState {
  bool alarm = false, alarm_pending = false, low_pending = false, suspicious = false;
  bool night_active = false, vent_active = false, vent_done = false, vent_pending = false;
  uint32_t alarm_ms = 0, low_ms = 0, tick_ms = 0, vent_ms = 0, vent_below_ms = 0;
  uint32_t vent_seconds = 0;
  uint16_t vent_start = 0, vent_current = 0, peak = 0;
  float slope = 0;
  bool trend_valid = false;
} analysis;
struct CalibrationState {
  bool active = false, dark_ready = false, bright_ready = false;
  int stage = 0, count = 0, minimum = 4095, maximum = 0;
  uint32_t started_ms = 0;
  double sum = 0, sum_squared = 0;
  float dark = 0, bright = 0, dark_noise = 0, bright_noise = 0;
  char result[96] = "Not started";
} calibration;
struct OpticalTest {
  bool active = false;
  int stage = 0, count = 0;
  uint32_t stage_ms = 0;
  float sum = 0, means[7] = {0,0,0,0,0,0,0};
} optical;
struct EventRecord { uint32_t seconds; char message[96]; };
EventRecord events[16];
uint8_t event_head = 0, event_count = 0;
bool display_dirty = true;
uint32_t display_ms = 0, log_ms = 0, screen_ms = 0, wake_ms = 0, brightness_preview_ms = 0;
int preview_led_brightness = -1, preview_oled_brightness = -1;
int screen_page = 0, previous_contrast = -1;
bool screen_awake = false, display_sleeping = false;
uint64_t uptime_ms = 0;
uint32_t uptime_previous = 0;

// Explicit declarations avoid Arduino auto-prototype ordering for custom types.
bool save_settings();
void archive_history();
uint32_t epoch_now();
void platform_setup();
void platform_loop();
void draw_network_screen();
int network_icon_state();
void settings_json(JsonObject out, bool secrets);
void append_history(const HistoryRecord& record);
void stats_json(JsonObject out, const PeriodStats& stats);
void apply_runtime_settings(const Settings& previous);
bool apply_settings(JsonObjectConst input, char* error, size_t size);

inline bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}
inline uint32_t stale_ms() { return cfg.eco ? 90000 : 15000; }
inline bool sample_fresh(uint32_t now) {
  return sensor.valid && sensor.phase == SensorPhase::measuring && !elapsed(now, sensor.sample_ms, stale_ms());
}
void event_log(const char* message) {
  events[event_head].seconds = uptime_ms / 1000;
  snprintf(events[event_head].message, sizeof(events[event_head].message), "%s", message);
  event_head = (event_head + 1) % 16;
  if (event_count < 16) ++event_count;
  Serial.println(message);
}
void wake_display(int page) {
  screen_page = page;
  screen_ms = wake_ms = millis();
  screen_awake = true;
  display_dirty = true;
}

void settings_json(JsonObject out, bool secrets) {
  out["schema"] = cfg.schema;
#define OUT(name) out[#name] = cfg.name
  OUT(dark_adc); OUT(bright_adc); OUT(light_low); OUT(light_high); OUT(gamma);
  OUT(led_day); OUT(led_night); OUT(oled_day); OUT(oled_night); OUT(led_enabled);
  OUT(smooth_light); OUT(oled_sleep); OUT(alarm_pulse); OUT(gesture); OUT(eco); OUT(archive);
  OUT(co2_hysteresis); OUT(alarm_ppm); OUT(alarm_seconds); OUT(vent_target);
  OUT(wifi_tx_dbm); OUT(wifi_enabled); OUT(homekit); OUT(sensor_override); OUT(asc); OUT(temperature_offset); OUT(altitude); OUT(pressure_pa);
  OUT(timezone); OUT(ssid); OUT(mqtt_enabled); OUT(mqtt_host); OUT(mqtt_user); OUT(mqtt_port);
  if (secrets) { OUT(wifi_password); OUT(mqtt_password); }
#undef OUT
  JsonArray gains = out["gains"].to<JsonArray>();
  JsonArray thresholds = out["thresholds"].to<JsonArray>();
  JsonArray colors = out["colors"].to<JsonArray>();
  for (int i = 0; i < 3; ++i) gains.add(cfg.gains[i]);
  for (int i = 0; i < 4; ++i) thresholds.add(cfg.thresholds[i]);
  for (int i = 0; i < 5; ++i) {
    JsonArray color = colors.add<JsonArray>();
    for (int j = 0; j < 3; ++j) color.add(cfg.colors[i][j]);
  }
}

bool apply_settings(JsonObjectConst input, char* error, size_t size) {
  Settings next = cfg;
  auto fail = [&](const char* name) { snprintf(error,size,"Invalid setting: %s",name); return false; };
  // Reject unknown keys and wrong types; all changes are atomic.
  JsonDocument known;
  settings_json(known.to<JsonObject>(), true);
  for (JsonPairConst item : input) if (!known[item.key()].is<JsonVariant>()) return fail(item.key().c_str());
#define INTEGER(name,lo,hi) if (input[#name].is<JsonVariantConst>()) { if (!input[#name].is<int>() || input[#name].as<int>() < lo || input[#name].as<int>() > hi) return fail(#name); next.name = input[#name].as<int>(); }
#define FLOATING(name,lo,hi) if (input[#name].is<JsonVariantConst>()) { if (!input[#name].is<float>() || !isfinite(input[#name].as<float>()) || input[#name].as<float>() < lo || input[#name].as<float>() > hi) return fail(#name); next.name = input[#name].as<float>(); }
#define BOOLEAN(name) if (input[#name].is<JsonVariantConst>()) { if (!input[#name].is<bool>()) return fail(#name); next.name = input[#name].as<bool>(); }
#define TEXT(name) if (input[#name].is<JsonVariantConst>()) { if (!input[#name].is<const char*>()) return fail(#name); const char* s = input[#name]; if (strlen(s) >= sizeof(next.name)) return fail(#name); for (const char* p=s; *p; ++p) if (static_cast<unsigned char>(*p)<32) return fail(#name); strcpy(next.name,s); }
  INTEGER(schema,1,1); INTEGER(dark_adc,0,4094); INTEGER(bright_adc,1,4095);
  FLOATING(light_low,0.01f,0.95f); FLOATING(light_high,0.05f,0.99f); FLOATING(gamma,1.0f,3.0f);
  INTEGER(led_day,0,255); INTEGER(led_night,0,255); INTEGER(oled_day,0,255); INTEGER(oled_night,0,255);
  INTEGER(co2_hysteresis,0,100); INTEGER(alarm_ppm,400,10000); INTEGER(alarm_seconds,5,1800);
  INTEGER(vent_target,400,2000); INTEGER(altitude,0,3000); INTEGER(pressure_pa,0,120000);
  FLOATING(wifi_tx_dbm,8.5f,19.5f);
  FLOATING(temperature_offset,0.0f,20.0f); INTEGER(mqtt_port,1,65535);
  BOOLEAN(led_enabled); BOOLEAN(smooth_light); BOOLEAN(oled_sleep); BOOLEAN(alarm_pulse);
  BOOLEAN(gesture); BOOLEAN(eco); BOOLEAN(archive); BOOLEAN(wifi_enabled); BOOLEAN(homekit); BOOLEAN(sensor_override); BOOLEAN(asc); BOOLEAN(mqtt_enabled);
  TEXT(timezone); TEXT(ssid); TEXT(wifi_password); TEXT(mqtt_host); TEXT(mqtt_user); TEXT(mqtt_password);
#undef INTEGER
#undef FLOATING
#undef BOOLEAN
#undef TEXT
  auto array = [&](const char* key, int* dest, int count, int lo, int hi) {
    if (!input[key].is<JsonVariantConst>()) return true;
    JsonArrayConst a = input[key].as<JsonArrayConst>();
    if (a.isNull() || static_cast<int>(a.size()) != count) return false;
    for (int i=0;i<count;++i) {
      if (!a[i].is<int>() || a[i].as<int>()<lo || a[i].as<int>()>hi) return false;
      dest[i]=a[i];
    }
    return true;
  };
  if (!array("gains",next.gains,3,0,255)) return fail("gains");
  if (!array("thresholds",next.thresholds,4,200,10000)) return fail("thresholds");
  if (input["colors"].is<JsonVariantConst>()) {
    JsonArrayConst colors=input["colors"].as<JsonArrayConst>();
    if (colors.isNull() || colors.size()!=5) return fail("colors");
    for (int i=0;i<5;++i) {
      JsonArrayConst c=colors[i].as<JsonArrayConst>();
      if (c.isNull() || c.size()!=3) return fail("colors");
      for (int j=0;j<3;++j) {
        if (!c[j].is<int>() || c[j].as<int>()<0 || c[j].as<int>()>255) return fail("colors");
        next.colors[i][j]=c[j];
      }
    }
  }
  if (next.bright_adc<=next.dark_adc) return fail("bright_adc > dark_adc");
  if (next.light_low>=next.light_high) return fail("light_low < light_high");
  if (next.pressure_pa && next.pressure_pa<70000) return fail("pressure_pa: 0 or 70000..120000");
  if (next.led_night>next.led_day || next.oled_night>next.oled_day) return fail("night brightness <= day brightness");
  for (int i=1;i<4;++i) if (next.thresholds[i]-next.thresholds[i-1] <= 2*next.co2_hysteresis) return fail("threshold spacing");
  if (next.mqtt_enabled && !strlen(next.mqtt_host)) return fail("mqtt_host");
  const size_t password_length = strlen(next.wifi_password);
  if (password_length && (password_length<8 || password_length>63)) return fail("wifi_password: 8..63 or empty");
  cfg=next;
  return true;
}

int read_light_adc() {
  uint32_t sum=0;
  for (int i=0;i<16;++i) sum+=analogRead(LIGHT_SENSOR_PIN);
  return (sum+8)/16;
}
void update_light(uint32_t now) {
  if (!elapsed(now,light.poll_ms,LIGHT_POLL_MS)) return;
  light.poll_ms=now;
  light.raw=read_light_adc();
  if (optical.active) {
    // Stages: off, R, G, B, OLED, combined, final off. Discard settling time.
    if (elapsed(now,optical.stage_ms,500)) { optical.sum+=light.raw; ++optical.count; }
    if (elapsed(now,optical.stage_ms,3000)) {
      optical.means[optical.stage]=optical.count ? optical.sum/optical.count : 0;
      optical.sum=0; optical.count=0; optical.stage_ms=now;
      if (++optical.stage>=7) { optical.active=false; event_log("Optical test completed"); }
      previous_contrast=-1; display_dirty=true;
    }
    return;
  }
  if (calibration.active) {
    calibration.sum+=light.raw; calibration.sum_squared+=static_cast<double>(light.raw)*light.raw;
    calibration.minimum=min(calibration.minimum,light.raw); calibration.maximum=max(calibration.maximum,light.raw); ++calibration.count;
    if (elapsed(now,calibration.started_ms,5000)) {
      const float mean=calibration.sum/calibration.count;
      const float noise=sqrtf(max(0.0,calibration.sum_squared/calibration.count-mean*mean));
      if (calibration.stage==0) { calibration.dark=mean; calibration.dark_noise=noise; calibration.dark_ready=true; }
      else { calibration.bright=mean; calibration.bright_noise=noise; calibration.bright_ready=true; }
      calibration.active=false;
      snprintf(calibration.result,sizeof(calibration.result),"%s captured: %.1f +/- %.1f ADC",calibration.stage==0?"Dark":"Bright",mean,noise);
      event_log(calibration.result);
    }
  }
  // Fast dimming (time constant ~0.45s), slower brightening (~2.5s).
  const float alpha=light.raw<light.filtered ? 0.2f : 0.04f;
  light.filtered+=alpha*(light.raw-light.filtered);
  light.normalized=constrain((light.filtered-cfg.dark_adc)/(cfg.bright_adc-cfg.dark_adc),0.0f,1.0f);
  const bool desired=light.normalized<cfg.light_low ? true : light.normalized>cfg.light_high ? false : light.night;
  if (desired==light.night) light.pending=false;
  else if (!light.pending || light.candidate!=desired) {
    light.pending=true; light.candidate=desired; light.candidate_ms=now;
  } else if (elapsed(now,light.candidate_ms,5000)) {
    light.night=desired; light.pending=false; display_dirty=true;
    event_log(light.night?"Night mode":"Day mode");
  }
  float target=light.night ? 0.0f : cfg.smooth_light ? powf(light.normalized,cfg.gamma) : 1.0f;
  light.brightness+=(target<light.brightness?0.2f:0.04f)*(target-light.brightness);
  // Gesture is an opt-in cover/release in established daylight, never an admin action.
  if (cfg.gesture && !calibration.active && !light.night && cfg.bright_adc-cfg.dark_adc>=20) {
    const float raw_norm=static_cast<float>(light.raw-cfg.dark_adc)/(cfg.bright_adc-cfg.dark_adc);
    if (!light.covered && raw_norm<cfg.light_low && light.normalized>cfg.light_high) { light.covered=true; light.gesture_ms=now; }
    if (light.covered && raw_norm>cfg.light_high) {
      const uint32_t duration=now-light.gesture_ms;
      if (duration>=300 && duration<=2000) wake_display((screen_page+1)%4);
      light.covered=false;
    }
    if (light.covered && elapsed(now,light.gesture_ms,2000)) light.covered=false;
  } else light.covered=false;
}

void sensor_retry(uint32_t now, const char* operation, int error) {
  char message[96]; snprintf(message,sizeof(message),"SCD40 %s: %d; retry in 10s",operation,error); event_log(message);
  sensor.phase=SensorPhase::retry; sensor.phase_ms=now; sensor.valid=false; sensor.errors=0;
  if (sensor.command!=SensorCommand::none) snprintf(sensor.result,sizeof(sensor.result),"Failed: %s (%d)",operation,error);
  sensor.command=SensorCommand::none; ++sensor.restarts; display_dirty=true;
}
void sensor_error(uint32_t now, const char* operation, int error) {
  ++sensor.failures;
  if (++sensor.errors>=3) sensor_retry(now,operation,error);
}
uint8_t scd_crc(uint16_t word) {
  uint8_t crc=0xff;
  for (int shift=8;shift>=0;shift-=8) {
    crc^=static_cast<uint8_t>(word>>shift);
    for (int b=0;b<8;++b) crc=(crc&0x80)?(crc<<1)^0x31:crc<<1;
  }
  return crc;
}
int send_sensor_command(uint16_t command, bool argument=false, uint16_t value=0) {
  Wire.beginTransmission(SCD_ADDRESS); Wire.write(command>>8); Wire.write(command&255);
  if (argument) { Wire.write(value>>8); Wire.write(value&255); Wire.write(scd_crc(value)); }
  return Wire.endTransmission();
}
void on_sample(uint32_t now) {
  ++minute_state.count; minute_state.co2_sum+=sensor.co2;
  minute_state.temperature_sum+=sensor.temperature; minute_state.humidity_sum+=sensor.humidity;
  analysis.peak=max(analysis.peak,sensor.co2);
  (void)now;
}
void update_sensor(uint32_t now) {
  if (sensor.phase==SensorPhase::retry) {
    if (!elapsed(now,sensor.phase_ms,SENSOR_RETRY_MS)) return;
    Wire.end();
    if (!Wire.begin(I2C_SDA,I2C_SCL,100000)) { sensor_retry(now,"I2C begin",-1); return; }
    Wire.setTimeOut(50);
    const int error=send_sensor_command(0x3f86);
    if (error) { sensor_retry(now,"stop",error); return; }
    sensor.phase=SensorPhase::stopping; sensor.phase_ms=millis(); display_dirty=true; return;
  }
  if (sensor.phase==SensorPhase::stopping) {
    if (!elapsed(now,sensor.phase_ms,500)) return;
    if (sensor.command!=SensorCommand::none) {
      int error=send_sensor_command(sensor.command==SensorCommand::self_test?0x3639:0x362f,sensor.command==SensorCommand::recalibrate,sensor.reference);
      if (error) { sensor_retry(now,"service command",error); return; }
      sensor.phase=SensorPhase::command_wait; sensor.phase_ms=millis(); return;
    }
    int16_t error=0;
    if (cfg.sensor_override) {
      error=scd4x.setTemperatureOffset(cfg.temperature_offset);
      if (!error) error=scd4x.setSensorAltitude(cfg.altitude);
      if (!error) error=scd4x.setAutomaticSelfCalibrationEnabled(cfg.asc?1:0);
    }
    if (!error) error=scd4x.getSerialNumber(sensor.serial);
    if (!error) error=scd4x.getTemperatureOffset(sensor.offset);
    if (!error) error=scd4x.getSensorAltitude(sensor.altitude);
    if (!error) error=scd4x.getAutomaticSelfCalibrationEnabled(sensor.asc);
    if (!error) error=cfg.eco?scd4x.startLowPowerPeriodicMeasurement():scd4x.startPeriodicMeasurement();
    if (!error && cfg.sensor_override && cfg.pressure_pa) error=scd4x.setAmbientPressure(cfg.pressure_pa);
    if (error) { sensor_retry(now,"configure/start",error); return; }
    sensor.phase=SensorPhase::measuring; sensor.phase_ms=sensor.poll_ms=millis();
    event_log(cfg.eco?"SCD40 measuring every 30s":"SCD40 measuring every 5s"); display_dirty=true; return;
  }
  if (sensor.phase==SensorPhase::command_wait) {
    if (!elapsed(now,sensor.phase_ms,sensor.command==SensorCommand::self_test?10000:400)) return;
    if (Wire.requestFrom(SCD_ADDRESS,static_cast<uint8_t>(3))!=3) { sensor_retry(now,"service response",-1); return; }
    uint16_t result=static_cast<uint16_t>(Wire.read())<<8; result|=Wire.read();
    if (Wire.read()!=scd_crc(result)) { sensor_retry(now,"service CRC",-1); return; }
    if (sensor.command==SensorCommand::self_test) snprintf(sensor.result,sizeof(sensor.result),"Self-test %s (0x%04x)",result==0?"passed":"FAILED",result);
    else if (result==0xffff) snprintf(sensor.result,sizeof(sensor.result),"Recalibration FAILED");
    else snprintf(sensor.result,sizeof(sensor.result),"Recalibrated: correction %d ppm",static_cast<int>(result)-32768);
    event_log(sensor.result); sensor.command=SensorCommand::none;
    sensor.phase=SensorPhase::stopping; sensor.phase_ms=now-500; return;
  }
  const uint32_t reference=sensor.valid?sensor.sample_ms:sensor.phase_ms;
  if (elapsed(now,reference,stale_ms())) { sensor_retry(now,"sample timeout",-1); return; }
  if (!elapsed(now,sensor.poll_ms,cfg.eco?5000:1000)) return;
  sensor.poll_ms=now;
  bool ready=false;
  int16_t error=scd4x.getDataReadyStatus(ready);
  if (error) { sensor_error(now,"ready",error); return; }
  if (!ready) return;
  uint16_t co2=0; float t=NAN,h=NAN;
  error=scd4x.readMeasurement(co2,t,h);
  if (error || co2==0 || co2>40000 || !isfinite(t) || !isfinite(h) || t < -10 || t>60 || h<0 || h>100) {
    sensor_error(now,"measurement",error?error:-1); return;
  }
  sensor.co2=co2; sensor.temperature=t; sensor.humidity=h; sensor.sample_ms=millis(); sensor.valid=true; sensor.errors=0;
  on_sample(now); display_dirty=true;
}

void compute_trend() {
  // Linear regression of the last five one-minute slots; no bridging missing data.
  analysis.trend_valid=false; analysis.slope=0;
  if (history_count<3) return;
  double sx=0,sy=0,sxx=0,sxy=0; int n=0;
  for (int back=min(static_cast<int>(history_count),5)-1;back>=0;--back) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-1-back)%HISTORY_SIZE];
    if (!r.valid) { sx=sy=sxx=sxy=0; n=0; continue; }
    double x=-back; sx+=x; sy+=r.co2; sxx+=x*x; sxy+=x*r.co2; ++n;
  }
  const double denominator=n*sxx-sx*sx;
  if (n>=3 && denominator>0) { analysis.slope=(n*sxy-sx*sy)/denominator; analysis.trend_valid=true; }
}
void append_history(const HistoryRecord& record) {
  history[history_head]=record; history_head=(history_head+1)%HISTORY_SIZE;
  if (history_count<HISTORY_SIZE) ++history_count;
}
void update_analytics(uint32_t now) {
  if (elapsed(now,minute_state.last_ms,60000)) {
    const uint32_t slots=(now-minute_state.last_ms)/60000;
    // A long blocked interval must become gaps, not a falsely continuous chart.
    for (uint32_t i=1;i<min(slots,static_cast<uint32_t>(HISTORY_SIZE));++i) {
      HistoryRecord gap; gap.uptime_minute=uptime_ms/60000-(slots-i); append_history(gap);
    }
    HistoryRecord record; record.epoch=epoch_now(); record.uptime_minute=uptime_ms/60000; record.night=light.night;
    if (minute_state.count && slots==1) {
      record.valid=1; record.co2=minute_state.co2_sum/minute_state.count;
      record.temperature=lroundf(minute_state.temperature_sum/minute_state.count*100);
      record.humidity=lroundf(minute_state.humidity_sum/minute_state.count*100);
    }
    append_history(record); minute_state=MinuteState{}; minute_state.last_ms=now;
    compute_trend(); display_dirty=true;
  }
  if (!elapsed(now,analysis.tick_ms,1000)) return;
  display_dirty=true;
  uint32_t seconds=min((now-analysis.tick_ms)/1000,static_cast<uint32_t>(2)); analysis.tick_ms=now;
  const bool fresh=sample_fresh(now);
  if (light.night && !analysis.night_active) { night_stats=PeriodStats{}; analysis.night_active=true; }
  if (!light.night && analysis.night_active) { last_night=night_stats; analysis.night_active=false; event_log("Night summary ready"); }
  if (analysis.night_active && fresh) {
    night_stats.sum+=static_cast<uint64_t>(sensor.co2)*seconds; night_stats.seconds+=seconds;
    if (sensor.co2>=cfg.alarm_ppm) night_stats.above_seconds+=seconds;
    night_stats.peak=max(night_stats.peak,sensor.co2);
  }
  if (!fresh) { analysis.alarm=analysis.alarm_pending=analysis.low_pending=analysis.suspicious=false; analysis.vent_pending=false; }
  else {
    if (sensor.co2>=cfg.alarm_ppm) {
      if (!analysis.alarm_pending) { analysis.alarm_pending=true; analysis.alarm_ms=now; }
      if (!analysis.alarm && elapsed(now,analysis.alarm_ms,cfg.alarm_seconds*1000UL)) { analysis.alarm=true; event_log("CO2 threshold exceeded persistently"); }
    } else {
      analysis.alarm_pending=false;
      if (sensor.co2<cfg.alarm_ppm-cfg.co2_hysteresis) analysis.alarm=false;
    }
    if (sensor.co2<300) {
      if (!analysis.low_pending) { analysis.low_pending=true; analysis.low_ms=now; }
      if (elapsed(now,analysis.low_ms,120000)) analysis.suspicious=true;
    } else analysis.low_pending=analysis.suspicious=false;
  }
  if (analysis.vent_active) {
    analysis.vent_seconds=(now-analysis.vent_ms)/1000;
    if (fresh) {
      analysis.vent_current=sensor.co2;
      if (sensor.co2<=cfg.vent_target) {
        if (!analysis.vent_pending) { analysis.vent_pending=true; analysis.vent_below_ms=now; }
        if (elapsed(now,analysis.vent_below_ms,60000)) { analysis.vent_active=false; analysis.vent_done=true; event_log("Ventilation target maintained for 60s"); }
      } else analysis.vent_pending=false;
    }
    if (analysis.vent_seconds>=86400) { analysis.vent_active=false; event_log("Ventilation session ended after 24h"); }
  }
}

int co2_band(uint16_t co2) {
  if (co2<cfg.thresholds[0]) return 0;
  for (int i=1;i<4;++i) if (co2<=cfg.thresholds[i]) return i;
  return 4;
}
void update_led(uint32_t now) {
  if (!elapsed(now,led.step_ms,LED_STEP_MS)) return;
  led.step_ms=now;
  if (led.preview>=0 && elapsed(now,led.preview_ms,10000)) led.preview=-1;
  const bool fresh=sample_fresh(now);
  if (fresh) {
    int desired=co2_band(sensor.co2);
    if (led.band<0) led.band=desired;
    else if (desired>led.band && sensor.co2>cfg.thresholds[led.band]+cfg.co2_hysteresis) led.band=desired;
    else if (desired<led.band && sensor.co2<cfg.thresholds[led.band-1]-cfg.co2_hysteresis) led.band=desired;
  } else led.band=-1;
  if (elapsed(now,brightness_preview_ms,10000)) preview_led_brightness=preview_oled_brightness=-1;
  float brightness=light.night ? cfg.led_night : cfg.led_night+(cfg.led_day-cfg.led_night)*light.brightness;
  if (cfg.alarm_pulse && analysis.alarm && !light.night) brightness*=0.7f+0.3f*sinf(now/1000.0f);
  if (preview_led_brightness>=0) brightness=preview_led_brightness;
  int color=led.preview>=0?led.preview:led.band;
  for (int i=0;i<3;++i) {
    float target=(color>=0 && cfg.led_enabled)?cfg.colors[color][i]:0;
    if (optical.active) { target=((optical.stage==i+1)||optical.stage==5)?255:0; brightness=cfg.led_day; }
    led.current[i]+=constrain(target-led.current[i],-3.0f,3.0f);
    float channel=optical.active?target:led.current[i];
    uint32_t duty=lroundf(channel*brightness*cfg.gains[i]*LED_MAX/(255.0f*255.0f*255.0f));
    if (!cfg.led_enabled && !optical.active) duty=0;
    duty=min(duty,LED_MAX);
    if (led.attached[i] && duty!=led.duty[i]) {
      if (ledcWrite(LED_PINS[i],duty)) led.duty[i]=duty;
    }
  }
}
void draw_history(int count, int y, int height) {
  count=min(count,static_cast<int>(history_count));
  if (count<2) return;
  uint16_t low=65535,high=0;
  for (int i=0;i<count;++i) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-count+i)%HISTORY_SIZE];
    if (r.valid) { low=min(low,r.co2); high=max(high,r.co2); }
  }
  if (!high) return;
  const int range=max(100,static_cast<int>(high-low));
  int last_x=-1,last_y=0;
  for (int i=0;i<count;++i) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-count+i)%HISTORY_SIZE];
    if (!r.valid) { last_x=-1; continue; }
    int x=i*124/(count-1)+1,py=y+height-1-(r.co2-low)*(height-1)/range;
    if (last_x>=0) display.drawLine(last_x,last_y,x,py);
    last_x=x; last_y=py;
  }
}
// Native monochrome icons; one bit per pixel, leftmost pixel in the high bit.
void draw_icon(int x,int y,const uint16_t* rows,int width,int height) {
  for (int row=0;row<height;++row) for (int col=0;col<width;++col)
    if (rows[row] & (1U<<(width-1-col))) display.drawLine(x+col,y+row,x+col,y+row);
}
void draw_status_icons() {
  static const uint16_t sun[]={0x010,0x092,0x054,0x038,0x17d,0x038,0x054,0x092,0x010};
  static const uint16_t moon[]={0x038,0x070,0x0e0,0x0c0,0x0c0,0x0e1,0x073,0x03e,0x01c};
  static const uint16_t wifi[]={0x07f0,0x180c,0x2002,0x03e0,0x0410,0x0808,0x00c0,0x0120,0x0000,0x0080};
  static const uint16_t house[]={0x10,0x38,0x6c,0xc6,0x44,0x54,0x54,0x7c};
  static const uint16_t phone[]={0x7c,0x44,0x44,0x44,0x44,0x44,0x54,0x7c};
  static const uint16_t waiting[]={0x7c,0x44,0x28,0x10,0x10,0x28,0x44,0x7c};
  draw_icon(69,1,light.night?moon:sun,9,9);
  draw_icon(89,0,wifi,14,10);
  const int state=network_icon_state();
  if (state==0) display.drawLine(89,10,103,0);
  else draw_icon(110,1,state==3?house:state==2?phone:waiting,8,8);
}
void update_display(uint32_t now) {
  if (!elapsed(now,display_ms,DISPLAY_MS)) return;
  display_ms=now;
  if (screen_awake && elapsed(now,wake_ms,60000)) { screen_awake=false; screen_page=0; display_dirty=true; }
  const bool sleeping=optical.active ? !(optical.stage==4||optical.stage==5) : cfg.oled_sleep && light.night && !screen_awake;
  if (sleeping!=display_sleeping) { display.setPowerSave(sleeping?1:0); display_sleeping=sleeping; display_dirty=true; }
  int contrast=optical.active ? cfg.oled_day : light.night ? cfg.oled_night : lroundf(cfg.oled_night+(cfg.oled_day-cfg.oled_night)*light.brightness);
  if (!optical.active && preview_oled_brightness>=0) contrast=preview_oled_brightness;
  if (contrast!=previous_contrast) { display.setContrast(contrast); previous_contrast=contrast; }
  static uint32_t shift_ms=0;
  if (elapsed(now,shift_ms,60000)) { shift_ms=now; display_dirty=true; }
  if (!display_dirty || sleeping) return;
  display_dirty=false;
  uint8_t old_buffer[1024]; memcpy(old_buffer,display.getBufferPtr(),sizeof(old_buffer));
  display.clearBuffer(); display.setFont(u8g2_font_6x12_tf);
  const int shift=(now/60000)%2;
  if (optical.active) {
    display.drawStr(shift,12,"Optical test"); display.drawStr(shift,32,"Keep room light"); display.drawStr(shift,48,"unchanged");
  } else if (screen_page==4) draw_network_screen();
  else if (screen_page==1 || screen_page==2) {
    display.drawStr(shift,11,screen_page==1?"CO2 / last hour":"CO2 / last 24h");
    draw_history(screen_page==1?60:HISTORY_SIZE,18,43);
  } else if (screen_page==3) {
    char line[32];
    if (analysis.vent_active || analysis.vent_done) {
      display.drawStr(shift,11,analysis.vent_active?"Ventilation":"Ventilation done");
      snprintf(line,sizeof(line),"%u -> %u ppm",analysis.vent_start,analysis.vent_current); display.drawStr(shift,32,line);
      snprintf(line,sizeof(line),"%lu min / target %d",static_cast<unsigned long>(analysis.vent_seconds/60),cfg.vent_target); display.drawStr(shift,52,line);
    } else {
      const PeriodStats& stats=analysis.night_active?night_stats:last_night;
      display.drawStr(shift,11,"Night summary");
      snprintf(line,sizeof(line),"Avg %u  Max %u",stats.seconds?static_cast<unsigned>(stats.sum/stats.seconds):0,stats.peak); display.drawStr(shift,32,line);
      snprintf(line,sizeof(line),"Above: %lu min",static_cast<unsigned long>(stats.above_seconds/60)); display.drawStr(shift,52,line);
    }
  } else if (sample_fresh(now)) {
    display.drawStr(shift,10,"CO2 ppm");
    draw_status_icons();
    char line[32]; snprintf(line,sizeof(line),"%u",sensor.co2);
    display.setFont(sensor.co2<1000?u8g2_font_logisoso42_tn:sensor.co2<10000?u8g2_font_logisoso32_tn:u8g2_font_logisoso24_tf);
    display.drawStr(shift,53,line);
    display.drawLine(82,15,82,52);
    display.setFont(u8g2_font_8x13B_tf);
    snprintf(line,sizeof(line),"%d",static_cast<int>(sensor.temperature));
    display.drawStr(85,29,line);
    const int degree_x=85+strlen(line)*8+1;
    static const uint16_t degree[]={2,5,2}; draw_icon(degree_x,18,degree,3,3);
    display.drawStr(degree_x+5,29,"C");
    snprintf(line,sizeof(line),"%.0f%%",sensor.humidity); display.drawStr(85,49,line);
    display.setFont(u8g2_font_6x12_tf);
    const char* trend=!analysis.trend_valid?"WAIT":analysis.slope>10?"RISING":analysis.slope < -10?"FALLING":"STEADY";
    const char* status=analysis.suspicious?"CHECK CO2":analysis.alarm?"VENTILATE":"CO2 trend";
    display.drawStr(shift,63,status); display.drawStr(85,63,trend);

  } else {
    display.drawStr(shift,11,"CO2 monitor");
    display.drawStr(shift,32,sensor.command!=SensorCommand::none?"Sensor service...":sensor.phase==SensorPhase::retry?"Sensor unavailable":"Waiting for sample");
    display.drawStr(shift,52,cfg.eco?"Eco: sample / 30s":"Sample / 5s");
  }
  // U8g2 full-buffer partial updates use 8x8 tiles. Only changed rows are sent.
  for (uint8_t row=0;row<8;++row) if (memcmp(old_buffer+row*128,display.getBufferPtr()+row*128,128)) display.updateDisplayArea(0,row,16,1);
}

void setup() {
  Serial.begin(115200);
  // Keep the lamp off while HomeSpan and persistent settings initialize.
  for (int i=0;i<3;++i) { pinMode(LED_PINS[i],OUTPUT); digitalWrite(LED_PINS[i],LOW); }
  platform_setup();
  for (int i=0;i<3;++i) {
    pinMode(LED_PINS[i],OUTPUT); digitalWrite(LED_PINS[i],LOW);
    led.attached[i]=ledcAttach(LED_PINS[i],LED_FREQ,LED_RES);
    if (led.attached[i]) ledcWrite(LED_PINS[i],0); else event_log("PWM attachment failed");
  }
  analogReadResolution(12); analogSetPinAttenuation(LIGHT_SENSOR_PIN,ADC_11db); pinMode(LIGHT_SENSOR_PIN,INPUT);
  light.raw=read_light_adc(); light.filtered=light.raw;
  light.normalized=constrain((light.filtered-cfg.dark_adc)/(cfg.bright_adc-cfg.dark_adc),0.0f,1.0f);
  light.night=light.normalized<=cfg.light_high; light.brightness=light.night?0:powf(light.normalized,cfg.gamma);
  display.begin(); display.setContrast(light.night?cfg.oled_night:cfg.oled_day); display.setPowerSave(0);
  scd4x.begin(Wire,SCD_ADDRESS);
  const uint32_t now=millis(); uptime_previous=now;
  sensor.phase_ms=now-SENSOR_RETRY_MS+1000; minute_state.last_ms=now;
  display_ms=now-DISPLAY_MS; light.poll_ms=now;
  wake_display(4); update_display(now);
  event_log("CO2 Monitor 4.1.2 started");
}
void loop() {
  uint32_t now=millis(); uptime_ms+=static_cast<uint32_t>(now-uptime_previous); uptime_previous=now;
  update_light(millis()); update_sensor(millis()); update_analytics(millis()); update_led(millis()); update_display(millis());
  platform_loop();
  if (elapsed(millis(),log_ms,5000)) {
    log_ms=millis(); Serial.printf("adc=%d filtered=%.1f fresh=%u co2=%u T=%.1f RH=%.1f\n",light.raw,light.filtered,sample_fresh(log_ms),sensor.co2,sensor.temperature,sensor.humidity);
  }
  delay(1);
}

#ifndef CO2_HOST_TEST
// Networking is kept in this sketch; only static web assets live in web_ui.h.
RTC_NOINIT_ATTR uint32_t diagnostic_magic;
RTC_NOINIT_ATTR uint32_t diagnostic_stage;
uint32_t previous_network_stage=0;
RTC_NOINIT_ATTR uint32_t retained_boot_count;
uint32_t boot_id=0, stage_started_ms=0, stage_max_ms[9]={};
std::atomic<uint32_t> wifi_disconnect_count{0}, wifi_got_ip_count{0};
std::atomic<uint32_t> wifi_disconnect_reason{0}, wifi_disconnect_ms{0};
std::atomic<int> wifi_disconnect_rssi{0};
void begin_network_stage(uint32_t stage) { diagnostic_stage=stage; stage_started_ms=millis(); }
void end_network_stage() {
  if (diagnostic_stage<9) stage_max_ms[diagnostic_stage]=max(stage_max_ms[diagnostic_stage],static_cast<uint32_t>(millis()-stage_started_ms));
  diagnostic_stage=0;
}

Preferences preferences;
// Bounded response writes: NetworkClient::write can wait for 10s or longer.
class BoundedWebServer : public WebServer {
public:
  using WebServer::WebServer;
  uint32_t response_started=0;
  bool response_active=false;
  uint32_t aborted_responses=0;
  void begin_cycle() { response_active=false; }
protected:
  size_t _currentClientWrite(const char* data,size_t length) override {
    if (!length) return 0;
    if (!response_active) { response_started=millis(); response_active=true; }
    size_t sent=0;
    uint32_t progress=millis();
    while (sent<length && _currentClient.connected()) {
      if (static_cast<uint32_t>(millis()-response_started)>=2500 || static_cast<uint32_t>(millis()-progress)>=500) break;
      int result=::send(_currentClient.fd(),data+sent,length-sent,MSG_DONTWAIT);
      if (result>0) { sent+=result; progress=millis(); }
      else if (result<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) break;
      else delay(1);
    }
    if (sent!=length) { ++aborted_responses; _currentClient.stop(); }
    return sent;
  }
  size_t _currentClientWrite_P(PGM_P data,size_t length) override { return _currentClientWrite(data,length); }
}; // bounded HTTP transport
BoundedWebServer web(80);
DNSServer dns;
WiFiClient mqtt_transport;
PubSubClient mqtt(mqtt_transport);
char device_id[24], ap_name[32], admin_password[17], pairing_code[9];
bool homekit_started=false, ap_active=false, mdns_started=false, fs_ready=false;
bool network_was_connected=false, reboot_pending=false, ota_upload=false, ota_failed=false;
bool archive_loaded=false, watchdog_ready=false, save_pending=false, storage_ready=false;
bool ota_verified=false, ota_marker_found=false;
size_t ota_marker_matched=0;
uint32_t config_revision=1;
uint32_t reboot_ms=0, disconnected_ms=0, connected_ms=0, archive_ms=0, mqtt_ms=0, mqtt_publish_ms=0;
uint32_t homekit_ms=0, health_ms=0, access_ms=0;
bool time_backfilled=false;
String last_network_error;
SpanCharacteristic *hk_co2=nullptr,*hk_detected=nullptr,*hk_peak=nullptr,*hk_temperature=nullptr,*hk_humidity=nullptr;
SpanCharacteristic *hk_fault[3]={nullptr,nullptr,nullptr},*hk_active[3]={nullptr,nullptr,nullptr},*hk_led=nullptr;

// Defer bootloader validation until the complete firmware passes its boot health check.
extern "C" bool verifyRollbackLater() { return true; }

uint32_t epoch_now() {
  const time_t value=time(nullptr);
  return value>1700000000 ? static_cast<uint32_t>(value) : 0;
}
bool save_settings() {
  JsonDocument document; settings_json(document.to<JsonObject>(),true);
  String data; serializeJson(document,data);
  if (preferences.getString("settings","")==data) return true;
  bool saved=preferences.putString("settings",data)==data.length();
  if (saved) ++config_revision;
  return saved;
}
void schedule_reboot() { reboot_pending=true; reboot_ms=millis(); }

struct HomeLed : Service::Switch {
  HomeLed() {
    new Characteristic::ConfiguredName("Monitor backlight");
    hk_led=new Characteristic::On(cfg.led_enabled);
  }
  boolean update() override {
    const bool enabled=hk_led->getNewVal();
    if (cfg.led_enabled!=enabled) { cfg.led_enabled=enabled; ++config_revision; save_pending=true; }
    return true;
  }
};
void start_homekit() {
  if (!cfg.wifi_enabled || !cfg.homekit || !strlen(cfg.ssid)) return;
  homeSpan.setLogLevel(-1).setSerialInputDisable(true).setPortNum(1201).setHostNameSuffix("");
  homeSpan.setWifiCredentials(cfg.ssid,cfg.wifi_password);
  homeSpan.setWifiBegin([](const char* ssid,const char* password) { WiFi.begin(ssid,password); WiFi.setTxPower(static_cast<wifi_power_t>(lroundf(cfg.wifi_tx_dbm*4))); });
  if (!preferences.getBool("hk_code",false)) {
    homeSpan.setPairingCode(pairing_code);
    preferences.putBool("hk_code",true);
  }
  homeSpan.begin(Category::Sensors,"CO2 Monitor",device_id,"SCD40 Monitor");
  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify(); new Characteristic::Name("CO2 Monitor");
  new Characteristic::Manufacturer("DIY"); new Characteristic::Model("SCD40 ESP32");
  new Characteristic::SerialNumber(device_id); new Characteristic::FirmwareRevision(FIRMWARE_VERSION);
  new Service::CarbonDioxideSensor();
  new Characteristic::ConfiguredName("CO2");
  hk_detected=new Characteristic::CarbonDioxideDetected(0);
  hk_co2=new Characteristic::CarbonDioxideLevel(0); hk_peak=new Characteristic::CarbonDioxidePeakLevel(0);
  hk_fault[0]=new Characteristic::StatusFault(1); hk_active[0]=new Characteristic::StatusActive(false);
  new Service::TemperatureSensor(); new Characteristic::ConfiguredName("Temperature");
  hk_temperature=new Characteristic::CurrentTemperature(0); hk_temperature->setRange(-10,60,0.1);
  hk_fault[1]=new Characteristic::StatusFault(1); hk_active[1]=new Characteristic::StatusActive(false);
  new Service::HumiditySensor(); new Characteristic::ConfiguredName("Humidity");
  hk_humidity=new Characteristic::CurrentRelativeHumidity(0);
  hk_fault[2]=new Characteristic::StatusFault(1); hk_active[2]=new Characteristic::StatusActive(false);
  new HomeLed();
  homekit_started=true;
}
void update_homekit(uint32_t now) {
  if (!homekit_started) return;
  homeSpan.poll();
  if (!elapsed(now,homekit_ms,1000)) return;
  homekit_ms=now;
  const bool fresh=sample_fresh(now);
  for (int i=0;i<3;++i) {
    if (hk_fault[i]->getVal<int>()!=(fresh?0:1)) hk_fault[i]->setVal(fresh?0:1);
    if (hk_active[i]->getVal<bool>()!=fresh) hk_active[i]->setVal(fresh);
  }
  // HomeKit has no null numeric value. Preserve last reading and publish inactive/fault.
  if (fresh) {
    if (hk_co2->getVal<float>()!=sensor.co2) hk_co2->setVal(sensor.co2);
    if (hk_peak->getVal<float>()!=analysis.peak) hk_peak->setVal(analysis.peak);
    float t=roundf(sensor.temperature*10)/10,h=roundf(sensor.humidity);
    if (hk_temperature->getVal<float>()!=t) hk_temperature->setVal(t);
    if (hk_humidity->getVal<float>()!=h) hk_humidity->setVal(h);
  }
  if (hk_detected->getVal<bool>()!=analysis.alarm) hk_detected->setVal(analysis.alarm);
  if (hk_led->getVal<bool>()!=cfg.led_enabled) hk_led->setVal(cfg.led_enabled);
}

void start_access_point() {
  access_ms=millis();
  if (ap_active) return;
  WiFi.mode(cfg.wifi_enabled && strlen(cfg.ssid)?WIFI_AP_STA:WIFI_AP);
  if (!WiFi.softAP(ap_name,admin_password)) { event_log("Setup AP failed"); return; }
  if (!WiFi.setTxPower(static_cast<wifi_power_t>(lroundf(cfg.wifi_tx_dbm*4)))) event_log("Wi-Fi TX power setup failed");
  ap_active=true; dns.start(53,"*",WiFi.softAPIP());
  wake_display(4); event_log("Setup Wi-Fi ready; credentials are on OLED");
}
int network_icon_state() {
  if (WiFi.status()==WL_CONNECTED) return 3;
  if (ap_active) return 2;
  return cfg.wifi_enabled?1:0;
}
void draw_network_screen() {
  char line[32];
  display.drawStr(0,10,ap_active?ap_name:"CO2 Monitor / Wi-Fi");
  String address=ap_active?WiFi.softAPIP().toString():WiFi.localIP().toString();
  display.drawStr(0,23,address.c_str());
  display.drawStr(0,36,"Login: admin");
  snprintf(line,sizeof(line),"Pass: %s",admin_password); display.drawStr(0,49,line);
  if (homekit_started) { snprintf(line,sizeof(line),"Home: %.3s-%.2s-%.3s",pairing_code,pairing_code+3,pairing_code+5); display.drawStr(0,62,line); }
  else display.drawStr(0,62,"Open in phone browser");
}

// History archive: two alternating snapshots with CRC, never format on mount failure.
struct ArchiveHeader { uint32_t magic,version,sequence,count,crc; };
uint32_t archive_sequence=0;
bool inspect_archive(const char* path, ArchiveHeader& header);
uint32_t crc32_bytes(uint32_t crc,const uint8_t* data,size_t count) {
  for (size_t i=0;i<count;++i) {
    crc^=data[i];
    for (int bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1)?0xedb88320UL:0);
  }
  return crc;
}
bool inspect_archive(const char* path,ArchiveHeader& header) {
  File file=LittleFS.open(path,"r");
  if (!file || file.read(reinterpret_cast<uint8_t*>(&header),sizeof(header))!=sizeof(header) ||
      header.magic!=0x434f3248 || header.version!=1 || header.count>HISTORY_SIZE ||
      file.size()!=sizeof(header)+header.count*sizeof(HistoryRecord)) return false;
  uint32_t crc=0xffffffff;
  for (uint32_t i=0;i<header.count;++i) {
    HistoryRecord record;
    if (file.read(reinterpret_cast<uint8_t*>(&record),sizeof(record))!=sizeof(record)) return false;
    crc=crc32_bytes(crc,reinterpret_cast<uint8_t*>(&record),sizeof(record));
  }
  return (crc^0xffffffff)==header.crc;
}
void restore_archive() {
  if (!fs_ready || !cfg.archive || !epoch_now() || archive_loaded) return;
  archive_loaded=true;
  ArchiveHeader a{},b{};
  bool a_ok=inspect_archive("/history-a",a),b_ok=inspect_archive("/history-b",b);
  if (!a_ok && !b_ok) return;
  const char* path=a_ok && (!b_ok || a.sequence>b.sequence)?"/history-a":"/history-b";
  ArchiveHeader header=(strcmp(path,"/history-a")==0)?a:b;
  archive_sequence=header.sequence;
  HistoryRecord* merged=new(std::nothrow) HistoryRecord[HISTORY_SIZE]{};
  if (!merged) { event_log("Not enough RAM to restore history"); return; }
  const uint32_t end_minute=epoch_now()/60;
  const uint32_t first_minute=end_minute-HISTORY_SIZE+1;
  uint16_t first_used=HISTORY_SIZE;
  auto insert=[&](HistoryRecord record) {
    if (!record.epoch) return;
    uint32_t m=record.epoch/60;
    if (m<first_minute || m>end_minute) return;
    int index=m-first_minute; merged[index]=record; first_used=min(first_used,static_cast<uint16_t>(index));
  };
  File file=LittleFS.open(path,"r"); file.seek(sizeof(header));
  for (uint32_t i=0;i<header.count;++i) { HistoryRecord record; file.read(reinterpret_cast<uint8_t*>(&record),sizeof(record)); insert(record); }
  for (int i=0;i<history_count;++i) {
    HistoryRecord record=history[(history_head+HISTORY_SIZE-history_count+i)%HISTORY_SIZE];
    if (!record.epoch) record.epoch=(end_minute-(uptime_ms/60000-record.uptime_minute))*60;
    insert(record);
  }
  history_head=history_count=0;
  for (int i=first_used;i<HISTORY_SIZE;++i) {
    merged[i].epoch=(first_minute+i)*60;
    append_history(merged[i]);
  }
  delete[] merged; compute_trend(); event_log("History archive restored");
}
void archive_history() {
  if (!fs_ready || !cfg.archive || !epoch_now() || !history_count) return;
  ArchiveHeader header{0x434f3248,1,archive_sequence+1,history_count,0};
  uint32_t crc=0xffffffff;
  for (int i=0;i<history_count;++i) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-history_count+i)%HISTORY_SIZE];
    crc=crc32_bytes(crc,reinterpret_cast<const uint8_t*>(&r),sizeof(r));
  }
  header.crc=crc^0xffffffff;
  const char* path=header.sequence%2?"/history-a":"/history-b";
  File file=LittleFS.open(path,"w");
  bool ok=file && file.write(reinterpret_cast<const uint8_t*>(&header),sizeof(header))==sizeof(header);
  for (int i=0;ok && i<history_count;++i) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-history_count+i)%HISTORY_SIZE];
    ok=file.write(reinterpret_cast<const uint8_t*>(&r),sizeof(r))==sizeof(r);
  }
  file.close();
  if (ok) archive_sequence=header.sequence; else event_log("History archive write failed");
}

void stats_json(JsonObject out,const PeriodStats& stats) {
  out["seconds"]=stats.seconds; out["above_seconds"]=stats.above_seconds; out["peak"]=stats.peak;
  if (stats.seconds) out["average"]=static_cast<uint32_t>(stats.sum/stats.seconds); else out["average"]=nullptr;
}
void network_diagnostics_json(JsonObject out) {
  out["version"]=FIRMWARE_VERSION; out["boot_id"]=boot_id; out["uptime_seconds"]=uptime_ms/1000;
  out["retained_boot_count"]=retained_boot_count; out["reset_reason"]=static_cast<int>(esp_reset_reason());
  out["previous_network_stage"]=previous_network_stage;
  out["wifi_connected"]=WiFi.status()==WL_CONNECTED;
  out["wifi_rssi"]=WiFi.status()==WL_CONNECTED?WiFi.RSSI():0;
  out["wifi_tx_dbm"]=static_cast<int>(WiFi.getTxPower())/4.0f;
  out["ip"]=WiFi.localIP().toString(); out["ap"]=ap_active;
  out["wifi_disconnect_count"]=wifi_disconnect_count.load(); out["wifi_got_ip_count"]=wifi_got_ip_count.load();
  out["wifi_disconnect_reason"]=wifi_disconnect_reason.load(); out["wifi_disconnect_ms"]=wifi_disconnect_ms.load();
  out["wifi_disconnect_rssi"]=wifi_disconnect_rssi.load();
  out["http_aborted_responses"]=web.aborted_responses;
  out["heap_free"]=ESP.getFreeHeap(); out["heap_min"]=ESP.getMinFreeHeap(); out["fresh"]=sample_fresh(millis());
  if (sample_fresh(millis())) out["co2"]=sensor.co2; else out["co2"]=nullptr;
  JsonArray durations=out["stage_max_ms"].to<JsonArray>(); for (uint32_t value:stage_max_ms) durations.add(value);
}
void status_json(JsonObject out) {
  network_diagnostics_json(out);
  const bool fresh=sample_fresh(millis());
  out["firmware_family"]=FIRMWARE_MARKER; out["config_revision"]=config_revision;
  out["version"]=FIRMWARE_VERSION; out["device_id"]=device_id; out["uptime_seconds"]=uptime_ms/1000;
  out["epoch"]=epoch_now(); out["fresh"]=fresh;
  if (fresh) { out["co2"]=sensor.co2; out["temperature"]=sensor.temperature; out["humidity"]=sensor.humidity; }
  else { out["co2"]=nullptr; out["temperature"]=nullptr; out["humidity"]=nullptr; }
  out["sample_age_seconds"]=sensor.valid?(millis()-sensor.sample_ms)/1000:0;
  out["night"]=light.night; out["adc"]=light.raw; out["filtered_adc"]=light.filtered; out["light"]=light.normalized;
  out["alarm"]=analysis.alarm; out["suspicious"]=analysis.suspicious;
  if (analysis.trend_valid && fresh) out["slope"]=analysis.slope; else out["slope"]=nullptr;
  out["night_active"]=analysis.night_active;
  stats_json(out["night_stats"].to<JsonObject>(),analysis.night_active?night_stats:last_night);
  JsonObject vent=out["vent"].to<JsonObject>(); vent["active"]=analysis.vent_active; vent["done"]=analysis.vent_done;
  vent["start"]=analysis.vent_start; vent["current"]=analysis.vent_current; vent["seconds"]=analysis.vent_seconds;
  out["history_count"]=history_count; out["led_enabled"]=cfg.led_enabled;
  out["wifi_tx_dbm"]=static_cast<int>(WiFi.getTxPower())/4.0f;
  out["previous_network_stage"]=previous_network_stage;
  out["http_aborted_responses"]=web.aborted_responses;
  out["wifi_rssi"]=WiFi.status()==WL_CONNECTED?WiFi.RSSI():0;
  out["wifi_enabled"]=cfg.wifi_enabled;
  out["wifi_connected"]=WiFi.status()==WL_CONNECTED; out["ip"]=WiFi.localIP().toString(); out["ap"]=ap_active;
  out["mqtt_connected"]=mqtt.connected(); out["homekit_enabled"]=homekit_started;
  out["homekit_paired"]=homekit_started && homeSpan.controllerListBegin()!=homeSpan.controllerListEnd();
  out["pairing_code"]=pairing_code;
  out["sensor_result"]=sensor.result; out["sensor_phase"]=static_cast<int>(sensor.phase);
  out["sensor_errors"]=sensor.failures; out["sensor_restarts"]=sensor.restarts;
  char serial[20]; snprintf(serial,sizeof(serial),"%012llX",static_cast<unsigned long long>(sensor.serial)); out["sensor_serial"]=serial;
  out["sensor_offset"]=sensor.offset; out["sensor_altitude"]=sensor.altitude; out["sensor_asc"]=sensor.asc;
  out["heap_free"]=ESP.getFreeHeap(); out["heap_min"]=ESP.getMinFreeHeap(); out["chip"]=ESP.getChipModel();
  out["flash_bytes"]=ESP.getFlashChipSize(); out["reset_reason"]=static_cast<int>(esp_reset_reason());
  out["settings_storage"]=storage_ready;
  out["pwm_ready"]=led.attached[0] && led.attached[1] && led.attached[2];
  out["filesystem"]=fs_ready; out["archive_enabled"]=cfg.archive; out["last_network_error"]=last_network_error;
  out["ota_rollback"]=true;
  JsonObject cal=out["calibration"].to<JsonObject>();
  cal["active"]=calibration.active; cal["dark_ready"]=calibration.dark_ready; cal["bright_ready"]=calibration.bright_ready;
  cal["dark"]=calibration.dark; cal["bright"]=calibration.bright;
  cal["dark_noise"]=calibration.dark_noise; cal["bright_noise"]=calibration.bright_noise; cal["result"]=calibration.result;
  JsonObject test=out["optical"].to<JsonObject>(); test["active"]=optical.active; test["stage"]=optical.stage;
  JsonArray levels=test["levels"].to<JsonArray>(); for (float value:optical.means) levels.add(value);
  JsonArray log=out["events"].to<JsonArray>();
  for (int i=0;i<event_count;++i) {
    const EventRecord& e=events[(event_head+16-event_count+i)%16]; JsonObject item=log.add<JsonObject>(); item["seconds"]=e.seconds; item["message"]=e.message;
  }
}
bool authorized(bool mutation=false) {
  if (!web.authenticate("admin",admin_password)) { web.requestAuthentication(); return false; }
  // Browsers cannot attach this header cross-origin without a preflight we do not grant.
  if (mutation && web.header("X-CO2-Request")!="1") { web.send(403,"application/json","{\"error\":\"Missing request header\"}"); return false; }
  web.sendHeader("Cache-Control","no-store"); web.sendHeader("X-Content-Type-Options","nosniff");
  return true;
}
void json_response(JsonDocument& document,int code=200) {
  String body; serializeJson(document,body); web.send(code,"application/json",body);
}
void api_error(int code,const char* message) {
  JsonDocument document; document["error"]=message; json_response(document,code);
}
bool request_json(JsonDocument& document) {
  if (web.arg("plain").length()>8192) { api_error(413,"Request too large"); return false; }
  if (deserializeJson(document,web.arg("plain")) || !document.is<JsonObject>()) { api_error(400,"Expected a JSON object"); return false; }
  return true;
}
void apply_runtime_settings(const Settings& previous) {
  if (previous.wifi_tx_dbm!=cfg.wifi_tx_dbm && WiFi.getMode()!=WIFI_OFF) WiFi.setTxPower(static_cast<wifi_power_t>(lroundf(cfg.wifi_tx_dbm*4)));
  preview_led_brightness=preview_oled_brightness=-1;
  led.band=-1; light.pending=false; analysis.alarm_pending=analysis.alarm=false; previous_contrast=-1; display_dirty=true;
  const bool sensor_changed=previous.eco!=cfg.eco || previous.sensor_override!=cfg.sensor_override || previous.asc!=cfg.asc ||
    previous.temperature_offset!=cfg.temperature_offset || previous.altitude!=cfg.altitude || previous.pressure_pa!=cfg.pressure_pa;
  if (sensor_changed) { sensor.valid=false; sensor.phase=SensorPhase::retry; sensor.phase_ms=millis()-SENSOR_RETRY_MS; }
  if (previous.wifi_enabled!=cfg.wifi_enabled || strcmp(previous.ssid,cfg.ssid) || strcmp(previous.wifi_password,cfg.wifi_password) || previous.homekit!=cfg.homekit ||
      strcmp(previous.timezone,cfg.timezone) || previous.mqtt_enabled!=cfg.mqtt_enabled || strcmp(previous.mqtt_host,cfg.mqtt_host) ||
      strcmp(previous.mqtt_user,cfg.mqtt_user) || strcmp(previous.mqtt_password,cfg.mqtt_password) || previous.mqtt_port!=cfg.mqtt_port) schedule_reboot();
  event_log("Settings updated");
}
void handle_settings() {
  if (!authorized(true)) return;
  if (sensor.command!=SensorCommand::none || optical.active || calibration.active) { api_error(409,"Wait for the active diagnostic procedure"); return; }
  if (web.header("X-CO2-Revision")!=String(config_revision)) { api_error(409,"Settings changed elsewhere; reload before saving"); return; }
  JsonDocument document; if (!request_json(document)) return;
  Settings previous=cfg; char error[96];
  if (!apply_settings(document.as<JsonObjectConst>(),error,sizeof(error))) { api_error(400,error); return; }
  if (!save_settings()) { cfg=previous; api_error(507,"Could not save settings"); return; }
  apply_runtime_settings(previous);
  JsonDocument result; result["ok"]=true; result["reboot"]=reboot_pending; result["revision"]=config_revision; json_response(result);
}
void handle_history(bool csv) {
  if (!authorized()) return;
  const char* prefix=csv?"epoch,uptime_minute,co2_ppm,temperature_c,humidity_percent,night,valid\n":"[";
  char line[192];
  auto format_record=[&](int i) {
    const HistoryRecord& r=history[(history_head+HISTORY_SIZE-history_count+i)%HISTORY_SIZE];
    if (csv) {
      if (r.valid) snprintf(line,sizeof(line),"%lu,%lu,%u,%.2f,%.2f,%u,1\n",static_cast<unsigned long>(r.epoch),static_cast<unsigned long>(r.uptime_minute),r.co2,r.temperature/100.0,r.humidity/100.0,r.night);
      else snprintf(line,sizeof(line),"%lu,%lu,,,,%u,0\n",static_cast<unsigned long>(r.epoch),static_cast<unsigned long>(r.uptime_minute),r.night);
    } else {
      if (r.valid) snprintf(line,sizeof(line),"%s[%lu,%lu,%u,%.2f,%.2f,%u]",i?",":"",static_cast<unsigned long>(r.epoch),static_cast<unsigned long>(r.uptime_minute),r.co2,r.temperature/100.0,r.humidity/100.0,r.night);
      else snprintf(line,sizeof(line),"%s[%lu,%lu,null,null,null,%u]",i?",":"",static_cast<unsigned long>(r.epoch),static_cast<unsigned long>(r.uptime_minute),r.night);
    }
  };
  size_t length=strlen(prefix)+(csv?0:1);
  for (int i=0;i<history_count;++i) { format_record(i); length+=strlen(line); }
  // Fixed length avoids WebServer's direct blocking chunk-footer writes.
  web.setContentLength(length);
  if (csv) web.sendHeader("Content-Disposition","attachment; filename=co2-history.csv");
  web.send(200,csv?"text/csv":"application/json","");
  web.sendContent(prefix);
  for (int i=0;i<history_count && web.client().connected();++i) { format_record(i); web.sendContent(line); }
  if (!csv && web.client().connected()) web.sendContent("]");
}
void handle_action() {
  if (!authorized(true)) return;
  JsonDocument document; if (!request_json(document)) return;
  const char* action=document["action"] | "";
  const uint32_t now=millis();
  if (optical.active || calibration.active || sensor.command!=SensorCommand::none) { api_error(409,"A diagnostic procedure is already running"); return; }
  if (!strcmp(action,"screen")) {
    if (!document["page"].is<int>() || document["page"].as<int>()<0 || document["page"].as<int>()>4) { api_error(400,"Page must be 0..4"); return; }
    wake_display(document["page"]);
  } else if (!strcmp(action,"brightness_preview")) {
    int value=document["value"] | -1;
    const char* target=document["target"] | "";
    if (value<0 || value>255 || (strcmp(target,"led") && strcmp(target,"oled"))) { api_error(400,"Invalid brightness preview"); return; }
    if (!strcmp(target,"led")) preview_led_brightness=value; else { preview_oled_brightness=value; wake_display(screen_page); }
    brightness_preview_ms=now;
  } else if (!strcmp(action,"preview")) {
    int index=document["color"] | -1;
    if (index<0 || index>4) { api_error(400,"Color must be 0..4"); return; }
    led.preview=index; led.preview_ms=now;
  } else if (!strcmp(action,"vent_start")) {
    if (!sample_fresh(now)) { api_error(409,"Wait for fresh measurements"); return; }
    analysis.vent_active=true; analysis.vent_done=false; analysis.vent_pending=false;
    analysis.vent_start=analysis.vent_current=sensor.co2; analysis.vent_ms=now; analysis.vent_seconds=0; wake_display(3);
  } else if (!strcmp(action,"vent_stop")) { analysis.vent_active=false; analysis.vent_done=true; wake_display(3);
  } else if (!strcmp(action,"cal_dark") || !strcmp(action,"cal_bright")) {
    calibration.active=true; calibration.stage=!strcmp(action,"cal_bright")?1:0;
    calibration.count=0; calibration.sum=calibration.sum_squared=0; calibration.minimum=4095; calibration.maximum=0; calibration.started_ms=now;
    if (calibration.stage==0) calibration.dark_ready=false; else calibration.bright_ready=false;
    snprintf(calibration.result,sizeof(calibration.result),"Sampling for 5 seconds...");
  } else if (!strcmp(action,"cal_apply")) {
    float minimum_span=max(10.0f,3*(calibration.dark_noise+calibration.bright_noise));
    if (!calibration.dark_ready || !calibration.bright_ready || calibration.bright-calibration.dark<minimum_span) { api_error(409,"Insufficient light range or excessive noise; capture both points again"); return; }
    Settings previous=cfg; cfg.dark_adc=lroundf(calibration.dark); cfg.bright_adc=lroundf(calibration.bright);
    if (!save_settings()) { cfg=previous; api_error(507,"Could not save calibration"); return; }
    apply_runtime_settings(previous); snprintf(calibration.result,sizeof(calibration.result),"Calibration saved");
  } else if (!strcmp(action,"optical_test")) {
    optical=OpticalTest{}; optical.active=true; optical.stage_ms=now; led.preview=-1; display_dirty=true;
  } else if (!strcmp(action,"self_test") || !strcmp(action,"recalibrate")) {
    if (!sample_fresh(now)) { api_error(409,"Wait for fresh measurements"); return; }
    const bool recalibrate=!strcmp(action,"recalibrate");
    if (recalibrate) {
      int reference=document["reference"] | 0;
      if (reference<400 || reference>2000 || !elapsed(now,sensor.phase_ms,180000) || document["confirm"].as<String>()!="REFERENCE") {
        api_error(400,"FRC requires 3 uninterrupted minutes, known reference 400..2000 ppm and REFERENCE confirmation"); return;
      }
      sensor.reference=reference;
    }
    sensor.command=recalibrate?SensorCommand::recalibrate:SensorCommand::self_test;
    snprintf(sensor.result,sizeof(sensor.result),"Running..."); sensor.valid=false;
    sensor.phase=SensorPhase::retry; sensor.phase_ms=now-SENSOR_RETRY_MS;
  } else if (!strcmp(action,"homekit_pair")) {
    if (!homekit_started) { api_error(409,"Enable HomeKit and connect to Wi-Fi first"); return; }
    wake_display(4);
  } else if (!strcmp(action,"homekit_reset")) {
    if (!homekit_started || document["confirm"].as<String>()!="UNPAIR") { api_error(400,"UNPAIR confirmation required; HomeKit must be enabled"); return; }
    homeSpan.processSerialCommand("U"); wake_display(4); event_log("HomeKit pairing removed");
  } else if (!strcmp(action,"wifi_reset")) {
    if (document["confirm"].as<String>()!="RESET WIFI") { api_error(400,"RESET WIFI confirmation required"); return; }
    Settings previous=cfg; cfg.ssid[0]=cfg.wifi_password[0]=0;
    if (!save_settings()) { cfg=previous; api_error(507,"Could not save Wi-Fi reset"); return; }
    schedule_reboot();
  } else if (!strcmp(action,"archive_format")) {
    if (document["confirm"].as<String>()!="FORMAT ARCHIVE") { api_error(400,"FORMAT ARCHIVE confirmation required"); return; }
    LittleFS.end(); fs_ready=LittleFS.format() && LittleFS.begin(false);
    if (!fs_ready) { api_error(500,"Filesystem format failed"); return; }
    archive_sequence=0; archive_loaded=true;
  } else if (!strcmp(action,"reboot")) schedule_reboot();
  else { api_error(400,"Unknown action"); return; }
  web.send(200,"application/json","{\"ok\":true}");
}

void handle_ota_chunk() {
  HTTPUpload& upload=web.upload();
  if (upload.status==UPLOAD_FILE_START) {
    ota_failed=false; ota_upload=false; ota_verified=false; ota_marker_found=false; ota_marker_matched=0;
    if (!web.authenticate("admin",admin_password) || web.header("X-CO2-Request")!="1") { ota_failed=true; return; }
    if (sensor.command!=SensorCommand::none || optical.active || calibration.active) { ota_failed=true; return; }
    const esp_partition_t* next=esp_ota_get_next_update_partition(nullptr);
    if (!next || !Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)) { ota_failed=true; return; }
    ota_upload=true; event_log("Firmware upload started");
  } else if (upload.status==UPLOAD_FILE_WRITE && ota_upload && !ota_failed) {
    // Require this firmware family's marker as well as ESP image/chip validation.
    // This prevents accidental uploads of unrelated sketches; it is not a signature.
    for (size_t i=0;i<upload.currentSize;++i) {
      if (upload.buf[i]==static_cast<uint8_t>(FIRMWARE_MARKER[ota_marker_matched])) {
        if (++ota_marker_matched==strlen(FIRMWARE_MARKER)) { ota_marker_found=true; ota_marker_matched=0; }
      } else ota_marker_matched=upload.buf[i]==static_cast<uint8_t>(FIRMWARE_MARKER[0])?1:0;
    }
    if (Update.write(upload.buf,upload.currentSize)!=upload.currentSize) { ota_failed=true; Update.abort(); }
    if (watchdog_ready) esp_task_wdt_reset();
  } else if (upload.status==UPLOAD_FILE_END && ota_upload) {
    if (ota_failed || !ota_marker_found || !upload.totalSize || !Update.end(true)) { ota_failed=true; Update.abort(); event_log("Firmware upload rejected"); }
    else { ota_verified=true; event_log("Firmware verified; reboot pending"); schedule_reboot(); }
    ota_upload=false;
  } else if (upload.status==UPLOAD_FILE_ABORTED) { Update.abort(); ota_upload=false; ota_failed=true; }
}
void start_web() {
  const char* headers[]={"X-CO2-Request","X-CO2-Revision"}; web.collectHeaders(headers,2);
  web.on("/",HTTP_GET,[](){ if (!authorized()) return; web.sendHeader("Content-Security-Policy","default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'"); web.send_P(200,"text/html; charset=utf-8",WEB_UI); });
  web.on("/api/diagnostics",HTTP_GET,[](){ if (!authorized()) return; JsonDocument d; network_diagnostics_json(d.to<JsonObject>()); json_response(d); });
  web.on("/api/status",HTTP_GET,[](){ if (!authorized()) return; JsonDocument d; status_json(d.to<JsonObject>()); json_response(d); });
  web.on("/api/settings",HTTP_GET,[](){ if (!authorized()) return; JsonDocument d; settings_json(d.to<JsonObject>(),false); d["_revision"]=config_revision; json_response(d); });
  web.on("/api/settings",HTTP_POST,handle_settings);
  web.on("/api/action",HTTP_POST,handle_action);
  web.on("/api/history",HTTP_GET,[](){handle_history(false);});
  web.on("/api/history.csv",HTTP_GET,[](){handle_history(true);});
  web.on("/api/backup",HTTP_GET,[](){ if (!authorized()) return; web.sendHeader("Content-Disposition","attachment; filename=co2-settings.json"); JsonDocument d; settings_json(d.to<JsonObject>(),false); json_response(d); });
  web.on("/api/update",HTTP_POST,[](){ if (!authorized(true)) return; if (ota_failed || !ota_verified) api_error(400,"No verified firmware uploaded or image rejected"); else web.send(200,"application/json","{\"ok\":true,\"reboot\":true}"); },handle_ota_chunk);
  web.onNotFound([](){ if (web.method()!=HTTP_GET) { api_error(404,"Not found"); return; } web.sendHeader("Location","/"); web.send(302,"text/plain",""); });
  web.begin();
}

String mqtt_base() { return String("co2/")+device_id; }
void mqtt_discovery() {
  struct Definition { const char* key; const char* label; const char* unit; const char* device_class; };
  const Definition definitions[]={{"co2","CO2","ppm","carbon_dioxide"},{"temperature","Temperature","°C","temperature"},{"humidity","Humidity","%","humidity"}};
  for (const auto& definition:definitions) {
    JsonDocument d; d["name"]=definition.label; d["unique_id"]=String(device_id)+"_"+definition.key;
    d["state_topic"]=mqtt_base()+"/state"; d["value_template"]=String("{{ value_json.")+definition.key+" }}";
    d["unit_of_measurement"]=definition.unit; d["device_class"]=definition.device_class; d["state_class"]="measurement";
    d["availability_topic"]=mqtt_base()+"/availability";
    JsonObject device=d["device"].to<JsonObject>(); device["identifiers"].to<JsonArray>().add(device_id); device["name"]="CO2 Monitor"; device["manufacturer"]="DIY"; device["model"]="SCD40 ESP32"; device["sw_version"]=FIRMWARE_VERSION;
    String payload; serializeJson(d,payload); mqtt.publish((String("homeassistant/sensor/")+device_id+"/"+definition.key+"/config").c_str(),payload.c_str(),true);
  }
}
void update_mqtt(uint32_t now) {
  if (!cfg.mqtt_enabled || WiFi.status()!=WL_CONNECTED) return;
  if (!mqtt.connected()) {
    if (!elapsed(now,mqtt_ms,30000)) return; mqtt_ms=now;
    if (mqtt.connect(device_id,strlen(cfg.mqtt_user)?cfg.mqtt_user:nullptr,strlen(cfg.mqtt_password)?cfg.mqtt_password:nullptr,(mqtt_base()+"/availability").c_str(),1,true,"offline")) {
      mqtt_discovery(); mqtt_publish_ms=now-5000;
    } else last_network_error="MQTT connection failed";
    return;
  }
  mqtt.loop();
  if (!elapsed(now,mqtt_publish_ms,5000)) return; mqtt_publish_ms=now;
  const bool fresh=sample_fresh(now);
  mqtt.publish((mqtt_base()+"/availability").c_str(),fresh?"online":"offline",true);
  JsonDocument d; d["fresh"]=fresh; d["alarm"]=analysis.alarm; d["epoch"]=epoch_now();
  if (fresh) { d["co2"]=sensor.co2; d["temperature"]=sensor.temperature; d["humidity"]=sensor.humidity; }
  else { d["co2"]=nullptr; d["temperature"]=nullptr; d["humidity"]=nullptr; }
  String payload; serializeJson(d,payload); mqtt.publish((mqtt_base()+"/state").c_str(),payload.c_str(),true);
}
void serial_commands() {
  static char buffer[192]; static size_t length=0; static bool overflow=false;
  for (int budget=0;budget<64 && Serial.available();++budget) {
    char c=Serial.read(); if (c=='\r') continue;
    if (c=='\n') {
      buffer[length]=0;
      if (overflow) Serial.println("Command too long");
      else if (!strcmp(buffer,"access")) {
        start_access_point(); wake_display(4);
        Serial.printf("AP: %s | URL http://%s | user admin | password %s\n",ap_name,WiFi.softAPIP().toString().c_str(),admin_password);
      } else if (!strcmp(buffer,"status")) {
        Serial.printf("chip=%s flash=%lu fresh=%u co2=%u errors=%lu\n",ESP.getChipModel(),static_cast<unsigned long>(ESP.getFlashChipSize()),sample_fresh(millis()),sensor.co2,static_cast<unsigned long>(sensor.failures));
      } else if (!strcmp(buffer,"diagnostics")) {
        JsonDocument d; network_diagnostics_json(d.to<JsonObject>()); serializeJson(d,Serial); Serial.println();
      } else if (!strcmp(buffer,"network")) {
        Serial.printf("mode=%u ap=%u clients=%u channel=%ld tx_quarter_dbm=%d station_status=%d ap_ip=%s station_ip=%s\n",static_cast<unsigned>(WiFi.getMode()),ap_active,WiFi.softAPgetStationNum(),static_cast<long>(WiFi.channel()),static_cast<int>(WiFi.getTxPower()),static_cast<int>(WiFi.status()),WiFi.softAPIP().toString().c_str(),WiFi.localIP().toString().c_str());
      } else if (!strcmp(buffer,"wifi reset")) {
        cfg.ssid[0]=cfg.wifi_password[0]=0; if (save_settings()) schedule_reboot();
      } else if (!strncmp(buffer,"wifi ",5)) {
        char* divider=strchr(buffer+5,'|');
        if (!divider) Serial.println("Use wifi SSID|PASSWORD");
        else {
          *divider=0; JsonDocument d; d["ssid"]=buffer+5; d["wifi_password"]=divider+1;
          Settings previous=cfg; char error[96];
          if (apply_settings(d.as<JsonObjectConst>(),error,sizeof(error)) && save_settings()) schedule_reboot();
          else { cfg=previous; Serial.println("Wi-Fi settings rejected"); }
        }
      } else if (!strcmp(buffer,"homekit reset") && homekit_started) homeSpan.processSerialCommand("U");
      else if (!strcmp(buffer,"reboot")) schedule_reboot();
      else Serial.println("Commands: status, network, diagnostics, access, wifi SSID|PASSWORD, wifi reset, homekit reset, reboot");
      memset(buffer,0,sizeof(buffer)); length=0; overflow=false;
    } else if (length<sizeof(buffer)-1) buffer[length++]=c; else overflow=true;
  }
}
void platform_setup() {
  previous_network_stage=diagnostic_magic==0x434f3245?diagnostic_stage:0;
  retained_boot_count=diagnostic_magic==0x434f3245?retained_boot_count+1:1;
  diagnostic_magic=0x434f3245; diagnostic_stage=5; boot_id=esp_random();
  storage_ready=preferences.begin("co2-monitor",false);
  if (!storage_ready) event_log("Settings storage unavailable");
  config_revision=esp_random();
  String stored=preferences.getString("settings","");
  if (stored.length()) {
    JsonDocument d; char error[96];
    if (deserializeJson(d,stored) || !d.is<JsonObject>() || !apply_settings(d.as<JsonObjectConst>(),error,sizeof(error))) event_log("Saved configuration invalid; using defaults");
  }
  snprintf(device_id,sizeof(device_id),"co2-%06lx",static_cast<unsigned long>(ESP.getEfuseMac()&0xffffff));
  snprintf(ap_name,sizeof(ap_name),"CO2-Monitor-%06lx",static_cast<unsigned long>(ESP.getEfuseMac()&0xffffff));
  String password=preferences.getString("admin","");
  if (password.length()!=12) {
    const char alphabet[]="abcdefghjkmnpqrstuvwxyz23456789";
    for (int i=0;i<12;++i) admin_password[i]=alphabet[esp_random()%(sizeof(alphabet)-1)]; admin_password[12]=0;
    preferences.putString("admin",admin_password);
  } else password.toCharArray(admin_password,sizeof(admin_password));
  String code=preferences.getString("pin","");
  if (code.length()!=8) { snprintf(pairing_code,sizeof(pairing_code),"37%06lu",static_cast<unsigned long>(esp_random()%1000000)); preferences.putString("pin",pairing_code); }
  else code.toCharArray(pairing_code,sizeof(pairing_code));
  fs_ready=LittleFS.begin(false);
  if (!fs_ready) event_log("Archive filesystem unformatted; use web format action if needed");
  WiFi.onEvent([](WiFiEvent_t event,WiFiEventInfo_t info) {
    // Event task only updates atomics; no Serial, JSON, or application state here.
    if (event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      wifi_disconnect_reason.store(info.wifi_sta_disconnected.reason);
      wifi_disconnect_rssi.store(info.wifi_sta_disconnected.rssi);
      wifi_disconnect_ms.store(millis()); wifi_disconnect_count.fetch_add(1);
    } else if (event==ARDUINO_EVENT_WIFI_STA_GOT_IP) wifi_got_ip_count.fetch_add(1);
  });
  WiFi.persistent(false); WiFi.setHostname(device_id);
  start_homekit();
  if (cfg.wifi_enabled && !homekit_started && strlen(cfg.ssid)) { WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true); WiFi.begin(cfg.ssid,cfg.wifi_password); WiFi.setTxPower(static_cast<wifi_power_t>(lroundf(cfg.wifi_tx_dbm*4))); }
  if (!cfg.wifi_enabled || !strlen(cfg.ssid)) start_access_point();
  disconnected_ms=millis();
  start_web();
  mqtt_transport.setConnectionTimeout(1000); mqtt_transport.setTimeout(1000);
  mqtt.setServer(cfg.mqtt_host,cfg.mqtt_port); mqtt.setSocketTimeout(1); mqtt.setBufferSize(1024); mqtt.setKeepAlive(15);
  // Watch only this task; idle cores can sleep while network stacks work.
  esp_task_wdt_config_t watchdog={10000,0,true};
  esp_err_t result=esp_task_wdt_init(&watchdog);
  if (result==ESP_ERR_INVALID_STATE) result=esp_task_wdt_reconfigure(&watchdog);
  watchdog_ready=result==ESP_OK && esp_task_wdt_add(nullptr)==ESP_OK;
  health_ms=millis();
}
void platform_loop() {
  const uint32_t now=millis();
  if (watchdog_ready) esp_task_wdt_reset();
  begin_network_stage(1); update_homekit(now); end_network_stage();
  begin_network_stage(2); web.begin_cycle(); web.handleClient(); end_network_stage();
  begin_network_stage(6); if (ap_active) dns.processNextRequest(); end_network_stage();
  begin_network_stage(7); serial_commands(); end_network_stage();
  bool connected=WiFi.status()==WL_CONNECTED;
  if (connected && !network_was_connected) {
    connected_ms=now; configTzTime(cfg.timezone,"pool.ntp.org","time.cloudflare.com");
    if (!homekit_started) mdns_started=MDNS.begin(device_id);
    else mdns_started=true;
    if (mdns_started) MDNS.addService("http","tcp",80);
    event_log("Home Wi-Fi connected"); wake_display(4);
  }
  if (!connected && network_was_connected) { disconnected_ms=now; event_log("Home Wi-Fi disconnected"); }
  network_was_connected=connected;
  if (cfg.wifi_enabled && !connected && elapsed(now,disconnected_ms,60000) && !ap_active) start_access_point();
  if (!cfg.wifi_enabled && ap_active && elapsed(now,access_ms,60000) && WiFi.softAPgetStationNum()==0) { dns.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); ap_active=false; display_dirty=true; }
  if (connected && ap_active && elapsed(now,connected_ms,60000) && WiFi.softAPgetStationNum()==0) {
    dns.stop(); WiFi.softAPdisconnect(true); ap_active=false; display_dirty=true;
  }
  begin_network_stage(3); update_mqtt(now); end_network_stage();
  if (save_pending) { begin_network_stage(8); save_pending=false; if (!save_settings()) event_log("Could not persist HomeKit change"); end_network_stage(); }
  if (!time_backfilled && epoch_now()) {
    time_backfilled=true;
    const uint32_t current_minute=epoch_now()/60;
    for (int i=0;i<history_count;++i) {
      HistoryRecord& record=history[(history_head+HISTORY_SIZE-history_count+i)%HISTORY_SIZE];
      if (!record.epoch) record.epoch=(current_minute-(uptime_ms/60000-record.uptime_minute))*60;
    }
  }
  begin_network_stage(4); restore_archive(); end_network_stage();
  if (elapsed(now,archive_ms,900000)) { archive_ms=now; begin_network_stage(4); archive_history(); end_network_stage(); }
  static bool boot_validated=false;
  if (!boot_validated && elapsed(now,health_ms,60000) && sample_fresh(now) && storage_ready && led.attached[0] && led.attached[1] && led.attached[2]) {
    esp_ota_mark_app_valid_cancel_rollback(); boot_validated=true; event_log("Boot health check passed");
  }
  if (!boot_validated && elapsed(now,health_ms,180000)) {
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(),&state)==ESP_OK && state==ESP_OTA_IMG_PENDING_VERIFY) {
      event_log("OTA health check failed; rolling back"); esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    boot_validated=true; // A first serial-flashed image has no previous OTA image to restore.
  }
  if (reboot_pending && elapsed(now,reboot_ms,1500)) {
    if (mqtt.connected()) { mqtt.publish((mqtt_base()+"/availability").c_str(),"offline",true); mqtt.disconnect(); }
    archive_history(); ESP.restart();
  }
}
#else
uint32_t epoch_now() { return 0; }
bool save_settings() { return true; }
void archive_history() {}
void platform_setup() {}
void platform_loop() {}
void draw_network_screen() {}
int network_icon_state() { return 3; }
#endif
