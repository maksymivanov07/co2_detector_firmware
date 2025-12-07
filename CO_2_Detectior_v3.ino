#include <Wire.h>
#include <U8g2lib.h>
#include <SensirionI2cScd4x.h>
#include <esp32-hal-ledc.h>

// --- Піни та базові налаштування I2C/OLED ---
constexpr uint8_t I2C_SDA = 8;   // загальна шина + SCD40
constexpr uint8_t I2C_SCL = 9;
constexpr uint8_t OLED_SCL = 3;  // софт-I2C для дисплея
constexpr uint8_t OLED_SDA = 4;

// --- RGB LED (KY-016, CC) ---
constexpr uint8_t LED_R = 20;
constexpr uint8_t LED_G = 5;
constexpr uint8_t LED_B = 6;
constexpr uint32_t LED_FREQ = 5000;
constexpr uint8_t LED_RES = 8;
uint8_t BRIGHT_DAY = 200;
uint8_t BRIGHT_NIGHT = 1;  // вимкнено у нічному режимі
uint8_t BRIGHT_CUR = 200;
uint8_t GAIN_R = 160, GAIN_G = 180, GAIN_B = 255;

// --- TEMT6000 (ADC) ---
constexpr uint8_t LIGHT_SENSOR_PIN = 2;
int DARK_ADC = 2219;    // під себе
int BRIGHT_ADC = 2224;  // під себе
constexpr float SMOOTH_ALPHA = 0.4f;
constexpr float TH_LOW = 0.30f;   // < — ніч
constexpr float TH_HIGH = 0.70f;  // > — день
constexpr uint32_t MODE_DWELL_MS = 5000;
constexpr uint32_t LUX_POLL_MS = 500;

// --- CO2 пороги для кольорів ---
constexpr uint16_t CO2_T1 = 300;
constexpr uint16_t CO2_T2 = 600;
constexpr uint16_t CO2_T3 = 1000;
constexpr uint16_t CO2_T4 = 1500;

// --- Стан системи ---
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /*scl=*/OLED_SCL, /*sda=*/OLED_SDA, /*reset=*/U8X8_PIN_NONE);
SensirionI2cScd4x scd4x;
uint16_t co2 = 0, prevCo2 = 0;
float tempC = 0, rh = 0, prevT = NAN, prevRH = NAN;
bool nightMode = false;
float smoothLux = 0;
uint32_t lastSwitchMs = 0;
uint32_t lastLuxMs = 0;

// --- LED утиліти ---
inline uint8_t scaleWithGain(uint8_t v, uint8_t gain) {
  uint16_t x = static_cast<uint16_t>(v) * BRIGHT_CUR / 255;
  x = static_cast<uint16_t>(x) * gain / 255;
  return x > 255 ? 255 : static_cast<uint8_t>(x);
}

void ledInit() {
  ledcAttach(LED_R, LED_FREQ, LED_RES);
  ledcAttach(LED_G, LED_FREQ, LED_RES);
  ledcAttach(LED_B, LED_FREQ, LED_RES);
  ledcWrite(LED_R, 0);
  ledcWrite(LED_G, 0);
  ledcWrite(LED_B, 0);
}

inline void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(LED_R, scaleWithGain(r, GAIN_R));
  ledcWrite(LED_G, scaleWithGain(g, GAIN_G));
  ledcWrite(LED_B, scaleWithGain(b, GAIN_B));
}

inline void C_LIGHT_BLUE() { setRGB(0, 60, 255); }
inline void C_BLUE() { setRGB(0, 0, 255); }
inline void C_VIOLET() { setRGB(80, 0, 255); }
inline void C_ORANGE() { setRGB(255, 50, 0); }
inline void C_RED() { setRGB(255, 0, 0); }

void updateLedByCO2(uint16_t co2val) {
  if (nightMode && BRIGHT_CUR == 0) {  // 0 — повністю вимкнено
    setRGB(0, 0, 0);
    return;
  }
  if (co2val < CO2_T1) C_LIGHT_BLUE();
  else if (co2val <= CO2_T2) C_BLUE();
  else if (co2val <= CO2_T3) C_VIOLET();
  else if (co2val <= CO2_T4) C_ORANGE();
  else C_RED();
}

// --- Освітленість ---
float readLightNorm() {
  const int raw = analogRead(LIGHT_SENSOR_PIN);
  smoothLux = SMOOTH_ALPHA * raw + (1.0f - SMOOTH_ALPHA) * smoothLux;
  const int delta = max(BRIGHT_ADC - DARK_ADC, 1);
  const float norm = (smoothLux - DARK_ADC) / static_cast<float>(delta);
  return constrain(norm, 0.0f, 1.0f);
}

void applyBrightnessByLight() {
  const float norm = readLightNorm();
  const bool wantNight = (norm < TH_LOW);
  const bool wantDay = (norm > TH_HIGH);
  const uint32_t now = millis();

  if (now - lastSwitchMs > MODE_DWELL_MS) {
    if (!nightMode && wantNight) {
      nightMode = true;
      lastSwitchMs = now;
    }
    if (nightMode && wantDay) {
      nightMode = false;
      lastSwitchMs = now;
    }
  }

  BRIGHT_CUR = nightMode ? BRIGHT_NIGHT : BRIGHT_DAY;
  u8g2.setContrast(nightMode ? 60 : 200);
}

// --- Відображення на OLED ---
void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, nightMode ? "CO2 Monitor (Night)" : "CO2 Monitor");

  u8g2.setFont(u8g2_font_logisoso24_tf);
  char co2buf[16];
  snprintf(co2buf, sizeof(co2buf), "%4u", co2);
  u8g2.drawStr(0, 45, co2buf);

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(90, 45, "ppm");
  char line2[28];
  snprintf(line2, sizeof(line2), "T: %.1fC  H: %.0f%%", tempC, rh);
  u8g2.drawStr(0, 62, line2);
  u8g2.sendBuffer();
}

// --- Ініціалізація ---
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(50);

  u8g2.begin();
  u8g2.setPowerSave(0);

  ledInit();

  analogReadResolution(12);
  analogSetPinAttenuation(LIGHT_SENSOR_PIN, ADC_11db);
  pinMode(LIGHT_SENSOR_PIN, INPUT);

  scd4x.begin(Wire, 0x62);
  scd4x.stopPeriodicMeasurement();
  scd4x.startPeriodicMeasurement();

  for (int i = 0; i < 8; i++) {
    readLightNorm();
    delay(20);
  }

  applyBrightnessByLight();
  drawScreen();
}

// --- Основний цикл ---
void loop() {
  const uint32_t now = millis();
  if (now - lastLuxMs > LUX_POLL_MS) {
    lastLuxMs = now;
    applyBrightnessByLight();
    updateLedByCO2(co2);  // переобрати яскравість/колір під активний режим
  }

  bool ready = false;
  const uint16_t err = scd4x.getDataReadyStatus(ready);
  if (!err && ready) {
    uint16_t c;
    float t, h;
    if (!scd4x.readMeasurement(c, t, h) && c != 0) {
      co2 = c;
      tempC = t;
      rh = h;
      updateLedByCO2(co2);
      const bool changed = (co2 != prevCo2) || (tempC != prevT) || (rh != prevRH);
      if (changed) {
        drawScreen();
        prevCo2 = co2;
        prevT = tempC;
        prevRH = rh;
      }
    }
  }

  delay(50);
}
