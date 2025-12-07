#include <Wire.h>
#include <U8g2lib.h>
#include <SensirionI2cScd4x.h>
#include <esp32-hal-ledc.h>

// -------- I2C --------
#define I2C_SDA 8           // SCD40 + загальна шина
#define I2C_SCL 9

// -------- OLED (SW I2C: SCL=3, SDA=4) --------
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /*scl=*/3, /*sda=*/4, /*reset=*/U8X8_PIN_NONE);

// -------- SCD40 --------
SensirionI2cScd4x scd4x;
uint16_t co2 = 0, prevCo2 = 0;
float tempC = 0, rh = 0, prevT = NAN, prevRH = NAN;

// -------- RGB LED (KY-016, CC) --------
#define LED_R 20
#define LED_G 5
#define LED_B 6
const uint32_t LED_FREQ = 5000;
const uint8_t  LED_RES  = 8;
uint8_t BRIGHT_DAY  = 200;
uint8_t BRIGHT_NIGHT= 30;
uint8_t BRIGHT_CUR  = 200;
uint8_t GAIN_R = 160, GAIN_G = 180, GAIN_B = 255;

inline uint8_t scale(uint8_t v, uint8_t gain){
  uint16_t x = (uint16_t)v * BRIGHT_CUR / 255;
  x = (uint16_t)x * gain / 255;
  return x > 255 ? 255 : (uint8_t)x;
}
void ledInit(){
  ledcAttach(LED_R, LED_FREQ, LED_RES);
  ledcAttach(LED_G, LED_FREQ, LED_RES);
  ledcAttach(LED_B, LED_FREQ, LED_RES);
  ledcWrite(LED_R, 0); ledcWrite(LED_G, 0); ledcWrite(LED_B, 0);
}
inline void setRGB(uint8_t r,uint8_t g,uint8_t b){
  ledcWrite(LED_R, scale(r,GAIN_R));
  ledcWrite(LED_G, scale(g,GAIN_G));
  ledcWrite(LED_B, scale(b,GAIN_B));
}
inline void C_LIGHT_BLUE(){ setRGB(  0, 60,255); }
inline void C_BLUE()      { setRGB(  0,  0,255); }
inline void C_VIOLET()    { setRGB( 80,  0,255); }
inline void C_ORANGE()    { setRGB(255, 50,  0); }
inline void C_RED()       { setRGB(255,  0,  0); }
void updateLedByCO2(uint16_t co2val){
  if      (co2val < 300)            C_LIGHT_BLUE();
  else if (co2val <= 600)           C_BLUE();
  else if (co2val <= 1000)          C_VIOLET();
  else if (co2val <= 1500)          C_ORANGE();
  else                              C_RED();
}

// -------- TEMT6000 (ADC) --------
#define LIGHT_SENSOR_PIN 2
int  DARK_ADC   = 2219;   // під себе
int  BRIGHT_ADC = 2224;   // під себе
const float SMOOTH_ALPHA = 0.4f;
bool  nightMode = false;
float smoothLux = 0;
const float TH_LOW  = 0.30f;     // < — ніч
const float TH_HIGH = 0.70f;     // > — день
const uint32_t MODE_DWELL_MS = 5000;
uint32_t lastSwitchMs = 0;

float readLightNorm(){
  int raw = analogRead(LIGHT_SENSOR_PIN);
  smoothLux = SMOOTH_ALPHA*raw + (1.0f-SMOOTH_ALPHA)*smoothLux;
  int delta = max(BRIGHT_ADC - DARK_ADC, 1);
  float norm = (smoothLux - DARK_ADC) / float(delta);
  return constrain(norm, 0.0f, 1.0f);
}
void applyBrightnessByLight(){
  float norm = readLightNorm();
  bool wantNight = (norm < TH_LOW);
  bool wantDay   = (norm > TH_HIGH);
  if (millis() - lastSwitchMs > MODE_DWELL_MS) {
    if (!nightMode && wantNight) { nightMode = true;  lastSwitchMs = millis(); }
    if ( nightMode && wantDay)   { nightMode = false; lastSwitchMs = millis(); }
  }
  BRIGHT_CUR = nightMode ? BRIGHT_NIGHT : BRIGHT_DAY;
  u8g2.setContrast(nightMode ? 60 : 200);
}

// -------- UI --------
void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, nightMode ? "CO2 Monitor (Night)" : "CO2 Monitor");
  u8g2.setFont(u8g2_font_logisoso24_tf);
  char co2buf[16]; snprintf(co2buf, sizeof(co2buf), "%4u", co2);
  u8g2.drawStr(0, 45, co2buf);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(90, 45, "ppm");
  char line2[28]; snprintf(line2, sizeof(line2), "T: %.1fC  H: %.0f%%", tempC, rh);
  u8g2.drawStr(0, 62, line2);
  u8g2.sendBuffer();
}

// -------- setup/loop --------
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

  for (int i=0;i<8;i++){ readLightNorm(); delay(20); }
  applyBrightnessByLight();
  drawScreen();
}

void loop() {
  static uint32_t lastLuxMs=0;
  if (millis()-lastLuxMs > 500){
    lastLuxMs = millis();
    applyBrightnessByLight();
    updateLedByCO2(co2);          // переобрати яскравість/колір
  }

  bool ready=false;
  uint16_t err = scd4x.getDataReadyStatus(ready);
  if (!err && ready){
    uint16_t c; float t,h;
    if (!scd4x.readMeasurement(c,t,h) && c!=0){
      co2=c; tempC=t; rh=h;
      updateLedByCO2(co2);
      bool changed = (co2!=prevCo2) || (tempC!=prevT) || (rh!=prevRH);
      if (changed){
        drawScreen();
        prevCo2=co2; prevT=tempC; prevRH=rh;
      }
    }
  }
  delay(50);
}
