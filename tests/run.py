#!/usr/bin/env python3
"""Compile the actual sketch against deterministic hardware stubs and ArduinoJson."""
from pathlib import Path
import os
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
json_root = Path(os.environ.get('ARDUINOJSON_DIR', str(Path.home() / 'Documents/Arduino/libraries/ArduinoJson/src')))
if not (json_root / 'ArduinoJson.h').exists():
    raise SystemExit('Set ARDUINOJSON_DIR to ArduinoJson 7.4.2 src directory')
with tempfile.TemporaryDirectory(prefix='co2-tests-') as folder:
    work = Path(folder)
    (work / 'Arduino.h').write_text(r'''
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <algorithm>
using std::min; using std::max;
#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define ADC_11db 3
#define U8G2_R0 0
#define U8X8_PIN_NONE 255
static uint32_t clock_ms=0;
static int adc_value=2224;
inline uint32_t millis(){return clock_ms;}
inline void delay(unsigned ms){clock_ms+=ms;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline void analogReadResolution(int){}
inline void analogSetPinAttenuation(int,int){}
inline int analogRead(int){return adc_value;}
template<class T>T constrain(T x,T lo,T hi){return std::min(hi,std::max(lo,x));}
struct SerialMock{void begin(int){} void println(const char*){} template<class... A>void printf(const char*,A...){} }Serial;
''')
    (work / 'Wire.h').write_text(r'''
#pragma once
struct WireMock{
 int error=0,response_count=0,response_index=0,response[3]={0,0,0}; void end(){} bool begin(int,int,int){return true;} void setTimeOut(int){}
 void beginTransmission(int){} void write(int){} int endTransmission(){return error;}
 int requestFrom(int,int){response_index=0;return response_count;} int read(){return response[response_index++];}
}Wire;
''')
    (work / 'SensirionI2cScd4x.h').write_text(r'''
#pragma once
class SensirionI2cScd4x{public:
 int error=0,starts=0,polls=0,eco_starts=0; bool ready=true; uint16_t co2=800; float temperature=22,humidity=50;
 void begin(WireMock&,int){} int startPeriodicMeasurement(){++starts;return error;}
 int startLowPowerPeriodicMeasurement(){++eco_starts;return error;}
 int getDataReadyStatus(bool& r){++polls;r=ready;return error;}
 int readMeasurement(uint16_t& c,float& t,float& h){c=co2;t=temperature;h=humidity;return error;}
 int setTemperatureOffset(float){return error;} int setSensorAltitude(int){return error;}
 int setAutomaticSelfCalibrationEnabled(int){return error;} int setAmbientPressure(int){return error;}
 int getSerialNumber(uint64_t& s){s=1;return error;} int getTemperatureOffset(float& f){f=4;return error;}
 int getSensorAltitude(uint16_t& n){n=0;return error;} int getAutomaticSelfCalibrationEnabled(uint16_t& n){n=1;return error;}
};
''')
    (work / 'U8g2lib.h').write_text(r'''
#pragma once
const int u8g2_font_6x12_tf=0,u8g2_font_logisoso24_tf=1,u8g2_font_logisoso42_tn=2,u8g2_font_logisoso32_tn=3,u8g2_font_8x13B_tf=4;
class U8G2_SSD1306_128X64_NONAME_F_SW_I2C{public:
 uint8_t buffer[1024]={}; int frames=0,contrast=0; bool sleeping=false;
 U8G2_SSD1306_128X64_NONAME_F_SW_I2C(int,int,int,int){}
 void begin(){} void setContrast(int c){contrast=c;} void setPowerSave(int n){sleeping=n;}
 void clearBuffer(){memset(buffer,0,1024);} void setFont(int){} void drawStr(int,int,const char*){++buffer[0];}
 void drawLine(int,int,int,int){++buffer[1];} uint8_t* getBufferPtr(){return buffer;}
 void updateDisplayArea(int,int,int,int){++frames;}
};
''')
    (work / 'esp32-hal-ledc.h').write_text(r'''
#pragma once
inline bool ledcAttach(int,int,int){return true;} inline bool ledcWrite(int,uint32_t){return true;}
''')
    (work / 'test.cpp').write_text('#define CO2_HOST_TEST\n#include "' + str(root / 'CO_2_Detectior_v3.ino') + '"\n' + (root / 'tests/core.cpp').read_text())
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                    '-I',str(work),'-I',str(json_root),str(work/'test.cpp'),'-o',str(work/'test')],check=True)
    subprocess.run([str(work/'test')],check=True)
    if os.environ.get('CO2_PREVIEW_SETTINGS'):
        Path(os.environ['CO2_PREVIEW_SETTINGS']).write_bytes(subprocess.check_output([str(work/'test'),'settings']))
