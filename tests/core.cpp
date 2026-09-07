#include <assert.h>
#include <iostream>
void reset() {
  cfg=Settings{}; sensor=SensorState{}; light=LightState{}; led=LedState{};
  minute_state=MinuteState{}; analysis=AnalysisState{}; calibration=CalibrationState{}; optical=OpticalTest{};
  night_stats=last_night=PeriodStats{}; history_head=history_count=0;
  scd4x=SensirionI2cScd4x{}; Wire.error=0; Wire.response_count=0; clock_ms=0; uptime_ms=0; display_dirty=false;
}
void sample(uint32_t t,uint16_t co2) {
  clock_ms=t; sensor.valid=true; sensor.phase=SensorPhase::measuring;
  sensor.co2=co2; sensor.sample_ms=t;
}
void tick(uint32_t t) { clock_ms=t; update_analytics(t); }
bool patch(const char* json) {
  JsonDocument d; assert(!deserializeJson(d,json)); char error[96];
  return apply_settings(d.as<JsonObjectConst>(),error,sizeof(error));
}
int main(int argc,char**) {
  if(argc>1){JsonDocument d;settings_json(d.to<JsonObject>(),false);serializeJson(d,std::cout);return 0;}
  reset(); adc_value=2219; setup(); assert(light.night && light.filtered==2219);
  clock_ms=999; update_sensor(clock_ms); assert(scd4x.starts==0);
  clock_ms=1000; update_sensor(clock_ms); assert(sensor.phase==SensorPhase::stopping);
  clock_ms=1499; update_sensor(clock_ms); assert(scd4x.starts==0);
  clock_ms=1500; update_sensor(clock_ms); assert(scd4x.starts==1);
  clock_ms=2499; update_sensor(clock_ms); assert(scd4x.polls==0);
  clock_ms=2500; update_sensor(clock_ms); assert(sample_fresh(clock_ms));
  clock_ms=17500; update_sensor(clock_ms); assert(!sensor.valid && sensor.phase==SensorPhase::retry);
  clock_ms=27500; update_sensor(clock_ms); assert(sensor.phase==SensorPhase::stopping);
  clock_ms=28000; update_sensor(clock_ms);
  clock_ms=29000; update_sensor(clock_ms); assert(sensor.valid);
  scd4x.co2=0;
  for(int i=0;i<3;i++){clock_ms+=1000;update_sensor(clock_ms);}
  assert(!sensor.valid && sensor.phase==SensorPhase::retry);
  reset(); cfg.eco=true; sensor.phase=SensorPhase::stopping;
  clock_ms=500; update_sensor(clock_ms); assert(scd4x.eco_starts==1);
  scd4x.ready=false; clock_ms=30500; update_sensor(clock_ms); assert(sensor.phase==SensorPhase::measuring);
  clock_ms=90500; update_sensor(clock_ms); assert(sensor.phase==SensorPhase::retry);
  reset();sensor.phase=SensorPhase::measuring;scd4x.temperature=NAN;
  for(int i=1;i<=3;++i){clock_ms=i*1000;update_sensor(clock_ms);}assert(!sensor.valid&&sensor.phase==SensorPhase::retry);
  assert(scd_crc(0x8000)==0xa2);
  reset(); assert(co2_band(299)==0 && co2_band(300)==1 && co2_band(600)==1);
  assert(co2_band(601)==2 && co2_band(1000)==2 && co2_band(1001)==3 && co2_band(1500)==3 && co2_band(1501)==4);
  sample(20,599);update_led(20);assert(led.band==1);
  sample(40,610);update_led(40);assert(led.band==1);
  sample(60,626);update_led(60);assert(led.band==2);
  sample(80,590);update_led(80);assert(led.band==2);
  sample(100,574);update_led(100);assert(led.band==1);
  reset();light.night=true;for(int i=0;i<3;++i)led.attached[i]=true;
  for(int i=1;i<=100;++i){sample(i*20,1200);update_led(i*20);}
  assert(led.duty[0]>led.duty[1]&&led.duty[1]>0&&led.duty[2]==0);
  cfg.led_night=0;sample(2020,1200);update_led(2020);for(auto d:led.duty)assert(d==0);
  cfg.led_night=1;cfg.led_enabled=false;sample(2040,1200);update_led(2040);for(auto d:led.duty)assert(d==0);
  reset();light.night=false;light.filtered=DARK_ADC;adc_value=DARK_ADC;
  clock_ms=100;update_light(clock_ms);clock_ms=5099;update_light(clock_ms);assert(!light.night);
  clock_ms=5200;update_light(clock_ms);assert(light.night);
  reset();light.night=false;light.filtered=DARK_ADC;adc_value=DARK_ADC;clock_ms=100;update_light(clock_ms);
  light.filtered=2221.5f;adc_value=2222;clock_ms=1000;update_light(clock_ms);assert(!light.pending);
  reset();sample(UINT32_MAX-1000,800);assert(sample_fresh(500));assert(!sample_fresh(14000));
  // Sensor service is nonblocking, and verifies the returned CRC before trusting it.
  reset();sensor.phase=SensorPhase::stopping;sensor.command=SensorCommand::self_test;
  clock_ms=500;update_sensor(clock_ms);assert(sensor.phase==SensorPhase::command_wait);
  clock_ms=10499;update_sensor(clock_ms);assert(sensor.phase==SensorPhase::command_wait);
  Wire.response_count=3;Wire.response[0]=0;Wire.response[1]=0;Wire.response[2]=scd_crc(0);
  clock_ms=10500;update_sensor(clock_ms);assert(sensor.command==SensorCommand::none&&strstr(sensor.result,"passed"));
  update_sensor(clock_ms);assert(sensor.phase==SensorPhase::measuring);
  reset();sensor.phase=SensorPhase::command_wait;sensor.command=SensorCommand::recalibrate;
  Wire.response_count=3;Wire.response[0]=0x80;Wire.response[1]=5;Wire.response[2]=scd_crc(0x8005);
  clock_ms=400;update_sensor(clock_ms);assert(strstr(sensor.result,"5 ppm"));
  reset();sensor.phase=SensorPhase::command_wait;sensor.command=SensorCommand::self_test;
  Wire.response_count=3;Wire.response[0]=0;Wire.response[1]=0;Wire.response[2]=0;
  clock_ms=10000;update_sensor(clock_ms);assert(sensor.phase==SensorPhase::retry&&strstr(sensor.result,"CRC"));
  // An opt-in cover/release gesture works in daylight and is suppressed at night.
  reset();cfg.gesture=true;cfg.dark_adc=0;cfg.bright_adc=100;light.filtered=100;light.night=false;screen_page=0;
  adc_value=0;clock_ms=100;update_light(clock_ms);assert(light.covered);
  adc_value=100;clock_ms=600;update_light(clock_ms);assert(screen_page==1&&!light.covered);
  light.night=true;screen_page=0;adc_value=0;clock_ms=700;update_light(clock_ms);assert(!light.covered&&screen_page==0);
  // Stable alarm, hysteresis, invalid data reset, and no stale ventilation success.
  reset();cfg.alarm_seconds=5;sample(1000,1600);tick(1000);assert(!analysis.alarm);
  sample(5999,1600);tick(5999);assert(!analysis.alarm);
  sample(7000,1600);tick(7000);assert(analysis.alarm);
  sample(8000,1490);tick(8000);assert(analysis.alarm);
  sample(9000,1470);tick(9000);assert(!analysis.alarm);
  sample(10000,1600);tick(10000);sensor.valid=false;tick(11000);assert(!analysis.alarm_pending);
  reset();analysis.vent_active=true;analysis.vent_ms=1000;
  sample(1000,700);tick(1000);assert(analysis.vent_pending);
  sensor.valid=false;tick(62000);assert(analysis.vent_active&&!analysis.vent_pending);
  sample(63000,700);tick(63000);sample(123000,700);tick(123000);assert(!analysis.vent_active&&analysis.vent_done);
  reset();light.night=true;sample(1000,1600);tick(1000);sample(2000,800);tick(2000);
  assert(night_stats.seconds==2&&night_stats.sum==2400&&night_stats.above_seconds==1);
  light.night=false;tick(3000);assert(last_night.seconds==2&&!analysis.night_active);
  // Minute aggregation, missing slots and ring wrap.
  reset();for(int i=1;i<=3;++i){sample(i*60000-1,600+i*20);on_sample(clock_ms);uptime_ms=i*60000;tick(i*60000);}
  assert(history_count==3&&analysis.trend_valid&&fabs(analysis.slope-20)<.01);
  tick(240000);assert(history_count==4&&!analysis.trend_valid&&!history[3].valid);
  sample(419999,800);on_sample(clock_ms);tick(420000);assert(history_count==7&&!history[6].valid);
  HistoryRecord r;r.valid=1;r.co2=800;for(int i=0;i<1500;++i)append_history(r);
  assert(history_count==HISTORY_SIZE);
  // All-or-nothing config validation, unknown fields, nulls, array shape, passwords.
  reset();assert(patch("{\"led_day\":100,\"eco\":true}"));assert(cfg.led_day==100&&cfg.eco);
  assert(!patch("{\"led_day\":80,\"bright_adc\":1}"));assert(cfg.led_day==100);
  assert(!patch("{\"mystery\":1}"));assert(!patch("{\"led_day\":null}"));
  assert(!patch("{\"led_day\":\"100\"}"));assert(!patch("{\"eco\":1}"));
  assert(!patch("{\"gains\":[1,2]}"));assert(!patch("{\"colors\":[[0,0,0]]}"));
  assert(!patch("{\"thresholds\":[300,320,1000,1500]}"));assert(!patch("{\"wifi_password\":\"short\"}"));
  assert(!patch("{\"pressure_pa\":1013}"));assert(patch("{\"pressure_pa\":101300}"));
  JsonDocument d;settings_json(d.to<JsonObject>(),false);assert(d["wifi_password"].isNull()&&d["mqtt_password"].isNull());
  assert(patch("{\"schema\":1,\"light_low\":0.2,\"light_high\":0.8}"));
  assert(patch("{\"wifi_tx_dbm\":13}"));assert(cfg.wifi_tx_dbm==13);
  assert(!patch("{\"wifi_tx_dbm\":20}"));assert(!patch("{\"wifi_tx_dbm\":0}"));
  // Calibration is sampled without blocking, and diagnostic light cannot change mode.
  reset();calibration.active=true;calibration.stage=0;adc_value=100;
  for(int i=1;i<=50;++i){clock_ms=i*100;update_light(clock_ms);}
  assert(calibration.dark_ready&&!calibration.active&&calibration.dark==100&&calibration.dark_noise==0);
  reset();optical.active=true;light.night=false;adc_value=0;
  for(int i=1;i<=210;++i){clock_ms=i*100;update_light(clock_ms);}
  assert(!optical.active&&optical.stage==7&&!light.night);
  reset();cfg.oled_sleep=true;light.night=true;screen_awake=false;display_dirty=true;
  update_display(250);assert(display.sleeping);clock_ms=300;wake_display(0);update_display(500);assert(!display.sleeping);
  puts("PASS: actual firmware state machines, eco timeout, recovery, CRC, night PWM, hysteresis, alarms, ventilation, history, settings validation, diagnostics and OLED sleep");
}
