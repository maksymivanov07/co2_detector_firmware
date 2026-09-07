#!/usr/bin/env python3
"""Local UI fixture only; this does NOT emulate ESP32 networking or HomeKit."""
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
root=Path(__file__).resolve().parents[1]
settings=json.loads(Path('/tmp/co2-defaults.json').read_text())
settings['_revision']=1
now=int(time.time())
state=dict(version='4.0.0 · DEMO',device_id='co2-preview',uptime_seconds=7200,epoch=now,fresh=True,
           co2=846,temperature=23.4,humidity=46.2,sample_age_seconds=1,night=False,adc=2450,filtered_adc=2448.6,
           light=.8,alarm=False,suspicious=False,slope=-12.4,night_active=False,
           night_stats=dict(seconds=21600,above_seconds=1200,peak=1620,average=960),
           vent=dict(active=False,done=False,start=0,current=0,seconds=0),history_count=120,
           led_enabled=True,wifi_enabled=True,wifi_connected=True,ip='192.0.2.5',ap=False,mqtt_connected=False,
           homekit_enabled=True,homekit_paired=False,pairing_code='37123456',sensor_result='Not run',
           sensor_phase=3,sensor_errors=0,sensor_restarts=0,sensor_serial='000000123456',sensor_offset=4.0,
           sensor_altitude=0,sensor_asc=1,heap_free=142000,heap_min=130000,chip='ESP32-S3 (preview)',flash_bytes=4194304,
           reset_reason=1,filesystem=True,archive_enabled=False,last_network_error='',ota_rollback=True,
           calibration=dict(active=False,dark_ready=False,bright_ready=False,dark=0.0,bright=0.0,dark_noise=0.0,bright_noise=0.0,result='Not started'),
           optical=dict(active=False,stage=0,levels=[0.0]*7),events=[dict(seconds=0,message='Browser fixture: no hardware connected')])
records=[[now-(119-i)*60,i,round(1000+220*math.sin(i/17)),23.4,46.2,0] for i in range(120)]
for i in range(65,70):records[i][2:5]=[None,None,None]
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def respond(self,data,code=200,mime='application/json'):
        body=data.encode() if isinstance(data,str) else json.dumps(data).encode()
        self.send_response(code);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_GET(self):
        if self.path=='/':self.respond((root/'web_ui.h').read_text().split('R"CO2HTML(',1)[1].rsplit(')CO2HTML"',1)[0],mime='text/html; charset=utf-8')
        elif self.path=='/api/status':self.respond(state)
        elif self.path in ['/api/settings','/api/backup']:self.respond(settings)
        elif self.path=='/api/history':self.respond(records)
        elif self.path=='/api/history.csv':self.respond('epoch,co2\n0,846\n',mime='text/csv')
        else:self.respond({'error':'Not found'},404)
    def do_POST(self):
        data=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0'))))
        if self.headers.get('X-CO2-Request')!='1':return self.respond({'error':'Missing header'},403)
        if self.path=='/api/settings':
            if data.get('led_night',settings['led_night'])>data.get('led_day',settings['led_day']):return self.respond({'error':'night brightness <= day brightness'},400)
            settings.update(data);state['led_enabled']=settings['led_enabled'];settings['_revision']+=1;self.respond({'ok':True,'reboot':False,'revision':settings['_revision']})
        elif self.path=='/api/action':
            action=data.get('action')
            if action=='vent_start':state['vent']=dict(active=True,done=False,start=846,current=846,seconds=0)
            elif action=='vent_stop':state['vent']['active']=False;state['vent']['done']=True
            elif action=='cal_dark':state['calibration'].update(dark=100.0,dark_noise=1.0,dark_ready=True,result='Dark captured')
            elif action=='cal_bright':state['calibration'].update(bright=2000.0,bright_noise=2.0,bright_ready=True,result='Bright captured')
            elif action=='cal_apply':state['calibration']['result']='Calibration saved'
            self.respond({'ok':True})
        else:self.respond({'error':'Not found'},404)
if __name__=='__main__':
    print('UI fixture: http://127.0.0.1:8765 (not device firmware)',flush=True)
    ThreadingHTTPServer(('127.0.0.1',8765),Handler).serve_forever()
