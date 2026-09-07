import importlib.util
import json
from pathlib import Path
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
spec=importlib.util.spec_from_file_location('collector',Path(__file__).resolve().parents[1]/'tools/collect_diagnostics.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def do_GET(self):
        if self.path.startswith('/redirect'):
            self.send_response(302);self.send_header('Location','http://127.0.0.1:1/');self.end_headers();return
        body=json.dumps({'boot_id':5,'uptime_seconds':12,'password':'do-not-save','pairing_code':'do-not-save'}).encode()
        self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers()
        try:
            if self.path.startswith('/slow'):
                for value in body:
                    self.wfile.write(bytes([value]));self.wfile.flush();time.sleep(.03)
            else:self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError):pass
class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.url='http://127.0.0.1:'+str(cls.server.server_port)
    @classmethod
    def tearDownClass(cls):cls.server.shutdown();cls.server.server_close();cls.thread.join()
    def test_normal_and_secret_filter(self):
        self.assertEqual(m.HttpSource(self.url,'admin','secret',1).read(),{'boot_id':5,'uptime_seconds':12})
    def test_slow_response_then_recovery(self):
        start=time.monotonic()
        with self.assertRaises(TimeoutError):m.HttpSource(self.url+'/slow','admin','secret',.1).read()
        self.assertLess(time.monotonic()-start,.5)
        self.assertEqual(m.HttpSource(self.url,'admin','secret',1).read()['boot_id'],5)
    def test_redirect_not_followed(self):
        with self.assertRaises(m.urllib.request.HTTPError) as e:m.HttpSource(self.url+'/redirect','admin','secret',1).read()
        self.assertEqual(e.exception.code,302);e.exception.close()
    def test_reboot_and_no_false_reboot_after_gap(self):
        t=m.Tracker();self.assertEqual(t.add({'boot_id':1,'uptime_seconds':10}),[])
        t.errors+=1;self.assertEqual(t.add({'boot_id':1,'uptime_seconds':20}),[])
        self.assertEqual(t.add({'boot_id':2,'uptime_seconds':2}),['boot_changed','uptime_decreased'])
        self.assertEqual(t.boot_changes,1)
    def test_usb_timeout_keeps_port_open(self):
        class Fake:
            closed=False
            def write(self,data):pass
            def read_until(self,*args,**kwargs):return b''
            def close(self):self.closed=True
        source=m.SerialSource.__new__(m.SerialSource);source.timeout=.01;source.connection=Fake()
        with self.assertRaises(TimeoutError):source.read()
        self.assertFalse(source.connection.closed)
    def test_usb_fragmented_json(self):
        class Fake:
            parts=iter([b'{"boot_',b'id":5}\\n'.replace(b'\\n',b'\n')])
            def write(self,data):pass
            def read_until(self,*args,**kwargs):return next(self.parts,b'')
            def close(self):pass
        source=m.SerialSource.__new__(m.SerialSource);source.timeout=.1;source.connection=Fake()
        self.assertEqual(source.read(),{'boot_id':5})
    def test_empty_or_invalid_payload(self):
        with self.assertRaises(ValueError):m.sanitize([])
        self.assertEqual(m.sanitize({'password':'secret'}),{})
if __name__=='__main__':unittest.main()
