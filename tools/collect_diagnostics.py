#!/usr/bin/env python3
"""Bounded foreground capture of CO2 diagnostics over HTTP or USB; no settings writes."""
import argparse
import base64
from datetime import datetime, timezone
import getpass
import json
import os
from pathlib import Path
import time
import urllib.request
from urllib.parse import urlparse

FIELDS = frozenset('version boot_id uptime_seconds retained_boot_count reset_reason previous_network_stage wifi_connected wifi_rssi wifi_tx_dbm ip ap wifi_disconnect_count wifi_got_ip_count wifi_disconnect_reason wifi_disconnect_ms wifi_disconnect_rssi http_aborted_responses heap_free heap_min fresh co2 stage_max_ms'.split())

def sanitize(data):
    if not isinstance(data, dict):
        raise ValueError('Expected a JSON object')
    return {key: value for key, value in data.items() if key in FIELDS}

class Tracker:
    def __init__(self):
        self.previous = None
        self.samples = self.errors = self.boot_changes = self.uptime_drops = 0
    def add(self, data):
        flags = []
        self.samples += 1
        if self.previous:
            old = self.previous
            if 'boot_id' in old and 'boot_id' in data and old['boot_id'] != data['boot_id']:
                self.boot_changes += 1
                flags.append('boot_changed')
            if isinstance(old.get('uptime_seconds'), (int, float)) and isinstance(data.get('uptime_seconds'), (int, float)) and data['uptime_seconds'] < old['uptime_seconds']:
                self.uptime_drops += 1
                flags.append('uptime_decreased')
        self.previous = data
        return flags

class HttpSource:
    def __init__(self, url, username, password, timeout):
        if not url.startswith('http://') or urlparse(url).username:
            raise ValueError('Use a local http:// URL without embedded credentials')
        self.url = url.rstrip('/') + '/api/status'
        self.timeout = timeout
        self.authorization = 'Basic ' + base64.b64encode((username + ':' + password).encode()).decode()
        # Do not follow redirects carrying authorization to another host.
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                return None
        self.opener = urllib.request.build_opener(NoRedirect)
    def read(self):
        request = urllib.request.Request(self.url, headers={'Authorization': self.authorization})
        deadline = time.monotonic() + self.timeout
        body = bytearray()
        with self.opener.open(request, timeout=self.timeout) as response:
            while len(body) <= 65536:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Response deadline')
                part = response.read1(min(4096, 65537-len(body)))
                if not part:
                    break
                body.extend(part)
        if len(body) > 65536:
            raise ValueError('Response too large')
        return sanitize(json.loads(body))
    def close(self):
        pass

class SerialSource:
    def __init__(self, port, timeout):
        import serial
        self.serial_module = serial
        self.port, self.timeout, self.connection = port, timeout, None
    def read(self):
        try:
            if self.connection is None:
                self.connection = self.serial_module.Serial(self.port, baudrate=115200, timeout=0.2, write_timeout=self.timeout)
            self.connection.write(b'diagnostics\n')
            deadline = time.monotonic() + self.timeout
            pending = bytearray()
            while time.monotonic() < deadline:
                part = self.connection.read_until(b'\n', size=8193)
                pending.extend(part)
                if len(pending) > 8192:
                    pending.clear()
                if not part.endswith(b'\n'):
                    continue
                raw = bytes(pending)
                pending.clear()
                if raw.startswith(b'{') and len(raw) <= 8192:
                    try:
                        data = sanitize(json.loads(raw))
                        if 'boot_id' in data:
                            return data
                    except (ValueError, UnicodeError):
                        pass
            raise TimeoutError('No diagnostics response')
        except TimeoutError:
            # A slow boot/response is not a disconnected USB port. Reopening can reset the MCU.
            raise
        except Exception:
            self.close()
            raise
    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--url', help='Device URL on the home LAN')
    mode.add_argument('--port', help='USB serial port; requires pyserial and firmware 4.1.2+')
    parser.add_argument('--duration', type=float, default=900, help='Capture seconds, default 900')
    parser.add_argument('--interval', type=float, default=5)
    parser.add_argument('--timeout', type=float, default=3)
    parser.add_argument('--credentials', type=Path, help='Local JSON with user/password; otherwise prompted')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if min(args.duration, args.interval, args.timeout) <= 0:
        parser.error('Duration, interval and timeout must be positive')
    if args.url:
        credentials = json.loads(args.credentials.read_text()) if args.credentials else {'user': 'admin', 'password': getpass.getpass('Device password: ')}
        source = HttpSource(args.url, credentials['user'], credentials['password'], args.timeout)
    else:
        source = SerialSource(args.port, args.timeout)
    output = args.output or Path('build/diagnostics') / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '.jsonl')
    output.parent.mkdir(parents=True, exist_ok=True)
    tracker = Tracker()
    start = time.monotonic()
    try:
        with os.fdopen(os.open(output, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600), 'w') as log:
            print(f'Capturing to {output.resolve()}; Ctrl-C stops safely.', flush=True)
            while time.monotonic() - start < args.duration:
                began = time.monotonic()
                record = {'time_utc': datetime.now(timezone.utc).isoformat()}
                try:
                    data = source.read()
                    record.update(ok=True, data=data, flags=tracker.add(data))
                except Exception as error:
                    tracker.errors += 1
                    # Error type only: never write response bodies, headers, or passwords.
                    record.update(ok=False, error=type(error).__name__)
                record['elapsed_ms'] = round((time.monotonic() - began) * 1000)
                log.write(json.dumps(record, ensure_ascii=False) + '\n')
                log.flush()
                remaining = args.duration - (time.monotonic() - start)
                time.sleep(max(0, min(remaining, args.interval - (time.monotonic() - began))))
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
    print(json.dumps(vars(tracker) | {'previous': None, 'output': str(output.resolve())}, ensure_ascii=False))

if __name__ == '__main__':
    main()
