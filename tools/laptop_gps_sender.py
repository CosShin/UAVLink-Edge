#!/usr/bin/env python3
"""Serve a localhost geolocation page and relay fixes to the Pi receiver."""

from __future__ import annotations

import argparse
import json
import threading
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = """<!doctype html>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Laptop GPS sender</title>
<style>body{font:16px sans-serif;max-width:760px;margin:30px auto;padding:0 16px;background:#111;color:#eee}button{font-size:18px;padding:12px 20px}pre{white-space:pre-wrap;background:#222;padding:14px;border-radius:8px}.ok{color:#5fda79}.bad{color:#ff7676}</style>
<h1>Laptop GPS to Pi</h1>
<p>Keep Pixhawk DISARMED. Allow Location when the browser asks.</p>
<button id=start>Start location</button> <button id=stop disabled>Stop</button>
<pre id=status>Not started</pre>
<script>
let watchId=null,lastFix=null,timer=null;
const status=document.getElementById('status');
async function send(){
 if(!lastFix)return;
 try{const r=await fetch('/gps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(lastFix)});const j=await r.json();status.className=j.ok?'ok':'bad';status.textContent=JSON.stringify({...lastFix,pi:j},null,2)}
 catch(e){status.className='bad';status.textContent='Cannot send location to Pi: '+e}
}
document.getElementById('start').onclick=()=>{
 if(!navigator.geolocation){status.textContent='Browser does not support Geolocation';return}
 watchId=navigator.geolocation.watchPosition(p=>{const c=p.coords;lastFix={latitude:c.latitude,longitude:c.longitude,accuracy_m:c.accuracy,altitude_m:c.altitude,altitude_accuracy_m:c.altitudeAccuracy,speed_m_s:c.speed,heading_deg:c.heading,browser_timestamp_ms:p.timestamp};send()},e=>{status.className='bad';status.textContent='Geolocation error '+e.code+': '+e.message},{enableHighAccuracy:true,maximumAge:0,timeout:15000});
 timer=setInterval(send,1000);document.getElementById('start').disabled=true;document.getElementById('stop').disabled=false;
};
document.getElementById('stop').onclick=()=>{if(watchId!==null)navigator.geolocation.clearWatch(watchId);clearInterval(timer);watchId=null;lastFix=null;status.textContent='Stopped';document.getElementById('start').disabled=false;document.getElementById('stop').disabled=true};
</script>""".encode("utf-8")


def make_handler(pi_url: str, token: str):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status: int, content_type: str, body: bytes):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._reply(200, "text/html; charset=utf-8", HTML)
            else:
                self._reply(404, "text/plain", b"not found")

        def do_POST(self):
            if self.path != "/gps":
                self._reply(404, "application/json", b'{"ok":false}')
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = self.rfile.read(length)
                json.loads(payload.decode("utf-8"))
                request = urllib.request.Request(
                    pi_url,
                    data=payload,
                    headers={"Content-Type": "application/json", "X-GPS-Token": token},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=3) as response:
                    body = response.read()
                    status = response.status
            except urllib.error.HTTPError as exc:
                status, body = exc.code, exc.read()
            except Exception as exc:
                status = 502
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
            self._reply(status, "application/json", body)

        def log_message(self, format, *args):
            pass

    return Handler


def build_parser():
    parser = argparse.ArgumentParser(description="Laptop browser geolocation sender for UAVLink Pi")
    parser.add_argument("--pi-host", required=True)
    parser.add_argument("--pi-port", type=int, default=8766)
    parser.add_argument("--token", required=True)
    parser.add_argument(
        "--listen-port",
        type=int,
        default=0,
        help="Local browser port; 0 lets Windows choose an available port",
    )
    parser.add_argument("--no-browser", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if len(args.token) < 16:
        raise SystemExit("--token must contain at least 16 characters")
    pi_url = f"http://{args.pi_host}:{args.pi_port}/gps"
    try:
        server = ThreadingHTTPServer(
            ("127.0.0.1", args.listen_port),
            make_handler(pi_url, args.token),
        )
    except PermissionError as exc:
        raise SystemExit(
            "Windows blocked Python from opening a localhost port. "
            "Test: python -m http.server 0 --bind 127.0.0.1. "
            "If that also fails, allow python.exe in Windows Security/antivirus."
        ) from exc
    actual_port = int(server.server_address[1])
    local_url = f"http://127.0.0.1:{actual_port}/"
    print(f"Laptop page: {local_url}")
    print(f"Relaying fixes to: {pi_url}")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(local_url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
