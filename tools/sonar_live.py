#!/usr/bin/env python3
"""Live sonar readout on :8903 - a big number in the browser, refreshed twice
a second, straight from the ESP32 serial stream. For tape-measure checks
(2026-08-26: the sonar read 0.08 m for two hours and nobody could SEE it).
Run with the dimos stack STOPPED (it owns the ESP port otherwise)."""
import re, threading, time
from http.server import BaseHTTPRequestHandler, HTTPServer
import serial
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vector_dimos.esp_sensors import ESP_BAUD, ESP_PORT

latest = {"m": None, "t": 0.0, "raw": ""}

def reader():
    while True:
        try:
            ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=2.0)
            for _ in iter(int, 1):
                ln = ser.readline().decode(errors="replace").strip()
                if not ln:
                    continue
                latest["raw"] = ln
                m = re.search(r"SONAR ([0-9.]+)", ln)
                if m:
                    latest["m"] = float(m.group(1)); latest["t"] = time.time()
        except Exception:
            time.sleep(2.0)

PAGE = """<meta http-equiv="refresh" content="0.5"><body style="background:#111;color:#eee;
font-family:system-ui;display:flex;flex-direction:column;align-items:center;justify-content:center;height:95vh">
<div style="font-size:18vw;font-weight:700;color:%s">%s</div>
<div style="font-size:3vw;color:#888">sonar &middot; %s &middot; last line: %s</div></body>"""

class H(BaseHTTPRequestHandler):
    def do_GET(self):
        m, age = latest["m"], time.time() - latest["t"]
        if m is None or age > 3:
            txt, col, sub = "&mdash;", "#888", "no data"
        else:
            txt, col, sub = f"{m:.2f} m", ("#e33" if m < 0.30 else "#3c3"), f"{age:.1f}s old"
        body = (PAGE % (col, txt, sub, latest["raw"][:60])).encode()
        self.send_response(200); self.send_header("Content-Type", "text/html"); self.end_headers()
        self.wfile.write(body)
    def log_message(self, *a): pass

threading.Thread(target=reader, daemon=True).start()
print("sonar live on http://0.0.0.0:8903", flush=True)
HTTPServer(("0.0.0.0", 8903), H).serve_forever()
