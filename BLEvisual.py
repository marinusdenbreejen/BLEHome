#!/usr/bin/env python3
"""ble_visual_server.py
=================================
Interactive Web visualisation of ESPresense rooms, nodes and BLE devices.

New features requested
----------------------
1. **Selectable floors** – check‑boxes let you toggle each floor layer on/off.
2. **Labels** – every node and device is drawn with its name (device‑name or id).
3. **Device list** – a sidebar lists all live devices. Click one to *select* it.
4. **Distance circles** – when a device is selected, circles are drawn around each
   *used* node at a radius equal to the reported distance; each circle is labelled
   with that distance (metres, 1‑decimal).

The server still:
* uses **paho‑mqtt** (same as BLEtracker.py) for live updates.
* reads **config.yaml** for geometry, map settings and nodes.
* serves a single SPA via Flask (no extra static files required).

Run with:
```bash
pip install flask pyyaml paho-mqtt numpy
python ble_visual_server.py
``` 
then browse to *http://localhost:5000/*. 
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Dict, Any, List

import yaml
import numpy as np
from flask import Flask, jsonify, render_template_string
import paho.mqtt.client as mqtt  # ⭐ SAME LIBRARY as BLEtracker.py

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CONFIG_PATH = Path(__file__).with_name("config.yaml")
with CONFIG_PATH.open() as f:
    CONFIG: Dict[str, Any] = yaml.safe_load(f)

MAP_CFG = CONFIG.get("map", {})
FLIP_X = bool(MAP_CFG.get("flip_x", False))
FLIP_Y = bool(MAP_CFG.get("flip_y", True))

FLOORS: List[Dict[str, Any]] = CONFIG.get("floors", [])
NODES_CFG: Dict[str, Dict[str, Any]] = {n["name"].lower(): n for n in CONFIG.get("nodes", [])}

MQTT_CFG = CONFIG.get("mqtt", {})
MQTT_HOST = MQTT_CFG.get("host", "localhost")
MQTT_PORT = int(MQTT_CFG.get("port", 1883))
MQTT_USER = MQTT_CFG.get("username")
MQTT_PASS = MQTT_CFG.get("password")
MQTT_SSL  = bool(MQTT_CFG.get("ssl", False))

MQTT_TOPIC = "espresense/BLEtracker/detail/#"

# ---------------------------------------------------------------------------
# Runtime state (updated by MQTT thread)
# ---------------------------------------------------------------------------
DEVICES_LOCK = threading.Lock()
DEVICES: Dict[str, Dict[str, Any]] = {}  # did → payload + "_recv" ts

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def transform_point(x: float, y: float) -> tuple[float, float]:
    """Flip/translate according to map settings so JS can draw in screen space."""
    if FLIP_X:
        x = FLOORS[0]["bounds"][1][0] - x  # width − x
    if FLIP_Y:
        y = FLOORS[0]["bounds"][1][1] - y  # height − y
    return x, y


def floor_id_for_z(z: float) -> str | None:
    """Return floor id whose z‑bounds contain *z*, else None."""
    for f in FLOORS:
        z0, z1 = f["bounds"][0][2], f["bounds"][1][2]
        if z0 <= z <= z1:
            return f["id"]
    return None

# ---------------------------------------------------------------------------
# MQTT thread
# ---------------------------------------------------------------------------

def on_connect(client: mqtt.Client, userdata, flags, rc):
    if rc == 0:
        print("[ble_visual_server] MQTT connected")
        client.subscribe(MQTT_TOPIC)
    else:
        print("[ble_visual_server] MQTT connect failed", rc)


def on_message(client: mqtt.Client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        did = payload.get("device_id") or payload.get("id")
        if not did:
            return
        with DEVICES_LOCK:
            DEVICES[did] = payload | {"_recv": time.time()}
    except Exception as e:
        print("[ble_visual_server] MQTT message error:", e)


def mqtt_thread():
    client = mqtt.Client()
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    if MQTT_SSL:
        client.tls_set()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_forever(retry_first_connection=True)

# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
app = Flask(__name__)

# load INDEX_HTML from file
INDEX_HTML = Path(__file__).with_name("BLEvisual.html").read_text(encoding="utf-8")

@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

@app.route("/data")
def data_endpoint():
    now = time.time()
    # Prune stale (> 30 s) devices
    with DEVICES_LOCK:
        stale = [k for k, v in DEVICES.items() if now - v.get("_recv", 0) > 30]
        for k in stale:
            DEVICES.pop(k, None)
        dev_copy = {k: v.copy() for k, v in DEVICES.items()}

    # Global XY bounds (for all floors)
    x_min = min(f["bounds"][0][0] for f in FLOORS)
    y_min = min(f["bounds"][0][1] for f in FLOORS)
    x_max = max(f["bounds"][1][0] for f in FLOORS)
    y_max = max(f["bounds"][1][1] for f in FLOORS)
    bounds = {"x": x_min, "y": y_min, "w": x_max - x_min, "h": y_max - y_min}

    # Floors and room polygons (XY transformed only once)
    floors_js = []
    for f in FLOORS:
        rooms_js = []
        for r in f.get("rooms", []):
            pts = [list(transform_point(*pt)) for pt in r.get("points", [])]
            rooms_js.append({"name": r.get("name"), "points": pts})
        floors_js.append({"id": f["id"], "name": f["name"], "rooms": rooms_js})

    # Nodes
    nodes_js = {}
    for name, n in NODES_CFG.items():
        node_pt = transform_point(n["point"][0], n["point"][1])
        nodes_js[name] = {
            "name": n["name"],
            "point": node_pt,
            "floor_ids": n.get("floors", []) or []
        }

    # Devices
    devices_js = {}
    for did, d in dev_copy.items():
        pos = d.get("position") or [0, 0, 0]
        x_t, y_t = transform_point(pos[0], pos[1])
        floor_id = floor_id_for_z(pos[2]) or "unknown"

        # pull out used_nodes_info
        used_info = d.get("used_nodes_info", [])

        # build used_nodes and distances
        used_nodes = [info.get("node") for info in used_info]
        node_distances = {info.get("node"): info.get("distance") for info in used_info}

        devices_js[did] = {
            "device_name": d.get("name", did),
            "position": [x_t, y_t],
            "floor_id": floor_id,
            "room_fast": d.get("room_fast"),
            "room_stable": d.get("room_stable"),
            "used_nodes": used_nodes,
            "node_distances": node_distances,
            "used_nodes_info": used_info
        }

    return jsonify({
        "timestamp": time.time(),
        "bounds": bounds,
        "floors": floors_js,
        "nodes": nodes_js,
        "devices": devices_js
    })

# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------
def main(host: str = "0.0.0.0", port: int = 8000):
    th = threading.Thread(target=mqtt_thread, daemon=True)
    th.start()
    print(f"[ble_visual_server] Serving http://{host}:{port}/  –  MQTT {MQTT_HOST}:{MQTT_PORT}")
    app.run(host=host, port=port, threaded=True, use_reloader=False)

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="ESPResense BLE visualisation Web server")
    ap.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--port", type=int, default=8000, help="HTTP port")
    args = ap.parse_args()
    main(args.host, args.port)
