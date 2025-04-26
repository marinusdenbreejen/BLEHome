#!/usr/bin/env python3
"""ble_visual.py
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

MQTT_TOPIC = "espresense/BLEtracker/#"

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

#load INDEX_HTML from file


INDEX_HTML = """<!doctype html>
<html lang=\"en\"><head><meta charset=\"utf-8\">
<title>ESPResense BLE Visualiser</title>
<style>
  html,body{margin:0;height:100%;width:100%;background:#111;font-family:sans-serif;color:#eee}
  #root{display:flex;height:100%}
  #canvas{flex:1}
  #sidebar{width:220px;background:#1b1b1b;padding:10px;box-sizing:border-box;overflow-y:auto}
  h3{margin:8px 0 4px;font-size:16px;border-bottom:1px solid #333}
  label{display:block;margin-bottom:4px}
  .device{cursor:pointer;padding:4px;border-radius:4px}
  .device:hover{background:#333}
  .selected{background:#555 !important}
</style></head><body>
<div id=\"root\">
  <canvas id=\"canvas\"></canvas>
  <div id=\"sidebar\">
    <h3>Floors</h3><div id=floorsUI></div>
    <h3>Devices</h3><div id=deviceUI></div>
  </div>
</div>
<script>
const canvas=document.getElementById('canvas');const ctx=canvas.getContext('2d');
let W,H;function resize(){W=canvas.width=window.innerWidth-220;H=canvas.height=window.innerHeight;}window.addEventListener('resize',resize);resize();
let data,lastStamp=0,visibleFloors=new Set(),selectedDevice=null;
const floorsUI=document.getElementById('floorsUI');const deviceUI=document.getElementById('deviceUI');
function makeFloorControls(floors){floorsUI.innerHTML='';floors.forEach(f=>{const id='fl_'+f.id;const c=document.createElement('label');c.innerHTML=`<input type=checkbox id=${id} checked> ${f.name}`;floorsUI.appendChild(c);document.getElementById(id).addEventListener('change',e=>{e.target.checked?visibleFloors.add(f.id):visibleFloors.delete(f.id);draw();});visibleFloors.add(f.id);});}
function makeDeviceList(devices){deviceUI.innerHTML='';Object.entries(devices).forEach(([did,d])=>{const div=document.createElement('div');div.className='device';div.textContent=d.device_name||did;div.onclick=()=>{selectedDevice=selectedDevice===did?null:did;draw();Array.from(deviceUI.children).forEach(el=>el.classList.remove('selected'));if(selectedDevice)div.classList.add('selected');};deviceUI.appendChild(div);});}
function tx(pt,b){return[(pt[0]-b.x)*b.s,(pt[1]-b.y)*b.s];}
function draw(){if(!data)return;ctx.clearRect(0,0,W,H);const b={x:data.bounds.x,y:data.bounds.y,s:Math.min(W/data.bounds.w,H/data.bounds.h)*0.9};const offX=(W-b.s*data.bounds.w)/2,offY=(H-b.s*data.bounds.h)/2;ctx.save();ctx.translate(offX,offY);
  // rooms
  data.floors.filter(f=>visibleFloors.has(f.id)).forEach(f=>{f.rooms.forEach(r=>{ctx.beginPath();r.points.forEach((p,i)=>{const[tX,tY]=tx(p,b);i?ctx.lineTo(tX,tY):ctx.moveTo(tX,tY);});ctx.closePath();ctx.fillStyle='rgba(80,80,80,.25)';ctx.fill();ctx.strokeStyle='#555';ctx.lineWidth=1;ctx.stroke();});});
  // nodes
  Object.values(data.nodes).forEach(n=>{if(!n.floor_ids.some(fid=>visibleFloors.has(fid)))return;const[pX,pY]=tx(n.point,b);ctx.beginPath();ctx.arc(pX,pY,5,0,2*Math.PI);ctx.fillStyle='#0ff';ctx.fill();ctx.strokeStyle='#005';ctx.stroke();ctx.fillStyle='#0ff';ctx.font='12px sans-serif';ctx.textAlign='left';ctx.fillText(n.name,pX+6,pY-6);});
  // devices
  Object.entries(data.devices).forEach(([did,d])=>{if(!visibleFloors.has(d.floor_id))return;const[pX,pY]=tx(d.position,b);ctx.beginPath();ctx.arc(pX,pY,6,0,2*Math.PI);ctx.fillStyle=did===selectedDevice?'#ff0':'#f80';ctx.fill();ctx.strokeStyle='#500';ctx.stroke();ctx.fillStyle='#f80';ctx.font='12px sans-serif';ctx.textAlign='left';ctx.fillText(d.device_name||did,pX+6,pY-6);});
  // distance circles for selected device
  if(selectedDevice&&data.devices[selectedDevice]){const d=data.devices[selectedDevice];for(const [n,dist] of Object.entries(d.node_distances||{})){const node=data.nodes[n];if(!node||!node.floor_ids.some(fid=>visibleFloors.has(fid)))continue;const[cx,cy]=tx(node.point,b);const radius=dist*b.s;ctx.beginPath();ctx.arc(cx,cy,radius,0,2*Math.PI);ctx.strokeStyle='rgba(255,255,0,.6)';ctx.lineWidth=1;ctx.setLineDash([6,4]);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#ff0';ctx.font='12px sans-serif';ctx.textAlign='center';ctx.fillText(dist.toFixed(1)+' m',cx,cy-radius-4);}}
  ctx.restore();}
async function poll(){try{const res=await fetch('/data');const d=await res.json();if(d.timestamp!==lastStamp){lastStamp=d.timestamp;data=d;if(floorsUI.childElementCount===0)makeFloorControls(d.floors);makeDeviceList(d.devices);draw();}}catch(e){console.error(e);}finally{setTimeout(poll,1000);} }poll();
</script></body></html>"""


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
        devices_js[did] = {
            "device_name": d.get("device_name", did),
            "position": [x_t, y_t],
            "floor_id": floor_id,
            "used_nodes": d.get("used_nodes", []),
            "node_distances": d.get("node_distances", {})
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

def main(host: str = "0.0.0.0", port: int = 5000):
    th = threading.Thread(target=mqtt_thread, daemon=True)
    th.start()
    print(f"[ble_visual_server] Serving http://{host}:{port}/  –  MQTT {MQTT_HOST}:{MQTT_PORT}")
    app.run(host=host, port=port, threaded=True, use_reloader=False)

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="ESPResense BLE visualisation Web server")
    ap.add_argument("--host", default="0.0.0.0", help="HTTP bind address (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=5000, help="HTTP port (default 5000)")
    args = ap.parse_args()
    main(args.host, args.port)
