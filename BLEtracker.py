#!/usr/bin/env python

import time
import json
import yaml
import numpy as np
from datetime import datetime
from scipy.optimize import least_squares
import paho.mqtt.client as mqtt

# Global configuration and runtime tracking variables
config = None
allowed_devices = []
nodes_dict = {}
node_measurements = {}  # device_id -> node_name -> measurement
device_positions = {}  # smoothed position per device
room_histories = {}  # room history per device
discovery_published = set()

# Smoothing and stability parameters
alpha = 0.2
room_stability_threshold = 10

MQTT_SUB_TOPIC = "espresense/devices/#"
MQTT_PUB_TOPIC_BASE = "espresense/BLEtracker"

# Load the YAML configuration for nodes, devices, and settings
def load_config():
    global config, nodes_dict, allowed_devices, away_timeout
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)
    nodes_dict = {n["name"].lower(): n for n in config.get("nodes", [])}
    allowed_devices = config.get("devices", [])
    away_timeout = config.get("away_timeout", 120)
    print("[BLEtracker] Configuration loaded.")

# Check if the incoming device matches the whitelist in the config
def is_device_allowed(device_data):
    device_id = device_data.get("id", "")
    device_name = device_data.get("name", "")
    for entry in allowed_devices:
        if entry.get("name") == "*" and device_name:
            return True
        eid = entry.get("id")
        if eid and eid.endswith("*") and device_id.startswith(eid[:-1]):
            return True
        if eid and device_id == eid:
            return True
    return False

# Multilateration via weighted least squares
# Given RSSI-derived distances to at least three fixed nodes, solve for the device position that best
# fits the measured distances. Returns the estimated XYZ position, the list of nodes used, and a
# residual-based accuracy metric (lower is better).
def multilaterate(nodes, measurements):
    # 1. Gather valid node positions and their corresponding measured distances
    pts, dists, used, wts = [], [], [], []
    for nm, node in nodes.items():
        if nm in measurements:
            p = np.array(node["point"], dtype=float)  # fixed node coordinates
            dist = measurements[nm]  # measured distance to node
            pts.append(p)
            dists.append(dist)
            # Weight inversely proportional to distance (avoid division by zero)
            wts.append(1.0 / max(dist, 0.1))
            used.append(nm)
    # If no measurements are available, cannot multilaterate
    if not pts:
        raise ValueError("No measurements for multilateration.")

    # Convert lists to numpy arrays for vectorized computation
    pts = np.array(pts)
    dists = np.array(dists)
    wts = np.array(wts)

    # 2. Initial guess: centroid of node positions
    x0 = np.mean(pts, axis=0)

    # 3. Define residuals function: difference between estimated and measured distances,
    # scaled by weights
    def residuals(X):
        return wts * (np.linalg.norm(pts - X, axis=1) - dists)

    # 4. Run least-squares optimization to minimize weighted residuals
    result = least_squares(residuals, x0)

    # 5. Compute accuracy as RMS of normalized residuals
    acc = np.sqrt(np.mean((residuals(result.x) / wts)**2))

    # Return estimated position, which nodes were used, and accuracy metric
    return result.x, used, acc

# Check if a 2D point is inside a polygon using ray-casting algorithm
def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y) and y <= max(p1y, p2y) and x <= max(p1x, p2x):
            if p1y != p2y:
                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
            if p1x == p2x or x <= xinters:
                inside = not inside
        p1x, p1y = p2x, p2y
    return inside

# Determine which room a 3D point belongs to, or fallback to closest room
def find_room_for_point(P):
    x, y, z = P
    candidates = []
    for floor in config.get("floors", []):
        zmin, zmax = floor["bounds"][0][2], floor["bounds"][1][2]
        if not (zmin <= z <= zmax):
            continue
        for room in floor.get("rooms", []):
            if point_in_polygon(x, y, room["points"]):
                candidates.append(room.get("name", "Unknown"))
    if candidates:
        return candidates[0], 100.0
    # fallback to closest room centroid
    closest_room, min_dist = "Unknown", float('inf')
    for floor in config.get("floors", []):
        for room in floor.get("rooms", []):
            pts = room.get("points", [])
            if not pts: continue
            centroid = np.mean(pts, axis=0)
            d = np.linalg.norm(np.array([x, y]) - centroid)
            if d < min_dist:
                min_dist, closest_room = d, room.get("name", "Unknown")
    return closest_room, 0.0

# Smoothed room transitions
def get_stable_room(device_id, current_guess):
    room_histories.setdefault(device_id, []).append(current_guess)
    hist = room_histories[device_id]
    if len(hist) > room_stability_threshold:
        hist.pop(0)
    if len(set(hist)) == 1:
        return current_guess
    return hist[-2] if len(hist) > 1 else current_guess

# Nearest-node fallback locator
def nearest_node_locator(fresh):
    cfg = config.get("locators", {}).get("nearest_node", {})
    if not cfg.get("enabled", False) or not fresh:
        return None
    max_d = cfg.get("max_distance")
    name, dist = min(fresh.items(), key=lambda x: x[1])
    if max_d is not None and dist > max_d:
        return None
    point = np.array(nodes_dict[name]["point"], dtype=float)
    return point, [name], dist

# MQTT message handler
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if not is_device_allowed(data):
            return
        dist = data.get("distance")
        if data.get("var", 0) > 0.5:
            return
        node = msg.topic.split("/")[-1].lower()
        did = data.get("id", "unknown")
        node_measurements.setdefault(did, {})[node] = {
            "distance": dist,
            "timestamp": time.time(),
            "name": data.get("name", "unknown")
        }
    except Exception as e:
        print("[BLEtracker] Error processing MQTT message:", e)

# MQTT connect callback
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[BLEtracker] Connected to {mqtt_host}:{mqtt_port}")
        client.subscribe(MQTT_SUB_TOPIC)
    else:
        print(f"[BLEtracker] MQTT connect failed: {rc}")

def setup_mqtt_client():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    if mqtt_username:
        client.username_pw_set(mqtt_username, mqtt_password)
    if mqtt_ssl:
        client.tls_set()
    client.connect(mqtt_host, mqtt_port, 60)
    client.loop_start()
    return client

# Exponential smoothing
def update_smoothed_position(did, pos):
    if did not in device_positions:
        device_positions[did] = pos
    else:
        device_positions[did] = alpha*pos + (1-alpha)*device_positions[did]

# Main tracking cycle
def process_tracking_cycle(client):
    now = time.time()
    for did, meas in node_measurements.items():
        # fresh measurements for multilateration
        fresh = {n: m["distance"] for n, m in meas.items() if now - m["timestamp"] <= measurement_timeout}
        # time since last seen
        last_seen = max((m["timestamp"] for m in meas.values()), default=0)
        since = now - last_seen
        dname = next((m.get("name") for m in meas.values()), "unknown")
        if since > away_timeout:
            stable, fast = "away", "away"
            acc, used, spos = None, [], [0,0,0]
        else:
            # attempt multilateration
            est_pos, used, acc = None, [], None
            if len(fresh) >= 3:
                try:
                    est_pos, used, acc = multilaterate(nodes_dict, fresh)
                except Exception as e:
                    print(f"[BLEtracker] LS error for {did}:", e)
            # fallback if no estimate
            if est_pos is None:
                fb = nearest_node_locator(fresh)
                if fb:
                    est_pos, used, acc = fb
                else:
                    stable, fast = "unknown", "unknown"
                    spos = [0,0,0]
                    accuracy = None
                    used_nodes = []
                    goto_publish = True
            if est_pos is not None:
                update_smoothed_position(did, est_pos)
                spos = device_positions[did]
                raw, conf = find_room_for_point(spos)
                stable = get_stable_room(did, raw)
                fast = raw
        # publish if interval elapsed
        if now - last_publish_time.get(did, 0) >= publish_interval:
            ts = datetime.now().isoformat()
            payload = {
                "fast_room": fast,
                "stable_room": stable,
                "used_nodes": used,
                "device_name": dname,
                "device_id": did,
                "timestamp": ts,
                "accuracy": acc,
                "position": list(map(float, spos))
            }
            topic = f"{MQTT_PUB_TOPIC_BASE}/{did}"
            client.publish(topic, json.dumps(payload))
            print(f"[BLEtracker] {did} at {stable} ({acc if acc else 'N/A'})")
            last_publish_time[did] = now

# Initialization
def initialize_tracker():
    global mqtt_host, mqtt_port, mqtt_username, mqtt_password, mqtt_ssl
    global publish_interval, last_publish_time, measurement_timeout
    load_config()
    mcfg = config.get("mqtt", {})
    mqtt_host = mcfg.get("host", "localhost")
    mqtt_port = mcfg.get("port", 1883)
    mqtt_username = mcfg.get("username")
    mqtt_password = mcfg.get("password")
    mqtt_ssl = mcfg.get("ssl", False)
    publish_interval = 1
    last_publish_time = {}
    measurement_timeout = config.get("timeout", 30)
    return setup_mqtt_client()

# Entry point
if __name__ == "__main__":
    client = initialize_tracker()
    print("[BLEtracker] Running BLE tracker...")
    try:
        while True:
            process_tracking_cycle(client)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[BLEtracker] Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()
