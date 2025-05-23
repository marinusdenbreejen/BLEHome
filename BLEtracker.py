#!/usr/bin/env python

import time
import json
import yaml
import numpy as np


from datetime import datetime
from scipy.optimize import least_squares
import paho.mqtt.client as mqtt

import numpy as np
from scipy.optimize import minimize
import logging

# Set up a logger
logger = logging.getLogger("BLEtracker")
logging.basicConfig(level=logging.INFO)

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




def multilaterate_nm(nodes, measurements,
                     weight_fn=lambda d: 1.0/max(d, 0.1),
                     nm_options=None):
    """
    Multilateration via Nelder–Mead simplex.
    
    Args:
        nodes (dict): mapping node names → {"point": [x,y,z], ...}
        measurements (dict): mapping node names → measured distance (float)
        weight_fn (callable): fn(distance) → weight; defaults to 1/max(d,0.1)
        nm_options (dict): passed to scipy.optimize.minimize(method='Nelder-Mead')
        
    Returns:
        x_est (ndarray): estimated [x,y,z] device position
        used (list): list of node names actually used
        acc (float): RMS of un‐weighted residuals (lower is better)
    """
    # 1. Collect valid anchors
    pts, dists, used, wts = [], [], [], []
    for name, node in nodes.items():
        if name in measurements:
            dist = float(measurements[name])
            pts.append(np.array(node["point"], dtype=float))
            dists.append(dist)
            wts.append(weight_fn(dist))
            used.append(name)
    if len(pts) < 3:
        raise ValueError("Need at least 3 measurements for 3D multilateration.")
    
    pts  = np.vstack(pts)         # shape (M,3)
    dists= np.array(dists)        # shape (M,)
    wts  = np.array(wts)          # shape (M,)
    
    # 2. Initial guess = centroid of anchors
    x0 = pts.mean(axis=0)
    
    # 3. Objective = sum of squared weighted residuals
    def obj(x):
        res = wts * (np.linalg.norm(pts - x, axis=1) - dists)
        return np.sum(res**2)
    
    # 4. Run Nelder–Mead
    if nm_options is None:
        nm_options = {'xatol':1e-6, 'fatol':1e-6, 'maxiter':1000}
    res = minimize(obj, x0, method='Nelder-Mead', options=nm_options)
    
    # 5. Compute un‑weighted RMS error for interpretability
    final_res = np.linalg.norm(pts - res.x, axis=1) - dists
    acc = np.sqrt(np.mean(final_res**2))
    
    return res.x, used, acc



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
    floors = config.get("floors", [])
    if not floors:
        return "Unknown", 0.0

    # Find global zmin and zmax across all floors
    zmins = [floor["bounds"][0][2] for floor in floors]
    zmaxs = [floor["bounds"][1][2] for floor in floors]
    global_zmin = min(zmins)
    global_zmax = max(zmaxs)

    # Clamp z to within building bounds
    if z < global_zmin:
        z = global_zmin
    elif z > global_zmax:
        z = global_zmax

    candidates = []
    for floor in floors:
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
    for floor in floors:
        for room in floor.get("rooms", []):
            pts = room.get("points", [])
            if not pts:
                continue
            centroid = np.mean(pts, axis=0)
            d = np.linalg.norm(np.array([x, y]) - centroid)
            if d < min_dist:
                min_dist, closest_room = d, room.get("name", "Unknown")
    return closest_room, 0.0

# Smoothed room transitions
def get_stable_room(device_id, current_guess):
    """
    Determines the most stable room for a given device based on its recent room history.

    This function maintains a history of room guesses for each device and determines
    the most stable room by checking if the recent guesses are consistent. If all
    guesses in the history are the same, it returns the current guess as the stable room.
    Otherwise, it returns the second-to-last guess if available, or the current guess
    as a fallback.

    Args:
        device_id (str): The unique identifier for the device.
        current_guess (str): The current room guess for the device.

    Returns:
        str: The most stable room for the device based on its recent history.
    """
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
        if dist is None or dist > 10:
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
        fresh = {
            n: m["distance"]
            for n, m in meas.items()
            if now - m["timestamp"] <= measurement_timeout
        }
        last_seen = max((m["timestamp"] for m in meas.values()), default=0)
        since = now - last_seen
        dname = next((m.get("name") for m in meas.values()), "unknown")

        # will hold detailed per-node info
        used_nodes_info = []

        if since > away_timeout:
            stable, fast = "away", "away"
            acc, spos = None, [0, 0, 0]
        else:
            # attempt multilateration
            est_pos, used_nodes, acc = None, [], None
            if len(fresh) >= 3:
                try:
                    est_pos, used_nodes, acc = multilaterate_nm(nodes_dict, fresh)
                except Exception as e:
                    print(f"[BLEtracker] LS error for {did}:", e)

            if est_pos is None:
                fb = nearest_node_locator(fresh)
                if fb:
                    est_pos, used_nodes, acc = fb
                else:
                    stable, fast = "unknown", "unknown"
                    spos = [0, 0, 0]
                    acc = None
                    # publish with empty node info
                    publish_device_state(
                        client, did, fast, stable, dname,
                        acc, spos, fresh, now, used_nodes_info
                    )
                    continue

            # got a position → update smoothing and room detection
            update_smoothed_position(did, est_pos)
            spos = device_positions[did]
            raw, conf = find_room_for_point(spos)
            stable = get_stable_room(did, raw)
            fast = raw

            # build detailed per-node info
            for node in used_nodes:
                m = meas.get(node, {})
                used_nodes_info.append({
                    "node": node,
                    "distance": m.get("distance"),
                    "last_seen": m.get("timestamp")
                })

        # publish only the detailed info list
        publish_device_state(
            client, did, fast, stable, dname,
            acc, spos, fresh, now, used_nodes_info
        )


def publish_device_state(
    client, did, fast, stable, dname,
    acc, spos, fresh, now, used_nodes_info
):
    """
    Publishes the device state to the MQTT broker.
    """
    payload = {
        "id": did,
        "name": dname,
        "room_fast": fast,
        "room_stable": stable,
        "used_nodes_info": used_nodes_info,  # detailed per-node info only
        "accuracy": acc,
        "position": list(map(float, spos)),
        "fresh": fresh,
        "timestamp": now
    }
    topic = f"{MQTT_PUB_TOPIC_BASE}/detail/{did}"
    try:
        client.publish(topic, json.dumps(payload), qos=0, retain=True)
    except Exception as e:
        print(f"[BLEtracker] Failed to publish state for {did}: {e}")

    # Only publish short message if room_stable has changed
    prev_room = last_publish_time.get(did, {}).get("room_stable")
    if prev_room != stable:
        short_payload = {
            "room_stable": stable,
            "timestamp": now
        }
        short_topic = f"{MQTT_PUB_TOPIC_BASE}/{did}"
        try:
            client.publish(short_topic, json.dumps(short_payload), qos=0, retain=True)
        except Exception as e:
            print(f"[BLEtracker] Failed to publish short state for {did}: {e}")
        # Track last published stable room
        last_publish_time.setdefault(did, {})["room_stable"] = stable


# get all devices from mqtt base topic and clear retained messages, even if they are not in the config file
def clear_retained_messages(client):
    """
    Clears retained MQTT messages for each tracked device by publishing an empty payload 
    with retain=True on its detail and short‐state topics.
    """
    # If we have a list of device IDs from last_publish_time:
    for did in last_publish_time.keys():
        for suffix in ("", "detail/"):
            topic = f"{MQTT_PUB_TOPIC_BASE}/{suffix}{did}"
            try:
                client.publish(topic, payload="", qos=0, retain=True)
                logger.debug("Cleared retained message on %s", topic)
            except Exception:
                logger.exception("Failed to clear retained message on %s", topic)




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

    # Remove all devices from MQTT topics by publishing "away" state and clearing retained messages
    client = setup_mqtt_client()
    clear_retained_messages(client)
    print("[BLEtracker] MQTT client setup complete.")
    return client

# Entry point
if __name__ == "__main__":
    client = initialize_tracker()
    print("[BLEtracker] Running BLE tracker...")
    try:
        while True:
            process_tracking_cycle(client)
            time.sleep(1)
    except KeyboardInterrupt:
        print("[BLEtracker] Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()
