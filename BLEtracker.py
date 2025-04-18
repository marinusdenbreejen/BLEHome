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
alpha = 0.3
room_stability_threshold = 5

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

# Estimate position using weighted least squares multilateration

def multilaterate(nodes, measurements):
    positions, distances, used_nodes, weights = [], [], [], []
    for name, node in nodes.items():
        if name in measurements:
            point = np.array(node["point"], dtype=float)
            distance = measurements[name]
            weight = 1.0 / max(distance, 0.1)
            positions.append(point)
            distances.append(distance)
            weights.append(weight)
            used_nodes.append(name)

    if not positions:
        raise Exception("No measurements available.")

    # Convert to numpy for optimization
    positions = np.array(positions)
    distances = np.array(distances)
    weights = np.array(weights)
    x0 = np.mean(positions, axis=0)  # Initial guess

    def residuals(X):
        return weights * (np.linalg.norm(positions - X, axis=1) - distances)

    result = least_squares(residuals, x0)
    accuracy = np.sqrt(np.mean((residuals(result.x) / weights)**2))
    return result.x, used_nodes, accuracy

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

    # Fallback: find the closest room centroid if no match
    closest_room = "Unknown"
    min_dist = float("inf")
    for floor in config.get("floors", []):
        for room in floor.get("rooms", []):
            points = room.get("points", [])
            if not points:
                continue
            centroid = np.mean(points, axis=0)
            dist = np.linalg.norm(np.array([x, y]) - centroid)
            if dist < min_dist:
                min_dist = dist
                closest_room = room.get("name", "Unknown")
    return closest_room, 0.0

# Smooth room transitions by checking recent room history for consistency

def get_stable_room(device_id, current_guess):
    if device_id not in room_histories:
        room_histories[device_id] = []
    history = room_histories[device_id]
    history.append(current_guess)
    if len(history) > room_stability_threshold:
        history.pop(0)
    if len(set(history)) == 1:
        return current_guess
    return history[-2] if len(history) > 1 else current_guess

# MQTT callback to handle incoming BLE measurements

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if not is_device_allowed(data):
            return

        distance = data.get("distance")
        variance = data.get("var", 0)
        if variance > 0.5:
            return

        node_name = msg.topic.split("/")[-1].lower()
        device_id = data.get("id", "unknown")
        if device_id not in node_measurements:
            node_measurements[device_id] = {}
        node_measurements[device_id][node_name] = {
            "distance": distance,
            "timestamp": time.time(),
            "name": data.get("name", "unknown"),
            "id": device_id
        }
    except Exception as e:
        print("[BLEtracker] Error processing MQTT message:", e)

# MQTT callback triggered upon connecting to the broker

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[BLEtracker] Connected to MQTT broker at {mqtt_host}:{mqtt_port}")
        client.subscribe(MQTT_SUB_TOPIC)
    else:
        print(f"[BLEtracker] MQTT connection failed with code {rc}")

# Create and configure the MQTT client

def setup_mqtt_client():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    if mqtt_username and mqtt_password:
        client.username_pw_set(mqtt_username, mqtt_password)
    if mqtt_ssl:
        client.tls_set()
    client.connect(mqtt_host, mqtt_port, 60)
    client.loop_start()
    return client

# Apply exponential smoothing to the device's estimated position

def update_smoothed_position(device_id, current_pos):
    if device_id not in device_positions:
        device_positions[device_id] = current_pos
    else:
        device_positions[device_id] = alpha * current_pos + (1 - alpha) * device_positions[device_id]

# Publish an MQTT discovery message for Home Assistant integration

def publish_mqtt_discovery(client, device_id, device_name):
    temp_data = {"id": device_id, "name": device_name}
    if not is_device_allowed(temp_data):
        return

    topic = f"homeassistant/device_tracker/{device_id}/config"
    payload = {
        "name": device_name,
        "unique_id": f"{device_id}_tracker",
        "state_topic": f"{MQTT_PUB_TOPIC_BASE}/{device_id}",
        "value_template": "{{ value_json.stable_room }}",
        "json_attributes_topic": f"{MQTT_PUB_TOPIC_BASE}/{device_id}",
        "device": {
            "identifiers": [device_id],
            "name": device_name,
            "model": "BLEtracker",
            "manufacturer": "DIY"
        }
    }
    client.publish(topic, json.dumps(payload), retain=True)

# Main loop to process and publish BLE device location estimates

def process_tracking_cycle(client):
    now = time.time()
    for device_id, measurements in node_measurements.items():
        # Filter only fresh measurements
        fresh = {
            n: d["distance"]
            for n, d in measurements.items()
            if now - d["timestamp"] <= measurement_timeout
        }

        device_name = next((d.get("name", "unknown") for d in measurements.values() if d), "unknown")

        time_since_last_seen = max([now - d["timestamp"] for d in measurements.values()], default=0)
        if time_since_last_seen > away_timeout:
            # Mark device as away
            stable_room = "away"
            fast_room = "away"
            accuracy = None
            used_nodes = []
            smoothed_pos = [0, 0, 0]
        else:
            try:
                if device_id not in discovery_published:
                    publish_mqtt_discovery(client, device_id, device_name)
                    discovery_published.add(device_id)

                est_pos, used_nodes, accuracy = multilaterate(nodes_dict, fresh)
                update_smoothed_position(device_id, est_pos)
                smoothed_pos = device_positions[device_id]
                raw_room, confidence = find_room_for_point(smoothed_pos)
                stable_room = get_stable_room(device_id, raw_room)
                fast_room = raw_room
            except Exception as e:
                print(f"[BLEtracker] Multilateration error for {device_id}:", e)
                stable_room = "unknown"
                fast_room = "unknown"
                accuracy = None
                used_nodes = []
                smoothed_pos = [0, 0, 0]

        if now - last_publish_time.get(device_id, 0) >= publish_interval:
            # Publish tracking results
            timestamp = datetime.now().isoformat()
            payload = {
                "fast_room": fast_room,
                "stable_room": stable_room,
                "used_nodes": used_nodes,
                "device_name": device_name,
                "device_id": device_id,
                "timestamp": timestamp,
                "accuracy": accuracy,
                "position": list(map(float, smoothed_pos))
            }

            pub_topic = f"{MQTT_PUB_TOPIC_BASE}/{device_id}"
            client.publish(pub_topic, json.dumps(payload))

            print(f"[BLEtracker] {device_id} at {stable_room} ({accuracy if accuracy is not None else 'N/A'} accuracy)")

            last_publish_time[device_id] = now

# Initialize the system, load config, and start the MQTT client

def initialize_tracker():
    global mqtt_host, mqtt_port, mqtt_username, mqtt_password, mqtt_ssl
    global publish_interval, last_publish_time, measurement_timeout

    load_config()
    mqtt_config = config.get("mqtt", {})
    mqtt_host = mqtt_config.get("host", "localhost")
    mqtt_port = mqtt_config.get("port", 1883)
    mqtt_username = mqtt_config.get("username")
    mqtt_password = mqtt_config.get("password")
    mqtt_ssl = mqtt_config.get("ssl", False)

    publish_interval = 1
    last_publish_time = {}
    measurement_timeout = config.get("timeout", 30)

    return setup_mqtt_client()

# Entry point
if __name__ == "__main__":
    client = initialize_tracker()
    print("[BLEtracker] Running multi-device BLE tracker. Press Ctrl+C to exit.")
    try:
        while True:
            process_tracking_cycle(client)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[BLEtracker] Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()
