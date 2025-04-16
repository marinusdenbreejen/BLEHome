#!/usr/bin/env python

import time
import json
import yaml
import numpy as np
from datetime import datetime
from scipy.optimize import least_squares
import paho.mqtt.client as mqtt

config = None
allowed_devices = []
nodes_dict = {}
node_measurements = {}
smoothed_position = None
alpha = 0.3
room_stability_threshold = 3
room_history = []

MQTT_SUB_TOPIC = "espresense/devices/#"
MQTT_PUB_TOPIC = "espresense/BLEtracker"

# Load configuration from YAML file and parse node/device info
def load_config():
    global config, nodes_dict, allowed_devices
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)
    nodes_dict = {n["name"].lower(): n for n in config.get("nodes", [])}
    allowed_devices = config.get("devices", [])
    print("[BLEtracker] Configuration loaded.")

# Check if a device is allowed based on the config whitelist
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

# Estimate device location using weighted multilateration
def multilaterate(nodes, measurements):
    positions, distances, used_nodes, weights = [], [], [], []
    for name, node in nodes.items():
        if name in measurements:
            point = np.array(node["point"], dtype=float)
            distance = measurements[name]
            floor_weight = 1.0
            if "floors" in node and room_history and room_history[-1] in node["floors"]:
                floor_weight = 1.5  # Boost for nodes on same floor as last known room
            weight = floor_weight / max(distance, 0.1)  # Prioritize close nodes
            positions.append(point)
            distances.append(distance)
            weights.append(weight)
            used_nodes.append(name)

    if not positions:
        raise Exception("No measurements available.")
    positions = np.array(positions)
    distances = np.array(distances)
    weights = np.array(weights)
    x0 = np.mean(positions, axis=0)

    def residuals(X):
        return weights * (np.linalg.norm(positions - X, axis=1) - distances)

    result = least_squares(residuals, x0)
    accuracy = np.sqrt(np.mean((residuals(result.x) / weights)**2))
    return result.x, used_nodes, accuracy

# Determine if a point lies within a polygon (2D point-in-polygon test)
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

# Determine which room a point is in (or closest to if outside all rooms)
def find_room_for_point(P, config):
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
        confidence = round(len(candidates) / sum(len(f["rooms"]) for f in config.get("floors", [])) * 100, 1)
        for room in reversed(room_history):
            if room in candidates:
                return room, confidence
        return candidates[0], confidence

    # Fallback: use closest room by centroid distance
    closest_room = None
    min_dist = float('inf')
    for floor in config.get("floors", []):
        for room in floor.get("rooms", []):
            points = np.array(room.get("points", []))
            if len(points) == 0:
                continue
            centroid = np.mean(points, axis=0)
            dist = np.linalg.norm(np.array([x, y]) - centroid)
            if dist < min_dist:
                min_dist = dist
                closest_room = room.get("name", "Unknown")
    return closest_room, 0.0

# Smooth room transitions by requiring multiple confirmations before switching
def get_stable_room(current_guess):
    global room_history
    room_history.append(current_guess)
    if len(room_history) > room_stability_threshold:
        room_history.pop(0)
    if len(set(room_history)) == 1:
        return current_guess
    return room_history[-2] if len(room_history) > 1 else current_guess

# Output a compact summary line to the console with device info and node data
def print_compact_line(room, quick_pos, smooth_pos, used_nodes, accuracy, confidence, device_name):
    fast_room, _ = find_room_for_point(quick_pos, config)
    node_parts = []
    for node in used_nodes:
        d = node_measurements.get(node, {})
        dist = d.get("distance")
        var = d.get("var", 0.0)
        if dist is not None:
            node_parts.append(f"{node}({dist:.2f}, var={var:.2f})")
    nodes_str = ", ".join(node_parts)
    print(f"[BLEtracker] stable_room='{room.split(' → ')[0]}', fast_room='{room.split(' → ')[1]}' "
      f"device='{device_name}' confidence={confidence:.1f}% via: {nodes_str}")

# MQTT callback for handling incoming BLE distance messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if not is_device_allowed(data):
            return

        distance = data.get("distance")
        variance = data.get("var", 0)
        if variance > 0.5:
            return  # discard noisy measurement

        node_name = msg.topic.split("/")[-1].lower()
        if distance is not None:
            node_measurements[node_name] = {
                "distance": distance,
                "timestamp": time.time(),
                "name": data.get("name", "unknown")
            }
    except Exception as e:
        print("[BLEtracker] Error processing MQTT message:", e)

# MQTT callback for successful connection, subscribes to topic
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[BLEtracker] Connected to MQTT broker at {mqtt_host}:{mqtt_port}")
        client.subscribe(MQTT_SUB_TOPIC)
    else:
        print(f"[BLEtracker] MQTT connection failed with code {rc}")


# Set up and return an MQTT client with appropriate credentials and handlers
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

# Print the tracking result to console with stable/quick room labels
def log_tracking_output(stable_room, est_pos, smoothed_position, used_nodes, accuracy, confidence, device_name):
    quick_room, _ = find_room_for_point(smoothed_position, config)
    print_compact_line(f"{stable_room} → {quick_room}", est_pos, smoothed_position, used_nodes, accuracy, confidence, device_name)

# Update global smoothed_position using exponential smoothing
def update_smoothed_position(current_pos):
    global smoothed_position
    if smoothed_position is None:
        smoothed_position = current_pos
    else:
        smoothed_position = alpha * current_pos + (1 - alpha) * smoothed_position

# Construct the JSON payload to publish to MQTT, including node distances
def build_mqtt_payload(stable_room, est_pos, smoothed_pos, accuracy, timestamp, used_nodes, device_name):
    fast_room, _ = find_room_for_point(est_pos, config)
    used_nodes_data = {
        n: {
            "distance": round(node_measurements[n].get("distance", 0), 2),
            "var": round(node_measurements[n].get("var", 0), 2)
        } for n in used_nodes
    }
    return {
        "fast_room": fast_room,
        "stable_room": stable_room,"used_nodes": used_nodes_data,
        "device_name": device_name
        
    }

# Main loop logic: filter data, estimate position, determine room, publish & log
def process_tracking_cycle(client):
    global last_publish_time
    now = time.time()
    fresh = {
        n: d["distance"]
        for n, d in node_measurements.items()
        if now - d["timestamp"] <= measurement_timeout
    }

    # Get latest device name (from any measurement)
    device_name = next((d.get("name", "unknown") for d in node_measurements.values() if d), "unknown")

    if now - last_publish_time >= publish_interval:
        try:
            est_pos, used_nodes, accuracy = multilaterate(nodes_dict, fresh)
            update_smoothed_position(est_pos)
            raw_room, confidence = find_room_for_point(smoothed_position, config)
            stable_room = get_stable_room(raw_room)
            timestamp = datetime.now().isoformat()

            update_smoothed_position(est_pos)

            payload = build_mqtt_payload(stable_room, est_pos, smoothed_position, accuracy, timestamp, used_nodes, device_name)
            client.publish(MQTT_PUB_TOPIC, json.dumps(payload))

            log_tracking_output(stable_room, est_pos, smoothed_position, used_nodes, accuracy, confidence, device_name)

        except Exception as e:
            print("[BLEtracker] Multilateration error:", e)
        last_publish_time = now

# Load config and set up MQTT client with settings
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
    last_publish_time = time.time()
    measurement_timeout = config.get("timeout", 30)

    return setup_mqtt_client()

if __name__ == "__main__":
    client = initialize_tracker()
    print("[BLEtracker] Running BLE tracker. Press Ctrl+C to exit.")

    try:
        while True:
            process_tracking_cycle(client)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[BLEtracker] Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()
