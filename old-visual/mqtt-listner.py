import time
import json
import yaml
import paho.mqtt.client as mqtt
import importlib.util
import matplotlib.pyplot as plt
import trilateration

# Load the 3dhome.py module as "d3home" using importlib
spec = importlib.util.spec_from_file_location("d3home", "./3dhome.py")
d3home = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d3home)
print("🔧 [mqtt_listener] Loaded d3home module.")

# Load configuration for MQTT settings
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

mqtt_config = config.get("mqtt", {})
host = mqtt_config.get("host", "localhost")
port = mqtt_config.get("port", 1883)
username = mqtt_config.get("username")
password = mqtt_config.get("password")
use_ssl = mqtt_config.get("ssl", False)

# MQTT topic to listen to (wildcard on the node name)
topic = "espresense/devices/phone:iphonemarinus/#"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ [mqtt_listener] Connected to MQTT broker at {host}:{port}")
        client.subscribe(topic)
        print(f"📡 [mqtt_listener] Subscribed to topic: {topic}")
    else:
        print(f"❌ [mqtt_listener] Failed to connect. Return code: {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        distance = data.get("distance")
        # The node name is the last part of the topic (converted to lowercase)
        node_name = msg.topic.split("/")[-1].lower()
        if distance is not None:
            print(f"🔄 [mqtt_listener] Received update for node '{node_name}': distance {distance} m")
            d3home.queue_update(node_name, distance)
        else:
            print("⚠️ [mqtt_listener] No 'distance' found in message:", payload)
    except Exception as e:
        print("❌ [mqtt_listener] Error processing message:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

if username and password:
    client.username_pw_set(username, password)
if use_ssl:
    client.tls_set()

client.connect(host, port, 60)
client.loop_start()
print("🔄 [mqtt_listener] MQTT client loop started.")

d3home.load_config()
d3home.init_plot()

# Main loop: process MQTT updates and perform multilateration periodically.
multilat_interval = 5  # seconds
last_multilat_time = time.time()

print("🚀 [mqtt_listener] MQTT listener and 3D model running. Close the plot window to exit.")

while plt.fignum_exists(d3home.fig.number):
    d3home.process_updates()
    current_time = time.time()
    if current_time - last_multilat_time >= multilat_interval:
        measurements = d3home.node_measurements
        if len(measurements) > 0:
            try:
                # Use all nodes that have measurements
                estimated_pos = trilateration.multilaterate(d3home.nodes_dict, measurements)
                room = trilateration.find_room_for_point(estimated_pos, d3home.config)
                trilateration.plot_object(estimated_pos, "MyObject", room, d3home.ax)
            except Exception as e:
                print("❌ [mqtt_listener] Multilateration error:", e)
        last_multilat_time = current_time
    plt.pause(0.1)

client.loop_stop()
print("🛑 [mqtt_listener] MQTT client loop stopped. Exiting.")
