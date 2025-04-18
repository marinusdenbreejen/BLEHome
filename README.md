# 🛰️ BLEtracker: Indoor Positioning via MQTT & Multilateration

**BLEtracker** is a Python-based indoor positioning system designed to track Bluetooth Low Energy (BLE) devices using signal strength measurements from multiple nodes. It integrates seamlessly with [Espresense](https://github.com/espresense/espresense) and can be used with [Home Assistant](https://www.home-assistant.io/) to automate smart environments based on precise room-level location tracking.

---

## 🎯 Project Goals

- Track BLE tags (phones, wearables, etc.) using multiple fixed BLE receiver nodes (e.g., ESP32 with Espresense).
- Use **multilateration** to estimate 2D/3D positions from RSSI-derived distances.
- Transmit and receive all data via **MQTT**, compatible with smart home systems.
- Identify which room a tracked device is located in based on floor/room polygon definitions.
- Provide **stable room detection** using confidence smoothing and historical filtering.

---

## 🧩 Features

✅ MQTT integration (bidirectional)  
✅ Works with Espresense (uses its topic format)  
✅ Multilateration-based location estimation  
✅ Floor and room layout support  
✅ Room smoothing (stabilizes rapid switches)  
✅ Confidence scoring based on geometry  
✅ Compatible with **Home Assistant** via MQTT sensors  
✅ Open and extensible YAML configuration

---

## 📦 File Structure

```
.
├── bletracker.py            # Main tracking script (run this)
├── config.yaml              # Node/device/room layout configuration
├── requirements.txt         # Python dependencies
└── README.md                # This file
```

---

## ⚙️ Configuration (config.yaml)

Example structure:

```yaml
mqtt:
  host: "mqtt.local"
  port: 1883
  username: "user"
  password: "pass"
  ssl: false

devices:
  - id: "aa:bb:cc:dd:ee:ff"
    name: "MyPhone"

nodes:
  - name: "Node1"
    point: [0.0, 0.0, 2.0]
    floors: ["Ground"]

floors:
  - name: "Ground"
    bounds: [[-10, -10, 0], [10, 10, 3]]
    rooms:
      - name: "Living Room"
        points: [[0,0], [5,0], [5,5], [0,5]]
```

---

## 🚀 Running the Tracker

1. Install dependencies:

```bash
pip install -r requirements.txt
```

2. Configure `config.yaml` to match your MQTT and node setup.

3. Run the tracker:

```bash
python bletracker.py
```

4. Position and room data will be published to MQTT under:

```
espresense/BLEtracker/<device_id>
```

Payload includes:

```json
{
  "fast_room": "Living Room",
  "stable_room": "Living Room",
  "used_nodes": ["node1", "node2"],
  "device_name": "MyPhone",
  "device_id": "aa:bb:cc:dd:ee:ff",
  "timestamp": "2025-04-16T12:34:56.789Z",
  "accuracy": 0.85,
  "position": [1.23, 3.45, 2.0]
}
```

---

## 🧠 Integration with Home Assistant

Use the MQTT data to trigger room-based automations:

```yaml
sensor:
  - platform: mqtt
    name: "MyPhone Room"
    state_topic: "espresense/BLEtracker/aa:bb:cc:dd:ee:ff"
    value_template: "{{ value_json.stable_room }}"
```

---

## 🔧 Advanced Features

- 🔄 **Exponential Smoothing**: Stabilizes coordinate updates.
- 🧠 **Room Guessing + Confidence**: Chooses best-fit room even when coordinates are near boundaries.
- 🚫 **Device Whitelisting**: Only process selected device IDs.
- ⏱️ **Time-based Filtering**: Ignores stale measurements.
- ↔️ **Fallback Locators**: Nearest-node fallback when multilateration fails.

---

## 📌 Requirements

- Python 3.8+
- MQTT broker (e.g., [Mosquitto](https://mosquitto.org/))
- BLE scanning nodes running Espresense or similar firmware

Install dependencies:

```bash
pip install numpy scipy paho-mqtt pyyaml
```

---

## 🤝 Contributing

Contributions, suggestions, and PRs are welcome!  
Feel free to fork the project or open an issue to get involved.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Made with ❤️ for smarter spaces.

