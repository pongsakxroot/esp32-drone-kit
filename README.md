# ESP32 IoT Drone Platform 🚁

A comprehensive IoT drone platform built around the ESP32 microcontroller, featuring real-time telemetry, MQTT communication, mobile app control, and web dashboard monitoring.

## Features

### 🔧 Hardware
- **ESP32-WROOM-32** main controller with WiFi/Bluetooth
- **6-axis IMU** (MPU6050) for attitude estimation
- **Barometric pressure sensor** (BMP280) for altitude sensing
- **4x ESC motor drivers** for brushless motor control
- **Power management** with battery monitoring
- **RGB status LEDs** and audio buzzer feedback
- **Emergency stop button** for safety
- **Modular PCB design** with proper power planes

### 💻 Firmware
- **MQTT telemetry** at 10Hz for real-time data streaming
- **PID flight controller** with roll, pitch, and yaw stabilization
- **Safety systems** including emergency stop and low battery protection
- **WiFi connectivity** with automatic reconnection
- **Motor arming/disarming** with safety checks
- **Sensor calibration** and fault detection
- **Over-the-air configuration** via MQTT commands

### 📱 Mobile App (Flutter)
- **Real-time telemetry display** with IMU and battery data
- **Touch controls** for throttle and attitude setpoints
- **Motor status monitoring** with visual indicators
- **Emergency stop button** for immediate safety
- **Connection status** with automatic reconnection
- **Responsive UI** optimized for tablets and phones

### 🌐 Web Dashboard (React)
- **Live charts** for sensor data visualization
- **Motor control interface** with real-time feedback
- **System status monitoring** including WiFi and MQTT health
- **Battery level visualization** with alerts
- **Emergency controls** accessible from any device
- **Data logging** and export capabilities

## Project Structure

```
esp32-drone-kit/
├── hardware/                    # Hardware design files
│   ├── esp32_drone_schematic.json   # EasyEDA schematic
│   └── esp32_drone_pcb.json         # EasyEDA PCB layout
├── firmware/                    # ESP32 firmware
│   ├── esp32_drone_mqtt.ino        # Main MQTT-enabled firmware
│   ├── esp32_drone_main.ino        # Basic firmware (legacy)
│   └── esp32_drone_pid.ino         # PID controller example
├── mobile_app/                  # Flutter mobile application
│   └── lib/main.dart               # Main mobile app code
├── dashboard/                   # React web dashboard
│   └── src/App.js                  # Main dashboard component
├── docs/                       # Documentation
│   └── BOM.md                     # Bill of Materials
└── README.md                   # This file
```

## Hardware Setup

### Required Components
See [docs/BOM.md](docs/BOM.md) for complete bill of materials.

**Main Components:**
- ESP32-WROOM-32 development board
- MPU6050 6-axis IMU sensor
- BMP280 barometric pressure sensor
- 4x ESCs (Electronic Speed Controllers) 30A
- 4x Brushless motors
- LiPo battery (3S/4S, 2200-5000mAh)
- Power distribution board
- Frame and propellers

### Assembly Instructions

1. **PCB Assembly:**
   - Import `hardware/esp32_drone_schematic.json` into EasyEDA
   - Review the schematic and component placement
   - Order PCB using `hardware/esp32_drone_pcb.json`
   - Solder components according to the schematic

2. **Frame Assembly:**
   - Mount the PCB to the drone frame using M3 screws
   - Install motors on the frame arms
   - Connect ESCs to motors and power distribution

3. **Wiring:**
   - Connect ESC signal wires to ESP32 GPIO pins (2, 4, 12, 13)
   - Wire power distribution from battery to ESCs and main board
   - Connect sensors to I2C bus (SDA=21, SCL=22)
   - Install emergency stop button and status LEDs

4. **Calibration:**
   - Ensure motor rotation directions are correct
   - Calibrate ESCs according to manufacturer instructions
   - Mount IMU with X-axis pointing forward

## Firmware Setup

### Prerequisites
- **Arduino IDE** 1.8.x or **PlatformIO**
- **ESP32 Arduino Core** v2.0.x or later

### Required Libraries
Install these libraries through Arduino Library Manager:
```
- WiFi (built-in)
- PubSubClient v2.8.0+
- MPU6050 v0.6.0+
- ArduinoJson v6.19.0+
- ESP32Servo v0.11.0+
```

### Configuration

1. **WiFi Settings:**
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. **MQTT Broker:**
   ```cpp
   const char* mqtt_server = "192.168.1.100";  // Your MQTT broker IP
   const int mqtt_port = 1883;
   const char* mqtt_user = "drone_esp32";
   const char* mqtt_password = "drone_password";
   ```

3. **Upload Firmware:**
   - Open `firmware/esp32_drone_mqtt.ino` in Arduino IDE
   - Select ESP32 board and correct COM port
   - Compile and upload to ESP32

### MQTT Topics
The firmware publishes and subscribes to these topics:
- `drone/telemetry` - Real-time sensor data (10Hz)
- `drone/status` - System status updates (1Hz)
- `drone/control` - Command input (subscribe)
- `drone/emergency` - Emergency commands (bi-directional)

## Mobile App Setup

### Prerequisites
- **Flutter SDK** 3.0.0 or later
- **Dart SDK** 2.17.0 or later
- Android Studio or VS Code with Flutter extensions

### Installation

1. **Install Dependencies:**
   ```bash
   cd mobile_app
   flutter pub get
   ```

2. **Configure MQTT:**
   Edit `lib/main.dart` and update MQTT settings:
   ```dart
   String mqttServer = '192.168.1.100'; // Your MQTT broker IP
   int mqttPort = 1883;
   ```

3. **Build and Install:**
   ```bash
   # For Android
   flutter build apk
   flutter install
   
   # For iOS (requires Mac and Xcode)
   flutter build ios
   ```

### Required Permissions
Add to `android/app/src/main/AndroidManifest.xml`:
```xml
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.ACCESS_NETWORK_STATE" />
```

## Web Dashboard Setup

### Prerequisites
- **Node.js** 16.0.0 or later
- **npm** or **yarn** package manager

### Installation

1. **Install Dependencies:**
   ```bash
   cd dashboard
   npm install react react-dom
   npm install mqtt chart.js react-chartjs-2
   ```

2. **Configure MQTT:**
   Edit `src/App.js` and update the MQTT broker URL:
   ```javascript
   const [mqttConfig] = useState({
     broker: 'ws://192.168.1.100:8080', // WebSocket MQTT broker
     options: {
       clientId: 'react_drone_dashboard',
       username: 'drone_dashboard',
       password: 'drone_password',
     }
   });
   ```

3. **Start Development Server:**
   ```bash
   npm start
   ```

4. **Build for Production:**
   ```bash
   npm run build
   ```

### MQTT Broker WebSocket Support
Ensure your MQTT broker supports WebSocket connections on port 8080.

For **Mosquitto**, add to config:
```
listener 8080
protocol websockets
```

## MQTT Broker Setup

### Option 1: Mosquitto (Recommended)
```bash
# Install Mosquitto
sudo apt update
sudo apt install mosquitto mosquitto-clients

# Configure authentication
sudo mosquitto_passwd -c /etc/mosquitto/passwd drone_esp32
sudo mosquitto_passwd /etc/mosquitto/passwd drone_app
sudo mosquitto_passwd /etc/mosquitto/passwd drone_dashboard

# Add to /etc/mosquitto/mosquitto.conf:
password_file /etc/mosquitto/passwd
listener 1883
listener 8080
protocol websockets

# Start service
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

### Option 2: Cloud MQTT Services
- **AWS IoT Core**
- **Google Cloud IoT**
- **HiveMQ Cloud**
- **CloudMQTT**

## Usage Instructions

### Safety First! ⚠️
1. **Always remove propellers** during initial testing
2. **Test in open area** away from people and obstacles
3. **Keep emergency stop accessible** at all times
4. **Monitor battery voltage** - land when below 20%
5. **Check all connections** before each flight

### Initial Testing

1. **Power Up Sequence:**
   - Connect battery
   - Wait for green LED (system ready)
   - Check WiFi connection
   - Verify MQTT connectivity in dashboard

2. **Sensor Calibration:**
   - Place drone on level surface
   - Wait for IMU calibration (automatic)
   - Verify sensor readings in dashboard

3. **Motor Test (NO PROPELLERS):**
   - Arm motors via mobile app or dashboard
   - Test individual motor response
   - Verify emergency stop functionality
   - Disarm motors

### Flight Operations

1. **Pre-flight Checklist:**
   - ✅ Battery charged and connected
   - ✅ Propellers securely attached
   - ✅ All connections tight
   - ✅ Clear flight area
   - ✅ Emergency stop accessible
   - ✅ MQTT connection active

2. **Flight Procedure:**
   - Arm motors from mobile app
   - Gradually increase throttle
   - Monitor telemetry data
   - Use attitude controls for stabilization
   - Land safely and disarm motors

### Troubleshooting

**No WiFi Connection:**
- Check SSID and password in firmware
- Verify WiFi network is 2.4GHz
- Check antenna connection

**MQTT Not Connecting:**
- Verify broker IP address and port
- Check username/password
- Ensure broker is running and accessible

**Motors Not Responding:**
- Check ESC calibration
- Verify power connections
- Ensure motors are armed
- Check emergency stop status

**Sensor Data Invalid:**
- Check I2C connections
- Verify sensor power supply
- Recalibrate sensors

## Development and Customization

### Adding New Sensors
1. Update hardware schematic
2. Modify firmware to read sensor data
3. Add telemetry fields to MQTT messages
4. Update mobile app and dashboard displays

### Custom Control Algorithms
- Modify PID controller parameters in firmware
- Implement custom flight modes
- Add autonomous navigation features

### Data Logging
- Extend dashboard to log telemetry data
- Implement data export functionality
- Add historical data visualization

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For support and questions:
- Create an issue on GitHub
- Join our Discord community
- Check the documentation wiki

## Acknowledgments

- ESP32 community for excellent documentation
- Flutter team for cross-platform mobile framework
- React ecosystem for web dashboard tools
- Open source MQTT broker projects

---

**⚠️ IMPORTANT SAFETY NOTICE**

This is experimental software for educational purposes. Always follow local regulations for drone operation. The authors are not responsible for any damage or injury caused by the use of this project. Fly safely and responsibly!