import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'dart:convert';
import 'dart:async';
import 'dart:math';

void main() {
  runApp(DroneControlApp());
}

class DroneControlApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ESP32 Drone Control',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
        fontFamily: 'Roboto',
      ),
      home: DroneControlScreen(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class DroneControlScreen extends StatefulWidget {
  @override
  _DroneControlScreenState createState() => _DroneControlScreenState();
}

class _DroneControlScreenState extends State<DroneControlScreen>
    with TickerProviderStateMixin {
  // MQTT Configuration
  late MqttServerClient client;
  String mqttServer = '192.168.1.100'; // Change to your MQTT broker IP
  int mqttPort = 1883;
  String mqttUser = 'drone_app';
  String mqttPassword = 'drone_password';
  
  // Connection status
  bool isConnected = false;
  bool isConnecting = false;
  
  // Drone status
  bool motorsArmed = false;
  bool emergencyStop = false;
  bool sensorsReady = false;
  
  // Telemetry data
  Map<String, dynamic> telemetryData = {};
  Map<String, dynamic> statusData = {};
  
  // Control variables
  double throttle = 0.0;
  double rollSetpoint = 0.0;
  double pitchSetpoint = 0.0;
  double yawSetpoint = 0.0;
  
  // UI Controllers
  late AnimationController _connectionAnimationController;
  late AnimationController _emergencyAnimationController;
  Timer? _telemetryTimer;
  
  @override
  void initState() {
    super.initState();
    
    _connectionAnimationController = AnimationController(
      duration: Duration(seconds: 1),
      vsync: this,
    )..repeat(reverse: true);
    
    _emergencyAnimationController = AnimationController(
      duration: Duration(milliseconds: 500),
      vsync: this,
    )..repeat(reverse: true);
    
    initializeMQTT();
  }
  
  @override
  void dispose() {
    _connectionAnimationController.dispose();
    _emergencyAnimationController.dispose();
    _telemetryTimer?.cancel();
    client.disconnect();
    super.dispose();
  }
  
  void initializeMQTT() {
    client = MqttServerClient(mqttServer, 'flutter_drone_app');
    client.port = mqttPort;
    client.keepAlivePeriod = 30;
    client.onConnected = onConnected;
    client.onDisconnected = onDisconnected;
    client.onSubscribed = onSubscribed;
    
    connectToMQTT();
  }
  
  Future<void> connectToMQTT() async {
    setState(() {
      isConnecting = true;
    });
    
    try {
      await client.connect(mqttUser, mqttPassword);
    } catch (e) {
      print('MQTT connection failed: $e');
      setState(() {
        isConnecting = false;
        isConnected = false;
      });
    }
  }
  
  void onConnected() {
    setState(() {
      isConnected = true;
      isConnecting = false;
    });
    
    print('MQTT Connected');
    
    // Subscribe to drone topics
    client.subscribe('drone/telemetry', MqttQos.atMostOnce);
    client.subscribe('drone/status', MqttQos.atMostOnce);
    client.subscribe('drone/emergency', MqttQos.atMostOnce);
    
    // Listen for messages
    client.updates!.listen((List<MqttReceivedMessage<MqttMessage>> c) {
      final MqttPublishMessage message = c[0].payload as MqttPublishMessage;
      final payload = MqttPublishPayload.bytesToStringAsString(message.payload.message);
      
      handleMQTTMessage(c[0].topic, payload);
    });
    
    _connectionAnimationController.stop();
  }
  
  void onDisconnected() {
    setState(() {
      isConnected = false;
      isConnecting = false;
    });
    
    print('MQTT Disconnected');
    _connectionAnimationController.repeat(reverse: true);
  }
  
  void onSubscribed(String topic) {
    print('Subscribed to: $topic');
  }
  
  void handleMQTTMessage(String topic, String payload) {
    try {
      final data = json.decode(payload);
      
      setState(() {
        if (topic == 'drone/telemetry') {
          telemetryData = data;
          motorsArmed = data['armed'] ?? false;
          emergencyStop = data['emergency'] ?? false;
        } else if (topic == 'drone/status') {
          statusData = data;
          sensorsReady = data['sensors_ready'] ?? false;
        } else if (topic == 'drone/emergency') {
          emergencyStop = true;
          showEmergencyDialog(data['reason'] ?? 'Unknown');
        }
      });
    } catch (e) {
      print('Error parsing MQTT message: $e');
    }
  }
  
  void sendControlCommand(Map<String, dynamic> command) {
    if (!isConnected) return;
    
    final message = json.encode(command);
    final builder = MqttClientPayloadBuilder();
    builder.addString(message);
    
    client.publishMessage(
      'drone/control',
      MqttQos.atLeastOnce,
      builder.payload!,
    );
  }
  
  void sendEmergencyCommand(Map<String, dynamic> command) {
    if (!isConnected) return;
    
    final message = json.encode(command);
    final builder = MqttClientPayloadBuilder();
    builder.addString(message);
    
    client.publishMessage(
      'drone/emergency',
      MqttQos.atLeastOnce,
      builder.payload!,
    );
  }
  
  void armMotors() {
    if (!sensorsReady) {
      showErrorDialog('Cannot arm motors', 'Sensors not ready');
      return;
    }
    
    sendControlCommand({'command': 'arm'});
  }
  
  void disarmMotors() {
    sendControlCommand({'command': 'disarm'});
  }
  
  void emergencyStopAction() {
    sendEmergencyCommand({'command': 'stop'});
  }
  
  void resetEmergencyStop() {
    sendEmergencyCommand({'command': 'reset'});
  }
  
  void updateThrottle(double value) {
    throttle = value;
    sendControlCommand({
      'command': 'throttle',
      'value': value.round(),
    });
  }
  
  void updateSetpoints() {
    sendControlCommand({
      'command': 'setpoint',
      'roll': rollSetpoint,
      'pitch': pitchSetpoint,
      'yaw': yawSetpoint,
    });
  }
  
  void showEmergencyDialog(String reason) {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Row(
            children: [
              Icon(Icons.warning, color: Colors.red),
              SizedBox(width: 8),
              Text('EMERGENCY STOP'),
            ],
          ),
          content: Text('Emergency stop activated.\nReason: $reason'),
          actions: [
            TextButton(
              onPressed: () {
                Navigator.of(context).pop();
                resetEmergencyStop();
              },
              child: Text('RESET'),
            ),
          ],
        );
      },
    );
  }
  
  void showErrorDialog(String title, String message) {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(title),
          content: Text(message),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text('OK'),
            ),
          ],
        );
      },
    );
  }
  
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('ESP32 Drone Control'),
        backgroundColor: isConnected ? Colors.green : Colors.red,
        actions: [
          IconButton(
            icon: Icon(isConnected ? Icons.wifi : Icons.wifi_off),
            onPressed: isConnected ? null : connectToMQTT,
          ),
        ],
      ),
      body: Column(
        children: [
          _buildConnectionStatus(),
          _buildDroneStatus(),
          Expanded(
            child: SingleChildScrollView(
              child: Column(
                children: [
                  _buildControlSection(),
                  _buildTelemetrySection(),
                  _buildMotorStatus(),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
  
  Widget _buildConnectionStatus() {
    return Container(
      width: double.infinity,
      padding: EdgeInsets.all(12),
      color: isConnected ? Colors.green.shade100 : Colors.red.shade100,
      child: Row(
        children: [
          if (isConnecting)
            AnimatedBuilder(
              animation: _connectionAnimationController,
              builder: (context, child) {
                return Transform.rotate(
                  angle: _connectionAnimationController.value * 2 * pi,
                  child: Icon(Icons.sync, color: Colors.blue),
                );
              },
            )
          else
            Icon(
              isConnected ? Icons.check_circle : Icons.error,
              color: isConnected ? Colors.green : Colors.red,
            ),
          SizedBox(width: 8),
          Text(
            isConnecting
                ? 'Connecting to drone...'
                : isConnected
                    ? 'Connected to drone'
                    : 'Disconnected from drone',
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }
  
  Widget _buildDroneStatus() {
    return Container(
      padding: EdgeInsets.all(16),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          _buildStatusIndicator(
            'Sensors',
            sensorsReady,
            Icons.sensors,
          ),
          _buildStatusIndicator(
            'Armed',
            motorsArmed,
            Icons.power,
          ),
          _buildStatusIndicator(
            'Emergency',
            emergencyStop,
            Icons.warning,
            isAlert: true,
          ),
        ],
      ),
    );
  }
  
  Widget _buildStatusIndicator(
    String label,
    bool status,
    IconData icon, {
    bool isAlert = false,
  }) {
    Color color = status
        ? (isAlert ? Colors.red : Colors.green)
        : Colors.grey;
    
    Widget indicator = Column(
      children: [
        Icon(icon, color: color, size: 32),
        SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            color: color,
            fontWeight: FontWeight.bold,
          ),
        ),
      ],
    );
    
    if (isAlert && status) {
      return AnimatedBuilder(
        animation: _emergencyAnimationController,
        builder: (context, child) {
          return Transform.scale(
            scale: 1.0 + (_emergencyAnimationController.value * 0.1),
            child: indicator,
          );
        },
      );
    }
    
    return indicator;
  }
  
  Widget _buildControlSection() {
    return Card(
      margin: EdgeInsets.all(16),
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Drone Control',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            
            // Arm/Disarm buttons
            Row(
              children: [
                Expanded(
                  child: ElevatedButton(
                    onPressed: !motorsArmed ? armMotors : null,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      foregroundColor: Colors.white,
                    ),
                    child: Text('ARM'),
                  ),
                ),
                SizedBox(width: 16),
                Expanded(
                  child: ElevatedButton(
                    onPressed: motorsArmed ? disarmMotors : null,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.orange,
                      foregroundColor: Colors.white,
                    ),
                    child: Text('DISARM'),
                  ),
                ),
              ],
            ),
            
            SizedBox(height: 16),
            
            // Emergency stop button
            SizedBox(
              width: double.infinity,
              child: ElevatedButton(
                onPressed: emergencyStopAction,
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.red,
                  foregroundColor: Colors.white,
                  padding: EdgeInsets.symmetric(vertical: 16),
                ),
                child: Text(
                  'EMERGENCY STOP',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ),
            
            SizedBox(height: 16),
            
            // Throttle control
            Text('Throttle: ${throttle.round()}%'),
            Slider(
              value: throttle,
              min: 0,
              max: 100,
              divisions: 100,
              onChanged: motorsArmed && !emergencyStop ? updateThrottle : null,
            ),
            
            SizedBox(height: 16),
            
            // Attitude setpoints
            Text('Roll Setpoint: ${rollSetpoint.toStringAsFixed(1)}°'),
            Slider(
              value: rollSetpoint,
              min: -30,
              max: 30,
              divisions: 60,
              onChanged: (value) {
                setState(() {
                  rollSetpoint = value;
                });
                updateSetpoints();
              },
            ),
            
            Text('Pitch Setpoint: ${pitchSetpoint.toStringAsFixed(1)}°'),
            Slider(
              value: pitchSetpoint,
              min: -30,
              max: 30,
              divisions: 60,
              onChanged: (value) {
                setState(() {
                  pitchSetpoint = value;
                });
                updateSetpoints();
              },
            ),
            
            Text('Yaw Rate: ${yawSetpoint.toStringAsFixed(1)}°/s'),
            Slider(
              value: yawSetpoint,
              min: -180,
              max: 180,
              divisions: 72,
              onChanged: (value) {
                setState(() {
                  yawSetpoint = value;
                });
                updateSetpoints();
              },
            ),
          ],
        ),
      ),
    );
  }
  
  Widget _buildTelemetrySection() {
    return Card(
      margin: EdgeInsets.all(16),
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Telemetry Data',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            
            if (telemetryData.isNotEmpty) ...[
              _buildTelemetryRow('Temperature', 
                '${telemetryData['imu']?['temperature']?.toStringAsFixed(1) ?? 'N/A'}°C'),
              
              _buildTelemetryRow('Battery', 
                '${telemetryData['battery']?['voltage']?.toStringAsFixed(2) ?? 'N/A'}V '
                '(${telemetryData['battery']?['percentage']?.round() ?? 'N/A'}%)'),
              
              SizedBox(height: 8),
              Text('IMU Data:', style: TextStyle(fontWeight: FontWeight.bold)),
              
              _buildTelemetryRow('Accel X', 
                '${telemetryData['imu']?['accel']?['x']?.toStringAsFixed(2) ?? 'N/A'} g'),
              _buildTelemetryRow('Accel Y', 
                '${telemetryData['imu']?['accel']?['y']?.toStringAsFixed(2) ?? 'N/A'} g'),
              _buildTelemetryRow('Accel Z', 
                '${telemetryData['imu']?['accel']?['z']?.toStringAsFixed(2) ?? 'N/A'} g'),
              
              _buildTelemetryRow('Gyro X', 
                '${telemetryData['imu']?['gyro']?['x']?.toStringAsFixed(1) ?? 'N/A'}°/s'),
              _buildTelemetryRow('Gyro Y', 
                '${telemetryData['imu']?['gyro']?['y']?.toStringAsFixed(1) ?? 'N/A'}°/s'),
              _buildTelemetryRow('Gyro Z', 
                '${telemetryData['imu']?['gyro']?['z']?.toStringAsFixed(1) ?? 'N/A'}°/s'),
              
            ] else
              Text('No telemetry data available'),
          ],
        ),
      ),
    );
  }
  
  Widget _buildTelemetryRow(String label, String value) {
    return Padding(
      padding: EdgeInsets.symmetric(vertical: 2),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(value, style: TextStyle(fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }
  
  Widget _buildMotorStatus() {
    return Card(
      margin: EdgeInsets.all(16),
      child: Padding(
        padding: EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Motor Status',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 16),
            
            if (telemetryData['motors'] != null) ...[
              GridView.count(
                shrinkWrap: true,
                physics: NeverScrollableScrollPhysics(),
                crossAxisCount: 2,
                childAspectRatio: 2,
                children: [
                  _buildMotorIndicator('FL', telemetryData['motors'][0]),
                  _buildMotorIndicator('FR', telemetryData['motors'][1]),
                  _buildMotorIndicator('BL', telemetryData['motors'][2]),
                  _buildMotorIndicator('BR', telemetryData['motors'][3]),
                ],
              ),
            ] else
              Text('No motor data available'),
          ],
        ),
      ),
    );
  }
  
  Widget _buildMotorIndicator(String label, dynamic speed) {
    int motorSpeed = speed?.toInt() ?? 1000;
    double percentage = ((motorSpeed - 1000) / 10.0).clamp(0.0, 100.0);
    
    return Container(
      margin: EdgeInsets.all(4),
      padding: EdgeInsets.all(8),
      decoration: BoxDecoration(
        border: Border.all(color: Colors.grey),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(label, style: TextStyle(fontWeight: FontWeight.bold)),
          Text('$motorSpeed μs'),
          LinearProgressIndicator(
            value: percentage / 100,
            backgroundColor: Colors.grey.shade300,
            valueColor: AlwaysStoppedAnimation<Color>(
              percentage > 50 ? Colors.red : Colors.green,
            ),
          ),
        ],
      ),
    );
  }
}