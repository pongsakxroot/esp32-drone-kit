/*
 * ESP32 IoT Drone Platform - MQTT Telemetry & Control
 * 
 * Features:
 * - WiFi connectivity and MQTT communication
 * - Real-time telemetry (IMU, barometer, battery)
 * - Motor control via MQTT commands
 * - Safety shutdown and emergency stop
 * - PID stabilization system
 * - Status monitoring and alerts
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// WiFi Configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Configuration
const char* mqtt_server = "YOUR_MQTT_BROKER";
const int mqtt_port = 1883;
const char* mqtt_user = "drone_esp32";
const char* mqtt_password = "drone_password";

// MQTT Topics
const char* topic_telemetry = "drone/telemetry";
const char* topic_control = "drone/control";
const char* topic_status = "drone/status";
const char* topic_emergency = "drone/emergency";

// Hardware Pin Definitions
#define MOTOR_FL_PIN    2   // Front Left Motor (ESC PWM)
#define MOTOR_FR_PIN    4   // Front Right Motor (ESC PWM)
#define MOTOR_BL_PIN    12  // Back Left Motor (ESC PWM)
#define MOTOR_BR_PIN    13  // Back Right Motor (ESC PWM)
#define BUZZER_PIN      14
#define LED_RED_PIN     16
#define LED_GREEN_PIN   17
#define LED_BLUE_PIN    18
#define EMERGENCY_PIN   19
#define BATTERY_PIN     32  // ADC for battery voltage monitoring

// Motor Control
Servo motorFL, motorFR, motorBL, motorBR;
int motorSpeeds[4] = {1000, 1000, 1000, 1000}; // ESC signals (1000-2000 us)
bool motorsArmed = false;
bool emergencyStop = false;

// Sensors
MPU6050 mpu;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temperature;
float batteryVoltage;
float batteryPercentage;

// PID Control Variables
struct PIDController {
  float kp, ki, kd;
  float setpoint;
  float input;
  float output;
  float integral;
  float previous_error;
  unsigned long last_time;
};

PIDController pidRoll = {1.0, 0.1, 0.05, 0, 0, 0, 0, 0, 0};
PIDController pidPitch = {1.0, 0.1, 0.05, 0, 0, 0, 0, 0, 0};
PIDController pidYaw = {2.0, 0.1, 0.1, 0, 0, 0, 0, 0, 0};

// System Status
bool wifiConnected = false;
bool mqttConnected = false;
bool sensorsReady = false;
unsigned long lastTelemetryTime = 0;
unsigned long lastStatusTime = 0;
const unsigned long telemetryInterval = 100; // 10Hz telemetry
const unsigned long statusInterval = 1000;   // 1Hz status

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 IoT Drone Platform Starting...");
  
  // Initialize hardware
  initializePins();
  initializeMotors();
  initializeSensors();
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  // Connect to MQTT
  connectMQTT();
  
  Serial.println("System Ready - Drone Platform Initialized");
  setStatusLED(0, 255, 0); // Green LED for ready status
}

void loop() {
  // Handle MQTT connection
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
  
  // Check emergency stop
  checkEmergencyStop();
  
  // Read sensors
  readSensors();
  
  // Update PID controllers if armed
  if (motorsArmed && !emergencyStop) {
    updatePIDControllers();
    updateMotors();
  }
  
  // Send telemetry data
  if (millis() - lastTelemetryTime > telemetryInterval) {
    sendTelemetry();
    lastTelemetryTime = millis();
  }
  
  // Send status data
  if (millis() - lastStatusTime > statusInterval) {
    sendStatus();
    lastStatusTime = millis();
  }
  
  // Update status LED
  updateStatusLED();
  
  delay(10);
}

void initializePins() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);
  
  // Initial LED test
  setStatusLED(255, 0, 0); // Red
  delay(200);
  setStatusLED(0, 255, 0); // Green
  delay(200);
  setStatusLED(0, 0, 255); // Blue
  delay(200);
  setStatusLED(0, 0, 0);   // Off
}

void initializeMotors() {
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorBL.attach(MOTOR_BL_PIN);
  motorBR.attach(MOTOR_BR_PIN);
  
  // Initialize ESCs (important for proper ESC calibration)
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  
  delay(2000); // Wait for ESC initialization
  Serial.println("Motors initialized");
}

void initializeSensors() {
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
    sensorsReady = true;
    
    // Calibrate gyroscope
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);
    Serial.println("Sensor calibration complete");
  } else {
    Serial.println("MPU6050 connection failed");
    sensorsReady = false;
  }
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi connection failed!");
  }
}

void connectMQTT() {
  while (!client.connected() && wifiConnected) {
    Serial.print("Attempting MQTT connection...");
    
    String clientId = "ESP32Drone-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      mqttConnected = true;
      
      // Subscribe to control topics
      client.subscribe(topic_control);
      client.subscribe(topic_emergency);
      
      // Send initial status
      sendStatus();
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      mqttConnected = false;
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("MQTT Message: " + String(topic) + " - " + message);
  
  // Parse JSON message
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, message);
  
  if (String(topic) == topic_control) {
    handleControlCommand(doc);
  } else if (String(topic) == topic_emergency) {
    handleEmergencyCommand(doc);
  }
}

void handleControlCommand(DynamicJsonDocument& doc) {
  String command = doc["command"];
  
  if (command == "arm") {
    if (!emergencyStop && sensorsReady) {
      motorsArmed = true;
      Serial.println("Motors ARMED");
      soundBuzzer(2, 100); // 2 short beeps
    }
  } else if (command == "disarm") {
    motorsArmed = false;
    // Set all motors to minimum
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = 1000;
    }
    updateMotors();
    Serial.println("Motors DISARMED");
    soundBuzzer(1, 500); // 1 long beep
    
  } else if (command == "throttle") {
    if (motorsArmed && !emergencyStop) {
      int throttle = doc["value"];
      throttle = constrain(throttle, 0, 100);
      
      // Convert percentage to ESC signal (1000-2000 us)
      int baseSpeed = map(throttle, 0, 100, 1000, 2000);
      for (int i = 0; i < 4; i++) {
        motorSpeeds[i] = baseSpeed;
      }
    }
  } else if (command == "setpoint") {
    pidRoll.setpoint = doc["roll"];
    pidPitch.setpoint = doc["pitch"];
    pidYaw.setpoint = doc["yaw"];
  }
}

void handleEmergencyCommand(DynamicJsonDocument& doc) {
  String command = doc["command"];
  
  if (command == "stop") {
    emergencyStop = true;
    motorsArmed = false;
    
    // Immediately stop all motors
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = 1000;
    }
    updateMotors();
    
    Serial.println("EMERGENCY STOP ACTIVATED");
    soundBuzzer(5, 100); // 5 rapid beeps
    setStatusLED(255, 0, 0); // Red LED
  } else if (command == "reset") {
    emergencyStop = false;
    Serial.println("Emergency stop reset");
    setStatusLED(0, 255, 0); // Green LED
  }
}

void checkEmergencyStop() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 50) { // Check every 50ms
    if (digitalRead(EMERGENCY_PIN) == LOW) {
      if (!emergencyStop) {
        emergencyStop = true;
        motorsArmed = false;
        
        // Stop all motors immediately
        for (int i = 0; i < 4; i++) {
          motorSpeeds[i] = 1000;
        }
        updateMotors();
        
        Serial.println("HARDWARE EMERGENCY STOP");
        
        // Send emergency stop via MQTT
        DynamicJsonDocument doc(256);
        doc["emergency"] = true;
        doc["reason"] = "hardware_button";
        doc["timestamp"] = millis();
        
        String output;
        serializeJson(doc, output);
        client.publish(topic_emergency, output.c_str());
      }
    }
    lastCheck = millis();
  }
}

void readSensors() {
  if (sensorsReady) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to proper units
    accelX = ax / 16384.0; // g
    accelY = ay / 16384.0; // g
    accelZ = az / 16384.0; // g
    
    gyroX = gx / 131.0; // degrees/second
    gyroY = gy / 131.0; // degrees/second
    gyroZ = gz / 131.0; // degrees/second
    
    temperature = mpu.getTemperature() / 340.0 + 36.53; // Celsius
  }
  
  // Read battery voltage (assuming voltage divider)
  int adcValue = analogRead(BATTERY_PIN);
  batteryVoltage = (adcValue / 4095.0) * 3.3 * 4.0; // Assuming 4:1 voltage divider
  batteryPercentage = map(batteryVoltage * 100, 1100, 1260, 0, 100); // 11V-12.6V LiPo
  batteryPercentage = constrain(batteryPercentage, 0, 100);
}

float calculatePID(PIDController* pid, float input) {
  unsigned long now = millis();
  float dt = (now - pid->last_time) / 1000.0;
  
  if (dt > 0) {
    float error = pid->setpoint - input;
    pid->integral += error * dt;
    
    // Integral windup protection
    pid->integral = constrain(pid->integral, -100, 100);
    
    float derivative = (error - pid->previous_error) / dt;
    
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->output = constrain(pid->output, -100, 100);
    
    pid->previous_error = error;
    pid->last_time = now;
  }
  
  return pid->output;
}

void updatePIDControllers() {
  if (!sensorsReady) return;
  
  // Calculate roll and pitch from accelerometer
  float roll = atan2(accelY, accelZ) * 180 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  
  // Update PID controllers
  float rollCorrection = calculatePID(&pidRoll, roll);
  float pitchCorrection = calculatePID(&pidPitch, pitch);
  float yawCorrection = calculatePID(&pidYaw, gyroZ);
  
  // Apply corrections to motor speeds
  if (motorsArmed) {
    int baseSpeed = motorSpeeds[0]; // Use current base speed
    
    motorSpeeds[0] = baseSpeed + rollCorrection + pitchCorrection + yawCorrection; // FL
    motorSpeeds[1] = baseSpeed - rollCorrection + pitchCorrection - yawCorrection; // FR
    motorSpeeds[2] = baseSpeed + rollCorrection - pitchCorrection - yawCorrection; // BL
    motorSpeeds[3] = baseSpeed - rollCorrection - pitchCorrection + yawCorrection; // BR
    
    // Constrain motor speeds
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = constrain(motorSpeeds[i], 1000, 2000);
    }
  }
}

void updateMotors() {
  motorFL.writeMicroseconds(motorSpeeds[0]);
  motorFR.writeMicroseconds(motorSpeeds[1]);
  motorBL.writeMicroseconds(motorSpeeds[2]);
  motorBR.writeMicroseconds(motorSpeeds[3]);
}

void sendTelemetry() {
  if (!mqttConnected) return;
  
  DynamicJsonDocument doc(512);
  
  doc["timestamp"] = millis();
  doc["armed"] = motorsArmed;
  doc["emergency"] = emergencyStop;
  
  // IMU data
  JsonObject imu = doc.createNestedObject("imu");
  imu["accel"]["x"] = accelX;
  imu["accel"]["y"] = accelY;
  imu["accel"]["z"] = accelZ;
  imu["gyro"]["x"] = gyroX;
  imu["gyro"]["y"] = gyroY;
  imu["gyro"]["z"] = gyroZ;
  imu["temperature"] = temperature;
  
  // Motor speeds
  JsonArray motors = doc.createNestedArray("motors");
  for (int i = 0; i < 4; i++) {
    motors.add(motorSpeeds[i]);
  }
  
  // Battery data
  JsonObject battery = doc.createNestedObject("battery");
  battery["voltage"] = batteryVoltage;
  battery["percentage"] = batteryPercentage;
  
  // PID setpoints
  JsonObject pid = doc.createNestedObject("pid");
  pid["roll_setpoint"] = pidRoll.setpoint;
  pid["pitch_setpoint"] = pidPitch.setpoint;
  pid["yaw_setpoint"] = pidYaw.setpoint;
  
  String output;
  serializeJson(doc, output);
  client.publish(topic_telemetry, output.c_str());
}

void sendStatus() {
  if (!mqttConnected) return;
  
  DynamicJsonDocument doc(256);
  
  doc["timestamp"] = millis();
  doc["wifi_connected"] = wifiConnected;
  doc["mqtt_connected"] = mqttConnected;
  doc["sensors_ready"] = sensorsReady;
  doc["motors_armed"] = motorsArmed;
  doc["emergency_stop"] = emergencyStop;
  doc["uptime"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  
  String output;
  serializeJson(doc, output);
  client.publish(topic_status, output.c_str());
}

void setStatusLED(int red, int green, int blue) {
  analogWrite(LED_RED_PIN, red);
  analogWrite(LED_GREEN_PIN, green);
  analogWrite(LED_BLUE_PIN, blue);
}

void updateStatusLED() {
  static unsigned long lastUpdate = 0;
  static bool ledState = false;
  
  if (millis() - lastUpdate > 500) {
    if (emergencyStop) {
      // Flashing red for emergency
      setStatusLED(ledState ? 255 : 0, 0, 0);
    } else if (!wifiConnected) {
      // Flashing blue for WiFi issues
      setStatusLED(0, 0, ledState ? 255 : 0);
    } else if (!mqttConnected) {
      // Flashing yellow for MQTT issues
      setStatusLED(ledState ? 255 : 0, ledState ? 255 : 0, 0);
    } else if (motorsArmed) {
      // Solid green for armed and ready
      setStatusLED(0, 255, 0);
    } else {
      // Slow breathing green for disarmed but ready
      int brightness = (sin(millis() / 1000.0) + 1) * 127;
      setStatusLED(0, brightness, 0);
    }
    
    ledState = !ledState;
    lastUpdate = millis();
  }
}

void soundBuzzer(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(100);
  }
}