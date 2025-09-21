float Kp = 0.6, Ki = 0.2, Kd = 0.1;
float setpoint = 0, input = 0, output = 0;
float last_error = 0, integral = 0;

void updatePID(float measured, float target) {
  float error = target - measured;
  integral += error;
  float derivative = error - last_error;
  output = Kp * error + Ki * integral + Kd * derivative;
  last_error = error;
}

void loop() {
  // Example: read IMU and apply PID
  float measured_value = analogRead(32); // Replace with real IMU
  updatePID(measured_value, setpoint);
  analogWrite(2, output); // Apply to motor (as an example)
  delay(20);
}