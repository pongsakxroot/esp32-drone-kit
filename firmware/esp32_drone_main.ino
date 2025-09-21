#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  pinMode(2, OUTPUT); // Motor 1
  pinMode(4, OUTPUT); // Motor 2
  pinMode(12, OUTPUT); // Motor 3
  pinMode(13, OUTPUT); // Motor 4
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // Simple stabilization logic (placeholder)
  if (ax > 100) digitalWrite(2, HIGH); else digitalWrite(2, LOW);
  if (ay > 100) digitalWrite(4, HIGH); else digitalWrite(4, LOW);
  if (az > 100) digitalWrite(12, HIGH); else digitalWrite(12, LOW);
  delay(10);
}