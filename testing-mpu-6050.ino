#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Your wiring: SDA=17, SCL=16
  Wire.begin(17, 16);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring/power.");
    while (1) delay(10);
  }

  // Optional: set ranges/filters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accel (m/s^2): ");
  Serial.print(a.acceleration.x); Serial.print(", ");
  Serial.print(a.acceleration.y); Serial.print(", ");
  Serial.print(a.acceleration.z);

  Serial.print(" | Gyro (rad/s): ");
  Serial.print(g.gyro.x); Serial.print(", ");
  Serial.print(g.gyro.y); Serial.print(", ");
  Serial.print(g.gyro.z);

  Serial.print(" | Temp (C): ");
  Serial.println(temp.temperature);

  delay(200);
}