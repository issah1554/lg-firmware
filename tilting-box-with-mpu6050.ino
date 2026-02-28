#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float roll = 0.0f, pitch = 0.0f;
unsigned long lastMs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  // Your wiring: SDA=GPIO17, SCL=GPIO16
  Wire.begin(17, 16);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastMs = millis();
  Serial.println("ROLL,PITCH"); // header
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  lastMs = now;

  // Accel angles (degrees)
  float accRoll  = atan2(a.acceleration.y, a.acceleration.z) * 57.2957795f;
  float accPitch = atan2(-a.acceleration.x,
                         sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))
                   * 57.2957795f;

  // Gyro rates (deg/s) - gyro is in rad/s from library
  float gyroRollRate  = g.gyro.x * 57.2957795f;
  float gyroPitchRate = g.gyro.y * 57.2957795f;

  // Complementary filter
  const float alpha = 0.98f;
  roll  = alpha * (roll  + gyroRollRate  * dt) + (1 - alpha) * accRoll;
  pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * accPitch;

  // Send as CSV: roll,pitch
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.println(pitch, 2);

  delay(20); // ~50 Hz
}