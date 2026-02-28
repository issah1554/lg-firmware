#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables for initial calibration
float baseAccelX = 0, baseAccelY = 0, baseAccelZ = 0;
float baseGyroX = 0, baseGyroY = 0, baseGyroZ = 0;
bool calibrated = false;

// Thresholds for detecting motion (adjust these values as needed)
float ACCEL_THRESHOLD = 0.5;  // m/s² difference from base
float GYRO_THRESHOLD = 0.3;   // rad/s difference from base
int VIBRATION_SAMPLES = 5;     // Number of samples to check for vibration

// LED pins for different ESP32 boards
// Most ESP32 dev boards have built-in LED on GPIO2
// Some use GPIO5 or other pins - check your board
#define LED_BUILTIN 2  // Change this if your board uses a different pin

// Variables for LED control
unsigned long motionStartTime = 0;
bool ledState = false;
const unsigned long LED_ON_DURATION = 1000; // LED stays on for 1 second after motion stops

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Start with LED off
  
  Serial.println("ESP32 MPU6050 Motion Detector");
  Serial.println("--------------------------------");
  
  // Initialize I2C with your pins (SDA=17, SCL=16)
  Wire.begin(17, 16);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring");
    Serial.println("SDA=17, SCL=16");
    // Blink LED rapidly to indicate error
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  Serial.println("MPU6050 found!");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("Calibrating sensor... Keep sensor still");
  delay(2000);
  
  // Read initial values for calibration
  calibrateSensor();
  
  Serial.println("Calibration complete!");
  Serial.print("Base Accel (X,Y,Z): ");
  Serial.print(baseAccelX, 2); Serial.print(", ");
  Serial.print(baseAccelY, 2); Serial.print(", ");
  Serial.println(baseAccelZ, 2);
  
  Serial.print("Base Gyro (X,Y,Z): ");
  Serial.print(baseGyroX, 2); Serial.print(", ");
  Serial.print(baseGyroY, 2); Serial.print(", ");
  Serial.println(baseGyroZ, 2);
  
  Serial.println("--------------------------------");
  Serial.println("Monitoring for motion/vibration/tilt...");
  
  // Quick LED test
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate differences from base values
  float accelDiffX = abs(a.acceleration.x - baseAccelX);
  float accelDiffY = abs(a.acceleration.y - baseAccelY);
  float accelDiffZ = abs(a.acceleration.z - baseAccelZ);
  
  float gyroDiffX = abs(g.gyro.x - baseGyroX);
  float gyroDiffY = abs(g.gyro.y - baseGyroY);
  float gyroDiffZ = abs(g.gyro.z - baseGyroZ);
  
  // Check for tilt (change in accelerometer)
  bool tiltDetected = (accelDiffX > ACCEL_THRESHOLD || 
                       accelDiffY > ACCEL_THRESHOLD || 
                       accelDiffZ > ACCEL_THRESHOLD);
  
  // Check for vibration/rotation (change in gyroscope)
  bool vibrationDetected = (gyroDiffX > GYRO_THRESHOLD || 
                            gyroDiffY > GYRO_THRESHOLD || 
                            gyroDiffZ > GYRO_THRESHOLD);
  
  // If any motion detected
  if (tiltDetected || vibrationDetected) {
    // Turn LED on
    digitalWrite(LED_BUILTIN, HIGH);
    motionStartTime = millis(); // Record when motion started
    
    Serial.println("\n*** MOTION DETECTED! ***");
    
    // Print current readings
    Serial.print("Current Accel (m/s²): ");
    Serial.print(a.acceleration.x, 2); Serial.print(", ");
    Serial.print(a.acceleration.y, 2); Serial.print(", ");
    Serial.println(a.acceleration.z, 2);
    
    Serial.print("Diff from base: ");
    Serial.print(accelDiffX, 2); Serial.print(", ");
    Serial.print(accelDiffY, 2); Serial.print(", ");
    Serial.println(accelDiffZ, 2);
    
    Serial.print("Current Gyro (rad/s): ");
    Serial.print(g.gyro.x, 2); Serial.print(", ");
    Serial.print(g.gyro.y, 2); Serial.print(", ");
    Serial.println(g.gyro.z, 2);
    
    // Classify the motion
    if (tiltDetected && !vibrationDetected) {
      Serial.println("Type: TILT");
    } else if (!tiltDetected && vibrationDetected) {
      Serial.println("Type: VIBRATION/ROTATION");
    } else {
      Serial.println("Type: COMBINED MOTION");
    }
    
    Serial.println("------------------------");
  } else {
    // No motion detected, but keep LED on for LED_ON_DURATION after last motion
    if (motionStartTime > 0 && (millis() - motionStartTime) < LED_ON_DURATION) {
      // Still within the LED on period
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      // LED on period expired
      digitalWrite(LED_BUILTIN, LOW);
      motionStartTime = 0;
    }
  }
  
  delay(100); // Check every 100ms
}

void calibrateSensor() {
  const int numSamples = 50;
  float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  
  Serial.print("Calibrating");
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    sumAccelX += a.acceleration.x;
    sumAccelY += a.acceleration.y;
    sumAccelZ += a.acceleration.z;
    
    sumGyroX += g.gyro.x;
    sumGyroY += g.gyro.y;
    sumGyroZ += g.gyro.z;
    
    if (i % 10 == 0) {
      Serial.print(".");
    }
    delay(50);
  }
  
  Serial.println(" done!");
  
  // Calculate averages
  baseAccelX = sumAccelX / numSamples;
  baseAccelY = sumAccelY / numSamples;
  baseAccelZ = sumAccelZ / numSamples;
  
  baseGyroX = sumGyroX / numSamples;
  baseGyroY = sumGyroY / numSamples;
  baseGyroZ = sumGyroZ / numSamples;
  
  calibrated = true;
}