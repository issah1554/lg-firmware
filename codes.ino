#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// GPS Setup - Using Hardware Serial2 on ESP32
#define GPS_RX_PIN 22  // ESP32 RX pin connected to GPS TX
#define GPS_TX_PIN 23  // ESP32 TX pin connected to GPS RX

// Variables for initial calibration
float baseAccelX = 0, baseAccelY = 0, baseAccelZ = 0;
float baseGyroX = 0, baseGyroY = 0, baseGyroZ = 0;
bool calibrated = false;

// Thresholds for detecting motion
float ACCEL_THRESHOLD = 0.5;  // m/s² difference from base
float GYRO_THRESHOLD = 0.3;   // rad/s difference from base

// LED pins
#define LED_BUILTIN 2

// Variables for LED control
unsigned long motionStartTime = 0;
const unsigned long LED_ON_DURATION = 1000;

// Motion detection states
enum MotionState {
  IDLE,
  MOTION_DETECTED,
  WAITING_FOR_GPS,
  READING_GPS
};

MotionState currentState = IDLE;
unsigned long motionDetectedTime = 0;
const unsigned long GPS_WAIT_TIME = 10000; // 10 seconds wait after motion

// GPS Data for averaging
struct GPSReading {
  float latitude;
  float longitude;
  float altitude;
  bool valid;
};

const int MAX_GPS_SAMPLES = 10;  // Number of samples to average
GPSReading gpsSamples[MAX_GPS_SAMPLES];
int sampleCount = 0;
unsigned long gpsReadStartTime = 0;
const unsigned long GPS_READ_TIMEOUT = 30000; // 30 seconds timeout
const unsigned long SAMPLE_INTERVAL = 1000; // Take a sample every second

// Buffer for GPS sentences
char gpsBuffer[256];
int bufferIndex = 0;
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize GPS serial communication on Serial2
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("ESP32 MPU6050 Motion Detector with GPS Averaging");
  Serial.println("--------------------------------");
  
  // Initialize I2C
  Wire.begin(17, 16);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring");
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
  
  calibrateSensor();
  
  Serial.println("Calibration complete!");
  Serial.println("--------------------------------");
  Serial.println("Monitoring for motion/vibration/tilt...");
  
  // Quick LED test
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("GPS initialized on Serial2 (RX=22, TX=23)");
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
  
  // Check for motion
  bool tiltDetected = (accelDiffX > ACCEL_THRESHOLD || 
                       accelDiffY > ACCEL_THRESHOLD || 
                       accelDiffZ > ACCEL_THRESHOLD);
  
  bool vibrationDetected = (gyroDiffX > GYRO_THRESHOLD || 
                            gyroDiffY > GYRO_THRESHOLD || 
                            gyroDiffZ > GYRO_THRESHOLD);
  
  // State machine
  switch(currentState) {
    case IDLE:
      if (tiltDetected || vibrationDetected) {
        currentState = MOTION_DETECTED;
        motionDetectedTime = millis();
        digitalWrite(LED_BUILTIN, HIGH);
        
        Serial.println("\n*** MOTION DETECTED! ***");
        Serial.println("Waiting 10 seconds before reading GPS...");
      } else {
        if (motionStartTime > 0 && (millis() - motionStartTime) < LED_ON_DURATION) {
          digitalWrite(LED_BUILTIN, HIGH);
        } else {
          digitalWrite(LED_BUILTIN, LOW);
          motionStartTime = 0;
        }
      }
      break;
      
    case MOTION_DETECTED:
      digitalWrite(LED_BUILTIN, HIGH);
      
      if (millis() - motionDetectedTime >= GPS_WAIT_TIME) {
        Serial.println("\n10 seconds elapsed. Reading GPS position...");
        Serial.println("Taking 10 samples over 10 seconds for averaging...");
        currentState = READING_GPS;
        gpsReadStartTime = millis();
        sampleCount = 0;
        lastSampleTime = 0;
        
        // Clear GPS buffer
        while(Serial2.available()) {
          Serial2.read();
        }
      }
      break;
      
    case READING_GPS:
      // Process incoming GPS data
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        
        // Store in buffer
        if (bufferIndex < 255) {
          gpsBuffer[bufferIndex++] = c;
        }
        
        // Check for end of sentence
        if (c == '\n') {
          gpsBuffer[bufferIndex] = '\0';
          String sentence = String(gpsBuffer);
          
          // Parse GPGGA for position data
          if (sentence.startsWith("$GPGGA")) {
            parseGPGGA(sentence);
          }
          
          bufferIndex = 0;
        }
      }
      
      // Take samples at regular intervals
      if (sampleCount < MAX_GPS_SAMPLES && (millis() - lastSampleTime) >= SAMPLE_INTERVAL) {
        lastSampleTime = millis();
        
        // Try to get a valid GPS reading
        GPSReading reading = getCurrentGPSReading();
        if (reading.valid) {
          gpsSamples[sampleCount] = reading;
          sampleCount++;
          Serial.print("Sample ");
          Serial.print(sampleCount);
          Serial.print("/10 taken: ");
          Serial.print(reading.latitude, 6);
          Serial.print(", ");
          Serial.println(reading.longitude, 6);
        } else {
          Serial.println("No valid GPS fix for this sample, skipping...");
        }
      }
      
      // Check if we have enough samples or timeout
      if (sampleCount >= MAX_GPS_SAMPLES) {
        calculateAndPrintAverage();
        currentState = IDLE;
        motionStartTime = millis();
      } else if (millis() - gpsReadStartTime >= GPS_READ_TIMEOUT) {
        Serial.println("\nGPS timeout reached.");
        if (sampleCount > 0) {
          calculateAndPrintAverage();
        } else {
          Serial.println("No GPS samples acquired.");
        }
        currentState = IDLE;
        motionStartTime = millis();
      }
      break;
  }
  
  delay(100);
}

void parseGPGGA(String sentence) {
  // This function can be expanded if you want to parse in real-time
  // Currently we just store the raw sentence and parse when needed
}

GPSReading getCurrentGPSReading() {
  GPSReading reading;
  reading.valid = false;
  
  // Clear buffer and wait for fresh data
  while(Serial2.available()) {
    Serial2.read();
  }
  
  // Wait for a GPGGA sentence
  unsigned long timeout = millis() + 2000; // 2 second timeout
  String sentence = "";
  
  while (millis() < timeout) {
    if (Serial2.available()) {
      char c = Serial2.read();
      sentence += c;
      
      if (c == '\n') {
        if (sentence.startsWith("$GPGGA")) {
          // Parse GPGGA sentence
          int commaCount = 0;
          String fields[15];
          String currentField = "";
          
          for (int i = 0; i < sentence.length(); i++) {
            if (sentence[i] == ',' || sentence[i] == '\r' || sentence[i] == '\n') {
              if (commaCount < 15) {
                fields[commaCount] = currentField;
                currentField = "";
              }
              commaCount++;
            } else {
              currentField += sentence[i];
            }
          }
          
          // Check if we have fix (field 6 > 0)
          if (commaCount > 6 && fields[6].toInt() > 0) {
            // Parse latitude
            if (fields[2].length() > 0) {
              float lat = fields[2].toFloat();
              int latDeg = int(lat / 100);
              float latMin = lat - (latDeg * 100);
              reading.latitude = latDeg + (latMin / 60.0);
              if (fields[3] == "S") reading.latitude = -reading.latitude;
              
              // Parse longitude
              float lon = fields[4].toFloat();
              int lonDeg = int(lon / 100);
              float lonMin = lon - (lonDeg * 100);
              reading.longitude = lonDeg + (lonMin / 60.0);
              if (fields[5] == "W") reading.longitude = -reading.longitude;
              
              // Parse altitude
              if (fields[9].length() > 0) {
                reading.altitude = fields[9].toFloat();
              }
              
              reading.valid = true;
            }
          }
        }
        break;
      }
    }
  }
  
  return reading;
}

void calculateAndPrintAverage() {
  if (sampleCount == 0) {
    Serial.println("No valid GPS samples to average.");
    return;
  }
  
  float sumLat = 0, sumLon = 0, sumAlt = 0;
  int validCount = 0;
  
  for (int i = 0; i < sampleCount; i++) {
    if (gpsSamples[i].valid) {
      sumLat += gpsSamples[i].latitude;
      sumLon += gpsSamples[i].longitude;
      sumAlt += gpsSamples[i].altitude;
      validCount++;
    }
  }
  
  if (validCount > 0) {
    float avgLat = sumLat / validCount;
    float avgLon = sumLon / validCount;
    float avgAlt = sumAlt / validCount;
    
    Serial.println("\n=================================");
    Serial.println("*** AVERAGE GPS POSITION ***");
    Serial.print("Based on "); Serial.print(validCount); Serial.println(" samples");
    Serial.print("Latitude:  "); Serial.println(avgLat, 6);
    Serial.print("Longitude: "); Serial.println(avgLon, 6);
    Serial.print("Altitude:  "); Serial.print(avgAlt, 1); Serial.println(" m");
    Serial.println("=================================\n");
    
    // You can store or transmit this average position here
    // For example, you could send it via LoRa, WiFi, etc.
    
  } else {
    Serial.println("No valid GPS samples found.");
  }
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
  
  baseAccelX = sumAccelX / numSamples;
  baseAccelY = sumAccelY / numSamples;
  baseAccelZ = sumAccelZ / numSamples;
  
  baseGyroX = sumGyroX / numSamples;
  baseGyroY = sumGyroY / numSamples;
  baseGyroZ = sumGyroZ / numSamples;
  
  calibrated = true;
}