/*
WIRING YOU GAVE (interpreting Dx as GPIOx)

1) MPU6050 (I2C)
  INT  -> GPIO5
  SDA  -> GPIO17
  SCL  -> GPIO16

2) GPS NEO-6M (UART)
  GPS RX -> GPIO23   (ESP32 TX to GPS RX)
  GPS TX -> GPIO22   (ESP32 RX from GPS TX)

3) SIM800L (UART)
  SIM RXD -> GPIO26  (ESP32 TX to SIM RXD)
  SIM TXD -> GPIO27  (ESP32 RX from SIM TXD)
  DTR     -> GPIO33

This sketch:
- Initializes MPU6050 on custom I2C pins and prints raw accel/gyro.
- Prints GPS NMEA lines.
- Lets you send AT commands to SIM800L from Serial Monitor.
*/

#include <Wire.h>

// ---------------- Pins ----------------
static const int MPU_INT_PIN = 5;
static const int I2C_SDA     = 17;
static const int I2C_SCL     = 16;

static const int GPS_RX_PIN  = 22; // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN  = 23; // ESP32 TX  -> GPS RX

static const int SIM_RX_PIN  = 27; // ESP32 RX  <- SIM TXD
static const int SIM_TX_PIN  = 26; // ESP32 TX  -> SIM RXD
static const int SIM_DTR_PIN = 33;

// ---------------- UARTs ----------------
// ESP32 has 3 UARTs: Serial(0), Serial1, Serial2
HardwareSerial GPS(1);
HardwareSerial SIM(2);

// ---------------- MPU6050 minimal ----------------
static const uint8_t MPU_ADDR = 0x68;

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool mpuInit() {
  Wire.begin(I2C_SDA, I2C_SCL);

  uint8_t who = i2cRead8(MPU_ADDR, 0x75); // WHO_AM_I
  if (who != 0x68 && who != 0x69) return false;

  // Wake up (PWR_MGMT_1 = 0)
  i2cWrite8(MPU_ADDR, 0x6B, 0x00);

  // Optional: set gyro ±250dps, accel ±2g (defaults usually)
  i2cWrite8(MPU_ADDR, 0x1B, 0x00);
  i2cWrite8(MPU_ADDR, 0x1C, 0x00);

  return true;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((int)MPU_ADDR, 14);
  if (Wire.available() < 14) return false;

  auto rd16 = [&]() -> int16_t {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (hi << 8) | (lo & 0xFF);
  };

  ax = rd16();
  ay = rd16();
  az = rd16();
  (void)rd16(); // temp
  gx = rd16();
  gy = rd16();
  gz = rd16();
  return true;
}

// ---------------- MPU interrupt (optional) ----------------
volatile bool mpuIntFlag = false;
void IRAM_ATTR onMpuInt() { mpuIntFlag = true; }

// ---------------- Helpers ----------------
String simCmdLine;

void setup() {
  Serial.begin(115200);
  delay(300);

  // DTR: to avoid SIM800L sleeping, keep DTR HIGH (common practice).
  // If your module behaves opposite, set LOW instead.
  pinMode(SIM_DTR_PIN, OUTPUT);
  digitalWrite(SIM_DTR_PIN, HIGH);

  // GPS UART (NEO-6M often 9600)
  GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // SIM800L UART (often 9600, sometimes 115200)
  SIM.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  // MPU6050
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMpuInt, RISING);

  if (!mpuInit()) {
    Serial.println("MPU6050 not found on I2C (check SDA/SCL/power/address).");
  } else {
    Serial.println("MPU6050 OK.");
  }

  Serial.println("\nCommands:");
  Serial.println("  Type AT commands and press Enter -> sent to SIM800L");
  Serial.println("  GPS NMEA lines will print automatically");
  Serial.println("  MPU raw values print every 200ms (or on interrupt if it triggers)\n");

  // Quick SIM ping
  SIM.println("AT");
}

void loop() {
  // -------- GPS: print NMEA lines --------
  while (GPS.available()) {
    char c = (char)GPS.read();
    Serial.write(c); // prints raw NMEA stream
  }

  // -------- SIM800L: read replies --------
  while (SIM.available()) {
    Serial.write(SIM.read());
  }

  // -------- Send AT commands from Serial Monitor to SIM800L --------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;      // ignore CR
    if (c == '\n') {
      if (simCmdLine.length()) {
        SIM.println(simCmdLine);
        simCmdLine = "";
      }
    } else {
      simCmdLine += c;
    }
  }

  // -------- MPU6050: read/print raw accel+gyro --------
  static uint32_t lastMpuMs = 0;
  bool doRead = false;

  // If your INT pin is configured to fire (depends on MPU config), this will trigger reads.
  if (mpuIntFlag) {
    mpuIntFlag = false;
    doRead = true;
  }

  // Also do periodic read (works even if INT not configured)
  if (millis() - lastMpuMs >= 200) {
    lastMpuMs = millis();
    doRead = true;
  }

  if (doRead) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      Serial.print("MPU ax ay az gx gy gz: ");
      Serial.print(ax); Serial.print(' ');
      Serial.print(ay); Serial.print(' ');
      Serial.print(az); Serial.print("   ");
      Serial.print(gx); Serial.print(' ');
      Serial.print(gy); Serial.print(' ');
      Serial.println(gz);
    }
  }
}