#include <HardwareSerial.h>

HardwareSerial SIM(1);

#define SIM_RX 27   // ESP32 RX (from SIM TX)
#define SIM_TX 26   // ESP32 TX (to SIM RX)
#define SIM_DTR 33  // DTR control pin

void setup() {
  Serial.begin(115200);

  pinMode(SIM_DTR, OUTPUT);
  digitalWrite(SIM_DTR, LOW);  // HIGH = normal operation (awake)

  SIM.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  Serial.println("SIM800L Test with GPIO33 (DTR)");
}

void loop() {
  // Forward SIM → PC
  while (SIM.available()) {
    Serial.write(SIM.read());
  }

  // Forward PC → SIM
  while (Serial.available()) {
    SIM.write(Serial.read());
  }
}