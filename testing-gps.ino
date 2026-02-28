#include <HardwareSerial.h>

HardwareSerial SIM(1);

#define SIM_RX 27
#define SIM_TX 26
#define SIM_DTR 33

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SIM_DTR, OUTPUT);
  digitalWrite(SIM_DTR, HIGH);

  SIM.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  Serial.println("Bridge mode. Type AT commands in Serial Monitor.");
}

void loop() {
  while (SIM.available()) Serial.write(SIM.read());   // SIM -> PC
  while (Serial.available()) SIM.write(Serial.read()); // PC -> SIM
}