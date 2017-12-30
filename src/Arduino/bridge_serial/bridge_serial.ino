/*
 * Bridges messages between Serial and Serial2
 * 
 * Use this with teensy3.5 / teensy3.6 to test camera operation
 * 
 * Camera wiring
 * 
 * Camera Wire - Arduino Pin Name
 * -------------------------------
 * black       - ground
 * red         - 3.3v
 * white       - 9 (rx2)
 * yellow      - 10 (tx2)
 * 
 */

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  char incomingByte;
  while (Serial.available() > 0) {
      incomingByte = Serial.read();
      Serial2.write(incomingByte);
  }
  while (Serial2.available() > 0) {
    incomingByte = Serial2.read();
    Serial.write(incomingByte);
  }
}
