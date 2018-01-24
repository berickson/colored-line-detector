void setup() {
  // put your setup code here, to run once:
  while(!Serial) {
    
  }
  Serial4.begin(115200);
  Serial.print("waiting for serial ready");
  if(!Serial4) {
    delay(1);
  }
  Serial.print("ready");

}

void loop() {
  while(Serial4.available()) {
    Serial.write(Serial4.read());
  }
  while(Serial.available()) {
    Serial4.write(Serial.read());
  }
}
