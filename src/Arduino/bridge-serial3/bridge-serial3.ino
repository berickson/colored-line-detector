void setup() {
  // put your setup code here, to run once:
  while(!Serial) {
    
  }
  Serial3.begin(9600);
  Serial.print("waiting for serial ready");
  if(!Serial3) {
    delay(1);
  }
  Serial.print("ready");

}

void loop() {
  while(Serial3.available()) {
    Serial.write(Serial3.read());
  }
  while(Serial.available()) {
    Serial3.write(Serial.read());
  }
}
