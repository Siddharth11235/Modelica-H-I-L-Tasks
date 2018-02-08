void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write(analogRead(A5))/4;
  delay(5);
}
