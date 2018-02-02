void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write(analogRead(A5));
  delay(5);
}
