void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write(analogRead(A5))  ;
  //Serial.write(int(100*sin(millis()*3.1412/(20*180)))+100);
  delay(5);
}
