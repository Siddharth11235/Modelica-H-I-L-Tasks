int receivedChar;
int counter;
boolean newData = false;

void setup() 
{
    Serial.begin(115200);
}

void loop() 
{
    recvOneChar();
    showNewData();
}

void recvOneChar() 
{
    if (Serial.available() > 0) 
    {
      receivedChar = Serial.read();
      if (receivedChar>0)
      {
        newData = true;
      }
    }
}

void showNewData() 
{
    if (newData == true) 
    {
        Serial.write(receivedChar);
        newData = false;
    }
}


