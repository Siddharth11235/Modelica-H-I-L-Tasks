
int receivedChar;
int ip[] = {100,100};
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
      if (abs(receivedChar-ip[1]) <= abs(ip[1]-ip[0])*1.2)
      {
        newData = true;
        while (counter >2)
        {
          ip[0] = ip[1];
          ip[1] = receivedChar;  
        }
        
      }
      counter++;   
      
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


