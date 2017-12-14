#include <PID_v1.h>

double Setpoint, Input, Output;
double consKp=1 , consKi=0, consKd=0;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
void setup()
{
  Serial.begin(115200); //serial begin
  pinMode(A5, INPUT);

myPID.SetMode(AUTOMATIC);
}

void loop()
{
  String readStr = ""; //some variables
  String readVal = "";
  
  
  if (Serial.available())
  { //when serial data comes from modelica
  while(Serial.available())
  {
    char readChar = (char)Serial.read();
    readStr+=readChar; 
    if(readChar == '\n') break;
  } //read the data and store in a string
    for (int i = 1; i < (readStr.length()-1); i++)
    {
      readVal += readStr[i];  
    }
    Input = readVal.toDouble(); //extract value
    Setpoint = ((double)analogRead(A5)); 
    myPID.Compute();
    Serial.println(String(Output)); //send data in same format i.e. ending with \n character
    delay(1);
  } 
}
