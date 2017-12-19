#include <PID_v1.h>

double inVal, outVal,Setpoint;
double consKp=1 , consKi=0, consKd=0;
PID myPID(&inVal, &outVal, &Setpoint, consKp, consKi, consKd, DIRECT);

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
  
  
  if (Serial.available()){ //when serial data comes from modelica
  while(Serial.available()){
    char readChar = (char)Serial.read();
    readStr+=readChar; 
    if(readChar == '\n') break;
  } //read the data and store in a string
    for (int i = 1; i < (readStr.length()-1); i++)
    {
      readVal += readStr[i];
    }
    inVal = readVal.toDouble(); //extract value
    Setpoint = ((double)analogRead(A5))/4; 
    myPID.Compute();
    Serial.print("1," + String(Setpoint) + "\n"); //send data in same format i.e. ending with \n character
    delay(1);
  } 
}
