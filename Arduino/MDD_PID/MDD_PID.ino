#include <PID_v1.h>

double Setpoint, Input, Output;
double consKp=1, consKi=0, consKd=0;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
void setup() 
{
Serial.begin(115200);
pinMode(A5, INPUT);

myPID.SetMode(AUTOMATIC);

}

void loop() 
{
    Input=(double)Serial.read();
    Setpoint = ((double)analogRead(A5))/4; 
    myPID.Compute();
    
    Serial.write((int)Output/*&0xFF*/);
   // Serial.write(((int)Output>>8)&0xFF);
   // delay(5);
  
}


