

void setup()
{
  Serial.begin(115200); //serial begin
}

unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
//double Ti = 2;
//double Td = 0.5;
double kp = 62;
double ki =0;
double kd = 0.01;
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime)/10;
  
   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
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
  
   
      //Setpoint = 100*sin(millis()*3.1412/(20*180))+100;// This is for the case where we may not have a FG on hand.
      Setpoint = double(analogRead(A5));  

    
    Input = readVal.toDouble(); //extract value
    Compute();
    Serial.println("1,"+String(Output)+"\n"+"2,"+String(Setpoint)+"\n"); //send data in same format i.e. ending with \n character
    delay(5);
  } 
}
