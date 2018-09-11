void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //serial begin

}

unsigned long lastTime;
double Input[3];
double q, theta, theta_Ref, delEplus, delE;

void feedback(double termRef, double termErr, double Output)
{
  Output = termRef - termErr;
}



void Compute(double Input, double Output, double Setpoint, double kp, double ki,double kd)
{
    double errSum, lastErr;

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

void loop() {
 String readStr = ""; //some variables
  String readVal = "";
  char* p;
  char* q_c;
  char* theta_c;
  
  
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


     int readVal_len = readVal.length() + 1;
    char readF[readVal_len];
    readVal.toCharArray(readF, readVal_len);
    
    p = strtok(readF, ",");
     if(p)
    {
    q_c= p;
    }
    p = strtok(NULL, ",");

    if(p)
    {
     theta_c = p; 
     
    }
    String q_s(q_c);
    String theta_s(theta_c);
  
   
      //Setpoint = 100*sin(millis()*3.1412/(20*180))+100;// This is for the case where we may not have a FG on hand.
      theta_Ref = double(analogRead(A5));  

    q = q_s.toDouble();
    theta = theta_s.toDouble();

    
//    Serial.println("1,"+String(Output)+"\n"+"2,"+String(Setpoint)+"\n"); //send data in same format i.e. ending with \n character
    delay(5);
  } 

}
