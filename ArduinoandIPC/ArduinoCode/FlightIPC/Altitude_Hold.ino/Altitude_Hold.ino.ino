void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //serial begin

}

unsigned long lastTime;
double Input[3];
double q, theta, theta_Ref, delE2, delE1, delE;

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
  double inVal, outVal;
  int breakCount = 0;
  if (Serial.available()){ //when serial data comes from modelica
  while(Serial.available()){
    char readChar = (char)Serial.read();
    readStr+=readChar; 
    if(readChar == '\n') breakCount++;
    if (breakCount == 2) break;
  } //read the data and store in a string
    for (int i = 1; i < (readStr.length()-1); i++)
    {
      readVal += readStr[i];
    }
    char a[20];
    readVal.toCharArray(a,20);
    char* b = strtok(a,",");
    q= atof(b);
    b = strtok(NULL,",");
    theta = atof(b);

    double thetaRef = 0.1;
    Compute(theta, delE1, thetaRef, 0.2, 1/15, 0);
    delE = delE1 - q*0.002; 
    

    
   Serial.print("1," + String(delE) + "\n"); //send data in same format i.e. ending with \n character
    delay(2);
  } 

}
