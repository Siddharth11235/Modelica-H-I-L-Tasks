void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //serial begin

}

unsigned long lastTime;
double Input[2];
double q, theta, Output;
void func ()
{
  Output = q + theta;
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
    
  }
 for (int i = 1; i < (readStr.length()-1); i++)
  {
    readVal += readStr[i];  
  }
 
  String a = "100, 200";
  char ac[20];
 readVal.toCharArray(ac,20);
  char* b = strtok(ac, ",");
  
  double aI = atof(b);
  b = strtok(NULL, ",");
  double bI = atof(b);
  
   

  
  
    
    Serial.println("1,"+String(bI)+"\n"); //send data in same format i.e. ending with \n character
    delay(5);
  } 

}
