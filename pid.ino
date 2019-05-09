
int i=0;
int x[5];
long duration, cm;
int distance=0,dis=0;
#include<Servo.h>                                                  
#include<PID_v1.h>


const int servoPin = 9;                                              
const int TrigPin = 7;
const int EchoPin = 6;

 
 
float Kp = 0.8;                   // Proportional Gain
float Ki = 0.5;                   //Integral Gain
float Kd = 0.5;                   //Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       



PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);          
                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600);                                                
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  myServo.attach(servoPin);                                          //Attach Servo

  for(i=0;i<5;i++)
 {
  x[i] = readPosition();
 
 if(x[i]>46)
 x[i]=2;
}
 Serial.println((x[0]+x[1]+x[2]+x[3]+x[4])/5);                                          
 distance=((x[0]+x[1]+x[2]+x[3]+x[4])/5);
 Input=distance; 
 
  myPID.SetMode(AUTOMATIC);                                          
  myPID.SetOutputLimits(-15,10);                                     
}

void loop()
{
 
  Setpoint = 35;    
dis=readPosition();

if(dis>46)
dis=2;     
for(i=0;i<5;)
{
  x[i++]=x[i];
}                            
 x[4]=dis;
 distance=((x[0]+x[1]+x[2]+x[3]+x[4])/5);
 Serial.println(distance);
 Input=distance;
    
    myPID.Compute();                                                  

  ServoOutput=93+Output;                                            
  myServo.write(ServoOutput);    

  
}
      
      

float readPosition() 
{                                                           
   //pinMode(TrigPin, OUTPUT);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);


  //pinMode(EchoPin, INPUT);
  duration = pulseIn(EchoPin, HIGH);

  cm = duration/(29*2);
 
  return cm;                                         
}




