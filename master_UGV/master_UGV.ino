

#include <Encoder.h>
Encoder myEnc(2, 3); //0 1

const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

volatile long diffPosition = 0;
volatile long Position = 0;
long time1 = 0;
long time2 = 0;
double desiredM2 = 4500;
double kp = 0.04; //0.08
double ki = 0.01;
double offset = 0;
long totaloffset = 0;
double countps = 0;
double PWM2 = 0;
double countpsM2 = 0;
double directionM3=0;

void M2_advance(int Speed,int Direction) ///<Motor2 Advance
{
  digitalWrite(M3, Direction);
  analogWrite(E3, Speed);
}


void setup ()
{

  myEnc.write(0);
  Serial.begin (115200);
  pinMode(E1,OUTPUT);
  pinMode(E2,OUTPUT);
  pinMode(E3,OUTPUT);
  pinMode(E4,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  //digitalWrite(M1, LOW);
  //analogWrite(E1, 0);

}  // end of setup

void loop ()
{
  Serial.print ("Count = ");
  Position = myEnc.read();
  time2 = millis();
  time1 = time2 - time1;

  diffPosition = Position - diffPosition;
  countps = diffPosition / (float(time1) / 1000);
  
  offset = desiredM2 - countps;
  totaloffset += offset;
  
  //Serial.println (totaloffset);
  PWM2 = (offset * kp) + (totaloffset * ki);
  //countpsM2=map(PWM2,0,6400*5,0,255);
  if (PWM2 > 255)
  {
    PWM2 = 255;
  }
  if (PWM2 < -255)
  {
    PWM2 = 255;
  }
  if(PWM2>0)
  {
    directionM3=0;
  }
  else
  {
    directionM3=1;
  }
  Serial.println (countps);
  //M2_advance(PWM2,directionM3);


  diffPosition = Position;
  time1 = time2;
  delay(15);
}  // end of loop
