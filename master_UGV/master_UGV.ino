#include "ros.h"
#include "geometry_msgs/Twist.h"

#include <Encoder.h>
Encoder myEnc(2, 9); //0 
Encoder myEnc2(20, 21); //0 1
Encoder myEnc3(18, 13); //0 1
Encoder myEnc4(19, 10); //0 1

const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

float diffPosition1 = 0;
float Position1 = 0;
long diffPosition2 = 0;
long Position2 = 0;
long diffPosition3 = 0;
long Position3 = 0;
long diffPosition4 = 0;
long Position4 = 0;
float time1 = 0;
float time2 = 0;
float time3 = 0;
float time4 = 0;
long time5 = 0;
long time6 = 0;
long time7 = 0;
long time8 = 0;

float kp = 0.08; //0.08
float ki = 0.02;
float offset1 = 0;
float totaloffset1 = 0;
long offset2 = 0;
long totaloffset2 = 0;
long offset3 = 0;
long totaloffset3 = 0;
long offset4 = 0;
long totaloffset4 = 0;
float countps1 = 0;
int PWM1 = 0;
//float countpsM1 = 0;
float directionM1=0;
float countpsM2 = 0;
long PWM2 = 0;
//float countpsM2 = 0;
float directionM2=0;
//float countps3 = 0;
long PWM3 = 0;
float countpsM3 = 0;
float directionM3=0;
float countps4 = 0;
long PWM4 = 0;
//float countpsM4 = 0;
float directionM4=0;
float countpsM4 = 0;
float desiredM4 = 0;
float countpsM1 = 0;
float desiredM1 = 0;
//float countpsM3 = 0;
float desiredM3 = 0;
float interval=0;
float a=0.15;
float b=0.15;
float r=0.1;
float xvel=0; 
float yvel=0; 
float rot=0; 
float temp=0;


ros::NodeHandle nh;

void velCallback(  const geometry_msgs::Twist& vel)
{
     xvel = vel.linear.x; // I CAN USE VEL AS I WANT
     yvel = vel.linear.y;
     rot = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void M1_advance(float xvel,float yvel,float rot) 
{

  desiredM1=(1/r)*(xvel-yvel-(a+b)*rot);
  Position1 = myEnc.read();
  time4 = millis();
  time3 = time4 - time3;

  diffPosition1 = Position1 - diffPosition1;
  countpsM1 = diffPosition1 / (float(time3) / 1000.0f);
  
  offset1 = desiredM1-countpsM1*2.0*3.142/3078.0;
  
  totaloffset1 =totaloffset1 + offset1;
  
  PWM1 = (offset1 * kp) + (totaloffset1 * ki);

  if(PWM1>0)
  {
    directionM1=0;
  }
  else
  {
    directionM1=1;
    PWM1=PWM1*-1;
  }
  PWM1=map(PWM1,0,13.064,0,255);
  
  if (PWM1 > 255)
  {
    PWM1 = 255;
  }
  
//  Serial.println(PWM1);
  diffPosition1 = Position1;
  time3 = time4;
   
  digitalWrite(M1, directionM1);
//digitalWrite(M1, 0);
  analogWrite(E1, PWM1);
}

void M2_advance(float xvel,float yvel,float rot) 
{
  float countpsM2 = 0;
  float desiredM2 = 0;
  desiredM2=(1/r)*(xvel+yvel+(a+b)*rot);
  Position2 = myEnc2.read();
  time2 = millis();
  time1 = time2 - time1;

  diffPosition2 = Position2 - diffPosition2;
  countpsM2 = diffPosition2 / (float(time1) / 1000);
  
  offset2 = desiredM2 - (countpsM2*2*3.142/3078);
  totaloffset2 += offset2;
  PWM2 = (offset2 * kp) + (totaloffset2 * ki);

  if(PWM2>0)
  {
    directionM2=0;
  }
  else
  {
    directionM2=1;
    PWM2=PWM2*-1;
  }
  PWM2=map(PWM2,0,13.064,0,255);
  if (PWM2 > 255)
  {
    PWM2 = 255;
  }

//  Serial.print("Error = ");
//  Serial.print(offset2);
//  Serial.print(" Control = ");
//  Serial.println(PWM2);

  
  diffPosition2 = Position2;
  time1 = time2;
  digitalWrite(M2, directionM2);
//digitalWrite(M2, 0);
  analogWrite(E2, PWM2);
}

void M3_advance(float xvel,float yvel,float rot) 
{
  
  desiredM3=(1/r)*(xvel+yvel-(a+b)*rot);
  Position3 = myEnc3.read();
  time6 = millis();
  time5 = time6 - time5;

  diffPosition3 = Position3 - diffPosition3;
  countpsM3 = diffPosition3 / (float(time5) / 1000);
  
  offset3 = desiredM3 - (countpsM3*2.0*3.142/3078);
  totaloffset3 += offset3;
  PWM3 = (offset3 * kp) + (totaloffset3 * ki);


  if(PWM3>0)
  {
    directionM3=0;
  }
  else
  {
    directionM3=1;
    PWM3=PWM3*-1;
  }
  PWM3=map(PWM3,0,13.064,0,255);
  if (PWM3 > 255)
  {
    PWM3 = 255;
  }
  diffPosition3 = Position3;
  time5 = time6;
  //Serial.println(countpsM3);
  digitalWrite(M3, directionM3);
//digitalWrite(M3, 0);
  analogWrite(E3, PWM3);
}

void M4_advance(float xvel,float yvel,float rot)
{

  desiredM4=(1/r)*(xvel-yvel+(a+b)*rot);
  Position4 = myEnc4.read();
  time8 = millis();
  time7 = time8 - time7;

  diffPosition4 = Position4 - diffPosition4;
  countpsM4 = diffPosition4 / (float(time7) / 1000);
  
  offset4 = desiredM4 - (countpsM4*2*3.142/3078);
  totaloffset4 += offset4;
  PWM4 = (offset4 * kp) + (totaloffset4 * ki);


  if(PWM4>0)
  {
    directionM4=0;
  }
  else
  {
    directionM4=1;
    PWM4=PWM4*-1;
  }
  PWM4=map(PWM4,0,13.064,0,255);
  if (PWM4 > 255)
  {
    PWM4 = 255;
  }
  //Serial.println(countsps1);
  diffPosition4 = Position4;
  time7 = time8;
  //Serial.println(countpsM4);
  digitalWrite(M4, directionM4);
//digitalWrite(M4, 0);
  analogWrite(E4, PWM4);
}

void setup ()
{

  myEnc.write(0);
  myEnc2.write(0);
  myEnc3.write(0);
  myEnc4.write(0);
  //Serial.begin (115200);
  pinMode(E1,OUTPUT);
  pinMode(E2,OUTPUT);
  pinMode(E3,OUTPUT);
  pinMode(E4,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);
  pinMode(2,INPUT);
  pinMode(9,INPUT);
//  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);

}  // end of setup

void loop ()
{
  nh.spinOnce();
  delay(15);
  M1_advance(xvel,yvel,rot);
  M2_advance(xvel,yvel,rot);
  M3_advance(xvel,yvel,rot);
  M4_advance(xvel,yvel,rot);
  
}  // end of loop
