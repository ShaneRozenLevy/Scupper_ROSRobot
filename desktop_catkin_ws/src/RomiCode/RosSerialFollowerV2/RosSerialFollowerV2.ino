
#include <Romi32U4.h>
#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Wire.h>
#include <LSM6.h>
//todo 


Romi32U4Encoders encoders;
Romi32U4Motors motors;
LSM6 imu;



float xspd_ref = 0;
float thspd_ref=0;
double errorA=0;
double errorB=0;

float gyro_offset=0;


long last_posA=0;
long last_posB=0;
double spdA=0;
double spdB=0;
long tOld=0;
//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
geometry_msgs::Vector3 rpm_msg;
std_msgs::Float64 gyro_msg;


void messageCb(const geometry_msgs::Twist& msg)
{
  xspd_ref=msg.linear.x;
  thspd_ref=msg.angular.z;
 
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

ros::Publisher rpm("rpm", &rpm_msg);
ros::Publisher gyro("gyro", &gyro_msg);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rpm);
  nh.advertise(gyro);

  Wire.begin();
  imu.init();
  imu.enableDefault();

  imu.read();
  gyro_offset=imu.g.z;


  delay(100);
 
}

void loop()
{
  nh.spinOnce();
  int mB=0;
  int mA=0;
  int offset=23;
  float ki=95;
  int maxSpeed=6;
  float r=.13278/2.0;
  
  //read encoder posistions
  long temp_posA=-1*encoders.getCountsLeft();
  long temp_posB=-1*encoders.getCountsRight();
  long tNew=millis();
  double dt=tNew-tOld;
  //factor is based on ms to s, diamater of wheels and mm to m
  double factor=1000.0/1440.0*220.0/1000.0;

  //see if the encoders had an issue
  if(!encoders.checkErrorLeft())
  {
      if(abs((temp_posA-last_posA)/(dt))<maxSpeed)
      {
        spdA=(temp_posA-last_posA)/(dt);// multiplied by 1000 to change into units of s, divide by 1440 to turn into revs/s, times 220 for mm/s
        spdA=factor*constrain(spdA,-1*maxSpeed,maxSpeed);//prevent huge speeds when an error occurs
      }

  }

  if(!encoders.checkErrorRight())
  {
      if(abs((temp_posB-last_posB)/(dt))<maxSpeed)
      {
        spdB=(temp_posB-last_posB)/(dt);
        spdB=factor*constrain(spdB,-1*maxSpeed,maxSpeed);//prevent huge speeds when errors occur
      }
  }

  rpm_msg.x=spdA;
  rpm_msg.y=spdB;
  rpm_msg.z=dt;
  rpm.publish(&rpm_msg);

  imu.read();
  if(abs(spdA-spdB) < 0.02)//if the wheels are going close to the same speed rezero the gyro
  {
    gyro_offset=imu.g.z;
  }
  gyro_msg.data=(imu.g.z-gyro_offset)*8.75/1000*3.14159/180;
  gyro.publish(&gyro_msg);

  errorA=xspd_ref+thspd_ref*r-spdA+errorA; calculate the cumulative error
  errorB=xspd_ref-thspd_ref*r-spdB+errorB;

  if ( errorA > 0)
    {
      mA=(int)constrain(ki*errorA+offset,-300,300); //the offset is added in to deal with friction in the motor
    }
  else if (errorA<0)
    {
      mA=(int)constrain(ki*errorA-offset,-300,300);
    }
  else
    {
      mA=0;
    }

  if ( errorB > 0)
    {
      mB=(int)constrain(ki*errorB+offset,-300,300);
    }
  else if (errorB<0)
    {
      mB=(int)constrain(ki*errorB-offset,-300,300);
    }
  else
    {
      mB=0;
    }

  motors.setSpeeds(-mA,-mB);

 delay(10);
 tOld=tNew;
 last_posA=temp_posA;
 last_posB=temp_posB;
}

