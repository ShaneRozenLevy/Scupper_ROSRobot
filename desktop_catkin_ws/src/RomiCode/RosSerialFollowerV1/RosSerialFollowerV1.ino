
#include <Romi32U4.h>
#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
//todo 
//     test to see if can be teleop
//     publish spd data
//     publish gyro data
//     reduce memory usage

Romi32U4Encoders encoders;
Romi32U4Motors motors;


float xspd_ref = 0;
float thspd_ref=0;
double errorA=0;
double errorB=0;


long last_posA=0;
long last_posB=0;
double spdA=0;
double spdB=0;
long tOld=0;
//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
geometry_msgs::Vector3 rpm_msg;

void messageCb(const geometry_msgs::Twist& msg)
{
  xspd_ref=msg.linear.x;
  thspd_ref=msg.angular.z;
 
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);
ros::Publisher rpm("rpm", &rpm_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rpm);
  delay(100);
 
}

void loop()
{
  nh.spinOnce();
  int mB=0;
  int mA=0;
  int offset=23;
  float ki=100;
  int maxSpeed=6;
  float r=.13278/2.0;
  
  long temp_posA=-1*encoders.getCountsLeft();
  long temp_posB=-1*encoders.getCountsRight();
  long tNew=millis();
  double dt=tNew-tOld;
  double factor=1000.0/1440.0*220.0/1000.0;
  spdA=(temp_posA-last_posA)/(dt);// multiplied by 1000 to change into units of s, divide by 1440 to turn into revs/s, times 220 for mm/s
  
  spdB=(temp_posB-last_posB)/(dt);
  spdA=factor*constrain(spdA,-1*maxSpeed,maxSpeed);
  spdB=factor*constrain(spdB,-1*maxSpeed,maxSpeed);

  rpm_msg.x=spdA;
  rpm_msg.y=spdB;
  rpm_msg.z=dt;
  rpm.publish(&rpm_msg);

  errorA=xspd_ref+thspd_ref*r-spdA+errorA;
  errorB=xspd_ref-thspd_ref*r-spdB+errorB;

  if ( errorA > 0)
    {
      mA=(int)constrain(ki*errorA+offset,-300,300);
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

