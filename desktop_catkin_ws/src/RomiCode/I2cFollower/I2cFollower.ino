
#include <Romi32U4.h>

#include <Wire.h>
#include <LSM6.h>
#include <PololuRPiSlave.h>


Romi32U4Encoders encoders;
Romi32U4Motors motors;
LSM6 imu;

struct Data
{


  float x_ref, th_ref, a_spd, b_spd, gyro;
  uint16_t dt;

};

PololuRPiSlave<struct Data,5> slave;

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

void setup()
{

  Wire.begin();
  imu.init();
  imu.enableDefault();

  imu.read();
  gyro_offset=imu.g.z;

   slave.init(20);


  delay(100);
 
}

void loop()
{
  slave.updateBuffer();
  xspd_ref=slave.buffer.x_ref;
  thspd_ref=slave.buffer.th_ref;
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

  slave.buffer.a_spd=spdA;
  slave.buffer.b_spd=spdB;
  slave.buffer.dt=dt;

  

  imu.read();
  slave.buffer.gyro=(imu.g.z-gyro_offset)*8.75/1000*3.14159/180;

  slave.finalizeWrites();
  
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

