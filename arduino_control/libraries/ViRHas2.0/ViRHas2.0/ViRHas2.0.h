/*
  Fork by Michele Bertoni of library Triskar.h - Library for controlling the Triskar robot.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
  Modification made by Matteo Lucchelli including
    - encoders reader
    - PID speed control
  - Odometry
  Modification made by Michele Bertoni:
    - added support for MR001-004 drivers
    - minor changes to make this library work with ViRHaS robot
*/

#ifndef ViRHaS_h
#define ViRHaS_h

#include "Arduino.h"
#include "CytronMotorDriver.h"
#include "Encoder.h"

#define NMOTOR 3
#define wheel_radius  3.0f //cm
#define robot_radius  15.0f  //cm

class ViRHaS
{
private:

  #define KP  0.1f;
  #define KI  1.0f;
  #define _MAX_DTH  30.0f     // Maximum wheel angular speed [rad/s]
  #define _MAX_SP   400.0f   // Maximum setpoint
  #define _SP_SCALE (_MAX_SP / _MAX_DTH)
  #define LOOPTIME        25                     // PID loop time
  double speed_wheel[3];         //SETPOINT
  double speed_act[3];                              // speed (actual value)
  int countAnt[3];
  double last_error[3];
  double Iterm[3];
  float Kp;
  float Ki;
  float Kd;
  double posX;
  double posY;
  double posTh;
  double speedX;
  double speedY;
  double speedTh;
  double reqSpeedX;
  double reqSpeedY;
  double reqSpeedTh;

  unsigned long lastMilliLoop;
  unsigned long lastMillis[3];
  unsigned long deltaPid;

  CytronMD & _m1;
  CytronMD & _m2;
  CytronMD & _m3;
  Encoder & _e1;
  Encoder & _e2;
  Encoder & _e3;
  
  
  void getMotorCmS(long deltaT,int pos,int i);
  
  int updatePid(double targetValue, double currentValue, int i);
  void direct_kinematics(void);
  void makeOdometry(unsigned long int deltaT);
  void writeDebugInfo(char* debug_msg_static);

public:
  ViRHaS(CytronMD & m1,CytronMD & m2, CytronMD & m3,
      Encoder & e1, Encoder & e2, Encoder & e3);
  void run2(float strafe, float forward, float angular);
  void inverseKinematics(float strafeError, float forwardError, float angularError);
 
  void PIDLoop(char* debug_msg_static);

  double getSpeedX();
  double getSpeedY();
  double getSpeedTh();
  
  double getPosX();
  double getPosY();
  double getPosTh();

  void setPosX(double _posX);
  void setPosY(double _posY);
  void setPosTh(double _posTh);

  void setIterm(int i, double val);

  void setKp(double val);
  void resetKp();

  void setKi(double val);
  void resetKi();
    
  void setKpid(double val, double val1, double val2);

  void stop(void);


};

#endif /* Triskar_h */
