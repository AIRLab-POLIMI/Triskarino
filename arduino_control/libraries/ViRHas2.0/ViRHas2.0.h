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
  Modification made by Massimiliano Nigro:
    - Changed direct and inverse kinematics formulas
    - Added write debug string function to tune PID Controller
    - Changed PID control setpoint to robot twist
    - Changed PID control to 1 PID for linear and another for rotational speed
    - Added setters to modify wheel_radius, robot_radius and encoder PPR
    - Deleted unused functions and constants
*/

#ifndef ViRHaS_h
#define ViRHaS_h

#include "Arduino.h"
#include "CytronMotorDriver.h"
#include "Encoder.h"

#define NMOTOR 3

class ViRHaS
{
private:

  #define LOOPTIME        25                     // PID loop time
  double speed_wheel[3];         //SETPOINT
  double speed_act[3];                              // speed (actual value)
  int countAnt[3];
  double last_error[3];
  double Iterm[3];
  double lastSP[3];
  float Kp_xy;
  float Ki_xy;
  float Kd_xy;
  float Kp_th;
  float Ki_th;
  float Kd_th;
  double posX;
  double posY;
  double posTh;
  double speedX;
  double speedY;
  double speedTh;
  double reqSpeedX;
  double reqSpeedY;
  double reqSpeedTh;

  float wheel_radius;
  float robot_radius;
  float encoder_ppr;

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

  void setWheelRadius(float radius);
  void setRobotRadius(float radius);
  void setEncoderPPR(float ppr);

  void setIterm(int i, double val);
    
  void setKpid_xy(double val, double val1, double val2);
  void setKpid_th(double val, double val1, double val2);

  void stop(void);


};

#endif /* Triskar_h */
