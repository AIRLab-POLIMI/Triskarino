/*
  Fork by Michele Bertoni of library Triskar.cpp - Library for controlling the Triskar robot.
  Created by Martino Migliavacca, August 2, 2013.
  Released into the public domain.
  Modification made by Julian Angel
  -variables dth1, dth2 and dth2 now are global variables
  -add of new methods to get the points
  Modification made by Michele Bertoni
    -edited constructor and motor definitions, in order to make the code work with
     motor drivers "MR001-004"
  Modification made by Massimiliano Nigro:
    - Changed direct and inverse kinematics formulas
    - Added write debug string function to tune PID Controller
    - Changed PID control setpoint to robot twist
    - Changed PID control to 1 PID for linear and another for rotational speed
    - Added setters to modify wheel_radius, robot_radius and encoder PPR
    - Deleted unused functions and constants
*/

#include "ViRHas2.0.h"


ViRHaS::ViRHaS(CytronMD & m1, CytronMD & m2,CytronMD & m3, Encoder & e1, Encoder & e2, Encoder & e3)
: _m1(m1), _m2(m2), _m3(m3), _e1(e1),_e2(e2),_e3(e3) {
  //_m1m2.init2M();
  _m1.setSpeed(0);
  _m2.setSpeed(0);
  //_m3.init1M();
  _m3.setSpeed(0);
  //Variables to store odometry results
   posX=0;
   posY=0;
   posTh=0;
   speedX=0;
   speedY=0;
   speedTh=0;
   reqSpeedX=0;
   reqSpeedY=0;
   reqSpeedTh=0;
  //Speed of the wheels calculated from inverse kinematics
   speed_wheel[0] = 0.0;                      
   speed_wheel[1] = 0.0;
   speed_wheel[2] = 0.0;
  //Error term used for PID control
   last_error[0]=0.0;
   last_error[1]=0.0;
   last_error[2]=0.0;
   Iterm[0]=0.0;
   Iterm[1]=0.0;
   Iterm[2]=0.0;
  //Last position of wheels given by encoders
   countAnt[0] = 0;
   countAnt[1] = 0;
   countAnt[2] = 0;

  //Used to measure elapsed time in order to measure wheel actual speed
   lastMilliLoop=0;
   lastMillis[0]=0;
   lastMillis[1]=0;
   lastMillis[2]=0;
   deltaPid=0;
  
}

void ViRHaS::PIDLoop(char* debug_msg_static){
  deltaPid=millis()-lastMilliLoop;
  float strafeError;
  float forwardError;
  float angularError;

  if(deltaPid >= LOOPTIME)   {// enter tmed loop
     lastMilliLoop=millis();
     long deltaT;
     long ActualPos[3];
     ActualPos[0]=_e1.read();
     deltaT=millis()-lastMillis[0];
     getMotorCmS(deltaT,ActualPos[0],0);// calculate speed
     lastMillis[0] = millis();

     ActualPos[1]=_e2.read();
     deltaT=millis()-lastMillis[1];
     getMotorCmS(deltaT,ActualPos[1],1);
     lastMillis[1] = millis();

     ActualPos[2]=_e3.read();
     deltaT=millis()-lastMillis[2];
     getMotorCmS(deltaT,ActualPos[2],2);
     lastMillis[2] = millis();                                       // calculate speed,
     //Calculates twist (speedX,Y,Th) from actual wheel speed
     direct_kinematics();
     strafeError = updatePid(reqSpeedX,speedX,0);
     forwardError = updatePid(reqSpeedY,speedY,1);
     angularError = updatePid(reqSpeedTh,speedTh,2);
     inverseKinematics(strafeError,forwardError,angularError);
     _m1.setSpeed(speed_wheel[0]);
     _m2.setSpeed(speed_wheel[1]);
     _m3.setSpeed(speed_wheel[2]);
    
    //Calculates pose (posX,Y,Th) given twist
    makeOdometry(deltaPid);
    writeDebugInfo(debug_msg_static);
  }

}

//Writes in the debug string the setpoint and actual speed to allow for PID Tuning
void ViRHaS::writeDebugInfo(char* debug_msg_static){
  debug_msg_static[0]='(';
  dtostrf(reqSpeedX,3,2, &debug_msg_static[1]);
  debug_msg_static[4] = ',';
  dtostrf(reqSpeedY,3,2, &debug_msg_static[5]);
  debug_msg_static[8] = ',';
  dtostrf(reqSpeedTh,3,2, &debug_msg_static[9]);
  debug_msg_static[12] = ')';
  debug_msg_static[13] = ',';
  debug_msg_static[14] = '(';
  dtostrf(speedX,3,2, &debug_msg_static[15]);
  debug_msg_static[18] = ',';
  dtostrf(speedY,3,2, &debug_msg_static[19]);
  debug_msg_static[22] = ',';
  dtostrf(speedTh,3,2, &debug_msg_static[23]);
  debug_msg_static[26] = ')';
  debug_msg_static[27] = '\0';
}


void ViRHaS::run2(float strafe, float forward, float angular) { 
  reqSpeedX = strafe;
  reqSpeedY = forward;
  reqSpeedTh = angular;
}

void ViRHaS::inverseKinematics(float strafeError, float forwardError, float angularError) {

  speed_wheel[0] = (-robot_radius * angularError + strafeError)/wheel_radius;
  speed_wheel[1] = (-robot_radius * angularError -0.5f*strafeError - 0.866025404f*forwardError)/wheel_radius;
  speed_wheel[2] = (-robot_radius * angularError -0.5f*strafeError + 0.866025404f*forwardError)/wheel_radius;
}

void ViRHaS::stop(void){
  _m1.setSpeed(0);
  _m2.setSpeed(0);
  _m3.setSpeed(0);
  reqSpeedX = 0;
  reqSpeedY = 0;
  reqSpeedTh = 0;
  for(int i=0;i<NMOTOR;i++){
    speed_act[i]=0;
    speed_wheel[i]=0;
    Iterm[i]=0;
  }

}




//determina velocità in cm/s
void ViRHaS::direct_kinematics(void){
     speedTh= -wheel_radius*(speed_act[2]+speed_act[0]+speed_act[1])/(3.0*robot_radius);
     speedX = wheel_radius*(+2.0*speed_act[0] -speed_act[1] - speed_act[2])/3.0;
     speedY = sqrt(3)*wheel_radius*(speed_act[2] - speed_act[1])/3.0;
     

}

void ViRHaS::makeOdometry(unsigned long int deltaT){
  double delta_x = (speedX * cos(posTh) - speedY * sin(posTh)) * (deltaT/1000.0);
  double delta_y = (speedX * sin(posTh) + speedY * cos(posTh)) * (deltaT/1000.0);
  double delta_th = speedTh * (deltaT/1000.0);

  posX+=delta_x;
  posY+=delta_y;
  posTh+=delta_th;


}


int ViRHaS::updatePid(double targetValue, double currentValue, int i)   {// compute PWM value
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  double pidTerm =0;                                                            // PID correction
  double error=0;
  if(i == 2){
    //If it's th we use th PID
    Kp = Kp_th;
    Ki = Ki_th;
    Kd = Kd_th;
  }
  else{
    Kp = Kp_xy;
    Ki = Ki_xy;
    Kd = Kd_xy;
  }
  error = targetValue - currentValue;
  Iterm[i] += error*Ki;
  if(Iterm[i]>255) Iterm[i]=255;
  else if(Iterm[i]<-255) Iterm[i]=-255;
  double deltaError = error - last_error[i];
  pidTerm = (Kp * error) + (Iterm[i]) + (Kd * deltaError);
  last_error[i] = error;
  int output=constrain(int(pidTerm), -255, 255);
  return output;
}



void ViRHaS::getMotorCmS(long deltaT,int pos,int i)  {                                                       
// (Delta Pos * Distance per 1 pulse) / Delta Time
speed_act[i] = ((pos - countAnt[i]) * ((2.0f*(float)M_PI*wheel_radius)/encoder_ppr)) / (deltaT/1000.0f);    
countAnt[i] = pos;
}

void ViRHaS::setEncoderPPR(float ppr){
  encoder_ppr = ppr;
}

void ViRHaS::setWheelRadius(float radius){
  wheel_radius = radius;
}
void ViRHaS::setRobotRadius(float radius){
  robot_radius = radius;
}


double ViRHaS::getPosX(){
  return posX;
}
double ViRHaS::getPosY(){
  return posY;
}
double ViRHaS::getPosTh(){
  return posTh;
}

double ViRHaS::getSpeedX(){
  return speedX;
}
double ViRHaS::getSpeedY(){
  return speedY;
}
double ViRHaS::getSpeedTh(){
  return speedTh;
}

void ViRHaS::setPosX(double _posX){
  posX=_posX;
}
void ViRHaS::setPosY(double _posY){
  posY=_posY;
}
void ViRHaS::setPosTh(double _posTh){
  posTh=_posTh;
}

void ViRHaS::setIterm(int i, double val){
  Iterm[i]=val;
}


void ViRHaS::setKpid_xy(double val, double val1, double val2){
  Kp_xy=val;
  Ki_xy=val1;
  Kd_xy=val2;
}

void ViRHaS::setKpid_th(double val, double val1, double val2){
  Kp_th=val;
  Ki_th=val1;
  Kd_th=val2;
}
