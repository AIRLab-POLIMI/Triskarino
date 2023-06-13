#include <NewPing.h>
#include <CytronMotorDriver.h>
#include <Encoder.h>
#include <ViRHas.h>
#include <ros.h>
#include <triskarino_msgs/RawOdometry.h>
#include <triskarino_msgs/Sonar.h>
#include <geometry_msgs/Twist.h>
// SONARS

#define SONAR_NUM 4 
#define MAX_DISTANCE 300 // Max distance returned


unsigned long pingTimer[SONAR_NUM];

float dist[SONAR_NUM]; // Final distance communicated {F, R, L, B}
float cm[SONAR_NUM]; //Filtered sample
float cm_prec[SONAR_NUM]; // Filtered sample
float cm_raw[SONAR_NUM]; // Last raw
float cm_prec1[SONAR_NUM]; // second last raw etc..
float cm_prec2[SONAR_NUM];
float cm_prec3[SONAR_NUM];
float cm_prec4[SONAR_NUM];
float minDist = MAX_DISTANCE; //Support to calculate which sensor is working
int numActualData;

int minIndex;

const int trigPin = 32;
const int echoPin = 30;

const int trigPin1 = 28;
const int echoPin1 = 26;

const int trigPin2 = 38;
const int echoPin2 = 36;

const int trigPin3 = 44;
const int echoPin3 = 42;

NewPing sonar[SONAR_NUM] = {
  NewPing(trigPin2, echoPin2, MAX_DISTANCE), //Front sonar
  NewPing(trigPin1, echoPin1, MAX_DISTANCE), //Right sonar
  NewPing(trigPin, echoPin, MAX_DISTANCE), //left sonar
  NewPing(trigPin3, echoPin3, MAX_DISTANCE), //back sonar  
  };



//MOTORS 
//input pins
#define _EP11 19
#define _EP12 18
#define _EP31 21
#define _EP32 20
#define _EP21 2
#define _EP22 3

//driver pins
#define _1_1A 8
#define _1_1B 7
#define _1_2A 5
#define _1_2B 4
#define _2_1A 12
#define _2_1B 11

//Maximum speed wantd
#define _MAX_SPEED 80 //cm/s
#define _MAX_ANGULAR 6.28//rad/s

// Configure the motor driver.
CytronMD motor1(PWM_PWM, _1_1A, _1_1B); // PWM 2A = Pin 8, PWM 2B = Pin 7. Motor 1 : right robot
CytronMD motor2(PWM_PWM, _2_1A, _2_1B);   // PWM 1A = Pin 12, PWM 1B = Pin 11. Motor 2 : Atras
CytronMD motor3(PWM_PWM, _1_2A, _1_2B); // PWM 2A = Pin 5, PWM 2B = Pin 4. Motor 3 : left robot

//enable the encoders and set each eancoder of each sensor to which pin is connectes. take into coount that the
//order has to be Motor1,Motor2,Motor3. The order used in the motor has to joing with this one
Encoder ENCODER[] = { Encoder(_EP11, _EP12), Encoder(_EP21, _EP22), Encoder(_EP31, _EP32)};

//robot class
ViRHaS virhas = ViRHaS(motor1, motor2, motor3, ENCODER[0], ENCODER[1], ENCODER[2]);

//Uses the contents of the twist message to move the robot
void moveRobot(const geometry_msgs::Twist& twist_msg){
  if(twist_msg.linear.x == 0.0 && twist_msg.linear.y == 0.0 && twist_msg.angular.z == 0.0){
    virhas.stop();
  }else{
    virhas.run2(twist_msg.linear.y * _MAX_SPEED, twist_msg.linear.x * _MAX_SPEED, twist_msg.angular.z * _MAX_ANGULAR);
    virhas.PIDLoop();
  }
  publishSensorMsg();
  
}

<<<<<<< HEAD
//Json that will continuosly be updated to send odometry and sonar information over Serial
StaticJsonDocument<128> sensor_msg;
JsonArray sonarData = sensor_msg.createNestedArray("sonarData");
JsonArray odometryPos = sensor_msg.createNestedArray("odometryPos");
JsonArray odometryVel = sensor_msg.createNestedArray("odometryVel");

void setup() {
  initializeSensorMsg();
=======
ros::NodeHandle nh;
triskarino_msgs::RawOdometry odom_msg;
ros::Publisher odomPub("rawOdometry", &odom_msg);
triskarino_msgs::Sonar sonar_msg;
ros::Publisher sonarPub("sonar", &sonar_msg);
ros::Subscriber<geometry_msgs::Twist> twistSub("cmd_vel", &moveRobot );

void setup() {
>>>>>>> refs/remotes/origin/main
  virhas.setKpid(2, 1, 0.7);
  virhas.stop();
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(twistSub);
  nh.advertise(odomPub);
  nh.advertise(sonarPub);
  
}

void loop() {

<<<<<<< HEAD
  recvWithStartEndMarkers();

  if (newData == true){
     //Reading data from SONAR
     getSonarData();
     // I have cm[i] filled with information at this point, let's put it in the Json
     fillSonarMsg();
     //If twist message is equal to the default one, the robot does not move
     if(speedX == 0.0 & speedY == 0.0 & speedTh == 0.0){
       virhas.stop();
     }else{
       moveRobot();
    }

    fillOdometryMsg();
    serializeJson(sensor_msg, Serial);
    Serial.write("\n");
    newData = false;
  }
  
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    static byte valueCounter = 0;
    char startMarker = ':';
    char separator = ',';
    char endMarker = '\n';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
          //Accumulate characters in the buffer till the separator appears
            if (rc != separator) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else if (rc == separator) {
              //When the separator appears, put the received number in the respective speed by the value counter
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = true;
              ndx = 0;
              switch (valueCounter){
                case 0: speedX = atof(receivedChars);
                        break;
                case 1: speedY = atof(receivedChars);
                        break;
                case 2: speedTh = atof(receivedChars);
                        break;
              }
              valueCounter++;  
            }
            else if (rc == endMarker){
              //When the endMarker appears exit from the loop and the speed variables should be full
              newData = true;
              valueCounter = 0;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void initializeSensorMsg(){
  sonarData.add(0.0);
  sonarData.add(0.0);
  sonarData.add(0.0);
  sonarData.add(0.0);
  odometryPos.add(0.0);
  odometryPos.add(0.0);
  odometryPos.add(0.0);
  odometryVel.add(0.0);
  odometryVel.add(0.0);
  odometryVel.add(0.0);
}



//Uses the contents of the twist message to move the robot
void moveRobot(){
 
  virhas.run2(speedY * _MAX_SPEED, speedX * _MAX_SPEED, speedTh * _MAX_ANGULAR);
  virhas.PIDLoop();
}


=======
 nh.spinOnce();
  
}




void publishSensorMsg(){
  getSonarData();
  fillSonarMsg();
  fillOdometryMsg();
  sonarPub.publish(&sonar_msg);
  odomPub.publish(&odom_msg);
}


>>>>>>> refs/remotes/origin/main

void fillOdometryMsg(){
  odom_msg.odometryPos[0] = virhas.getPosX();
  odom_msg.odometryPos[1] = virhas.getPosY();
  odom_msg.odometryPos[2] = virhas.getPosTh();
  odom_msg.odometryVel[0] = virhas.getSpeedX();
  odom_msg.odometryVel[1] = virhas.getSpeedY();
  odom_msg.odometryVel[2] = virhas.getSpeedTh();
}

void fillSonarMsg(){
 for(uint8_t i = 0; i < SONAR_NUM; i++){
  sonar_msg.distances[i]=cm[i];
 }
}



void getSonarData(){
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    
    cm_prec4[i] = cm_prec3[i];
    cm_prec3[i] = cm_prec2[i];
    cm_prec2[i] = cm_prec1[i];
    cm_prec1[i] = cm_raw[i];
    cm_prec[i] = cm[i];
  }
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    cm_raw[i] = sonar[i].ping_cm();
    numActualData = 0;
    if(cm_raw[i] == 0){
      if(cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i] == 0) cm[i] = MAX_DISTANCE;  //If it's 0 (ping not returned) and all the precedent are zero, then is a real zero
      else{
        numActualData = 0;
        if (cm_prec1[i] != 0) numActualData++;
        if (cm_prec2[i] != 0) numActualData++;
        if (cm_prec3[i] != 0) numActualData++;
        if (cm_prec4[i] != 0) numActualData++;
        cm[i] = (cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i])/ numActualData; //If it's 0 but there i data in the "buffer" than the real result is the mean of the buffer
      }
    }
    else{
      if (cm_prec1[i] != 0) numActualData++;
      if (cm_prec2[i] != 0) numActualData++;
      if (cm_prec3[i] != 0) numActualData++;
      if (cm_prec4[i] != 0) numActualData++;
      cm[i] = (cm_raw[i] + cm_prec1[i] + cm_prec2[i] + cm_prec3[i] + cm_prec4[i])/(numActualData+1);      
    }
  }
}
