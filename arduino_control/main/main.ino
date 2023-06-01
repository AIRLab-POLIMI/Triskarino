#include <NewPing.h>
#include <CytronMotorDriver.h>
#include <Encoder.h>
#include <ArduinoJson.h>
#include <ViRHas.h>


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

//Variables needed to read data reliably
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Configure the motor driver.
CytronMD motor1(PWM_PWM, _1_1A, _1_1B); // PWM 2A = Pin 8, PWM 2B = Pin 7. Motor 1 : right robot
CytronMD motor2(PWM_PWM, _2_1A, _2_1B);   // PWM 1A = Pin 12, PWM 1B = Pin 11. Motor 2 : Atras
CytronMD motor3(PWM_PWM, _1_2A, _1_2B); // PWM 2A = Pin 5, PWM 2B = Pin 4. Motor 3 : left robot

//enable the encoders and set each eancoder of each sensor to which pin is connectes. take into coount that the
//order has to be Motor1,Motor2,Motor3. The order used in the motor has to joing with this one
Encoder ENCODER[] = { Encoder(_EP11, _EP12), Encoder(_EP21, _EP22), Encoder(_EP31, _EP32)};

//robot class
ViRHaS virhas = ViRHaS(motor1, motor2, motor3, ENCODER[0], ENCODER[1], ENCODER[2]);


//Json that will continuosly be updated to send odometry and sonar information over Serial
StaticJsonDocument<128> sensor_msg;
JsonArray sonarData = sensor_msg.createNestedArray("sonarData");
JsonArray odometryPos = sensor_msg.createNestedArray("odometryPos");
JsonArray odometryVel = sensor_msg.createNestedArray("odometryVel");
//Json that will continuosly be updated with the twist messages
StaticJsonDocument<32> twist_msg;
JsonArray twistData = twist_msg.createNestedArray("twist");

void setup() {
  initializeSensorMsg();
  initializeTwistMsg();
  virhas.setKpid(2, 1, 0.7);
  virhas.stop();
  Serial.begin(115200);
  
}

void loop() {
  
  // Reading data from SONAR
  getSonarData();

  // I have cm[i] filled with information at this point, let's put it in the Json
  fillSonarMsg();
  
  recvWithStartEndMarkers();
  fillTwistMsg();
  float speedY = twistData[1];
  float speedX = twistData[0];
  float speedTh = twistData[2];
  //If twist message is equal to the default one, the robot does not move
  if(speedX == 0.0 & speedY == 0.0 & speedTh == 0.0){
    virhas.stop();
  }else{
    moveRobot();
  }

  fillOdometryMsg();
  //serializeJson(sensor_msg, Serial);
  //Serial.write("\n");
 
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '{';
    char endMarker = '}';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = rc;
                ndx++;
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
            receivedChars[ndx] = rc;
            ndx++;
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


void initializeTwistMsg(){
  twistData.add(0.0);
  twistData.add(0.0);
  twistData.add(0.0);
 
}

void resetTwistMsg(){
  twistData[0] = 0.0;
  twistData[1] = 0.0;
  twistData[2] = 0.0;
}


//Uses the contents of the twist message to move the robot
void moveRobot(){
  float speedY = twistData[1];
  float speedX = twistData[0];
  float speedTh = twistData[2];
  virhas.run2(speedY * _MAX_SPEED * 0.01, speedX * _MAX_SPEED * 0.01, speedTh * _MAX_ANGULAR * 0.01);
  virhas.PIDLoop();
}



void fillTwistMsg(){

   if (newData == true) {
      Serial.println(receivedChars);
      const auto deser_err = deserializeJson(twist_msg, receivedChars);
      // Test if parsing succeeds.
      if (deser_err) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(deser_err.f_str());
        return;
      }
      newData = false;
    }
}


void fillOdometryMsg(){
  odometryPos[0] = virhas.getPosX();
  odometryPos[1] = virhas.getPosY();
  odometryPos[2] = virhas.getPosTh();
  odometryVel[0] = virhas.getSpeedX();
  odometryVel[1] = virhas.getSpeedY();
  odometryVel[2] = virhas.getSpeedTh();
}

void fillSonarMsg(){
 for(uint8_t i = 0; i < SONAR_NUM; i++){
  sonarData[i]=cm[i];
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
