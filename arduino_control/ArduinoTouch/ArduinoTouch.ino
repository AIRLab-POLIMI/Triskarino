// ROS Libraries
#include <ros.h>
#include <std_msgs/String.h> //for debugging
#include <triskarino_msgs/touchdata.h>
                      
// Sensor libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Pressure sensor: https://docs.rs-online.com/9979/0900766b8138443.pdf
#define PRESS_PIN A0
const float ADC_mV = 4.8828125;     // conversion multiplier from Arduino ADC value
const float SensorOffset = 200.0;   // in mV taken from datasheet
const float sensitivity = 4.413;    // in mV/mmH2O taken from datasheet
const float mmH2O_cmH2O = 10;       // convert mmH2O to cmH2O

// Flex sensor:
#define FLEX_PIN A1
const float R = 10000;              // Voltage divider resistance in Ohm

// MPU sensor:
sensors_event_t a, g, temp;         // Get ready to read MPU data

Adafruit_MPU6050 mpu;               // Renaming the MPU6050

int16_t AccX, AccY, AccZ, gyroX, gyroY, gyroZ;
float elapsedTime, currentTime, previousTime;
float gyroangleX, gyroangleY;
float pitch_acc, roll_acc, pitch_gyro, roll_gyro, pitch_fusion, roll_fusion;

float pressure, flexion;

// ROS initialization: create a publisher
ros::NodeHandle nh;
triskarino_msgs::touchdata touch_msg;
ros::Publisher touch_pub("touch_data", &touch_msg);

void setup(){
  Serial.begin(9600); //set baud rate

  // Setup for MPU6050
 // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);       // Set bandwidth of the MPU6050
  //mpu.setAccelerometerRange(MPU6050_RANGE_8_G);     // Set the accelerometer range at 8g
  //mpu.setGyroRange(MPU6050_RANGE_1000_DEG);         // Set the gyroscope range at 1000deg

  // Setup node
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(touch_pub);
}

void loop(){
  publishSensorData();
  nh.spinOnce();
  delay(10);
}

void publishSensorData(){
  
  // Get the data
  //getMPUData();
  getPressureData();
  getFlexionData();

  // Get the current time with ros::Time object
  ros::Time current_time = nh.now();

  // Assign the values from the sensors to the message 
  touch_msg.roll_value = 0;
  touch_msg.pitch_value = 0;
  touch_msg.flex_value = flexion; 
  touch_msg.pressure_value = pressure;
  touch_msg.timestamp = current_time;

  // Publish the message
  touch_pub.publish(&touch_msg);
}


// Get the data from the MPU6050
void getMPUData(){
  mpu.getEvent(&a, &g, &temp);

  // Get accelerometer data
  AccX = (int16_t)(a.acceleration.x);
  AccY = (int16_t)(a.acceleration.y);
  AccZ = (int16_t)(a.acceleration.z);
  // Derive roll and pitch from accelerometer
  roll_acc = 180*atan(AccY/sqrt(pow(AccX,2)+pow(AccZ,2)))/M_PI;
  roll_acc = 180*atan(AccX/sqrt(pow(AccY,2)+pow(AccZ,2)))/M_PI;
  // Get gyroscope data
  gyroX = (int16_t)(g.gyro.x)*180/M_PI;
  gyroY = (int16_t)(g.gyro.y)*180/M_PI;
  gyroZ = (int16_t)(g.gyro.z)*180/M_PI;

  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  // Derive angle from gyroscope
  gyroangleX = gyroangleX + (gyroX) * elapsedTime;
  gyroangleY = gyroangleY + (gyroY) * elapsedTime;
  // Fuse accelerometer and gyroscope
  roll_fusion = 0.96 * gyroangleX + 0.04 * roll_acc;
  pitch_fusion = 0.96 * gyroangleY + 0.04 * pitch_acc;
}

// Get the data from tshe pressure sensor
void getPressureData(){
  pressure = (analogRead(PRESS_PIN)*ADC_mV - SensorOffset) / sensitivity / mmH2O_cmH2O;
}

// Get the data from the flex sensor
void getFlexionData(){
  float v = analogRead(FLEX_PIN)*5.0 / 1023.0;
  flexion = R*(5.0 / v-1 );
}
