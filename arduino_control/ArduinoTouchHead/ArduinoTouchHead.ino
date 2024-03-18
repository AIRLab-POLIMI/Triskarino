/*
This code is used to read the data from the pressure sensor and the flex sensors and publish them to the ROS network.
It is similar to the ArduinoTouchArray.ino code, but it publishes the data from the head sensors to the ROS network.

NOTE: no calibration is performed in this code, so the data is raw and not calibrated.

*/


// ROS Libraries
#include <ros.h>
#include <triskarino_msgs/touchdata.h>

// Pressure sensor: https://docs.rs-online.com/9979/0900766b8138443.pdf
const float ADC_mV = 4.8828125;     // conversion multiplier from Arduino ADC value
const float SensorOffset = 200.0;   // in mV taken from datasheet
const float sensitivity = 4.413;    // in mV/mmH2O taken from datasheet
const float mmH2O_cmH2O = 10;       // convert mmH2O to cmH2O

// Flex sensor
const float R = 10000;              // Voltage divider resistance in Ohm
const float V = 5.0;                // Voltage Arduino
const float Conv = 1023.0;           // Conversion

// ROS initialization
ros::NodeHandle nh;
triskarino_msgs::touchdata touch_msg;
ros::Publisher touch_head("touch_data_head", &touch_msg);



void setup(){
  //Set baud rate
  Serial.begin(9600);
  // Setup node
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(touch_head);

}

void loop(){
  // Head Sensors
  publishSensorData();
  nh.spinOnce();
}

void publishSensorData(){

  // Get the values to publish
  float pressure = getPressureData(A0);                      //get Pressure
  float flexionSx = getFlexionData(A1);                   //get FlexSx
  float flexionDx = getFlexionData(A2);                   //get FlexDx

  // Get the current time with ros::Time object
  ros::Time current_time = nh.now();

  // Assign the values to the ROS message
  touch_msg.flexSx_value = flexionSx;
  touch_msg.flexDx_value = flexionDx;  
  touch_msg.pressure_value = pressure;
  touch_msg.timestamp = current_time;
  
  // Publish the message
  touch_head.publish(&touch_msg);
}


// Get the data from tshe pressure sensor
float getPressureData(int pin){
    float pressureValue = ((analogRead(pin) * ADC_mV - SensorOffset) / sensitivity / mmH2O_cmH2O);
    return pressureValue;
}


// Get the data from the left flex sensor
float getFlexionData(int pin){
    float vsx = analogRead(pin) * V / Conv;
    float flexValue = R * (V / vsx - 1);
    return flexValue;  // Return the calculated flexion value
}