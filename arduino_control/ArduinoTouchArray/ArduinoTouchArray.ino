/*
This code is used to read the data from the pressure sensor and the flex sensors and publish them to the ROS network.
The data is published to the topic touch_data_array, which is an array of touchdata messages.
It is used for the touch sensors positioned on the body of the robot.

NOTE: no calibration is performed in this code, so the data is raw and not calibrated.
*/

// ROS Libraries
#include <ros.h>
#include <triskarino_msgs/touchdataarray.h>

// Pressure sensor: https://docs.rs-online.com/9979/0900766b8138443.pdf
const float ADC_mV = 4.8828125;     // conversion multiplier from Arduino ADC value
const float SensorOffset = 200.0;   // in mV taken from datasheet
const float sensitivity = 4.413;    // in mV/mmH2O taken from datasheet
const float mmH2O_cmH2O = 10;       // convert mmH2O to cmH2O

// Flex sensor
const float R = 10000;              // Voltage divider resistance in Ohm
const float V = 5.0;                // Voltage Arduino
const float Conv = 1023.0;           // Conversion

// Define number of pins for pressure, left flexion, and right flexion
const int numPins = 5;

// Define the pins for pressure, left flexion, and right flexion
const int pressurePins[numPins] = {A1, A0, A2, A15, A11};      // Update pin numbers as per your setup
const int flexSxPins[numPins] = {A12, A10, A8, A7, A9};       // Update pin numbers as per your setup
const int flexDxPins[numPins] = {A5, A13, A6, A4, A3};   // Update pin numbers as per your setup

// ROS initialization: 1 publisher - 1 topic - 1 message
ros::NodeHandle nh;
triskarino_msgs::touchdataarray touch_msg_array;
ros::Publisher touch_system_pub("touch_data_array", &touch_msg_array);


void setup(){
  //Set baud rate
  Serial.begin(38400);
  // Setup node
  nh.getHardware()->setBaud(38400);
  nh.initNode();
  nh.advertise(touch_system_pub);
}

void loop(){
  //Publish sensor data
  publishSensorData();

  nh.spinOnce();
}

void publishSensorData(){ 
  // Initialize arrays to store sensor data
  float pressureValues[numPins];
  float flexSxValues[numPins];
  float flexDxValues[numPins];

  // Read sensor data
  for (int i = 0; i < numPins; i++) {
    pressureValues[i] = getPressureData(pressurePins[i]);
    flexSxValues[i] = getFlexionData(flexDxPins[i]);
    flexDxValues[i] = getFlexionData(flexSxPins[i]);
  }

  // Convert sensor data to arrays
  float pressure_data[] = {pressureValues[0], pressureValues[1], pressureValues[2], pressureValues[3], pressureValues[4]};
  touch_msg_array.pressure_values = pressure_data;
  touch_msg_array.pressure_values_length = 5;

  float flexSx_data[] = {flexSxValues[0], flexSxValues[1], flexSxValues[2], flexSxValues[3], flexSxValues[4]};
  touch_msg_array.flex_sx_values = flexSx_data;
  touch_msg_array.flex_sx_values_length = 6;

  float flexDx_data[] = {flexDxValues[0], flexDxValues[1], flexDxValues[2], flexDxValues[3], flexDxValues[4]};
  touch_msg_array.flex_dx_values = flexDx_data;
  touch_msg_array.flex_dx_values_length = 6;
    
  // Add current time
  touch_msg_array.timestamp = nh.now();
  touch_msg_array.time_float = 0;
  touch_msg_array.time_float_length = 1;

  // Publish the message
  touch_system_pub.publish(&touch_msg_array);
 
}

// Function to get pressure data from a pin
float getPressureData(int pin) {
  float pressureValue = ((analogRead(pin) * ADC_mV - SensorOffset) / sensitivity / mmH2O_cmH2O);
  return pressureValue;
}

// Function to get left flexion data from a pin
float getFlexionData(int pin) {
    float vdx = analogRead(pin) * V / Conv;
    float flexValue = R * (V / vdx - 1);
    return flexValue;
}