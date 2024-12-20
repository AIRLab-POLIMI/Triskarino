/*
* This code is used to control the light of the robot. It uses the Adafruit_NeoPixel library to control the LED strip.

* You can set the color of the light, the brightness, and the delay between the color changes. The light can be set to blink, fade, or show a rainbow of colors.
* The ROS message is defined in the triskarino_msgs package and it is called Light. The message contains the following fields:
* - string action: the command to control the light (e.g., "Start", "RB", "A", "B", "Y", "X", "Null")
* - uint8[3] color: the RGB color of the light
*- uint8 brightness: the brightness of the light (0-255)
*-  uint16 delay: the delay between color changes in milliseconds

*/

/*****************************************************************************************************************
* Libraries
*****************************************************************************************************************/
// Arduino Libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <triskarino_msgs/Light.h>

//NeoPixel Libraries
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

/*****************************************************************************************************************
* Definition
*****************************************************************************************************************/
// PIN for strip of leds
#define LED_PIN   13
// N° leds on the strip
#define LED_COUNT 14

// Strip Object
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Strip Brightness
int brightness_default = 50;
// Strip Delay
int pause_time = 100;

// Boolean for Blinking
bool showColor = false;

// Boolean for Fading
bool fading = true;
// Int for Fading
int fading_step = 5;
// Int for Fading
int brightness_command = 50;

// Int for Rainbow
int firstPixelHue = 0;

// Initialize RGB color
uint32_t color = strip.Color(0, 0, 0);

// Initialize Message command
String lastCommand = "Null";

/*****************************************************************************************************************
* ROS Initialization: Publisher & Subscriber
*****************************************************************************************************************/
// ROS Initialization
ros::NodeHandle nh;

// Publisher Node for debugging
std_msgs::String debug_msg;
ros::Publisher msgPub("info_light", &debug_msg);

// Subscriber Node Callback Function
void getLight(const triskarino_msgs::Light& light_msg){
    std::string command = light_msg.action;          // Get the command from the message
    String arduinoString = String(command.c_str());   // Convert the command to String
    
    // Check if the command is different from the last one
    if (arduinoString != lastCommand){
      lastCommand = arduinoString;                    // Update the last command
      
      color = strip.Color(light_msg.color[0], light_msg.color[1], light_msg.color[2]);
      pause_time = light_msg.delay;
      brightness_default = light_msg.brightness;
      strip.setBrightness(brightness_default);
      
      showColor = true;                              // Set the boolean for blinking
      fading = true;                                  // Set the boolean for fading
      
      debug_msg.data = "Received command";
      msgPub.publish(&debug_msg);                    // Publish the debug command
      
    } else{
      lastCommand = "Null";                         // Update the last command to null
      showColor = false;                          // Set the boolean for blinking - additional method for turning off the led                 
    }
}
// Subscriber Node for reading Light Input
ros::Subscriber<triskarino_msgs::Light> lightSub("light", &getLight);

/*****************************************************************************************************************
* Arduino Setup & Loop
*****************************************************************************************************************/
void setup() {
  
  // Set up Serial
  Serial.begin(115200);
  // Set up Hardware
  nh.getHardware()->setBaud(115200);
  
  // Initialize ROS Comunication
  nh.initNode();
  nh.subscribe(lightSub);
  nh.advertise(msgPub);
  
  // Initialize NeoPixel Strip (use user default brightness)
  strip.begin();
  strip.show();
  strip.setBrightness(brightness_default);
}

void loop() {
  
  // If the boolean for showColor is true, start showing the color otherwise turn off the lights
  if (showColor){
    startColor(lastCommand, color, pause_time); // Show the color for a certain amount of time
  } else{
    strip.fill(0, 0, 0);                        // Set the fill color to black (off)
    strip.show();                               // Update the NeoPixel strip with all LEDs off
  }
  
  // Check for new messages
  nh.spinOnce();

}

/*****************************************************************************************************************
* Other Functions
*****************************************************************************************************************/

// Show the color for a certain amount of time
void colorWipe(uint32_t color, int wait) {
    // Fill the strip with the specified color
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color);
    }
    strip.show(); // Update the strip with the specified color
    delay(wait);  // Pause for a moment
}

// Rainbow Function
void rainbow() {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  for(int i=0; i<strip.numPixels(); i++) { 
    // For each pixel in strip...
    int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels()); // Offset pixel hue by an amount to make one full revolution of the color wheel (range of 65536)
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue))); // Set pixel's color (in gamma-corrected form)
  }
  strip.show();                                                      // Update strip with new contents
  delay(10);                                                         // Pause for a moment

  firstPixelHue += 256;
}


// Showing the color for a certain amount of time
void startColor(String lastCommand, uint32_t color, int pause_time) {

  if (lastCommand == "Start"){
    //Command Start: rainbow color - rainbow lights
    rainbow();

  }else if (lastCommand == "RB"){
    //Command RB: user way to turn off the lights
    colorWipe(strip.Color(0, 0, 0), pause_time);

  } else if (lastCommand == "A" || lastCommand == "B" || lastCommand == "Y"){
    //Command A: green color - flashing lights
    //Command B: red color - angry fast flashing lights
    //Command Y: white color - fearful rapidly flashing lights

    colorWipe(color, pause_time);
    colorWipe(strip.Color(0, 0, 0), pause_time);


  } else if (lastCommand == "X"){
    //Command X: blue color - sad slowly fading lights

    colorWipe(color,pause_time);                                      // Fill the strip with blue at max brightness    
    if (fading == true){                                              // If fading is true, decrease the brightness
      brightness_command = max(brightness_command - fading_step, 0);  // Decrease the brightness by the fading_step
      if (brightness_command == 0){                                   // If the brightness is 0, set fading to false (stop increasing the brightness)
        fading = false;
      }
    }else{                                                            // If fading is false, increase the brightness
      brightness_command = min(brightness_command + fading_step, brightness_default); // Increase the brightness by the fading_step
      if (brightness_command == brightness_default){                  // If the brightness is 50, set fading to true (stop decreasing the brightness)
        fading = true;  
      }
    }
    strip.setBrightness(brightness_command);                          // Set the brightness of the strip for the next iteration       

  }else {
    //Command Null: defautl way to turn off the lights
    colorWipe(strip.Color(0, 0, 0), 20); 
  }
}