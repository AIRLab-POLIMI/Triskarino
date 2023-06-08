/*****************************************************************************************************************
 * Description: This code stablish a connection with the raspberry pi and depending on the action that this one *
 * sets, then the system will set the corresponding light
 * ***************************************************************************************************************
 * Author: Ane San Martin Igarza                                                                                 *
 * ***************************************************************************************************************
 * Last Update: 23/06/2020                                                                                       *
 * **************************************************************************************************************/
// define the library needed
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// DEfine the pin used for the strip of leds
#define LED_PIN   13

// Number of leds on the strip
#define LED_COUNT 14

// Declaramos el objeto TIRA NEOPIXEL:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//Variables needed to read data reliably
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

//Json that will continuosly be updated with action, color and wait
StaticJsonDocument<48> action_msg;


void setup() {
    Serial.begin(115200);
// This line is specified in order that Adafruit Trinket supports 5V 16 MHz.
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  strip.begin();           // initialize the STRIP NEOPIXEL(REQUIRED)
  strip.show();            // TURN OFF the STRIP
  strip.setBrightness(50); // set the brightness (aproximatly al 1/5 (max = 255))
}


void loop() {
  //received the action that must be performed from the Raspberry board
   recvWithStartEndMarkers();

  if (newData == true){
    fillActionMsg();
    do_action();
    newData = false;
  }
}

uint32_t get_color(){
    return strip.Color(action_msg["color"][0],action_msg["color"][1],action_msg["color"][2]);
}
void do_action(){
    if(action_msg["action"] == "colorWipe")
        colorWipe(get_color(),action_msg["wait"]);
    else if action_msg["action"] == "rainbow"
        rainbow(action_msg["wait"]);
    else if action_msg["action"] == "rainbowane":
        rainbowane(wait);
    else if action_msg["action"] == "off":
        turn_off();

}

void fillActionMsg(){
    Serial.println(receivedChars)
    const auto deser_err = deserializeJson(action_msg, receivedChars);
    // Test if parsing succeeds.
    if (deser_err) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.print(deser_err.f_str());
        Serial.print(" ");
        Serial.println(receivedChars);

     }
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



//set each pixel to a different color
void rainbowane(int wait){
for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i,strip.Color(random(0,255), random(0,255), random(0,255)));// it must be turned on one by one
    strip.show();// Actualize the strip
    delay(wait);// set a small delay
  }
}

void turn_off(){
  for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i,strip.Color(0, 0, 0));        
    strip.show();                          
    delay(10);                          
  }
  }
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // set a fixed color to each led of the strip
    strip.setPixelColor(i, color);         
    strip.show();                          // Actualize the strip
    delay(wait);                           //make a small delay
  }
}


// Ciclo del arco iris a lo largo de toda la tira. 
// Pasa el tiempo de retraso (en ms) entre cuadros.
void rainbow(int wait) {
// El tono del primer píxel ejecuta 5 bucles completos a través de la rueda de colores. 
// La rueda de colores tiene un rango de 65536 pero está OK si le damos la vuelta, así que solo cuenta de 0 a 5 * 65536. 
// Agregar 256 a firstPixelHue cada vez, significa que haremos 5 * 65536/256 = 1280 pases a través de este bucle externo:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { 
      // Para cada píxel en la tira... 
      // Compense el tono del píxel en una cantidad para hacer una revolución completa 
      // de la rueda de colores (rango de 65536) a lo largo de la tira (strip.numPixels() pasos)  
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}


