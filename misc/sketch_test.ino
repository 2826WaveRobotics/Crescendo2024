#include "FastLED.h"
// include "FastLED.h" - was in original code, probaby not needed
#include "colorutils.h"
#define PIN1 12

// Make an array of RGB values 64 wide
#define NUM_LEDS 76
CRGB leds[NUM_LEDS -1];

int StartingLED = 0;
int EndingLED = NUM_LEDS;
String LightState = "default";

void setup() {
  Serial.begin(9600);
  delay(2000); // Delay because the original had it
  
  // Add LEDS with the WS2811 chipset, to be controlled with "PIN1" pointing to the CRGB array leds, with TypicalLEDStrip color correction
  FastLED.addLeds<WS2811, PIN1, GRB>(leds, NUM_LEDS); //.setCorrection( TypicalLEDStrip );
}

void loop() {
  FastLED.setBrightness(100); // Set brightness to 100% 
  
  CRGB WaveBlue = CRGB(0, 160, 195); // ACCURATE :)
  CRGB WaveGreen = CRGB(115,255,38); // OFFBRAND
  
  //Fill with a gradient starting with WaveBlue, ending with Wave Green.
  fill_gradient_RGB(leds, StartingLED, WaveBlue, EndingLED, WaveGreen); // (LEDarray, Startpos, StartColor, Endpos, endcolor)
  if (LightState == "default"){
    shiftColors();
  }

  
  FastLED.show(); //start leds
  
  delay(14); // Wait 7ms before updating again (don't slow down)
}

//Shift colors
void shiftColors() {  //Shifts gradient 
  
  if (StartingLED == NUM_LEDS) {
  
  StartingLED = 0;
  
  } else {
    
  StartingLED = StartingLED + 2;
  
  }
  
   if (EndingLED == NUM_LEDS) {
  
  EndingLED = 0;
  
  } else {
    
  EndingLED = EndingLED + 1;
  
  }
  
   
  
}

// void setSolidColor(int R, int G, int B) {
//   for (int i = 0; i < (NUM_LEDS); i++) {
//     leds[i] = CRGB::Blue;
//   }
// }
