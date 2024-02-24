#include "FastLED.h"
// include "FastLED.h" - was in original code, probaby not needed
#include "colorutils.h"
#define PIN1 12

// Make an array of RGB values 64 wide
#define NUM_LEDS 53
CRGB leds[NUM_LEDS];

int shiftAmount = 0;

enum LightState {
  // For some odd reason these are global and don't need LightState prepended
  preStartState,
  autoState,
  teleOpNoteIntookState,
  teleOpNoteReadyState,
  teleOpStaticState,

};

LightState state = preStartState;
// LightState state = autoState;

void setup() {
  Serial.begin(9600);
  delay(2000);  // Delay because the original had it

  // Add LEDS with the WS2811 chipset, to be controlled with "PIN1" pointing to the CRGB array leds, with TypicalLEDStrip color correction
  FastLED.addLeds<WS2811, PIN1, GRB>(leds, NUM_LEDS);  //.setCorrection( TypicalLEDStrip );
}

void loop() {
  FastLED.setBrightness(100);  // Set brightness to 100%
  if (state == preStartState) {
    defaultPattern();
  } else if (state == autoState){
    autoPattern();
  }

  shiftColors();
  
  FastLED.show();  //start leds
  delay(5);  // Wait 7ms before updating again (don't slow down)
}

void defaultPattern() {
  CRGB WaveBlue = CRGB(0, 160, 195);    // ACCURATE :)
  CRGB WaveGreen = CRGB(115, 255, 38);  // OFFBRAND

  //Fill with a gradient starting with WaveBlue, ending with Wave Green.
  fill_gradient_RGB(leds, 0, WaveBlue, floor(NUM_LEDS / 2), WaveGreen);
  fill_gradient_RGB(leds, floor(NUM_LEDS / 2), WaveGreen, NUM_LEDS, WaveBlue);
}

void autoPattern() {
  CRGB red = CRGB(255, 0, 0);
  CRGB blue = CRGB(0, 0, 255);
  fill_gradient_RGB(leds, 0, red, floor(NUM_LEDS / 2), blue);
  fill_gradient_RGB(leds, floor(NUM_LEDS / 2), blue, NUM_LEDS, red);
}

void teleopNoteIntookPattern() {
  CRGB red = CRGB(255, 255, 0);
  CRGB blue = CRGB(100, 100, 0);
  fill_gradient_RGB(leds, 0, red, NUM_LEDS, blue);  // (LEDarray, Startpos, StartColor, Endpos, endcolor)
}

void shiftColors() {
  shiftAmount = (shiftAmount + 1) % NUM_LEDS;

  CRGB newleds[NUM_LEDS];
  for(int i = 0; i < (NUM_LEDS); i++) {
    newleds[i] = leds[(i + shiftAmount) % NUM_LEDS];
  }

  for(int i = 0; i < (NUM_LEDS); i++) {
    leds[i] = newleds[i];
  }
  free(newleds);
}

// void setSolidColor(int R, int G, int B) {
//   for (int i = 0; i < (NUM_LEDS); i++) {
//     leds[i] = CRGB::Blue;
//   }
// }
