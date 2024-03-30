#include "FastLED.h"
#include "colorutils.h"
#define PIN1 12

#define NUM_LEDS 288
CRGB leds[NUM_LEDS - 1];

int shiftAmount = 0;

enum LightState {
  // In C++, these are global and don't need LightState prepended
  preStartState,
  autoState,
  teleopNoteIntookState,
  teleopTransportState,
  teleopNoteReadyState,
  teleopEjectingNoteState,
  teleopStaticState,

  lightStateCount
};

enum Alliance {
  blueAlliance,
  redAlliance,

  allianceCount
};

// LightState state = teleopStaticState;
LightState state = autoState;
Alliance alliance = blueAlliance;
uint8_t robotSpeed = 0;

int delayTime = 20;

int frame = 0;

#define PULSE_LUT_ENTRIES 10
uint8_t pulseLUT[PULSE_LUT_ENTRIES];

// // Executes whenever data is recieved from the RoboRIO.
// void onRecieveI2CData(int bytesRead) {
//   Serial.println(bytesRead + " bytes read \n");

//   // Our messages consist of 3 bytes: the light state, the alliance we're on, and the robot speed from 0 to 255.
//   if(Wire.available() < 3) return;
  
//   Serial.println(" Full message\n");
  
//   uint8_t lightState = Wire.read();
//   uint8_t allianceValue = Wire.read();
//   uint8_t speed = Wire.read();
  
//   // Values for light state must be between 0 and (lightStateCount - 1)
//   if(lightState >= lightStateCount) return;
//   // Values for alliance must be between 0 and (allianceCount - 1)
//   if(allianceValue >= allianceCount) return;

//   state = static_cast<LightState>(lightState);
//   alliance = static_cast<Alliance>(allianceValue);
//   robotSpeed = speed;
// }

// Executes whenever data is recieved from the RoboRIO.
void onRecieveSerialData() {
  // Our messages consist of 4 bytes: 0xFF, the light state, the alliance we're on, and the robot speed from 0 to 254.
  while(Serial.read() != 0xFF) {}

  uint8_t lightState = Serial.read();
  uint8_t allianceValue = Serial.read();
  uint8_t speed = Serial.read();
  
  // Values for light state must be between 0 and (lightStateCount - 1)
  if(lightState >= lightStateCount) return;
  // Values for alliance must be between 0 and (allianceCount - 1)
  if(allianceValue >= allianceCount) return;

  state = static_cast<LightState>(lightState);
  alliance = static_cast<Alliance>(allianceValue);
  robotSpeed = speed;

  // Avoid backlog
  while(Serial.available() > 0) Serial.read();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Setup\n");
  
  for(int i = 0; i < PULSE_LUT_ENTRIES; i++) {
    pulseLUT[i] = (uint8_t)((-cos((float)i / PULSE_LUT_ENTRIES * 2 * 3.1415926) * 0.5 + 0.5) * 255);
  }

  // Add LEDS with the WS2812B chipset, to be controlled with "PIN1" pointing to the CRGB array leds, with TypicalLEDStrip color correction
  FastLED.addLeds<WS2812B, PIN1, GRB>(leds, NUM_LEDS).setCorrection( Typical8mmPixel );
}

// For pulsing patterns, gets a fade value from 0 to 255 for the amount it should be faded.
uint8_t getPulseFade() {
  return pulseLUT[frame % PULSE_LUT_ENTRIES];
}

// Gets the multiplier for the color-shifting speed based on the robot speed.
int getShiftColorMultiplier() {
  // Maximum multiplier of 5.1 at robotSpeed 255 and miniumum multiplier of 1 at robotSpeed 0.
  return (int)(1 + ((float)robotSpeed / 25.0));
}

void loop() {
  if(Serial.available() >= 4) {
    // Our messages consist of 4 bytes: 0xFF, the light state, the alliance we're on, and the robot speed from 0 to 254.
    onRecieveSerialData();
  }

  FastLED.setBrightness(100);  // Set brightness to 100%
  frame++;

  int shiftColorMultiplier = getShiftColorMultiplier();

  switch(state) {
    case preStartState: {
      preStartPattern();
      shiftColors(1 * shiftColorMultiplier);
      break;
    }
    case autoState: {
      autoPattern();
      shiftColors(8 * shiftColorMultiplier);
      break;
    }
    case teleopNoteIntookState: {
      teleopNoteIntookPattern();
      shiftColors(8 * shiftColorMultiplier);
      break;
    }
    case teleopNoteReadyState: {
      teleopNoteReadyPattern();
      shiftColors(1 * shiftColorMultiplier);
      break;
    }
    case teleopEjectingNoteState: {
      teleopEjectingNotePattern();
      shiftColors(15 * shiftColorMultiplier);
      break;
    }
    case teleopTransportState: {
      teleopTransportPattern();
      shiftColors(8 * shiftColorMultiplier);
      break;
    }
    case teleopStaticState: {
      teleopStaticPattern();
      shiftColors(5 * shiftColorMultiplier);
      break;
    }
  }
  
  applyGammaCorrection();

  FastLED.show();  // Start leds
  delay(delayTime);  // Wait before updating again
}

void applyGammaCorrection() {
  for(int i = 0; i < (NUM_LEDS); i++) {
    napplyGamma_video(leds[i], 2);
  }
}

void smoothGradient(CRGB color1, CRGB color2) {
  fill_gradient_RGB(leds, 0, color1, floor(NUM_LEDS / 2), color2);
  fill_gradient_RGB(leds, floor(NUM_LEDS / 2), color2, NUM_LEDS, color1);
}

void repeatSequence(struct CRGB *sequence, int sequenceLength) {
  for(int i = 0; i < (NUM_LEDS); i++) {
    leds[i] = sequence[i % sequenceLength];
  }
}

// Start is inclusive, and end is not inclusive
void fill_solid_with_offset(struct CRGB *targetArray, int start, int end, const struct CRGB& color) {
  for(int i = start; i < end; i++) targetArray[i] = color;
}

// Alternating gradient colors used for chase sequences
void chase(CRGB color1, int pixels1, CRGB color2, int pixels2, int space) {
  CRGB sequence[pixels1 + space + pixels2 + space];

  CRGB colorOff = CRGB(0, 0, 0);

  fill_gradient_RGB(sequence, 0, color1, pixels1, colorOff);
  fill_solid_with_offset(sequence, pixels1, pixels1 + space, colorOff);
  fill_gradient_RGB(sequence, pixels1 + space, color2, pixels1 + space + pixels2, colorOff);
  fill_solid_with_offset(sequence, pixels1 + space + pixels2, pixels1 + space + pixels2 + space, colorOff);
  
  repeatSequence(sequence, pixels1 + space + pixels2 + space);
}

// Alternating gradient colors with every other color solid used for chase sequences
void chase_one_gradient(CRGB color1, int pixels1, CRGB color2, int pixels2, int space) {
  CRGB sequence[pixels1 + space + pixels2 + space];

  CRGB colorOff = CRGB(0, 0, 0);

  fill_solid_with_offset(sequence, 0, pixels1, color1);
  fill_solid_with_offset(sequence, pixels1, pixels1 + space, colorOff);
  fill_gradient_RGB(sequence, pixels1 + space, color2, pixels1 + space + pixels2, colorOff);
  fill_solid_with_offset(sequence, pixels1 + space + pixels2, pixels1 + space + pixels2 + space, colorOff);
  
  repeatSequence(sequence, pixels1 + space + pixels2 + space);
}

void rightRotate(CRGB arr[], int d, int n) {
  reverseArray(arr, 0, n-1);
  reverseArray(arr, 0, n-d-1);
  reverseArray(arr, n-d, n-1);
}

void reverseArray(CRGB arr[], int start, int end) {
  int i;
  CRGB temp;
  while(start < end) {
    temp = arr[start];
    arr[start] = arr[end];
    arr[end] = temp;
    start++;
    end--;
  }
}

void shiftColors(int speedMultiplier) {
  shiftAmount = (shiftAmount + speedMultiplier) % NUM_LEDS;

  rightRotate(leds, shiftAmount, NUM_LEDS);
}

// A slightly dark version of the red or blue color, depending on which alliance we're on.
CRGB darkAllianceColor() {
  if(alliance == blueAlliance) {
    return CRGB(0, 0, 50);
  } else {
    return CRGB(50, 0, 0);
  }
}
// A slightly light version of the red or blue color, depending on which alliance we're on.
CRGB lightAllianceColor() {
  if(alliance == blueAlliance) {
    return CRGB(50, 50, 255);
  } else {
    return CRGB(255, 0, 0);
  }
}

////////////////////////////////////////////////// Patterns //////////////////////////////////////////////////

// Before the match starts:
// Alternating stripes with the alliance color
void preStartPattern() {
  chase_one_gradient(darkAllianceColor(), 8, lightAllianceColor(), 8, 5);
}

// During auto:
// Smooth gradient with wave colors
void autoPattern() {
  smoothGradient(CRGB(0, 160, 195), CRGB(115, 255, 38));
}

// When intaking a note in teleop:
// Smooth orange gradient
void teleopNoteIntookPattern() {
  int fade = getPulseFade();
  CRGB intakeColor1 = CRGB(255, 116, 0).fadeLightBy(fade);
  CRGB intakeColor2 = CRGB(190, 80, 5).fadeLightBy(fade);
  smoothGradient(intakeColor1, intakeColor2);
}

// While transporting a note in teleop:
// Smooth purple gradient
void teleopTransportPattern() {
  int fade = getPulseFade();
  CRGB intakeColor1 = CRGB(233, 0, 255).fadeLightBy(fade);
  CRGB intakeColor2 = CRGB(190, 25, 230).fadeLightBy(fade);
  smoothGradient(intakeColor1, intakeColor2);
}

// When note is ready to launch in teleop:
// Alternating stripes with WAVE blue and green
void teleopNoteReadyPattern() {
  chase(CRGB(0, 160, 195), 8, CRGB(115, 255, 38), 8, 5);
}

// When note is being ejected in teleop:
// Rainbow flashing
void teleopEjectingNotePattern() {
  int fade = getPulseFade();
  int value = 255 - fade / 2;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV((int)((float)i / NUM_LEDS * 2000) % 360, 128, value); 
  }
}

// While doing nothing during teleop:
// Smooth gradient with alliance colors
void teleopStaticPattern() {
  smoothGradient(darkAllianceColor(), lightAllianceColor());
}