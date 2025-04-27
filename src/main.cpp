#include <Arduino.h>
#include <BlendableColor.h>
#include <ColorConverterLib.h>
#include <EEPROM.h>

#if defined ARDUINO_AVR_UNO
  #define R_PIN 11
  #define G_PIN 10
  #define B_PIN 9
  #define BRIGHTNESS_PIN A0
  #define SPEED_PIN A1
  #define CHANGE_PROGRAM_BUTTON_PIN A2

#elif defined ARDUINO_AVR_PRO
  #define R_PIN 3
  #define G_PIN 5
  #define B_PIN 6
  #define BRIGHTNESS_PIN A0
  #define SPEED_PIN A1
  #define CHANGE_PROGRAM_BUTTON_PIN A2

#elif defined ARDUINO_AVR_NANO
  #define R_PIN 3
  #define G_PIN 5
  #define B_PIN 6
  #define BRIGHTNESS_PIN A0
  #define SPEED_PIN A1
  #define CHANGE_PROGRAM_BUTTON_PIN A2
#endif

#define ENABLE_EEPROM_RW 1

#define CHANNEL_TEST 0

const int inputHysteresis = 5;

int lastAckedSpeedInput = 0;
int lastAckedBrightnessInput = 0;

bool bChangeProgramButtonIsPressed = false;

float blendSpeed = 0.0001;

void OnColorBlendFinished();

BlendableColor CurrentColor(Color(255, 0, 0), Color(0, 255, 255), 0, 0.001, OnColorBlendFinished);

uint8_t currentProgramIdx = 0;
uint8_t currentColorIndex = 0;


Color Programs3Color[3][3] = {
  {Color(255, 0, 0), Color(255, 0, 255), Color(0, 255, 255)}, // 0
  {Color(0, 255, 0), Color(255, 255, 0), Color(0, 255, 255)}, // 1
  {Color(0, 128, 255), Color(128, 0, 255), Color(0, 0, 255)}  // 2
  
};

Color Programs2Color[3][2] = {
  {Color(255, 60, 0), Color(255, 0, 0)},                      // 3
  {Color(0, 0, 255), Color(0, 150, 255)},                     // 4
  {Color(255, 0, 100), Color(100, 0, 255)}                    // 5
};

// Color TestProgram[5] = {
//   Color(255, 60, 0), Color(128, 66, 0), Color(128, 66, 0), Color(0, 0, 255), Color(0, 0, 255)
// };

#if CHANNEL_TEST
void channelTest() {
  while(1) {
    Serial.println("RED");
    digitalWrite(R_PIN, HIGH);
    digitalWrite(G_PIN, LOW);
    digitalWrite(B_PIN, LOW);
    delay(2000);
    
    Serial.println("GREEN");
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, HIGH);
    digitalWrite(B_PIN, LOW);
    delay(2000);

    Serial.println("BLUE");
    digitalWrite(R_PIN, LOW);
    digitalWrite(G_PIN, LOW);
    digitalWrite(B_PIN, HIGH);
    delay(2000);
  }
}
#endif

void OnColorBlendFinished() {

  // DEBUG
  // currentColorIndex++;
  // currentColorIndex %= 5;

  // Serial.println("OnColorBlendFinished");
  // Serial.print("currentColorIndex = ");
  // Serial.println(currentColorIndex);
  // CurrentColor.setDestination(TestProgram[currentColorIndex]);
  // return;
  // -------------------




  if(currentProgramIdx == 6) {
    // Custom color, do nothing
  }
  else if(currentProgramIdx == 7) {
    // Random, set new random color
    uint8_t r, g, b;
    RGBConverter::HsvToRgb(random(0, 1000) / 1000.0, 1, 1, r, g, b);
    CurrentColor.setDestination(Color(r, g, b));
  }
  else {
    // Advance to next color in program
    currentColorIndex++;
    if(currentProgramIdx > 2) {
      // 2-color program
      if(currentColorIndex > 1) {
        currentColorIndex = 0;
      }
      CurrentColor.setDestination(Programs2Color[currentProgramIdx-3][currentColorIndex]);
    }
    else {
      // 3-color program
      if(currentColorIndex > 2) {
        currentColorIndex = 0;
      }
      
      CurrentColor.setDestination(Programs3Color[currentProgramIdx][currentColorIndex]);
    }
  }
}

int readMappedBrightness() {
  return map(analogRead(BRIGHTNESS_PIN), 0, 1024, 128, 1024);
}

int readMappedSpeed(bool bRemap = true) {
  
  return map(analogRead(SPEED_PIN), 0, 1024, (bRemap ? 128 : 0), 1024);
}

void onChangeProgramPressed() {
  currentColorIndex = 0;
  currentProgramIdx = (currentProgramIdx + 1) % 8;

  if(currentProgramIdx < 6) {
    if(currentProgramIdx > 2) {
      // 2-color program
      CurrentColor.snap(Programs2Color[currentProgramIdx][0]);
    }
    else {
      // 3-color program
      CurrentColor.snap(Programs3Color[currentProgramIdx][0]);
    }
  }

  // Random, snap to red 
  if(currentProgramIdx == 7) {
    CurrentColor.snap(Color(255, 0, 0));
  }

  Serial.print("Program: ");
  Serial.println(currentProgramIdx);

#if ENABLE_EEPROM_RW
  EEPROM.write(0, currentProgramIdx);
#endif
}

void updateChangeProgramInput() {
  int currentButtonInput = analogRead(CHANGE_PROGRAM_BUTTON_PIN);

  if(!bChangeProgramButtonIsPressed && currentButtonInput < 128) {
    bChangeProgramButtonIsPressed = true;
    onChangeProgramPressed();
  }
  else if(bChangeProgramButtonIsPressed && currentButtonInput > 896) {
    bChangeProgramButtonIsPressed = false;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);

  pinMode(BRIGHTNESS_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  pinMode(CHANGE_PROGRAM_BUTTON_PIN, INPUT);

  // Set PWM frequency
  // Pins D9 and D10 - 4 kHz
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000010; // x8 phase correct

  // Pins D3 and D11 - 4 kHz
  TCCR2B = 0b00000010; // x8
  TCCR2A = 0b00000001; // phase correct

  lastAckedSpeedInput = readMappedSpeed();
  lastAckedBrightnessInput = readMappedBrightness();

#if ENABLE_EEPROM_RW
  currentProgramIdx = EEPROM.read(0);
#endif

  if(currentProgramIdx < 6) {
    if(currentProgramIdx < 3)
      CurrentColor.snap(Programs3Color[currentProgramIdx][0]);
    else
      CurrentColor.snap(Programs2Color[currentProgramIdx-3][0]);
  }
  //CurrentColor.snap(TestProgram[0]);
}

void loop() {

#if CHANNEL_TEST
  channelTest();
#endif

  updateChangeProgramInput();

  const int currentSpeedInput = readMappedSpeed(currentProgramIdx != 6);
  if(abs(currentSpeedInput - lastAckedSpeedInput) > inputHysteresis) {
    lastAckedSpeedInput = currentSpeedInput;
  }
  const float speedInput = lastAckedSpeedInput/16 + 2;

  const int currentBrightnessInput = readMappedBrightness();
  if(abs(currentBrightnessInput - lastAckedBrightnessInput) > inputHysteresis) {
    lastAckedBrightnessInput = currentBrightnessInput;
  }
  const float brightnessInput = lastAckedBrightnessInput * (1.0 / 1024.0);

  // Serial.print("Speed: ");
  // Serial.println(currentSpeedInput);
  // Serial.print("Brightness: ");
  // Serial.println(currentBrightnessInput);

  Color finalColor;
  if(currentProgramIdx == 6) {
    // Custom color; treat speed input as color wheel input
    uint8_t r, g, b;

    RGBConverter::HsvToRgb(currentSpeedInput / 1024.0, 1, 1, r, g, b);
    finalColor.r = r;
    finalColor.g = g;
    finalColor.b = b;
  }
  else {
    CurrentColor.setBlendSpeed(0.00001*speedInput);
    CurrentColor.update();
    
    finalColor = CurrentColor.getCurrent();
  }

  finalColor *= brightnessInput;
  analogWrite(R_PIN, finalColor.r);
  analogWrite(G_PIN, finalColor.g);
  analogWrite(B_PIN, finalColor.b);

  // Serial.print("R: ");
  // Serial.print(finalColor.r);
  // Serial.print(" G: ");
  // Serial.print(finalColor.g);
  // Serial.print(" B: ");
  // Serial.println(finalColor.b);

  delay(1);
}
