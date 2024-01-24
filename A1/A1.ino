#include "src/pitches/pitches.h"
#include "src/RGBConverter/RGBConverter.h"

// Clock
unsigned int currentTime = 0;

// Photoresistor
int photoPin = A0;
int photoLevel = 0;
float brightness = 0;

void photoToBrightness()
{
  photoLevel = analogRead(photoPin);
  int ledLevel = map(photoLevel, 200, 1023, 0, 255);
  ledLevel = constrain(ledLevel, 0, 255);
  brightness = ledLevel / 255.0;
}

// RGB LED
int redPin = 6;
int greenPin = 9;
int bluePin = 3;

RGBConverter _rgbConverter;
int hue_interval = 20; // interval in ms between incrementing hues
byte rgb_max = 255;

byte rgb[3];
int lastFadeTimestamp = 0;

float hue = 0; //hue varies between 0 - 1
float step = 0.001f;
 

void setColor(int red, int green, int blue, float brightness = 1)
{
  analogWrite(redPin, red * brightness);
  analogWrite(greenPin, green * brightness);
  analogWrite(bluePin, blue * brightness);  
}

void crossFade(float brightness=1)
{
  if (currentTime - lastFadeTimestamp > hue_interval)
  {
    _rgbConverter.hslToRgb(hue, 1, 0.5, rgb);
    setColor(rgb[0], rgb[1], rgb[2], brightness); 
    hue += step;

    if(hue > 1.0){
      hue = 0;
    }

    lastFadeTimestamp = currentTime;
  }
}

void analogHue(int analogSignal) {
  hue = analogSignal / 1023.0;
  _rgbConverter.hslToRgb(hue, 1, 0.5, rgb);
  setColor(rgb[0], rgb[1], rgb[2], 1);
}

void writeRed(float brightness = 1) {
  analogWrite(redPin, 255 * brightness);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void writeGreen(float brightness = 1) {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255 * brightness);
  analogWrite(bluePin, 0);
}

void writeBlue(float brightness = 1) {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255 * brightness);
}

// Microphone
int micPin = A1;
double micVoltage = 0;
unsigned int lastVoltageTimestamp = 0;
unsigned int signalMax = 0;
unsigned int signalMin = 1024;

unsigned int claps = 0;
int detectingClaps = 0;
int lastClapTimestamp = 0;

void processMicrophone()
{
  int sample = analogRead(micPin);
  if (sample < 1024)
  {
    if (sample > signalMax)
    {
      signalMax = sample;
    }

    if (sample < signalMin)
    {
      signalMin = sample;
    }
  }

  if (currentTime - lastVoltageTimestamp > 50) {
    int peakToPeak = signalMax - signalMin;
    micVoltage = (peakToPeak * 5.0) / 1024;
    signalMax = 0;
    signalMin = 1024;
    lastVoltageTimestamp = currentTime;
    if (detectingClaps)
    {
      processClaps();
    }
  }
}

void processClaps()
{
  if (micVoltage > 2.3)
  {
    if (currentTime - lastClapTimestamp > 500)
    {
      claps++;
      int color = claps % 3;
      if (color == 0)
      {
        writeRed();
      }
      else if (color == 1)
      {
        writeGreen();
      }
      else if (color == 2)
      {
        writeBlue();
      }
      lastClapTimestamp = currentTime;
    }
  }
}

// Buzzer
int buzzerPin = 10;

void playNote(int note, int duration = 1000) {
  tone(buzzerPin, note, duration);
}

unsigned int melody[] = {NOTE_A4, NOTE_A4, NOTE_FS5, NOTE_FS5, NOTE_FS5, NOTE_G5, NOTE_G5, NOTE_FS5, NOTE_E5, NOTE_A4,
 NOTE_A4, NOTE_FS5, NOTE_FS5, NOTE_A6, NOTE_AS6, NOTE_AS6, NOTE_AS6, NOTE_G5, NOTE_B4, NOTE_AS4, NOTE_AS6, NOTE_AS6, NOTE_AS6,
 NOTE_C6, NOTE_B6, NOTE_AS6, NOTE_GS5, NOTE_GS5, NOTE_B6, NOTE_AS6, NOTE_GS5, NOTE_FS5, NOTE_F5, NOTE_FS5, NOTE_GS5, NOTE_FS5, NOTE_GS5,
 NOTE_AS6, NOTE_F5
};
float durations[] = {.25, .25, .25, 2.25, .25, .25, .125, .125, 2.25, .25,
 .25, .25, 2.25, .25, .25, .125, .125, 2.25, .25, .25, .25, 2.25, .25,
 .25, .125, .125, 2.25, .25, .25, .125, .125, 1.25, .125, .125, .25, .125, .125,
 2, 1.5
};

int melodyLength = sizeof(melody) / sizeof(unsigned int);
int noteIndex = 0;
int playDuration = 0;
int lastNoteTimestamp = 0;
int baseNoteDuration = 1000;
int baseNoteGap = 30;

void resetPlayback() {
  noteIndex = 0;
  playDuration = durations[noteIndex] * baseNoteDuration;
  lastNoteTimestamp = currentTime;
  playNote(melody[noteIndex], playDuration);
}

void playSong()
{
  if (currentTime - lastNoteTimestamp > playDuration + baseNoteGap) {
    noteIndex++;
    if (noteIndex >= melodyLength) {
      noteIndex = 0;
    }
    playDuration = durations[noteIndex] * baseNoteDuration;
    lastNoteTimestamp = currentTime;
    playNote(melody[noteIndex], playDuration);
    hue = map(melody[noteIndex], NOTE_A4, NOTE_AS6, 1, 1023);
    analogHue(hue);
  }
}

// Mode
int mode = 0;
int previousMode = 0;

void incrementMode() {
  mode++;
  if (mode > 3) {
    mode = 0;
  }
}

// Mode Button

int buttonPin = 2;

int debounceWindow = 20;
int debounceStartTime = 0;
int previousButtonState = 0;

int previousDebouncedButtonState = 0;
int debouncedButtonState = 0;


void debounceButton() {
  // Get current raw button state
  int currentButtonState = digitalRead(buttonPin);

  // Check for change
  if (currentButtonState != previousButtonState) {
    debounceStartTime = currentTime;
  }

  // If raw state stable for debounce debounceWindow, update debounced states
  if (currentTime - debounceStartTime > debounceWindow) {
    previousDebouncedButtonState = debouncedButtonState;
    debouncedButtonState = currentButtonState;
  }
  
  // Update raw states
  previousButtonState = currentButtonState;
}

int wasModeButtonClicked() {
  if (previousDebouncedButtonState != debouncedButtonState) {
    if (previousButtonState == 0) {
      return 1;
    }
  }
  return 0;
}

// Potentiometer
int potPin = A2;
int potValue = 0;

void readPotentiometer() {
  potValue = analogRead(potPin);
}

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(photoPin, INPUT);
  pinMode(micPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Serial.begin(9600);
}

void loop() {
  // Process Clock
  currentTime = millis();

  // Process Photoresitor
  photoLevel = analogRead(A0);
  photoToBrightness();

  // Process Microphone
  processMicrophone();

  // Process Potentiometer
  readPotentiometer();

  // Process Mode Button
  debounceButton();
  int clicked = wasModeButtonClicked();

  // Dispatch Mode
  if (clicked == 1) 
  {
    incrementMode();
  }

  detectingClaps = 0;

  if (mode == 0) {
    crossFade(brightness);
  } 
  else if (mode == 1) 
  {
    if (previousMode != mode) {
      writeRed();
    }
    detectingClaps = 1;
  } 
  else if (mode == 2)
  {
    analogHue(potValue);
  }
  else if (mode == 3) {
     if (previousMode != mode) {
      resetPlayback();
    }
    playSong();
  }

  previousMode = mode;
}
