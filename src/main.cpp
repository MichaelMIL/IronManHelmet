#include <Servo.h>
#include <Arduino.h>
#include <FastLED.h>
#define DATA_PIN 10
#define NUM_LEDS 2

CRGB leds[NUM_LEDS];
Servo servo1;
Servo servo2;

const int enableServosPin = 3;      // the number of the pushbutton pin
const int buttonPin = 2;            // the number of the pushbutton pin
int buttonState = 0;                // variable for reading the debounced pushbutton status
int lastButtonState = LOW;          // variable to store the previous button state
unsigned long lastDebounceTime = 0; // the last time the button state was toggled
unsigned long debounceDelay = 50;   // the debounce time in milliseconds
bool isOpen = false;                // variable to track the state of the servos
int topPoseOpen = 20;
int topPoseClose = 165;
int bottomPoseOpen = 30;
int bottomPoseClose = 155;
int topCurrentPos = 0;
int bottomCurrentPos = 0;

class Button
{
private:
  int pin;
  int state;
  int lastState;
  unsigned long lastPressTime;
  unsigned long longPressDuration;
  bool stateChanged;

public:
  Button(int buttonPin, unsigned long longPressDuration)
  {
    pin = buttonPin;
    state = LOW;
    lastState = LOW;
    lastPressTime = 0;
    this->longPressDuration = longPressDuration;
    pinMode(pin, INPUT_PULLUP);
    stateChanged = false;
  }

  int isPressed()
  {
    int reading = digitalRead(pin);
    if (reading != lastState && reading == LOW)
    {
      Serial.println("State changed");
      lastPressTime = millis();
      stateChanged = true;
    }
    lastState = reading;

    if (millis() - lastPressTime >= longPressDuration)
    {
      if (reading == HIGH)
      {
        stateChanged = false;
      }
      if (stateChanged)
      {
        return 2; // Long press detected after state change
      }
    }
    else if (reading == HIGH && millis() - lastPressTime < longPressDuration)
    {
      if (stateChanged)
      {
        stateChanged = false;
        return 1; // Short press detected after state change
      }
    }
    return 0; // No press detected or no state change
  }
};

Button button(buttonPin, 500);

// create list of colors to cycle
CRGB colors[] = {
    CRGB::BlueViolet,
    CRGB::Red,
    CRGB::Green,
    CRGB::Blue,
    CRGB::Yellow,
    CRGB::Purple,
    CRGB::Cyan,
    CRGB::White,
    CRGB::Black};
int numColors = sizeof(colors) / sizeof(colors[0]);
void setup()
{
  Serial.begin(115200);
  servo1.attach(0); // attaches the servo on pin 0
  servo2.attach(1); // attaches the servo on pin 1

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(64);
  leds[0] = CRGB::BlueViolet;
  leds[1] = CRGB::BlueViolet;
  FastLED.show();
}
CRGB Scroll(int pos)
{
  CRGB color(0, 0, 0);
  if (pos < 85)
  {
    color.g = 0;
    color.r = ((float)pos / 85.0f) * 255.0f;
    color.b = 255 - color.r;
  }
  else if (pos < 170)
  {
    color.g = ((float)(pos - 85) / 85.0f) * 255.0f;
    color.r = 255 - color.g;
    color.b = 0;
  }
  else if (pos < 256)
  {
    color.b = ((float)(pos - 170) / 85.0f) * 255.0f;
    color.g = 255 - color.b;
    color.r = 1;
  }
  return color;
}
void writeServos(int top, int bottom)
{
  digitalWrite(enableServosPin, HIGH);
  servo1.write(top);
  servo2.write(bottom);
  delay(500);
  digitalWrite(enableServosPin, LOW);
}

bool isRainbow = false;
unsigned long lastRainbowChange = 0;
int rainbowIndex = 0;
void rainbowLoop()
{
  if (isRainbow && millis() - lastRainbowChange > 10)
  {
    if (rainbowIndex == 255)
    {
      rainbowIndex = 0;
    }
    leds[0] = Scroll(rainbowIndex);
    leds[1] = Scroll(rainbowIndex);
    FastLED.show();
    rainbowIndex++;
    lastRainbowChange = millis();
  }
}

unsigned long lastLongPress = 0;
unsigned long longPressDuration = 1000;
int currentColorIndex = 0;
void loop()
{
  rainbowLoop();
  int buttonState = button.isPressed();
  if (buttonState == 1)
  {
    Serial.println("Short press detected");
    if (isOpen)
    {
      Serial.println("Closing the door");
      topCurrentPos = topPoseClose;
      bottomCurrentPos = bottomPoseClose;
      isOpen = false;
      writeServos(topCurrentPos, bottomCurrentPos);
    }
    else
    {
      Serial.println("Opening the door");
      topCurrentPos = topPoseOpen;
      bottomCurrentPos = bottomPoseOpen;
      isOpen = true;
      writeServos(topCurrentPos, bottomCurrentPos);
    }
  }
  else if (buttonState == 2)
  {
    if (millis() - lastLongPress > longPressDuration)
    {
      lastLongPress = millis();
      currentColorIndex++;
      isRainbow = false;

      if (currentColorIndex == numColors)
      {
        Serial.println("Rainbow mode activated");
        isRainbow = true;
        return;
      }
      if (currentColorIndex > numColors)
      {
        currentColorIndex = 0;
      }

      leds[0] = colors[currentColorIndex];
      leds[1] = colors[currentColorIndex];
      FastLED.show();
    }
    // Serial.println("Long press detected");
  }
}
