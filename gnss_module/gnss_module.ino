///BME280 LIBRARIES///
#include <Wire.h>
#include <Adafruit_BME280.h>

///ENCODER LIBRARIES///
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

///INITIALIZE BME280///
Adafruit_BME280 bme; // use I2C interface for BME280 sensor 
// see library examples -> test/unified for different communication examples

///INITIALIZE ENCODER///
#define SS_SWITCH 24
#define SS_NEOPIX 6

#define SEESAW_ADDR 0x36

Adafruit_seesaw ss;

///GLOBAL BME280 VARIABLES///


///GLOBAL ENCODER VARIABLES///

  int encoderPosition1 = 0;
  int buttonState1 = 0;

int32_t encoder_position;
int32_t lastEncoderPosition;
bool isButtonPushed = LOW; 
int buttonState; 
int32_t lastButtonState = LOW;
unsigned long lastDebounceTime = 0; //last time button was toggled
unsigned long debounceDelay = 50; //debounce time (increase if debounce ocurs)
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Looking for seesaw!");
  
  if (! ss.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version  != 4991){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
  }
  Serial.println("Found Product 4991");
  
  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();
}

/**
 * @brief Reads the encoder position and sets the neopixel color accordingly. 
 * @return Returns the position if changed from last position.
 */
int readEncoderMovement() {
  int32_t new_position = ss.getEncoderPosition();
  if (encoder_position != new_position) {
    Serial.println(new_position);  // display new position
    encoder_position = new_position;
  }
  return encoder_position;
}


/**
 * @brief Reads the button state and prints a statement if the button is pressed.
 * @return Returns the button reading if changed from last button reading.
 */
int readEncoderButtonPressed() {
  int32_t reading = ss.digitalRead(SS_SWITCH); // Read button state

  if (reading != lastButtonState) { // If the button state changed from the last reading
    lastDebounceTime = millis(); // reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) { // If the button state has been stable for longer than debounceDelay
    if (reading != buttonState) { // If the button state has changed
      buttonState = reading;

      if (buttonState == LOW) { // If the new state is LOW (pressed)
        //isButtonPushed = !isButtonPushed;
        //Serial.println(isButtonPushed);
        Serial.println("Button pressed, state toggled!");
      }
    }
  }

  lastButtonState = reading; // Save reading for next loop
  return lastButtonState;
}
void loop() {
  
  

  encoderPosition1 = readEncoderMovement();
  buttonState1 = readEncoderButtonPressed();

  // don't overwhelm serial port
  delay(10);

  switch (encoder_position) {
      case 1:
        Serial.println("Case 1");
        break;
      case 2:
        Serial.println("Case 2");
        break;
      default:
        Serial.println("Case default");
        // default is optional
        break;
    }
}