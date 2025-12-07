#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define BUTTON_PIN 3
#define BUZZER_PIN 2
#define BLUE_LED_PIN 1

#define BUZZER_PWM_CHANNEL 0
#define BUZZER_PWM_FREQ 1000 
#define BUZZER_PWM_RESOLUTION 8
void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set button pin as input 
  pinMode(BLUE_LED_PIN, OUTPUT);      // Set LED pin as output
  pinMode(BUZZER_PIN, OUTPUT);        // Set buzzer pin as output

  // Set up PWM for the buzzer
  ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_PWM_FREQ, BUZZER_PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);  // Attach the buzzer pin to PWM channel
  ledcWrite(BUZZER_PWM_CHANNEL, 0);  // Initially turn off the buzzer (0 duty cycle)
}

void loop() {
  // Read the button state (LOW when pressed due to pull up resistor)
  if (digitalRead(BUTTON_PIN) == LOW) {
    // Button is pressed
    Serial.println("Button Pressed!");

    // Turn on the LED and activate the buzzer
    digitalWrite(BLUE_LED_PIN, HIGH);  // Turn on the LED
    ledcWrite(BUZZER_PWM_CHANNEL, 128);  // Set PWM duty cycle to 50% (makes sound)

    // Wait for button release to avoid continuous triggering
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(10);  // Wait for button to be released
    }

    // Turn off the buzzer and LED after the button is released
    ledcWrite(BUZZER_PWM_CHANNEL, 0);  // Turn off the buzzer (0 duty cycle)
    digitalWrite(BLUE_LED_PIN, LOW);   // Turn off the LED
  }
}
 