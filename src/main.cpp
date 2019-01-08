#include <Arduino.h>
#include <Stepper.h>

#define STEPS 100

Stepper stepper(STEPS, 4, 5, 3, 1);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  stepper.setSpeed(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.printf("Hi");
  stepper.step(100);
  delay(500);
}
