#include <Arduino.h>
#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::HALF4WIRE, 1, 5, 3, 4);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  stepper.setMaxSpeed(1000.0);
  // stepper.setAcceleration(100.0);
  stepper.setSpeed(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper.runSpeed();
}
