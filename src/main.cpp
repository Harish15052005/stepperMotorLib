#include "Arduino.h"
#include "StepperLibHarish.h"

#define stepPin PA0
#define dirPin PA1

StepperMotor m1(stepPin, dirPin, 200, 1, 8);



void setup() {
    Serial.begin(115200);
}

void loop() {

}
