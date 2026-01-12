#include "Arduino.h"
#include "StepperLibHarish.h"

#define shoulderStepPin PA0
#define shoulderDirPin PA1
#define elbowStepPin PA2
#define elbowDirPin PA3

StepperMotor shoulder(shoulderStepPin, shoulderDirPin, 255, 200, 55.625, 8);
StepperMotor elbow(elbowStepPin, elbowDirPin, 255, 200, 18.75, 8);

HardwareTimer *stepTimer;
HardwareTimer *plannerTimer;

void stepISR()
{
    shoulder.stepUpdate();
    elbow.stepUpdate();
}

void plannerISR()
{
    shoulder.plannerUpdate();
    elbow.plannerUpdate();
}

void setup()
{
    Serial.begin(115200);
    shoulder.begin();
    elbow.begin();

    stepTimer = new HardwareTimer(TIM1);
    stepTimer->setPrescaleFactor(72); // 1 MHz
    stepTimer->setOverflow(1000000 / STEP_ISR_FREQ);
    stepTimer->attachInterrupt(stepISR);
    stepTimer->resume();

    plannerTimer = new HardwareTimer(TIM2);
    plannerTimer->setPrescaleFactor(7200);
    plannerTimer->setOverflow(10); // 1 kHz
    plannerTimer->attachInterrupt(plannerISR);
    plannerTimer->resume();
}

int inp1 = 0, inp2 = 0;

void loop()
{
    if (Serial.available() >= 2)
    {
        inp1 = Serial.parseInt();
        inp2 = Serial.parseInt();

        elbow.moveToAngle(inp1, inp2);
    }

    Serial.print(inp1);
    Serial.print(" ");
    Serial.print(inp2);
    Serial.print(" ");
    Serial.print(shoulder.getCurrentAngle());
    Serial.print(" ");
    Serial.println(elbow.getCurrentAngle());
}