#include "Arduino.h"
#include "StepperLibHarish.h"


#define m1step PA0
#define m1dir PA1
#define m2step PA2
#define m2dir PA3

StepperMotor m1(m1step, m1dir, 200, 56, 8);
StepperMotor m2(m2step, m2dir, 200, 9, 8);

HardwareTimer *stepTimer;
HardwareTimer *plannerTimer;

void stepISR() {
  m1.stepUpdate();
  m2.stepUpdate();
}

// PLANNER ISR
void plannerISR() {
  m1.plannerUpdate();
  m2.plannerUpdate();
}


void setup() {
  Serial.begin(115200);
  m1.begin();
  m2.begin();

  stepTimer = new HardwareTimer(TIM1);
  stepTimer->setPrescaleFactor(72);                 // 1 MHz base
  stepTimer->setOverflow(1000000 / STEP_ISR_FREQ);  // period
  stepTimer->attachInterrupt(stepISR);
  stepTimer->resume();

  // PLANNER TIMER
  plannerTimer = new HardwareTimer(TIM2);
  plannerTimer->setPrescaleFactor(7200);
  plannerTimer->setOverflow(10);  // 1 kHz
  plannerTimer->attachInterrupt(plannerISR);
  plannerTimer->resume();
}

int inp = 0, inp1=0;

void loop() {
  
  if(Serial.available()) {
    inp = Serial.parseInt();
    inp1 = Serial.parseInt();

    m1.moveToAngle(inp, 5);
    m2.moveToAngle(inp1, 5);
  }

  Serial.print(inp);Serial.print(" ");
  Serial.println(inp1);
}
