#include "StepperLibHarish.h"

StepperMotor m1(PA0, PA1, 255, 200, 16);
StepperMotor m2(PA3, PA4, 255, 200, 16);
StepperMotor m3(PA6, PA7, 255, 200, 16);
StepperMotor m4(PB1, PB10, 255, 200, 16);

StepperMotor* motors[] = { &m1, &m2, &m3, &m4 };
const uint8_t MOTOR_COUNT = 4;

HardwareTimer *stepTimer;
HardwareTimer *plannerTimer;

void stepISR()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->stepUpdate();
}

void plannerISR()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->plannerUpdate();
}

void setup()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->begin();

    // STEP timer
    stepTimer = new HardwareTimer(TIM1);
    stepTimer->setPrescaleFactor(72); // 1 MHz
    stepTimer->setOverflow(1000000 / STEP_ISR_FREQ);
    stepTimer->attachInterrupt(stepISR);
    stepTimer->resume();

    // PLANNER timer
    plannerTimer = new HardwareTimer(TIM2);
    plannerTimer->setPrescaleFactor(7200);
    plannerTimer->setOverflow(10); // 1 kHz
    plannerTimer->attachInterrupt(plannerISR);
    plannerTimer->resume();

    // Move all joints in 1 second
    m1.moveToAngle( 45, 1.0f);
    m2.moveToAngle(-30, 1.0f);
    m3.moveToAngle( 90, 1.0f);
    m4.moveToAngle( 10, 1.0f);
}

void loop()
{
    // Example: online replanning
    static bool done = false;
    if (millis() > 3000 && !done) {
        m1.moveToAngle(120, 0.8f);
        done = true;
    }
}
