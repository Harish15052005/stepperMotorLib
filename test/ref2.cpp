#include "StepperMotor.h"

// ================= CONFIG =================
const uint32_t STEP_ISR_FREQ = 50000; // 50 kHz
#define PLANNER_FREQ 1000             // 1 kHz
// =========================================

// 4 motors
StepperMotor m1(PA0, PA1, PA2, 200, 16);
StepperMotor m2(PA3, PA4, PA5, 200, 16);
StepperMotor m3(PA6, PA7, PB0, 200, 16);
StepperMotor m4(PB1, PB10, PB11, 200, 16);

StepperMotor* motors[] = { &m1, &m2, &m3, &m4 };
const uint8_t MOTOR_COUNT = 4;

HardwareTimer *stepTimer;
HardwareTimer *plannerTimer;

// STEP ISR
void stepISR()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->stepUpdate();
}

// PLANNER ISR
void plannerISR()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->plannerUpdate();
}

void setup()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        motors[i]->begin();

    // STEP TIMER
    stepTimer = new HardwareTimer(TIM1);
    stepTimer->setPrescaleFactor(72);                 // 1 MHz base
    stepTimer->setOverflow(1000000 / STEP_ISR_FREQ); // period
    stepTimer->attachInterrupt(stepISR);
    stepTimer->resume();

    // PLANNER TIMER
    plannerTimer = new HardwareTimer(TIM2);
    plannerTimer->setPrescaleFactor(7200);
    plannerTimer->setOverflow(10); // 1 kHz
    plannerTimer->attachInterrupt(plannerISR);
    plannerTimer->resume();

    // Move all motors to different angles in 1 second
    m1.moveToAngle( 45.0f, 1.0f);
    m2.moveToAngle(-30.0f, 1.0f);
    m3.moveToAngle( 90.0f, 1.0f);
    m4.moveToAngle( 10.0f, 1.0f);
}

void loop()
{
    // Example of PREEMPTION after 3 seconds
    static bool replanDone = false;

    if (millis() > 3000 && !replanDone) {
        m1.moveToAngle(120.0f, 7.0f); // overrides old motion
        replanDone = true;
    }
}
