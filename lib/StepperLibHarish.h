#ifndef STEPPERLIBHARISH_H
#define STEPPERLIBHARISH_H

#define MAX_MOTORS 6
#define STEP_ISR_FREQ 50000.0f // 5#ifndef STEPPERLIBHARISH_H
#define STEPPERLIBHARISH_H

#define MAX_MOTORS 6
#define STEP_ISR_FREQ 50000.0f // 50 kHz
#define PLANNER_DT 0.001f      // 1 ms

#include <Arduino.h>

class StepperMotor
{
public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t stepsPerRevolution = 200, uint16_t gearRatio = 1, uint8_t microstep = 1);

    void begin();

    void moveToAngle(float angle, float duration);
    void plannerUpdate(); // called @ 1 kHz
    void stepUpdate();    // called @ STEP_ISR_FREQ

private:
    uint8_t _stepPin, _dirPin;

    float _stepsPerDegree;

    volatile int32_t _currentSteps = 0;
    volatile int32_t _targetSteps = 0;
    volatile int32_t _stepsRemaining = 0;

    volatile float _currentSpeed = 0.0f; // steps/sec
    volatile float _targetSpeed = 0.0f;
    volatile float _accel = 0.0f;

    volatile float _stepAccumulator = 0.0f;
    volatile bool _moving = false;
};

#endif0 kHz
#define PLANNER_DT 0.001f      // 1 ms

#include <Arduino.h>

class StepperMotor
{
public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t stepsPerRevolution = 200, uint16_t gearRatio = 1, uint8_t microstep = 1);

    void begin();

    void moveToAngle(float angle, float duration);
    void plannerUpdate(); // called @ 1 kHz
    void stepUpdate();    // called @ STEP_ISR_FREQ

private:
    uint8_t _stepPin, _dirPin;

    float _stepsPerDegree;

    volatile int32_t _currentSteps = 0;
    volatile int32_t _targetSteps = 0;
    volatile int32_t _stepsRemaining = 0;

    volatile float _currentSpeed = 0.0f; // steps/sec
    volatile float _targetSpeed = 0.0f;
    volatile float _accel = 0.0f;

    volatile float _stepAccumulator = 0.0f;
    volatile bool _moving = false;
};

#endif