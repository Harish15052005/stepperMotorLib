#pragma once
#include <Arduino.h>

#define MAX_MOTORS 6
#define STEP_ISR_FREQ 50000.0f // 50 kHz
#define PLANNER_DT 0.001f      // 1 ms

class StepperMotor {
public:
    StepperMotor(uint8_t stepPin,
                 uint8_t dirPin,
                 uint8_t enPin = 255,
                 float stepsPerRev = 200.0f,
                 float gearRatio = 1.0f,
                 uint8_t microsteps = 16);

    void begin();

    // Preemptable, safe motion command
    void moveToAngle(float angle_deg, float time_s);

    // Called from global ISRs
    void plannerUpdate();   // 1 kHz
    void stepUpdate();      // STEP_ISR_FREQ

    bool  isMoving();
    float getCurrentAngle();

private:
    // Pins
    uint8_t _stepPin, _dirPin, _enPin;

    // Conversion
    float _stepsPerDegree;

    // Position state
    volatile int32_t _currentSteps = 0;
    volatile int32_t _targetSteps  = 0;
    volatile int32_t _stepsRemaining = 0;

    // Motion state
    volatile float _currentSpeed = 0.0f;   // steps/s
    volatile float _targetSpeed  = 0.0f;

    volatile float _currentAccel = 0.0f;   // steps/s²
    volatile float _targetAccel  = 0.0f;

    volatile float _jerk         = 0.0f;   // steps/s³

    volatile float _stepAccumulator = 0.0f;
    volatile bool  _moving = false;
};
