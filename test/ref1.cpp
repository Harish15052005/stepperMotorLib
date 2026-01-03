#include "StepperMotor.h"

extern const uint32_t STEP_ISR_FREQ; // defined in main

StepperMotor::StepperMotor(uint8_t stepPin,
                           uint8_t dirPin,
                           uint8_t enPin,
                           float stepsPerRev,
                           float gearRatio)
{
    _stepPin = stepPin;
    _dirPin  = dirPin;
    _enPin   = enPin;

    _stepsPerDegree = (stepsPerRev * gearRatio) / 360.0f;
}

void StepperMotor::begin()
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enPin, OUTPUT);

    digitalWrite(_enPin, LOW);
}

float StepperMotor::getCurrentAngle()
{
    return _currentSteps / _stepsPerDegree;
}

bool StepperMotor::isMoving()
{
    return _moving;
}

void StepperMotor::moveToAngle(float angle_deg, float time_s)
{
    // PREEMPT CURRENT MOTION (key fix)
    planMove(angle_deg, time_s);
}

void StepperMotor::planMove(float targetAngle, float time_s)
{
    _targetSteps = (int32_t)(targetAngle * _stepsPerDegree);
    int32_t delta = _targetSteps - _currentSteps;

    _stepsRemaining = abs(delta);

    if (_stepsRemaining == 0) {
        _moving = false;
        return;
    }

    digitalWrite(_dirPin, delta >= 0 ? HIGH : LOW);

    _targetSpeed = _stepsRemaining / time_s;
    _accel = (2.0f * _targetSpeed) / time_s;

    if (_currentSpeed < 1)
        _currentSpeed = 1;

    _stepAccumulator = 0;
    _moving = true;
}

void StepperMotor::plannerUpdate()
{
    if (!_moving) return;

    float dt = 0.001f; // 1 ms

    // Deceleration distance check
    float decelDist = (_currentSpeed * _currentSpeed) / (2 * _accel);

    if (_stepsRemaining <= decelDist) {
        _currentSpeed -= _accel * dt;
    } else if (_currentSpeed < _targetSpeed) {
        _currentSpeed += _accel * dt;
    }

    if (_currentSpeed < 1)
        _currentSpeed = 1;
}

void StepperMotor::stepUpdate()
{
    if (!_moving) return;

    _stepAccumulator += _currentSpeed;

    if (_stepAccumulator >= STEP_ISR_FREQ) {
        _stepAccumulator -= STEP_ISR_FREQ;

        digitalWrite(_stepPin, HIGH);
        digitalWrite(_stepPin, LOW);

        _currentSteps += digitalRead(_dirPin) ? 1 : -1;
        _stepsRemaining--;

        if (_stepsRemaining <= 0) {
            _moving = false;
        }
    }
}
