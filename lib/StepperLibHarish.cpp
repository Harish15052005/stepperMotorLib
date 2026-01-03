#include "StepperLibHarish.h"

StepperMotor::StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t stepsPerRevolution, uint16_t gearRatio, uint8_t microstep)
{
    _stepPin = stepPin;
    _dirPin = dirPin;

    _stepsPerDegree = (stepsPerRevolution * gearRatio * microstep) / 360.0f;
}

void StepperMotor::begin()
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

void StepperMotor::moveToAngle(float angle, float duration)
{
    // Calculate target steps
    _targetSteps = (int32_t)(angle * _stepsPerDegree);
    int32_t delta = _targetSteps - _currentSteps;

    _stepsRemaining = abs(delta);

    if (_stepsRemaining == 0)
    {
        _moving = false;
        return;
    }

    digitalWrite(_dirPin, delta >= 0 ? HIGH : LOW);

    // Calculate required speed and acceleration to reach target in given duration
    _targetSpeed = _stepsRemaining / duration;
    _accel = (2.0f * _targetSpeed) / duration;

    if (_currentSpeed < 1)
        _currentSpeed = 1;

    _stepAccumulator = 0;
    _moving = true;
}

void StepperMotor::plannerUpdate()
{
    if (!_moving)
        return;

    float dt = 0.001f; // 1 ms

    // Deceleration distance check
    float decelDist = (_currentSpeed * _currentSpeed) / (2 * _accel);

    if (_stepsRemaining <= decelDist)
    {
        _currentSpeed -= _accel * dt;
    }
    else if (_currentSpeed < _targetSpeed)
    {
        _currentSpeed += _accel * dt;
    }

    if (_currentSpeed < 1)
        _currentSpeed = 1;
}

void StepperMotor::stepUpdate()
{
    if (!_moving)
        return;

    _stepAccumulator += _currentSpeed;

    if (_stepAccumulator >= STEP_ISR_FREQ)
    {
        _stepAccumulator -= STEP_ISR_FREQ;

        digitalWrite(_stepPin, HIGH);
        digitalWrite(_stepPin, LOW);

        _currentSteps += digitalRead(_dirPin) ? 1 : -1;
        _stepsRemaining--;

        if (_stepsRemaining <= 0)
        {
            _moving = false;
        }
    }
}
