#include "StepperLibHarish.h"

// ===== SAFETY & BEHAVIOR LIMITS =====
#define MAX_STEP_SPEED 8000.0f // steps/s
#define MAX_ACCEL 20000.0f     // steps/s²
#define MAX_JERK 24000.0f      // steps/s³
#define DEFAULT_MOVE_TIME_S 1.0f
#define SMALL_ANGLE_DEG 2.0f

/*
    Constructor for the StepperMotor class.
    @param stepPin: GPIO pin connected to the STEP input of the driver.
    @param dirPin: GPIO pin connected to the DIR input of the driver.
    @param enPin: GPIO pin connected to the ENABLE input of the driver. Pass 255 if not used.
    @param stepsPerRev: Number of steps per revolution of the motor.
    @param gearRatio: Gear ratio of the motor (optional, default 1).
    @param microsteps: Number of microsteps per step (optional, default 16).
*/

StepperMotor::StepperMotor(uint8_t stepPin,
                           uint8_t dirPin,
                           uint8_t enPin = 255,
                           float stepsPerRev,
                           float gearRatio,
                           uint8_t microsteps = 16)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _enPin = enPin;

    _stepsPerDegree = (stepsPerRev * gearRatio * microsteps) / 360.0f;
}

void StepperMotor::begin()
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    if (_enPin != 255)
    {
        pinMode(_enPin, OUTPUT);
        digitalWrite(_enPin, LOW);
    }
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
    int32_t newTargetSteps = (int32_t)(angle_deg * _stepsPerDegree);

    // Ignore redundant command
    if (newTargetSteps == _targetSteps)
        return;

    int32_t delta = newTargetSteps - _currentSteps;

    // Small move → snap to target, instead of acceleration to save time
    if (abs(delta) < (SMALL_ANGLE_DEG * _stepsPerDegree))
    {
        _currentSteps = newTargetSteps;
        _targetSteps = newTargetSteps;
        _moving = false;
        _currentSpeed = 0;
        _currentAccel = 0;
        return;
    }

    // Invalid / unsafe time
    if (time_s <= 0)
        time_s = DEFAULT_MOVE_TIME_S;

    float requiredSpeed = abs(delta) / time_s;
    if (requiredSpeed > MAX_STEP_SPEED)
        time_s = abs(delta) / MAX_STEP_SPEED;

    // Setup new motion (PREEMPT SAFE)
    _targetSteps = newTargetSteps;
    _stepsRemaining = abs(delta);
    _targetSpeed = abs(delta) / time_s;

    _targetAccel = min(MAX_ACCEL, _targetSpeed / (time_s * 0.3f));
    _jerk = MAX_JERK;

    digitalWrite(_dirPin, delta >= 0 ? HIGH : LOW);

    // Keep speed continuous (no jump)
    if (_currentSpeed < 1.0f)
        _currentSpeed = 1.0f;

    _moving = true;
}

void StepperMotor::plannerUpdate()
{
    if (!_moving)
        return;

    const float dt = 0.001f; // 1 ms

    // Distance needed to stop
    float decelDist = (_currentSpeed * _currentSpeed) / (2.0f * _targetAccel);

    // Decide accel direction
    if (_stepsRemaining <= decelDist)
    {
        _targetAccel = -MAX_ACCEL;
    }
    else
    {
        _targetAccel = MAX_ACCEL;
    }

    if (_currentAccel < _targetAccel)
    {
        _currentAccel += _jerk * dt;
        if (_currentAccel > _targetAccel)
            _currentAccel = _targetAccel;
    }
    else if (_currentAccel > _targetAccel)
    {
        _currentAccel -= _jerk * dt;
        if (_currentAccel < _targetAccel)
            _currentAccel = _targetAccel;
    }

    // Integrate speed
    _currentSpeed += _currentAccel * dt;

    if (_currentSpeed < 1.0f)
        _currentSpeed = 1.0f;
    if (_currentSpeed > _targetSpeed)
        _currentSpeed = _targetSpeed;
}

void StepperMotor::stepUpdate()
{
    if (!_moving)
        return;

    _stepAccumulator += _currentSpeed;

    if (_stepAccumulator >= STEP_ISR_FREQ)
    {
        _stepAccumulator -= STEP_ISR_FREQ;

        // STEP pulse
        digitalWrite(_stepPin, HIGH);
        digitalWrite(_stepPin, LOW);

        _currentSteps += digitalRead(_dirPin) ? 1 : -1;
        _stepsRemaining--;

        if (_stepsRemaining <= 0)
        {
            _moving = false;
            _currentSpeed = 0;
            _currentAccel = 0;
            _currentSteps = _targetSteps;
        }
    }
}
