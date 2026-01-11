#include "StepperLibHarish.h"

// ===== SAFETY & BEHAVIOR LIMITS =====
#define MAX_STEP_SPEED 10000.0f // steps/s
#define MAX_ACCEL 20000.0f      // steps/s²
#define MAX_JERK 120000.0f       // steps/s³
#define DEFAULT_MOVE_TIME_S 1.0f
#define SMALL_ANGLE_DEG 2.0f
#define NO_PIN           255

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
                           uint8_t enPin,
                           float stepsPerRev,
                           float gearRatio,
                           uint8_t microsteps)
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
    if (_enPin != NO_PIN)
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
    int32_t newTarget = (int32_t)(angle_deg * _stepsPerDegree);
    int32_t delta = newTarget - _currentSteps;
    if (delta == 0) return;

    digitalWrite(_dirPin, delta >= 0 ? HIGH : LOW);

    _targetSteps = newTarget;
    _stepsRemaining = abs(delta);

    // ---------- S-Curve motion PLANNING ----------
    float S = (float)_stepsRemaining;
    float Treq = time_s;

    _J = MAX_JERK;
    _A = MAX_ACCEL;

    // jerk ramp time
    _Tj = _A / _J;

    // minimum accel time to reach peak speed
    float Tmin_noCruise = 2.0f * _Tj;

    // minimum possible time (accel + decel only)
    float Tmin = sqrt(S / _A) * 2.0f;

    // if requested time impossible → clamp
    if (Treq < Tmin)
        Treq = Tmin;

    // compute peak speed needed
    _Vmax = S / (Treq - _Tj);

    if (_Vmax > MAX_STEP_SPEED)
        _Vmax = MAX_STEP_SPEED;

    // recompute accel time
    _Ta = _Vmax / _A;

    // cruise time
    _Tc = Treq - (2.0f * _Ta);

    if (_Tc < 0) {
        _Tc = 0;
        _Ta = sqrt(S / _A);
    }

    // reset runtime state
    _t = 0.0f;
    _currentSpeed = 0.0f;
    _currentAccel = 0.0f;
    _stepAccumulator = 0.0f;
    _moving = true;
}

void StepperMotor::plannerUpdate()
{
    if (!_moving) return;

    const float dt = 0.001f;
    _t += dt;

    if (_t < _Tj) {
        _currentAccel = _J * _t;
    }
    else if (_t < (_Ta - _Tj)) {
        _currentAccel = _A;
    }
    else if (_t < _Ta) {
        _currentAccel = _A - _J * (_t - (_Ta - _Tj));
    }
    else if (_t < (_Ta + _Tc)) {
        _currentAccel = 0;
        _currentSpeed = _Vmax;
        return;
    }
    else if (_t < (_Ta + _Tc + _Tj)) {
        _currentAccel = -_J * (_t - (_Ta + _Tc));
    }
    else if (_t < (2.0f * _Ta + _Tc - _Tj)) {
        _currentAccel = -_A;
    }
    else {
        _currentAccel = -_A + _J * (_t - (2.0f * _Ta + _Tc - _Tj));
    }

    _currentSpeed += _currentAccel * dt;
    if (_currentSpeed < 1.0f) _currentSpeed = 1.0f;
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
            _currentSpeed = 0;
            _currentAccel = 0;
            _currentSteps = _targetSteps;
        }
    }
}