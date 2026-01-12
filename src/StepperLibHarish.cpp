#include "StepperLibHarish.h"

// ===== SAFETY & BEHAVIOR LIMITS =====
#define MAX_STEP_SPEED 10000.0f
#define MAX_ACCEL 20000.0f
#define MAX_JERK 120000.0f
#define NO_PIN 255

// ================= CONSTRUCTOR =================
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

// ================= INIT =================
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

// ================= STATE =================
bool StepperMotor::isMoving()
{
    return _moving;
}

float StepperMotor::getCurrentAngle()
{
    return _currentSteps / _stepsPerDegree;
}

// ================= PLANNER =================
void StepperMotor::moveToAngle(float angle_deg, float time_s)
{
    int32_t newTarget = (int32_t)(angle_deg * _stepsPerDegree);
    int32_t delta = newTarget - _currentSteps;
    if (delta == 0)
        return;

    digitalWrite(_dirPin, delta >= 0 ? HIGH : LOW);

    _targetSteps = newTarget;
    _stepsRemaining = abs(delta);

    float S = (float)_stepsRemaining;
    float T = time_s;

    // ---- Physical limits ----
    _A = MAX_ACCEL;
    _J = MAX_JERK;

    // ---- Jerk ramp duration ----
    _Tj = _A / _J;

    // ---- Minimum possible time ----
    float Tmin = 2.0f * sqrt(S / _A);
    if (T < Tmin)
        T = Tmin;

    // ---- Compute peak speed ----
    _Vmax = S / (T - _Tj);
    if (_Vmax > MAX_STEP_SPEED)
        _Vmax = MAX_STEP_SPEED;

    // ---- Accel duration ----
    _Ta = _Vmax / _A;

    // ---- Cruise duration ----
    _Tc = T - 2.0f * _Ta;
    if (_Tc < 0)
    {
        _Tc = 0;
        _Ta = sqrt(S / _A);
    }

    // ---- Reset runtime state ----
    _t = 0.0f;
    _currentSpeed = 0.0f;
    _currentAccel = 0.0f;
    _currentJerk = 0.0f;
    _stepAccumulator = 0.0f;
    _phase = PHASE_ACCEL_JERK_UP;
    _moving = true;
}

// ================= PLANNER UPDATE (1 kHz) =================
void StepperMotor::plannerUpdate()
{
    if (!_moving)
        return;

    const float dt = 0.001f;
    _t += dt;

    // ---------- Phase state machine ----------
    switch (_phase)
    {
    case PHASE_ACCEL_JERK_UP:
        _currentJerk = _J;
        if (_t >= _Tj)
        {
            _phase = PHASE_ACCEL_CONST;
            _t = 0;
        }
        break;

    case PHASE_ACCEL_CONST:
        _currentJerk = 0;
        if (_t >= (_Ta - 2.0f * _Tj))
        {
            _phase = PHASE_ACCEL_JERK_DOWN;
            _t = 0;
        }
        break;

    case PHASE_ACCEL_JERK_DOWN:
        _currentJerk = -_J;
        if (_t >= _Tj)
        {
            _phase = (_Tc > 0) ? PHASE_CRUISE : PHASE_DECEL_JERK_UP;
            _t = 0;
        }
        break;

    case PHASE_CRUISE:
        _currentJerk = 0;
        _currentAccel = 0;
        if (_t >= _Tc)
        {
            _phase = PHASE_DECEL_JERK_UP;
            _t = 0;
        }
        break;

    case PHASE_DECEL_JERK_UP:
        _currentJerk = -_J;
        if (_t >= _Tj)
        {
            _phase = PHASE_DECEL_CONST;
            _t = 0;
        }
        break;

    case PHASE_DECEL_CONST:
        _currentJerk = 0;
        if (_t >= (_Ta - 2.0f * _Tj))
        {
            _phase = PHASE_DECEL_JERK_DOWN;
            _t = 0;
        }
        break;

    case PHASE_DECEL_JERK_DOWN:
        _currentJerk = _J;
        if (_t >= _Tj)
        {
            _moving = false;
        }
        break;
    }

    // ---------- Integrate jerk → accel → speed ----------
    _currentAccel += _currentJerk * dt;
    _currentSpeed += _currentAccel * dt;

    if (_currentSpeed < 1.0f)
        _currentSpeed = 1.0f;
}

// ================= STEP ISR (40–50 kHz) =================
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
            _currentSpeed = 0;
            _currentAccel = 0;
            _currentSteps = _targetSteps;
        }
    }
}
