#include "arm.h"
#include "main.h"
#include <cmath>

// ------------------------------------------------------------
//  CONFIG  (TUNE everything in this block)
// ------------------------------------------------------------
// Ports 5, 6 and ADI 'H' are unused in your robot_config.cpp - change if needed.
// Negative port = reversed motor.
pros::Motor arm_motorL(-4, pros::E_MOTOR_GEAR_RED);
pros::Motor arm_motorR(7, pros::E_MOTOR_GEAR_RED);
pros::ADIDigitalIn arm_limit('H');       // bottom limit switch

static const bool  USE_LIMIT_SWITCH = false;   // false if you have no switch
static const float ARM_MIN = 0;               // motor degrees, 0 = resting on bottom
static const float ARM_MAX = 100000000;             // lift arm by hand and read the position
static const int   DRIVER_MV = 12000;         // driver power (lower = slower arm)

// PID (same idea as a drive PID)
static const float KP = 45.0;     // mV per degree of error
static const float KI = 0.0;      // leave 0 until KP/KD are good
static const float KD = 130.0;    // damping (per 10 ms loop)
static const float I_ZONE = 40;   // only build integral within this many degrees
static const float I_MAX = 300;   // clamp on integral sum
static const float DONE_TOL = 4;  // "arrived" tolerance in degrees


//  STATE (shared between opcontrol/autonomous and the PID task)
static float armTarget = 0;
static bool armPidOn = false;
static bool armSettled = false;
static bool armReset = true;

static float armpos()
{
    return (arm_motorL.get_position() + arm_motorR.get_position()) / 2.0;
}

static void armvolt(int mv)
{
    arm_motorL.move_voltage(mv);
    arm_motorR.move_voltage(mv);
}

static void armbrake()
{
    arm_motorL.brake();
    arm_motorR.brake();
}

void arminit()
{
    arm_motorL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    arm_motorR.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    arm_motorL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motorR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motorL.tare_position(); // arm must be resting on its bottom stop here
    arm_motorR.tare_position();
}

// ---------------- background PID (auton) ----------------
void armtask()
{
    float integral = 0;
    float prevErr = 0;

    while (true)
    {
        if (armPidOn)
        {
            float err = armTarget - armpos();

            if (armReset) // new target: start clean
            {
                integral = 0;
                prevErr = err;
                armReset = false;
            }

            if (fabs(err) < I_ZONE)
            {
                integral += err;
                if (integral > I_MAX) integral = I_MAX;
                if (integral < -I_MAX) integral = -I_MAX;
            }
            else
            {
                integral = 0;
            }

            float derivative = err - prevErr;
            prevErr = err;

            float out = KP * err + KI * integral + KD * derivative;
            if (out > 1000000) out = 1000000;
            if (out < -2000) out = -2000;
            armvolt((int)out);

            armSettled = (fabs(err) < DONE_TOL && fabs(derivative) < 1.0);
        }
        pros::delay(10);
    }
}

// ---------------- driver control (two buttons) ----------------
void armdriver(bool up, bool down)
{
    armPidOn = false; // driver always wins over the PID task

    bool bottom = USE_LIMIT_SWITCH && arm_limit.get_value() == 1;
    if (bottom) // re-zero every time it touches the switch
    {
        arm_motorL.tare_position();
        arm_motorR.tare_position();
    }
    float pos = armpos();
    int power = 0;
    if (up && !down) power = DRIVER_MV;
    else if (down && !up) power = -DRIVER_MV;

    if (power > 0 && pos >= ARM_MAX) power = 0;
    if (power < 0 && (pos <= ARM_MIN || bottom)) power = 0;

    if (power == 0)
        armbrake(); // holds position
    else
        armvolt(power);
}

// ---------------- auton helpers ----------------
void setarmtarget(float deg)
{
    if (deg < ARM_MIN) deg = ARM_MIN;
    if (deg > ARM_MAX) deg = ARM_MAX;
    armTarget = deg;
    armSettled = false;
    armReset = true;
    armPidOn = true;
}

bool armwait(int timeoutms)
{
    int t = 0;
    while (t < timeoutms)
    {
        if (armSettled) return true;
        pros::delay(10);
        t += 10;
    }
    return false; // never hang auton
}

void armstop()
{
    armPidOn = false;
    armbrake();
}

/*void clawgrip()
{
    arm_claw.set_value(!CLAW_RELEASE);
}
 
void clawrelease()
{
    arm_claw.set_value(CLAW_RELEASE);
}
 
// Sends the arm to zero and releases the claw delayms after the arm STARTS going down,
// so the release happens while the arm is still moving. Blocking until the arm arrives
// or timeoutms (total, counted from the start) runs out. Returns true if it arrived.
bool armscore(int delayms, int timeoutms)
{
    setarmtarget(0);       // arm starts lowering in the background
    pros::delay(delayms);  // the timer runs while the arm is moving
    clawrelease();
 
    int remaining = timeoutms - delayms;
    if (remaining < 0) remaining = 0;
    return armwait(remaining);
}*/
 
