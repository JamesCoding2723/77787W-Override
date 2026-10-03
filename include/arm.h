#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

// DR6B arm: two-button driver control, background PID for auton.

extern pros::Motor arm_motorL;
extern pros::Motor arm_motorR;
extern pros::ADIDigitalIn arm_limit;

void arminit();                      // call once in initialize()
void armtask();                      // the PID task (start it in initialize())
void armdriver(bool up, bool down);  // call every opcontrol loop (turns PID off)
void setarmtarget(float deg);        // auton: non-blocking, turns PID on
bool armwait(int timeoutms);         // auton: optional, wait until arm arrives
void armstop();                      // auton: turn PID off and brake

/*void clawgrip();                      // close the claw
void clawrelease();                  // open the claw
bool armscore(int delayms = 300, int timeoutms = 1500); // lower arm, release claw delayms into the move*/
 
