#include "arm.h"
#include "pros/adi.hpp"
#include <cmath>
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "odom.h"


void auto_skills()
{
    
}

void front_right()
{
    setStartPose(0, 13, 180);
    //jeminmecht();
    moveForSec(-70, true, 0.12);
    moveForSec(70, true, 0.35);
    moveForSec(-70, true, 0.2);
    moveForSec(70, true, 0.35);
    moveForSec(-70, true, 0.435);
    pros::delay(200);
    pidTurnAbs(90, 3, 0.7);
    moveToPoint(23, 24, 2, 70, 10, 70, 1.0, 1, 15);
    moveForSec(50, true, 0.3);
    jeminmecht();
    moveForSec(70, true, 0.3);
    moveForSec(-60, true, 0.43);
    setintakespd(-100);
    moveToPoint(28, 63, 5, 80, 30, 100, 1.0, 1, 15);
    //stop();
    pidmove(13, 1, 30, 30, 1, 30);
    jeminmecht();
    
    setarmtarget(160);
    pros::delay(200);
    moveToPoint(26, 24, 2, 70, 10, 70, 1.0, 1, 15);
    moveForSec(70, true, 0.1);

    setintakespd(0);
    setarmtarget(0);
    pros::delay(500);
    jeminmecht();



    moveForSec(-70, true, 0.2);
    setintakespd(-100);
    moveToPoint(54, 23, 2, 70, 19, 70, 1.0, 1, 15);
    pidmove(12,2,30,30);
    jeminmecht();
    pros::delay(200);
    setarmtarget(220);
    moveToPoint(26, 24, 2, 70, 10, 70, 1.0, 1, 15);
    moveForSec(70, true, 0.2);
    setarmtarget(100);
    setintakespd(0);
    pros::delay(300);
    jeminmecht();
    moveForSec(-70, true, 0.2);
   // moveForSec(60, true, 0.2);
}