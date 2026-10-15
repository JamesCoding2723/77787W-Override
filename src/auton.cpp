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
    //setStartPose(0, 13, 180);
    imu.set_heading(180);
    jeminmecht();
    setarmtarget(90);
    moveForSec(-70, true, 0.12);
    moveForSec(70, true, 0.35);
    moveForSec(-70, true, 0.2);
    moveForSec(70, true, 0.35);
    //setintakespd(-50);
    pidmove(-15, 1, 3, 100, 1,100);
    setintakespd(0);
    pros::delay(200);
    pidTurnAbs(90, 3, 0.7);
    pidGyro(17, 90, 1, 70, 2, 100, 1, 15);
    moveForSec(40, true, 0.25);
    setintakespd(-50);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    setintakespd(100);
    moveForSec(70, true, 0.1);
    pidmove(-12, 3, 1);
    setintakespd(-100);
    //setintakespd(-100);
    //moveToPoint(13, 40, 5, 80, 1, 100, 1.0, 1, 15);
    //stop();
    //pros::delay(2000);
    pidGyro(23, 48, 1, 70, 2.5, 100, 2, 15);
    moveForSec(25, true, 0.35);

    setintakespd(-100);
    
    setarmtarget(160);
    //pros::delay(200);
    setintakespd(0);
    pidGyro(12, 165, 1.5, 70, 2, 100, 1, 15);
    moveForSec(70, true, 0.2);


    setarmtarget(0);
    moveForSec(80, true, 0.2);
    pidmove(-12, 2, 1);
    setintakespd(-100);
    pidGyro(24, 130, 2, 70, 2, 50, 1, 15);
    moveForSec(25, true, 0.35);
    moveForSec(50, true, 0.1);
    pros::delay(200);
    setarmtarget(235);
    setintakespd(0);
    pidGyro(20, 255, 1.1, 100, 2, 100, 1, 15);
    moveForSec(80, true, 0.1);
    setarmtarget(00);
    moveForSec(80, true, 0.3);
    pidmove(-8, 1, 2);

    setintakespd(-100);
    pidGyro(23, 215, 1.3, 100, 1, 100, 1, 15);
    moveForSec(35, true, 0.25);
    setintakespd(0);
    setarmtarget(330);
    pidmove(-22, 2, 3);
    //setarmtarget(330);
    pidGyro(20, 220, 1.1, 100, 2, 100, 1, 15);
    moveForSec(70, true, 0.2);
    setarmtarget(00);
    pros::delay(300);
    pidmove(-8, 1, 2);

}