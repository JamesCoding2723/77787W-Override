#include "arm.h"
#include "pros/adi.hpp"
#include <cmath>
#include <sys/types.h>
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "odom.h"


void auto_skills()
{
    imu.set_heading(180);
    jeminmecht();
    moveForSec(-70, true, 0.12);
    moveForSec(70, true, 0.35);
    moveForSec(-70, true, 0.2);
    moveForSec(70, true, 0.35);
    //setintakespd(-50);
    pidmove(-15, 1, 3, 100, 1,100);
    setintakespd(0);
    pros::delay(200);
    turn(-60);
    pros::delay(50);
    setarmtarget(96);
    pidTurnAbs(90, 3, 0.7);
    setintakespd(-100);
    pidGyro(17, 90, 1, 70, 2, 100, 1, 15);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);

    pidmove(-12, 1, 1);
    pros::delay(200);
    setintakespd(-100);
    //setintakespd(-100);
    //moveToPoint(13, 40, 5, 80, 1, 100, 1.0, 1, 15);
    //stop();
    //pros::delay(2000);
    pidGyro(23, 42, 2, 70, 2.5, 100, 2, 15);
    moveForSec(25, true, 0.35);

    setintakespd(-100);
    
    setarmtarget(164);
    //pros::delay(200);
    
    pidGyro(18, 165, 1.1, 70, 2, 100, 1, 15);
    setintakespd(0);
    moveForSec(30, true, 0.2);


    setarmtarget(0);
    moveForSec(35, true, 0.2);
    pidmove(-12, 1, 1);
    pros::delay(200);
    setintakespd(-100);
    pidGyro(24, 125, 2, 70, 2, 50, 1, 15);
    moveForSec(25, true, 0.35);
    //moveForSec(50, true, 0.1);
    //pros::delay(200);
    setarmtarget(239);
    pros::delay(200);
    setintakespd(0);
    pidGyro(21, 255, 1.1, 100, 2, 100, 1, 15);
    //moveForSec(80, true, 0.1);
    setarmtarget(00);
    moveForSec(30, true, 0.3);
    pidmove(-25, 1, 1);

    //pidwallGyro(11.5, 270, 2, 1);
    pidTurnAbs(180, 1, 0.7);
    setintakespd(-100);
    pidGyro(24, 180, 1);
    pros::delay(200);
    pidmove(-8, 1, 1);
    setarmtarget(333);
    pidGyro(24, 270, 1);
    moveForSec(40, true, 0.2);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);
    pidmove(-25, 1, 1);

    //pidwallGyro(11.5, 270, 2, 1);
    pidTurnAbs(180, 1, 0.7);
    setintakespd(-100);
    pidGyro(24, 180, 1);
    pros::delay(200);
    pidmove(-8, 1, 1);
    setarmtarget(427);
    pidGyro(24, 270, 1);
    moveForSec(40, true, 0.2);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);
    pidmove(-24, 1, 1);

    //pidwallGyro(11.5, 270, 2, 1);
    pidTurnAbs(180, 1, 0.7);
    setintakespd(-100);
    pidGyro(24, 180, 1);
    pros::delay(200);
    pidmove(-13, 1, 1);
    setarmtarget(521);
    pidGyro(24, 270, 1);
    moveForSec(40, true, 0.2);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);
    pidmove(-24, 1, 1);

}

void front_right_5pin() //TIME
{
    //setStartPose(0, 13, 180);
    imu.set_heading(180);
    jeminmecht();
    moveForSec(-70, true, 0.12);
    moveForSec(70, true, 0.35);
    moveForSec(-70, true, 0.2);
    moveForSec(70, true, 0.35);
    //setintakespd(-50);
    pidmove(-15, 1, 3, 100, 1,100);
    setintakespd(0);
    pros::delay(200);
    turn(-60);
    pros::delay(50);
    setarmtarget(96);
    pidTurnAbs(90, 3, 0.7);
    setintakespd(-100);
    pidGyro(17, 90, 1, 70, 2, 100, 1, 15);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);

    pidmove(-12, 1, 1);
    pros::delay(200);
    setintakespd(-100);
    //setintakespd(-100);
    //moveToPoint(13, 40, 5, 80, 1, 100, 1.0, 1, 15);
    //stop();
    //pros::delay(2000);
    pidGyro(23, 45, 2, 70, 2.5, 100, 2, 15);
    moveForSec(25, true, 0.35);

    setintakespd(-100);
    
    setarmtarget(164);
    //pros::delay(200);
    
    pidGyro(18, 165, 1.1, 70, 2, 100, 1, 15);
    setintakespd(0);
    moveForSec(30, true, 0.2);


    setarmtarget(0);
    moveForSec(35, true, 0.2);
    pidmove(-12, 1, 1);
    pros::delay(200);
    setintakespd(-100);
    pidGyro(24, 127, 2, 70, 2, 50, 1, 15);
    moveForSec(25, true, 0.35);
    //moveForSec(50, true, 0.1);
    //pros::delay(200);
    setarmtarget(239);
    pros::delay(200);
    setintakespd(0);
    pidGyro(21, 255, 1.1, 100, 2, 100, 1, 15);
    //moveForSec(80, true, 0.1);
    setarmtarget(00);
    moveForSec(30, true, 0.3);
    pidmove(-8, 1, 2);

    setintakespd(-100);
    pidGyro(21, 212, 1.3, 70, 1, 100, 1, 15);
    moveForSec(35, true, 0.25);
    setarmtarget(333);
    pidGyro(-18, 265, 1.2, 90, 4, 100, 1, 10000000, 1, 20);    
    setintakespd(0);
    //setarmtarget(330);
    pidmove(15, 2, 0.9, 100);
    arm_motorL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm_motorR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    setarmtarget(00);
    moveForSec(30, true, 0.2);
    //pros::delay(300);
    pidmove(-8, 1, 2);
}

void right_twostack_5pin() //need to finish. DO NOT USE
{
    imu.set_heading(270);
    jeminmecht();
    setarmtarget(100);
    pidGyro(17, 325, 1, 100, 2, 25, 1, 100000);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);


    pidGyro(-9, 225, 1.4, 70, 2, 100, 1, 100000, 1, 4);
    setintakespd(-100);
    pros::delay(100);
    pidmove(8, 1, 1);
    moveForSec(25, true, 0.2);
    setarmtarget(164);
    pidGyro(22, -15, 1.3, 70, 2, 25, 1, 15, 1);
    //pros::delay(200);
    setintakespd(0);
    moveForSec(30, true, 0.2);
    setarmtarget(0);
    moveForSec(35, true, 0.2);

    pidmove(-12, 1, 1);
    setintakespd(-100);
    pidGyro(21, -45, 1.3, 70, 1, 100, 1, 15);
    moveForSec(35, true, 0.25);
    setarmtarget(239);
    pidGyro(22, 90, 1.3, 70, 2, 25, 1, 15, 1);
    //pros::delay(200);
    setintakespd(0);
    moveForSec(30, true, 0.2);
    setarmtarget(0);
    moveForSec(35, true, 0.2);
    
    pidmove(-12, 1, 1);


}

void right_twostack_4pin() //ATTEMPTS A 5 PIN
{
    imu.set_heading(270);
    jeminmecht();
    setarmtarget(100);
    pidGyro(17, 325, 1, 100, 2, 25, 1, 100000);
    moveForSec(40, true, 0.2);
    setintakespd(-100);
    moveForSec(30, false, 0.1);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);


    pidGyro(-9, 225, 1.4, 70, 2, 100, 1, 100000, 1, 5);
    setintakespd(-100);
    pros::delay(200);
    pidmove(8, 1, 1);
    moveForSec(20, true, 0.13);
    setarmtarget(164);
    pidGyro(22, -15, 1.3, 70, 2, 25, 1, 15, 1);
    //pros::delay(200);
    setintakespd(0);
    moveForSec(30, true, 0.1);
    setarmtarget(0);
    moveForSec(35, true, 0.2);

    pidmove(-13, 1, 1);
    //pros::delay(200);
    setintakespd(-100);
    pidGyro(21, -48, 1.3, 70, 1, 100, 1, 15);
    moveForSec(35, true, 0.25);
    setintakespd(-20);
    setarmtarget(239);
    pidGyro(48, 15, 2, 90, 2, 100, 1);
    setintakespd(0);
    moveForSec(30, true, 0.1);
    setarmtarget(0);
    moveForSec(35, true, 0.2);

    /*pidmove(-13, 1, 1);
    setintakespd(-100);
    pidGyro(26, 49.4, 1.3, 100, 2, 100, 1, 20, 1);
    //pros::delay(300);
    pidGyro(5, 60, 0.3, 100, 1, 100, 1, 15, 2);
    setarmtarget(300);
    pidGyro(-18, 5, 1.2, 90, 4, 100, 1, 10000000, 1, 20);
    setintakespd(0);
    pidmove(15, 2, 0.9, 100);
    moveForSec(30, true, 0.1);
    setarmtarget(00);
    moveForSec(30, true, 0.2);*/


    pidmove(-5, 2, 0.5);
    pidTurnAbs(90, 1, 0.7);
    moveForSec(60, true, 1.5);

    //moveForSec(70, true, 0.35);
    moveForSec(-30, true, 0.3);
    pros::delay(150);
    moveForSec(80, true, 0.25);
    moveForSec(-30, true, 0.35);
    pros::delay(100);


    pidGyro(49, 172, 5);
    stop();
}