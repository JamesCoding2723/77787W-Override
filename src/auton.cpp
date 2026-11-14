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


void left_goal_loader(int _arm_deg){
    pidwallGyro(15, 87, 270, 2, 70, 0.5);
    setintakespd(0);
    setarmtarget(0);
    pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(14, 180, 1, 60, 1, 40, 1);
    moveForSec(35, true, 0.14);

    pidGyro(-13, 180, 1, 60, 1, 40, 1);
    setarmtarget(_arm_deg);
    pidTurnAbs(90, 10, 1);
    pidGyro(22, 90, 1, 70, 2, 100, 1, 15);
    moveForSec(40, true, 0.3);
    setintakespd(0);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
}

void right_goal_loader_close(int _arm_deg){
    pidwallGyro(16, 273, 90, 2, 70, 1);
    setintakespd(-100);
    setarmtarget(0);
    //pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(9.5, 180, 0.7, 60, 1, 40, 1);
    moveForSec(35, true, 0.1);
    setarmtarget(50);
    pros::delay(100);

    pidGyro(-10, 180, 1, 80, 1, 40, 1);
    setarmtarget(_arm_deg);
    pidTurnAbs(270, 10, 1);
    pidGyro(26.4, 270, 1.3, 80, 2, 100, 1, 15);
    moveForSec(40, true, 0.1);
    setintakespd(0);
    setarmtarget(0);
    moveForSec(30, true, 0.2);
    setintakespd(70);
    moveForSec(40, false, 0.1);
    setintakespd(0);
}

void right_goal_loader_far(int _arm_deg, double dis){
    moverightleft(-50, -60);
    pros::delay(400);
    setintakespd(0);
    setarmtarget(0);
    pidwallGyro(16, 273, 90, 0.8, 70, 1);
    setintakespd(0);
    setarmtarget(0);
    //pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(dis, 180, 1, 60, 1, 40, 1);
    moveForSec(35, true, 0.09);
    
    setarmtarget(10);
    moveForSec(-30, true, 0.1);
    setarmtarget(30);
    pidGyro(-13, 180, 1, 80, 1, 40, 1);
    setarmtarget(_arm_deg);
    pidGyro(16.5, -29, 1.5, 80, 1, 40, 1);
    moveForSec(40, true, 0.1);
    setintakespd(0);
    setarmtarget(0);
    moveForSec(70, true, 0.18);
    setintakespd(70);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(0);
}

/*oid auto_skills_108()
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

    pidmove(-14, 0.5, 1);
    pros::delay(200);
    setintakespd(-100);
    //setintakespd(-100);
    //moveToPoint(13, 40, 5, 80, 1, 100, 1.0, 1, 15);
    //stop();
    //pros::delay(2000);
    pidGyro(23, 43.5, 2, 70, 2.5, 100, 2, 15);
    moveForSec(25, true, 0.3);
    setarmtarget(40);
    moveForSec(40, true, 0.41);

    setintakespd(-100);
    
    setarmtarget(164);
    //pros::delay(200);
    
    pidGyro(18, 180, 1.1, 70, 2, 100, 1, 15);
    setintakespd(0);
    moveForSec(30, true, 0.2);


    setarmtarget(0);
    moveForSec(35, true, 0.2);
    pidmove(-14, 1, 1);

    pros::delay(400);
    setintakespd(-100);
    pidGyro(24, 135, 2, 70, 2, 50, 1, 15);
    moveForSec(25, true, 0.35);
    setarmtarget(40);
    moveForSec(45, true, 0.4);
    setarmtarget(239);
    pros::delay(200);
    setintakespd(0);
    pidGyro(21, 265, 1.1, 100, 2, 100, 1, 15);
    //moveForSec(80, true, 0.1);
    setarmtarget(00);
    moveForSec(30, true, 0.3);

    right_goal_loader_close(314);

    right_goal_loader_close(389);

    setintakespd(100);
    pidwallGyro(15, 273, 90, 2, 70, 0.5);
    setintakespd(0);
    setarmtarget(0);
    pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(14, 180, 1, 60, 1, 40, 1);
    moveForSec(35, true, 0.4);

    pidGyro(-13, 180, 1, 60, 1, 40, 1);
    setarmtarget(90);
    pidGyro(16, -24, 1.5, 60, 1, 40, 1);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.1);
    setintakespd(100);

    right_goal_loader_far(164);

    right_goal_loader_far(239);

    right_goal_loader_far(314);

    right_goal_loader_far(389);


    moverightleft(-40, -50);
    setintakespd(100);
    pros::delay(400);
    setintakespd(0);
    setarmtarget(0);
    pidwallGyro(15, 273, 90, 0.9, 70, 0.5);
    setintakespd(0);
    setarmtarget(0);


    pidGyro(-40, -183, 2, 80, 2, 100, 1, 15);
    pidTurnAbs(90, 10, 0.8);
    moveForSec(60, true, 0.3);
    moveForSec(50, false, 0.2);
    moveForSec(60, true, 0.3);

    moveForSec(-30, true, 0.35);

    pidmove(-48, 1, 5);
}  */

void auto_skills_118() 
{
    imu.set_heading(180);
    jeminmecht();
    setarmtarget(50);
    moveForSec(-60, true, 0.19);
    moveForSec(75, true, 0.35);
    moveForSec(-50, true, 0.2);
    moveForSec(75, true, 0.35);
    //setintakespd(-50);
    pidmove(-15, 1, 3, 100, 1,100);
    setintakespd(0);
    pros::delay(100);
    turn(-60);
    pros::delay(50);
    setarmtarget(96);
    pidTurnAbs(90, 3, 0.7);
    pidGyro(17, 90, 1, 70, 2, 100, 1, 15);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    armmax = 2500;
    setarmtarget(0);
    moveForSec(70, true, 0.2);
    moveForSec(40, false, 0.05);
    setintakespd(70);
    moveForSec(40, false, 0.05);
    setintakespd(70);

    pidmove(-14, 0.5, 1);
    armmax = 4500;
    pros::delay(200);
    setintakespd(-100);
    pidTurnAbs(43.5, 1, 0.8);
    pidGyro(23, 43.5, 2, 70, 2.5, 100, 2, 15);
    moveForSec(25, true, 0.27);
    setarmtarget(40);
    moveForSec(40, true, 0.4);

    setintakespd(-100);
    
    setarmtarget(164);
    //pros::delay(200);
    
    pidGyro(21, 180, 1.3, 70, 2, 100, 1, 15);
    setintakespd(0);
    moveForSec(30, true, 0.3);


    setarmtarget(0);
    moveForSec(35, true, 0.2);
    pidmove(-14, 0.5, 1);

    pros::delay(200);
    setintakespd(-100);
    pidTurnAbs(134, 1, 0.8);
    pidGyro(19, 134, 2, 70, 2, 50, 1, 15);
    moveForSec(25, true, 0.5);
    setarmtarget(40);
    moveForSec(45, true, 0.46);
    setarmtarget(239);
    pros::delay(100);
    setintakespd(0);
    pidGyro(26, 265, 1.1, 100, 2, 100, 1, 15);
    moveForSec(30, true, 0.2);
    armmax = 2500;
    setarmtarget(0);
    moveForSec(30, true, 0.2);
    setintakespd(100);
    armmax = 4500;

    right_goal_loader_close(314);

    right_goal_loader_close(389);

    setintakespd(100);
    pidwallGyro(16, 273, 90, 2, 70, 1);
    setintakespd(0);
    setarmtarget(0);
    pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(9, 180, 0.78, 60, 1, 40, 1);
    moveForSec(20, true, 0.4);
    setarmtarget(50);

    pidGyro(-13, 180, 1, 60, 1, 40, 1);
    setarmtarget(103);
    pidGyro(16.5, -28, 1.5, 60, 1, 40, 1);
    moveForSec(40, true, 0.25);
    setintakespd(-100);
    moveForSec(30, false, 0.11);
    setarmtarget(0);
    pros::delay(200);
    moveForSec(40, false, 0.1);
    setintakespd(100);

    right_goal_loader_far(164, 15.3);

    right_goal_loader_far(239, 15.7);

    right_goal_loader_far(310, 16.1);

    //

    moverightleft(-50, -60);
    pros::delay(400);
    setintakespd(0);
    setarmtarget(0);
    pidwallGyro(15.5, 273, 90, 1.1, 70, 1);
    setintakespd(0);
    setarmtarget(0);
    pros::delay(100);
    pidTurnAbs(180, 10, 1);
    setintakespd(-100);
    pidGyro(15, 180, 1, 60, 1, 40, 1);
    moveForSec(25, true, 0.35);
    pros::delay(300); //
    setarmtarget(50);

    
    pidGyro(-60, 183.5, 2, 90, 2, 100, 1, 15);
    pidTurnAbs(90, 10, 0.8);
    moveForSec(75, true, 0.3);
    moveForSec(50, false, 0.2);
    moveForSec(75, true, 0.3);

    setintakespd(0);
    moveForSec(-30, true, 0.35);

    pidTurnAbs(130, 10, 0.8);
    setarmtarget(175);
    pidGyro(-44, 90, 2, 70, 2, 100, 1, 1000000);

    pidGyro(15, 225, 1.1, 70, 2, 100, 1, 15);
    //setintakespd(-100);
    moveForSec(40, true, 0.25);
    setintakespd(-50);
    //moveForSec(30, false, 0.1)
    armmax = 1500; 
    setarmtarget(105);
    pros::delay(350);
    setintakespd(100);
    moveForSec(40, false, 0.1);
    setintakespd(100);

    pidmove(-6, 0.5, 1);
    armmax = 4500;
    setarmtarget(0);
    setintakespd(-100);

    pros::delay(100);
    pidTurnAbs(270, 1, 0.8);
    pidGyro(24.2, 270, 1.3, 80, 2, 100, 1, 7);
    pros::delay(100);
    pidGyro(9, 313, 1.1, 70, 2, 100, 1, 15);
    moveForSec(35, true, 0.3);

    setarmtarget(250);
    pros::delay(100);

    //pidGyro(31, 145, 1.1, 100, 2, 100, 1, 15, 1.6);
    pidTurnAbs(143, 1, 0.75);
    moveForSec(90, true, 0.8);
    setarmtarget(100);
    moveForSec(40, true, 0.4);

    //pidmove(-8, 1, 1);



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
    setarmtarget(106);
    pidGyro(17, 325, 1, 100, 2, 25, 1, 100000);
    moveForSec(40, true, 0.25);
    //setintakespd(-100);
    moveForSec(30, false, 0.11);
    setarmtarget(0);
    pros::delay(200);
    moveForSec(40, false, 0.1);
    setintakespd(100);


    pidGyro(-9, 225, 1.4, 55, 2, 100, 1, 100000, 1, 4.5);
    setintakespd(-100);
    pros::delay(200);
    pidmove(6, 1, 1);
    setarmtarget(50);
    moveForSec(30, true, 0.33);
    setarmtarget(164);
    pidGyro(22, -15, 1.3, 70, 2, 25, 1, 15, 1);
    //pros::delay(200);
    setintakespd(0);
    moveForSec(30, true, 0.24);
    setarmtarget(0);
    moveForSec(35, true, 0.2);

    pidGyro(-14, 0, 1.3, 80, 2, 100, 1, 10);
    pros::delay(200);
    setintakespd(-100);
    pidTurnAbs(-46, 1, 0.8);
    pidGyro(23, -46, 1.3, 70, 1, 100, 1, 15);
    setarmtarget(50);
    moveForSec(35, true, 0.35);
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
    moveForSec(-30, true, 0.3);
    pros::delay(100);


    pidGyro(49, 172, 5);
    stop();
}