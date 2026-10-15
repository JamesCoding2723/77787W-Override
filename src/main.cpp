#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <cmath>
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "main.h"
#include "odom.h"
#include "auton.h"
#include "arm.h"


#pragma region

void initialize()
{

    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    
    //leftintakem.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::lcd::initialize(); // initialize brain screen
    imu.tare();     // calibrate sensors
    imu.set_data_rate(5);
    // print position to brain screen

    while (imu.is_calibrating()) {
    pros::delay(10);
    }

    /*front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);*/

    // pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "encoder unit: %d", front_left_motor.get_encoder_units());
    arminit();                       // arm must be resting on its bottom stop


    pros::Task Walltask(imu_display_task);
    pros::Task odomTask(odometry);
    pros::Task Armtask(armtask);
    pros::Task Intake(intake);

    //pros::Task mmidscore(midscore);

    //pros::Task Lowgoal(lowscore);
    top_color_sensor.set_led_pwm(100);
    //pros::Task GPS(GPStracking);

    // pros::lcd::clear();
    // pros::lcd::print(5, "IUESHIUSEFIHUWER");

    //master.clear();

    //jeminmech.set_value(true);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled()
{
}

void competition_initialize()
{
}

/**

 */
void autonomous()
{   
    setintakespd(0);

    //moveToPoint(24, 24, 5, 80, 1, 20, 3, 1, 80);


    //pros::delay(2000);

    //pidGyro(24, 90, 2, 70, 1, 25, 15, 100000000);

    front_right();
    
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    setintakespd(0);


    front_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    const float Joystick_LowerDeadzone = 7;

   
    while (true)
    {
        

        // get left y and right x positions
        int rightX = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < Joystick_LowerDeadzone ? 0: master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int leftY = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < Joystick_LowerDeadzone ? 0: master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // move the robot
        moveleft(leftY + 1 * rightX);
        moveright(leftY - 1 * rightX);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            jeminmechtoggle = !jeminmechtoggle;
            jeminmech.set_value(jeminmechtoggle);
        }
        
        armdriver(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));

       
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            setintakespd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            setintakespd(100);
        }
        else 
        {
            setintakespd(0);
        }
        


        pros::c::delay(25);
    }
}
