#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include <cmath>
#include "pros/optical.hpp"
#include "robot_config.h"

// ROBOT CONFIG  //  PID CONFIG
#pragma region

pros::Motor rightintakem1(6, pros::E_MOTOR_GEAR_BLUE);
pros::Motor rightintakem2(15, pros::E_MOTOR_GEAR_BLUE);
pros::Motor leftintakem(-8, pros::E_MOTOR_GEAR_BLUE);


pros::ADIDigitalOut jeminmech('A', false);
pros::ADIDigitalOut jeminchop('E', false);
pros::ADIDigitalOut jemintake('C', false);
pros::ADIDigitalOut jeminloader('D', false);
pros::ADIDigitalOut jeminwing('B', false);
pros::ADIDigitalOut jeminpark('F', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor front_left_motor(-13, pros::E_MOTOR_GEAR_BLUE);   // front left motor -13
pros::Motor middle_left_motor(-12, pros::E_MOTOR_GEAR_GREEN); // middle left motorv -12
pros::Motor back_left_motor(-11, pros::E_MOTOR_GEAR_BLUE);    // back left motor -11
pros::Motor front_right_motor(9, pros::E_MOTOR_GEAR_BLUE); // front right motor 9
pros::Motor middle_right_motor(10, pros::E_MOTOR_GEAR_BLUE); // middle right motor 10
pros::Motor back_right_motor(17, pros::E_MOTOR_GEAR_BLUE); // back right motor 17


pros::Optical top_color_sensor(14); 
// pros::Optical mid_color_sensor(1); // 1 is temporary

// left group
pros::MotorGroup left_motor_group({front_left_motor, middle_left_motor, back_left_motor});
// right group
pros::MotorGroup right_motor_group({front_right_motor, middle_right_motor, back_right_motor});


// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

pros::Imu imu(2);

pros::Distance distance_sensor(19);
pros::Distance frontdistance(11); // 1 is temporary
pros::Distance middistance(18);

pros::Rotation horizontalEncoder(3);
pros::Rotation verticalEncoder(4);
// vertical tracking wheel encoder
//pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings


#pragma endregion