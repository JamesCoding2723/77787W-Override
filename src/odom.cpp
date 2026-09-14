#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "robot_config.h"
#include "basic_functions.h"
#include "odom.h"

// Global position
float posX = 0;
float posY = 0;
double posHeading = 0;

// Previous sensor values
double lastVertical = 0;
double lastHorizontal = 0;
double lastHeading = 0;

void odometry(void*) {
    

    const double wheelDiameter = 2;
    const double degreesToInches = (M_PI * wheelDiameter) / 360.0 / 100.0;

    const double verticalOffset = 0.0;
    const double horizontalOffset = 0.0;

    verticalEncoder.set_position(0);
    horizontalEncoder.set_position(0);

    lastVertical = 0;
    lastHorizontal = 0;
    lastHeading = imu.get_heading();

    pros::delay(4000);

    posX = 0;
    posY = 0;

    while (true) {

        double currentVertical =
            verticalEncoder.get_position() * degreesToInches;

        double currentHorizontal =
            horizontalEncoder.get_position() * degreesToInches;

        double currentHeading =
            imu.get_heading();

        double dVertical =
            currentVertical - lastVertical;

        double dHorizontal =
            currentHorizontal - lastHorizontal;

        double dHeading =
            currentHeading - lastHeading;

        if (dHeading > 180)
            dHeading -= 360;

        if (dHeading < -180)
            dHeading += 360;

        double dTheta =
            dHeading * M_PI / 180.0;

        dVertical -= verticalOffset * dTheta;
        dHorizontal -= horizontalOffset * dTheta;

        double averageHeading =
            (lastHeading + currentHeading) / 2.0;

        double theta =
            averageHeading * M_PI / 180.0;

        double deltaX =
            dHorizontal * cos(theta) +
            dVertical * sin(theta);

        double deltaY =
            dVertical * cos(theta) -
            dHorizontal * sin(theta);

        posX += deltaX;
        posY += deltaY;

        posHeading = currentHeading;

        lastVertical = currentVertical;
        lastHorizontal = currentHorizontal;
        lastHeading = currentHeading;

        pros::c::screen_print(pros::E_TEXT_MEDIUM, 3, "Vertical: %f, Horizontal: %f",posY, posX);
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 5, "theta: %f", dHeading);
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 7, "DVertical: %f, DHorizontal: %f",deltaY, deltaX);


        pros::delay(10);
    }
}

double angleRange(double angle) {

    while (angle > 180)
        angle -= 360;

    while (angle < -180)
        angle += 360;

    return angle;
}


void moveToPoint(double targetX, double targetY,double timeout,double max, double E_TOL, double D_TOL, double _settle, float spdmod) 
{

    // Drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;

    // Turn PID
    double kP_turn = 2.0;
    double kI_turn = 0.0;
    double kD_turn = 7.0;

    double driveS_error = 0;
    double turnS_error = 0;

    double driveError = 0;
    double drivePrevError = 0;

    double turnError = 0;
    double turnPrevError = 0;

    double settleTime = 0;
    int repeat = 0;


    while (true) {

        repeat++;


        // ====================================================
        // Position error
        // ====================================================

        double errorX = targetX - posX;
        double errorY = targetY - posY;

        double distance =
            sqrt(errorX * errorX + errorY * errorY);

        driveError = distance;


        // Calculate heading toward point

        double targetHeading =
            atan2(errorX, errorY) * 180.0 / M_PI;

        if (targetHeading < 0)
            targetHeading += 360;


        // Drive PID

        float driveP =
            driveError * kP_drive;

        float driveD =
            (driveError - drivePrevError) * kD_drive;

        driveS_error += driveError;

        driveS_error = fmin(driveS_error, 100);
        driveS_error = fmax(driveS_error, -100);

        if (driveError * drivePrevError < 0)
            driveS_error = 0;

        float driveI =
            kI_drive * driveS_error;

        double driveOutput =
            (driveP + driveI + driveD) * spdmod;


        // Turn PID


        turnError =
            angleRange(targetHeading - posHeading);

        float turnP =
            turnError * kP_turn;

        float turnD =
            (turnError - turnPrevError) * kD_turn;

        turnS_error += turnError;

        turnS_error = fmin(turnS_error, 100);
        turnS_error = fmax(turnS_error, -100);

        if (turnError * turnPrevError < 0)
            turnS_error = 0;

        float turnI =
            kI_turn * turnS_error;

        double turnOutput =
            (turnP + turnI + turnD) * spdmod;


        // ====================================================
        // Motor outputs
        // ====================================================

        double leftPower =
            driveOutput + turnOutput;

        double rightPower =
            driveOutput - turnOutput;


        // Scale both sides if necessary
        double maxMag =
            std::max(
                fabs(leftPower),
                fabs(rightPower)
            );

        if (maxMag > 100) {

            double scale =
                100 / maxMag;

            leftPower *= scale;
            rightPower *= scale;
        }

        leftPower =
            std::clamp(leftPower, -max, max);

        rightPower =
            std::clamp(rightPower, -max, max);


        moveleft(leftPower);
        moveright(rightPower);


        // Early jumpout
        // E_TOL = position error tolerance
        // D_TOL = speed/error-change tolerance
     

        double driveSpeed =
            fabs(driveError - drivePrevError);

        if (
            fabs(driveError) < E_TOL &&
            driveSpeed < D_TOL
        ) {
            settleTime += 1;
        }
        else {
            settleTime = 0;
        }


        // ====================================================
        // Save previous values
        // ====================================================

        drivePrevError = driveError;
        turnPrevError = turnError;


        // ====================================================
        // Timeout
        // ====================================================

        if (repeat > timeout * 50) {

            break;
        }


        // ====================================================
        // Settled
        // ====================================================

        if (settleTime > _settle) {

            break;
        }


        pros::c::screen_print(
            pros::E_TEXT_MEDIUM,
            5,
            "P: %f, X: %f, Y: %f, D: %f",
            leftPower,
            posX,
            posY,
            driveError
        );

        pros::delay(20);
    }
}


// ============================================================
// MOVE TO POSE
// ============================================================

void moveToPose(
    double targetX,
    double targetY,
    double targetHeading,
    double timeout,
    double max,
    double E_TOL,
    double D_TOL,
    double _settle,
    float spdmod
) {

    // Drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;

    // Turn PID
    double kP_turn = 2.0;
    double kI_turn = 0.0;
    double kD_turn = 7.0;

    double driveS_error = 0;
    double turnS_error = 0;

    double driveError = 0;
    double drivePrevError = 0;

    double turnError = 0;
    double turnPrevError = 0;

    double settleTime = 0;
    int repeat = 0;


    while (true) {

        repeat++;


        // ====================================================
        // Position error
        // ====================================================

        double errorX =
            targetX - posX;

        double errorY =
            targetY - posY;

        double distance =
            sqrt(
                errorX * errorX +
                errorY * errorY
            );

        driveError = distance;


        // ====================================================
        // Turn error
        // ====================================================

        turnError =
            angleRange(targetHeading - posHeading);


        // ====================================================
        // Drive PID
        // ====================================================

        float driveP =
            driveError * kP_drive;

        float driveD =
            (driveError - drivePrevError) * kD_drive;

        driveS_error += driveError;

        driveS_error = fmin(driveS_error, 100);
        driveS_error = fmax(driveS_error, -100);

        if (driveError * drivePrevError < 0)
            driveS_error = 0;

        float driveI =
            kI_drive * driveS_error;

        double driveOutput =
            (driveP + driveI + driveD) * spdmod;


        // ====================================================
        // Turn PID
        // ====================================================

        float turnP =
            turnError * kP_turn;

        float turnD =
            (turnError - turnPrevError) * kD_turn;

        turnS_error += turnError;

        turnS_error = fmin(turnS_error, 100);
        turnS_error = fmax(turnS_error, -100);

        if (turnError * turnPrevError < 0)
            turnS_error = 0;

        float turnI =
            kI_turn * turnS_error;

        double turnOutput =
            (turnP + turnI + turnD) * spdmod;


        // ====================================================
        // Motor outputs
        // ====================================================

        double leftPower =
            driveOutput + turnOutput;

        double rightPower =
            driveOutput - turnOutput;


        double maxMag =
            std::max(
                fabs(leftPower),
                fabs(rightPower)
            );

        if (maxMag > 100) {

            double scale =
                100 / maxMag;

            leftPower *= scale;
            rightPower *= scale;
        }

        leftPower =
            std::clamp(leftPower, -max, max);

        rightPower =
            std::clamp(rightPower, -max, max);


        moveleft(leftPower);
        moveright(rightPower);


        // ====================================================
        // Early jumpout
        //
        // Must be at the position AND moving slowly
        // AND at the correct final heading
        // ====================================================

        double driveSpeed =
            fabs(driveError - drivePrevError);

        if (
            fabs(driveError) < E_TOL &&
            driveSpeed < D_TOL &&
            fabs(turnError) < 2
        ) {
            settleTime += 1;
        }
        else {
            settleTime = 0;
        }


        // ====================================================
        // Save previous values
        // ====================================================

        drivePrevError = driveError;
        turnPrevError = turnError;


        // ====================================================
        // Timeout
        // ====================================================

        if (repeat > timeout * 50) {

            break;
        }


        // ====================================================
        // Settled
        // ====================================================

        if (settleTime > _settle) {

            break;
        }


        pros::c::screen_print(
            pros::E_TEXT_MEDIUM,
            5,
            "P: %f, X: %f, Y: %f, H: %f",
            leftPower,
            posX,
            posY,
            posHeading
        );

        pros::delay(20);
    }
}