#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "robot_config.h"
#include "basic_functions.h"
#include "odom.h"

// Robot position
double posX = 0;
double posY = 0;
double posHeading = 0;

// Previous sensor values
double lastVertical = 0;
double lastHorizontal = 0;
double lastHeading = 0;

void odometry(void*) {

    // Tracking wheel
    const double wheelDiameter = 2.75;   //NEED TO AJDUST

    // PROS Rotation gives centidegrees
    const double degreesToInches =
        (M_PI * wheelDiameter) / 36000.0;

    // Tracking wheel offsets from robot center.  //NEED TO AJDUST
    const double verticalOffset = 0.0;    
    const double horizontalOffset = 0.0;

    // Reset encoders
    //verticalEncoder.set_position(0);     //NEED TO AJDUST PORT
    //horizontalEncoder.set_position(0);      //NEED TO AJDUST PORT

    lastVertical = 0;
    lastHorizontal = 0;
    lastHeading = imu.get_heading();      //NEED TO AJDUST PORT

    posHeading = lastHeading;

    while (true) {

        // Get current sensor values
        double currentVertical = 0;
            //verticalEncoder.get_position() * degreesToInches;  NEED TO AJDUST PORT

        double currentHorizontal = 0;
            //horizontalEncoder.get_position() * degreesToInches; NEED TO AJDUST PORT

        double currentHeading =
            imu.get_heading();


        // Calculate changes
        double dVertical =
            currentVertical - lastVertical;

        double dHorizontal =
            currentHorizontal - lastHorizontal;

        double dHeading =
            currentHeading - lastHeading;


        // Handle heading wraparound
        if (dHeading > 180)
            dHeading -= 360;

        if (dHeading < -180)
            dHeading += 360;


        // Convert heading change to radians
        double dTheta =
            dHeading * M_PI / 180.0;


        // Remove movement caused by robot rotation
        double localY =
            dVertical - verticalOffset * dTheta;

        double localX =
            dHorizontal - horizontalOffset * dTheta;


        // Arc calculation
        double deltaX;
        double deltaY;

        if (fabs(dTheta) < 1e-5) {

            // Straight movement
            deltaX = localX;
            deltaY = localY;

        } 
        else {

            // Arc movement
            double sinTerm =
                sin(dTheta / 2.0);

            double scale =
                2.0 * sinTerm / dTheta;

            deltaX =
                scale * localX;

            deltaY =
                scale * localY;
        }


        // Convert robot-relative movement
        // into field-relative movement
        double theta =
            lastHeading * M_PI / 180.0;

        double globalX =
            deltaX * cos(theta) +
            deltaY * sin(theta);

        double globalY =
            deltaY * cos(theta) -
            deltaX * sin(theta);


        // Update position
        posX += globalX;
        posY += globalY;

        posHeading = currentHeading;


        lastVertical = currentVertical;
        lastHorizontal = currentHorizontal;
        lastHeading = currentHeading;

        pros::delay(5);
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

            stop();
            break;
        }


        // ====================================================
        // Settled
        // ====================================================

        if (settleTime > _settle) {

            stop();
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

            stop();
            break;
        }


        // ====================================================
        // Settled
        // ====================================================

        if (settleTime > _settle) {

            stop();
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