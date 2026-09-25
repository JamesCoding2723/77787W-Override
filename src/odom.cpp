#include "pros/screen.h"
#include <cmath>
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

const double verticalOffset = -0.75;  //NEED TO CHECK
const double horizontalOffset = 0.25;   //NEED TO CHECK

verticalEncoder.set_position(0);
horizontalEncoder.set_position(0);

lastVertical = 0.0;
lastHorizontal = 0.0;
lastHeading = imu.get_heading();

pros::delay(2000);


posX = 0.0;
posY = 0.0;

while (true) {

double currentVertical =
-verticalEncoder.get_position() * degreesToInches;

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

double dVerticalCorrected =
dVertical - verticalOffset * dTheta;

double dHorizontalCorrected =
dHorizontal - horizontalOffset * dTheta;

double averageHeading =
            (lastHeading + currentHeading) / 2.0;

double theta =
averageHeading * M_PI / 180.0;

// Arc-length correction: when the robot turns while moving between
// updates, the true displacement is a chord of an arc, not a straight
// line. This factor corrects for that; falls back to straight-line
// motion when dTheta is ~0 to avoid dividing by zero.
double localX, localY;

if (fabs(dTheta) < 1e-9) {
localX = dHorizontalCorrected;
localY = dVerticalCorrected;
}
else {
double sinFactor =
2.0 * sin(dTheta / 2.0);

localX =
sinFactor * (dHorizontalCorrected / dTheta + horizontalOffset);

localY =
sinFactor * (dVerticalCorrected / dTheta + verticalOffset);
}

double deltaX =
localX * cos(theta) +
localY * sin(theta);

double deltaY =
localY * cos(theta) -
localX * sin(theta);

posX += deltaX;
posY += deltaY;

posHeading = currentHeading;

lastVertical = currentVertical;
lastHorizontal = currentHorizontal;
lastHeading = currentHeading;

pros::c::screen_print(pros::E_TEXT_MEDIUM, 3, "Vertical: %f, Horizontal: %f",posY, posX);
        //pros::c::screen_print(pros::E_TEXT_MEDIUM, 7, "VerticalE: %f, HorizontalE: %f",verticalEncoder.get_position(), horizontalEncoder.get_position());

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


void moveToPoint(double targetX, double targetY, double timeout, double max,
                  double E_TOL, double D_TOL, double _settle, float spdmod,
                  double _turnscale, double _drivescale ,
                  double headingLockDist) {

    // Drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;

    // Turn PID
    double kP_turn = 1.8;
    double kI_turn = 0.0;
    double kD_turn = 7.5;

    double driveS_error = 0;
    double turnS_error = 0;

    double driveError = 0, drivePrevError = 0;
    double turnError = 0, turnPrevError = 0;

    double settleTime = 0;
    int repeat = 0;

    while (true) {
        repeat++;

        // ====================================================
        // Position error
        // ====================================================

        double errorX = targetX - posX;
        double errorY = targetY - posY;

        double distance = sqrt(errorX * errorX + errorY * errorY);
        driveError = distance;

        // Target heading toward point — frozen once close, to avoid
        // atan2 instability from tiny positional noise near the target
        double targetHeading;
        if (distance > headingLockDist) {
            targetHeading = atan2(errorX, errorY) * 180.0 / M_PI;
            if (targetHeading < 0) targetHeading += 360;
        } else {
            targetHeading = posHeading;
        }

        // ====================================================
        // Drive PID
        // ====================================================

        float driveP = driveError * kP_drive;
        float driveD = (driveError - drivePrevError) * kD_drive;

        driveS_error += driveError;
        driveS_error = fmin(driveS_error, 100);
        driveS_error = fmax(driveS_error, -100);
        if (driveError * drivePrevError < 0) driveS_error = 0;

        float driveI = kI_drive * driveS_error;

        double driveOutput = (driveP + driveI + driveD) * spdmod;

        // ====================================================
        // Turn PID
        // ====================================================

        turnError = angleRange(targetHeading - posHeading);

        // Wrap the derivative delta too — a raw subtraction of two
        // already-wrapped errors can spike hugely across the ±180 boundary
        double turnDelta = angleRange(turnError - turnPrevError);

        float turnP = turnError * kP_turn;
        float turnD = turnDelta * kD_turn;

        turnS_error += turnError;
        turnS_error = fmin(turnS_error, 100);
        turnS_error = fmax(turnS_error, -100);
        if (turnError * turnPrevError < 0) turnS_error = 0;

        float turnI = kI_turn * turnS_error;

        double turnOutput = (turnP + turnI + turnD) * spdmod;

        // ====================================================
        // Optional scale coupling — disabled (1.0) unless explicitly set
        // ====================================================

        double turnScale = (_turnscale <= 0)
            ? 1.0
            : 1.0 - std::min(fabs(turnError) / _turnscale, 1.0);

        double driveScale = (_drivescale <= 0)
            ? 1.0
            : 1.0 - std::min(fabs(driveError) / _drivescale, 1.0);

        driveOutput *= turnScale;
        turnOutput *= driveScale;

        // ====================================================
        // Motor outputs
        // ====================================================

        double leftPower = driveOutput + turnOutput;
        double rightPower = driveOutput - turnOutput;

        double maxMag = std::max(fabs(leftPower), fabs(rightPower));
        if (maxMag > 100) {
            double scale = 100 / maxMag;
            leftPower *= scale;
            rightPower *= scale;
        }

        leftPower = std::clamp(leftPower, -max, max);
        rightPower = std::clamp(rightPower, -max, max);

        moveleft(leftPower);
        moveright(rightPower);

        // ====================================================
        // Early jumpout
        // ====================================================

        if (fabs(driveError) < E_TOL && ((leftPower + rightPower) / 2) < D_TOL) {
            settleTime += 1;
        } else {
            settleTime = 0;
        }

        drivePrevError = driveError;
        turnPrevError = turnError;

        if (repeat > timeout * 50) break;
        if (settleTime > _settle) break;

        pros::c::screen_print(
            pros::E_TEXT_MEDIUM, 5,
            "P: %f, X: %f, Y: %f, D: %f",
            turnError, posX, posY, driveError
        );

        pros::delay(20);
    }
}



void moveToPose(
    double targetX,
    double targetY,
    double targetHeading,
    double timeout,
    double max,
    double E_TOL,
    double D_TOL,
    double _settle,
    float spdmod,
    double blendDist,
    double _turnscale,
    double _drivescale
) {

    // Drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;

    // Turn PID
    double kP_turn = 1.8;
    double kI_turn = 0.0;
    double kD_turn = 7.5;

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
        // Blended heading target
        //
        // Far from the point: chase the point's direction (like moveToPoint).
        // Near the point: blend toward the final targetHeading, so the
        // robot rotates into its final pose only once it has basically
        // arrived, instead of turning-in-place up front.
        // ====================================================

        double pointHeading =
            atan2(errorX, errorY) * 180.0 / M_PI;

        if (pointHeading < 0)
            pointHeading += 360;

        double blend =
            1.0 - std::min(distance / blendDist, 1.0);
        // blend: 0 = fully chase the point, 1 = fully chase targetHeading

        double headingDiff =
            angleRange(targetHeading - pointHeading);

        double desiredHeading =
            pointHeading + headingDiff * blend;

        if (desiredHeading < 0)
            desiredHeading += 360;

        if (desiredHeading >= 360)
            desiredHeading -= 360;


        // ====================================================
        // Turn error
        // ====================================================

        turnError =
            angleRange(desiredHeading - posHeading);

        // Wrap the derivative delta too — a raw subtraction of two
        // already-wrapped errors can spike hugely across the ±180 boundary
        double turnDelta =
            angleRange(turnError - turnPrevError);


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
            turnDelta * kD_turn;

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
        // Optional scale coupling — disabled (1.0) unless explicitly set
        // ====================================================

        double turnScale =
            (_turnscale <= 0)
                ? 1.0
                : 1.0 - std::min(fabs(turnError) / _turnscale, 1.0);

        double driveScale =
            (_drivescale <= 0)
                ? 1.0
                : 1.0 - std::min(fabs(driveError) / _drivescale, 1.0);

        driveOutput *= turnScale;
        turnOutput *= driveScale;


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
            fabs(angleRange(targetHeading - posHeading)) < 2
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
            "P: %f, X: %f, Y: %f, H: %f, blend: %f",
            leftPower,
            posX,
            posY,
            posHeading,
            blend
        );

        pros::delay(20);
    }
}