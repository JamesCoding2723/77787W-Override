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