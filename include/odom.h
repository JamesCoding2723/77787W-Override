
#pragma once

// Global robot position
extern double posX;
extern double posY;
extern double posHeading;

// Odometry task function
void odometry(void* param);
void odometry(void*);


void moveToPoint(
    double targetX,
    double targetY,
    double timeout,
    double max,
    double E_TOL,
    double D_TOL,
    double _settle,
    float spdmod
);

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
);