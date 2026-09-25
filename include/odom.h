
#pragma once

// Global robot position
extern float posX;
extern float posY;
extern double posHeading;

extern double localX;
extern double localY;

extern double dVertical;
extern double dHorizontal;

// Odometry task function
void odometry(void* param);
//void odometry(void*);


void moveToPoint(double targetX, double targetY, double timeout, double max,
                  double E_TOL, double D_TOL, double _settle, float spdmod,
                  double _turnscale = 0, double _drivescale = 0,
                  double headingLockDist = 2.0);

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
    double blendDist = 5.0,
    double _turnscale = 0,
    double _drivescale = 0
);