
#pragma once

// Global robot position
extern double posX;
extern double posY;
extern double posHeading;

// Odometry task function
void odometry(void* param);
void odometry(void*);