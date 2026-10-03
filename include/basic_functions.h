#include <cmath>

int sign(float);

extern int intakespd1;


void setintakespd(float);

void intake();

void intake2();

void intake3();

void moveleft(float);

void moveright(float);

void move(float);

double motorpos();

void resetmotorpos();

void turn(float);

void stop();

void moveDis(float, float);

void moveForSec(float, bool, float);

extern bool jeminmechtoggle;

void jeminmecht();

void moveforward(float, bool, float);

void wait(float);

float rad2deg(float);

float deg2rad(float);

void setwall_heading(float);

double getwallpos(float wall_heading);

void imu_display_task(void*);

double getfrontwallpos(float wall_heading);



