#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include <cmath>

class DualGPS
{
public:
    DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units);

    gps &Left_GPS;
    gps &Right_GPS;
    inertial &IMU;

    double x;
    double y;
    double heading;
    double Vx;
    double Vy;

    double quality;

    unsigned long Data_Timestamp;


private:
    static int updateThread(void* arg);
    vex::thread* update_thread;
    void updatePosition();
    float normalizeQuality(int quality);
    vex::distanceUnits Units;

    
};

#endif // ROBOT_NAVIGATION_H