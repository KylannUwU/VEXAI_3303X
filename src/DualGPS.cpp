#include "robot-config.h"

DualGPS::DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units)
: Left_GPS(Lgps),
  Right_GPS(Rgps), 
  IMU(imu),
  Units(units)

{
    Data_Timestamp = Brain.Timer.system();
    update_thread = new vex::thread(updateThread, static_cast<void*>(this));
}



int DualGPS::updateThread(void* arg) 
{
    DualGPS* instance = static_cast<DualGPS*>(arg);
    while (true) 
    {
        instance->updatePosition();
        vex::this_thread::sleep_for(5);
    }
}


void DualGPS::updatePosition()
{
    float LeftX = Left_GPS.xPosition(Units), LeftY = Left_GPS.yPosition(Units), LeftH = Left_GPS.heading();
    float RightX = Right_GPS.xPosition(Units), RightY = Right_GPS.yPosition(Units), RightH = Right_GPS.heading();

    int LeftQual = Left_GPS.quality();
    int RightQual = Right_GPS.quality();

    float LeftWeight = normalizeQuality(LeftQual);
    float RightWeight = normalizeQuality(RightQual);
    float totalWeight = LeftWeight + RightWeight;

    unsigned long timestamp_new = Left_GPS.timestamp();

    if (totalWeight > 0)
    {
        float new_x = (LeftX * LeftWeight + RightX * RightWeight) / totalWeight;
        float new_y = (LeftY * LeftWeight + RightY * RightWeight) / totalWeight;
        float new_heading = (LeftH * LeftWeight + RightH * RightWeight) / totalWeight;
        double new_Vx = 0 ;
        double new_Vy = 0;

        // Calculate GPS-based velocity
        float deltaTime = (timestamp_new - Data_Timestamp) / 1000.0; // Convert to seconds
        
        if (deltaTime > 0)
        {
            new_Vx = (new_x - this->x) / deltaTime;
            new_Vy = (new_y- this->y) / deltaTime;
        }

        x = new_x;
        y = new_y;
        heading = new_heading;
        Vx = new_Vx;
        Vy = new_Vy;
        

        if (RightQual >= 99 && LeftQual >= 99)
        {
            IMU.setHeading(heading, degrees);
        }
    }
    this->quality = totalWeight;

    Data_Timestamp = timestamp_new;
}

float DualGPS::normalizeQuality(int quality) 
{
    return (quality >= 90) ? (quality - 90.0) / 10.0 : 0.0;
}