/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "math.h"

using namespace std;
using namespace vex;

#define MoGo 0
#define RedRing 1
#define BlueRing 2



// Calculates the distance to the coordinates from the current robot position

//GENERAL BOOL
bool ValidTarget = false;
bool ValidTargetMogo = false;

#pragma region MainFuncs

float distanceTo(double target_x, double target_y, vex::distanceUnits unit = vex::distanceUnits::in)
{
    float distance = sqrt(pow((target_x - GPS.xPosition(vex::distanceUnits::cm)), 2) + pow((target_y - GPS.yPosition(vex::distanceUnits::cm)), 2));
    
    if(unit == vex::distanceUnits::in)
        distance = distance / 2.54;
    else if(unit == vex::distanceUnits::cm)
        return distance;

    return distance;
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY)
{
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }
    // Normalize to the range [-180, 180]
    // This allows turnTo to make the smallest turn, whether going forward or backwards.
    bearing_deg = fmod(bearing_deg, 360);
    if (bearing_deg > 180)
    {
        bearing_deg -= 360;
    }
    else if (bearing_deg < -180)
    {
        bearing_deg += 360;
    }
    // fprintf(fp,"Target bearing:%.2f Degrees\n",bearing_deg);
    return bearing_deg;
}


void moveToPoint(Point* Target, bool FrontFacing = true)
{   
    float ThresholdRad = 12; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrived2Target = false;
    
    fprintf(fp,"\rTMoving to target point (%.2f, %.2f)\n", Target->Xcord,Target->Ycord);
    while(!arrived2Target)
    {
     
        // if(((Brain.Timer.system() - startOfInteractionTime)/1000)>breakOutTime){
        //                fprintf(fp,"\rBREAK\n");
        //     fprintf(fp,"\rBREAK\n");
        //     fprintf(fp,"\rBREAK\n");
        //     break;
        // }
        double X_Pos = GPS.xPosition(vex::distanceUnits::cm);
        double Y_Pos = GPS.yPosition(vex::distanceUnits::cm);
        // Check to see if we have arrived to target 
        double threshold = pow((X_Pos - Target->Xcord), 2) + pow((Y_Pos - Target->Ycord),2);
        if(threshold <= pow(ThresholdRad, 3))
        {   
                fprintf(fp,"\rRobot is within the threshold of target\n");
                break;
        }
        // Turn Function
        double intialHeading = calculateBearing(X_Pos, Y_Pos, Target->Xcord, Target->Ycord);
        double diff = fabs(GPS.heading(vex::rotationUnits::deg) - intialHeading);
        double result = (diff <= 180.0) ? diff : 360.0 - diff;

        if((result > 90))
        {
            intialHeading +=  180;
        }
        Chassis.set_heading(GPS.heading(deg));
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        if((result > 90))
        {
            distance = distance * -1;
        }
        Chassis.drive_distance(distance);
        wait(20,msec);
   }

}

void MovetoMogo(Point* Target)
{   
        //Turn Function
        float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), Target->Xcord, Target->Ycord);
        Chassis.set_heading(GPS.heading(deg));
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        Chassis.drive_distance(distance - field.Front_Offset);
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, bool GetMobile, int Dspeed, int Tspeed)
{
    Chassis.drive_max_voltage = Dspeed * 0.12;
    Chassis.turn_max_voltage = Tspeed * 0.12;

    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm));

    if (true)//(!field.Check_Barrier_Intersects(&CurrentPoint, &Target, true))
    {
        fprintf(fp,"\rNo Barrier Intersection found moving to point\n");
        
        if(GetMobile)
        {
            MovetoMogo(&Target);
        }
        else
        {
            moveToPoint(&Target);
        }
    }
    else
    {
        fprintf(fp,"\rBarrier Intersection found! Creating Path to Target\n");
        Path Path2Follow = field.Create_Path_to_Target(&CurrentPoint, &Target);
        Path2Follow.calcPathLength();
        fprintf(fp,"\rPath Length: %.2f || Number of Points in Path %i || Start of Path: (%.2f, %.2f)\n", Path2Follow.pathlength, Path2Follow.PathPoints.size(), Path2Follow.PathPoints[0]->Xcord, Path2Follow.PathPoints[0]->Ycord);
        for (int i = 1; i < Path2Follow.PathPoints.size(); i++)
        {   
            fprintf(fp, "\r-> (%.2f, %.2f)\n", Path2Follow.PathPoints[i]->Xcord, Path2Follow.PathPoints[i]->Xcord);
            if(!GetMobile)
            {
                moveToPoint(Path2Follow.PathPoints[i]);
            }

            else
            {
                if(i == Path2Follow.PathPoints.size() - 1)
                {
                    MovetoMogo(Path2Follow.PathPoints[i]);
                }
                else
                {
                    moveToPoint(Path2Follow.PathPoints[i]);
                }
            }
        }
    }
    if (target_theta != -1)
    {
        Chassis.turn_to_angle(target_theta);
    }
}


DETECTION_OBJECT findMogo(bool CheckSide, bool CheckIso)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
   
    if(local_map.detectionCount > 0 && local_map.detectionCount !=NULL)
    {
        for (int i = 0; i < local_map.detectionCount; i++)
        {
            if(local_map.detections[i].classID == 0)
            {
                if(local_map.detections[i].probability > 0.70 && local_map.detections[i].probability <= 1) 
                {
                    double Mogo_Dist = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                    if (Mogo_Dist < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = Mogo_Dist; 
                        ValidTargetMogo = true;
                        fprintf(fp,"\rFound Viable mogo at (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
                        wait(20,msec);
                    }
                    else 
                        fprintf(fp,"\rNo Viable target Found\n");
                }
                else
                    fprintf(fp,"\rNo Viable target Found, probability out of range %d \n", local_map.detections[i].probability);
            }
            else
                fprintf(fp,"\rNo Viable target Found, no MOGO CLASS in local map\n");
        }
    }
    else
        fprintf(fp,"\rNo Viable target Found, no Objects in local map\n");
    return target;
}


DETECTION_OBJECT findRing(bool CheckSide, bool CheckIso)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    int Ring_Color = 0;
    if(field.Blue_Side)
        Ring_Color = RedRing;
    else if(field.Red_Side)
        Ring_Color = BlueRing;
    if(local_map.detectionCount > 0 && local_map.detectionCount !=NULL)
    {
        for (int i = 0; i < local_map.detectionCount; i++)
        {
            if(local_map.detections[i].classID == Ring_Color)
            {
                if(!field.Near_Intake(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
                {
                    if(local_map.detections[i].probability > 0.70 && local_map.detections[i].probability <= 1) 
                    {
                        double Ring_Dist = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                        if (Ring_Dist < lowestDist)
                        {
                            target = local_map.detections[i];
                            lowestDist = Ring_Dist; 
                            ValidTarget = true;
                            fprintf(fp,"\rFound Viable ring at (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
                            wait(20,msec);
                        }
                        else 
                        {
                            fprintf(fp,"\rNo Viable target Found\n");
                            wait(20,msec);
                        }
                    }
                }
            }
        }
    }
    return target;
}



bool getObject(bool CheckSide = false, bool CheckIso = false)
{
    bool HoldingBall = false; 
    int turn_step = 45;
    int turnItr = 0;

    while(!HoldingBall)
    {    
        // if(((Brain.Timer.system() - startOfInteractionTime)/1000)>breakOutTime){
        //                fprintf(fp,"\rBREAK\n");
        //     fprintf(fp,"\rBREAK\n");
        //     fprintf(fp,"\rBREAK\n");
        //     return false;
        // }
        DETECTION_OBJECT target = findRing(CheckSide,CheckIso);
        // if(Balldetect.isNearObject() && CheckBallColor())
        // {
        //     fprintf(fp,"\rThe robot is holding a Triball\n");
        //     Intake.stop(hold);
        //     HoldingBall = true;
        // } 
       
       // if(target.mapLocation.x && target.mapLocation.y != 0.00)
        if(ValidTarget == true)
        {
           // fprintf(fp,"\rFound Triball! || Triballl Location (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
          //  fprintf(fp,"\rProbability of this target being a Triball is %f%% \n", target.probability*100);
            ValidTarget = false;
            Intake.spin(vex::directionType::fwd);
            moveToPosition(target.mapLocation.x * 100, target.mapLocation.y * 100,-1,true,75,75);
            vex::wait(250,msec);
            HoldingBall = true;
        }
        else
        {
            if(turnItr > 4){
                moveToPosition(-50,16.68125,-1,false);
                turnItr=0;
            }
            //fprintf(fp,"\rSeanning for ball....\n");
            Chassis.turn_max_voltage = 9;
            fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading(deg) + turn_step);
            Chassis.turn_to_angle(GPS.heading(deg) + turn_step);
            turnItr=turnItr+1;
            vex::wait(500,msec);
            target = findRing(CheckSide, CheckIso);
        }
     wait(20,msec);
    }

    return HoldingBall;
}



void GetMogo()
{
    int turn_step = 45;
    int turnItr = 0;
    bool Holding = false;
    Clamp.set(false);
    while(!Holding)
    {

        DETECTION_OBJECT target = findMogo();
        if(ValidTargetMogo == true)
        {
            fprintf(fp, "\r MOGO FOUNDED\n");
            ValidTargetMogo = false;
            moveToPosition(target.mapLocation.x * 100, target.mapLocation.y * 100,-1,true,75,75);
            vex::wait(250,msec);
            Chassis.set_heading(GPS.heading(deg));
            double TargetAngle = calculateBearing(GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm), target.mapLocation.x * 100, target.mapLocation.y * 100);
            double desiredAngle = fmod(TargetAngle + 180, 360); 
            fprintf(fp, "\r Target in %.2f , %.2f going from %.2f , %.2f\n",
                    target.mapLocation.x * 100, target.mapLocation.y * 100,
                    GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm));
            fprintf(fp, "\r TURNING TO MOGO FROM %.2f to %.2f\n", TargetAngle, desiredAngle);

            Chassis.turn_to_angle(desiredAngle);

            fprintf(fp, "\r GOING BACK TO MOGO  \n"); 
            Chassis.drive_distance(-28);
            Clamp.set(true);
            wait(400,msec);
            Holding = true;
        }
        else
        {
            if(turnItr > 4){
                moveToPosition(110,110,-1,false);
                turnItr=0;
            }
            //fprintf(fp,"\rSeanning for ball....\n");
            Chassis.turn_max_voltage = 9;
            fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading(deg) + turn_step);
            Chassis.turn_to_angle(GPS.heading(deg) + turn_step);
            turnItr=turnItr+1;
            vex::wait(500,msec);
            target = findMogo();
        }


    }

}


#pragma endregion MainFuncs

#pragma region ControlFuncs

bool IntakeActivation = false;

int IntakeControl_24()
{
    IntakeActivation = true;
    int LastPosition = 0;
    int Timeout = 0;
    double Intakethreshold = 500;
    int attempDelay = 0;
    bool isStuck = false;
    
    while(IntakeActivation)
    {
        attempDelay ++;
        
        if(!isStuck)
            Intake.spin(fwd,100,pct);

        else
        {
            Intake.spin(vex::directionType::rev, 100, pct);
            wait(400, msec);
            isStuck = false;
            Timeout = 0;
            Intake.spin(fwd,100,pct);
        }
        
        if(Timeout > 5)
            isStuck = true;

        if(attempDelay == 10)
        { 

        if((Intake.position(deg) - LastPosition) < Intakethreshold )
            Timeout ++;

        else
            Timeout = 0;

        
        
        
            
            LastPosition = Intake.position(deg);
            attempDelay = 0;
        }

        wait(20,msec); 
    }
    return 0;
}

int IntakeControl_15()
{
    IntakeActivation = true;
    int LastPosition = 0;
    int Timeout = 0;
    double Intakethreshold = 500;
    int attempDelay = 0;
    bool isStuck = false;
    
    #ifdef Alliance
    int Hue = 115;
    #else    
    int Hue = 30;
    #endif

    while(IntakeActivation)
    {
        attempDelay ++;
        
        if(!isStuck)
        {
            
            Intake.spin(fwd,100,pct);
        }
        
        
        else
        {
            Intake.spin(vex::directionType::rev, 100, pct);
            wait(400, msec);
            isStuck = false;
            Timeout = 0;
            Intake.spin(fwd,100,pct);
        }
        
        if(Timeout > 5)
            isStuck = true;

        if(attempDelay == 10)
        { 

        if((Intake.position(deg) - LastPosition) < Intakethreshold )
            Timeout ++;

        else
            Timeout = 0;

        
        
        
            
            LastPosition = Intake.position(deg);
            attempDelay = 0;
        }

        wait(20,msec); 
    }
    return 0;
}



int trackingMogo()
{
  
  while(1)
  {

    DETECTION_OBJECT targetmogo = findMogo();

    wait(20,msec);

  }
  return 0;
}


DETECTION_OBJECT Multi_CheckforMogo()
{

    float mogoX;
    float mogoY;

    bool mogoChecked = false;
    int attemps = 0;
    DETECTION_OBJECT target;
    while (!mogoChecked)
    {
        attemps ++;

        target = findMogo();

        wait(100,msec);
        fprintf(fp, "\r MOGO ATEMP %d, founded at %.2f, %.2f  \n", attemps, target.mapLocation.x, target.mapLocation.y );
        if (target.mapLocation.x < 1)
        {
            fprintf(fp, "\r MOGO CLOSE AT %.2f, %.2f  \n", target.mapLocation.x, target.mapLocation.y );
            mogoChecked = true;  
            mogoX = target.mapLocation.x;
            mogoY = target.mapLocation.y;
        }
        else
        {
            if(attemps >= 3)
            {
            mogoChecked = true;
            fprintf(fp, "\r MOGO is too far at %.2f, %.2f  \n", target.mapLocation.x, target.mapLocation.y);
            mogoChecked = true;  
            mogoX = 1;
            mogoY = 1;
            break;
            }
        }
        wait(100,msec);
    }
    return target;
}

bool HoldingMogo()
{
    bool Mogo = false;
    if(MogoOptical.hue() > 55 &&  MogoOptical.hue() < 80)
    {
        if(Clamp.value())
        {
            fprintf(fp, "\r HOLDING MOGO\n");
            Mogo = true;
        }

        else
            fprintf(fp, "\rNO MOGO, clamp value is %d  \n", Clamp.value());
        
    }
    else
        fprintf(fp, "\rNO MOGO, hue is %d \n", MogoOptical.hue());
    return Mogo;
}



#pragma endregion ControlFuncs

#pragma region ISOLATION


void auto_Isolation_24()
{
  Chassis.DriveL.resetPosition();
  Chassis.DriveR.resetPosition();
 

  Chassis.drive_with_voltage(12,12);

  double currentPos = Chassis.get_left_position_in(); 
  while(currentPos < 22)
  {
    currentPos = Chassis.get_left_position_in();
    wait(20,msec);
  }
  Chassis.drive_with_voltage(8,1);

  while(Chassis.get_absolute_heading() > 350 || Chassis.get_absolute_heading() < 120 )
  wait(20,msec);
  Chassis.drive_with_voltage(0,0);
  Doinker.set(true);
  Chassis.drive_with_voltage(-12,-12);
  wait(600,msec);
  Doinker.set(false);
  Chassis.drive_with_voltage(0,0);
  
  wait(1500,msec);

  DETECTION_OBJECT target = Multi_CheckforMogo();
  float mogoX = target.mapLocation.x * 100;
  float mogoY = target.mapLocation.y * 100; 


  if(mogoX < 1)
  {
    fprintf(fp, "\r MOGO VIABLE \n");
    double currentAngle = GPS.heading(deg);
    double TargetAngle = calculateBearing(GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm), mogoX, mogoY);
    fprintf(fp, "\r Turning from %.2f to %.2f \n", currentAngle, TargetAngle);
    double desiredAngle = TargetAngle + 180; 
    if(desiredAngle > 360)
      desiredAngle = desiredAngle - 360;     
    Chassis.turn_to_angle(desiredAngle);
    fprintf(fp, "\r turning to %.2f \n", desiredAngle);
  }

  else
  {

    fprintf(fp, "\r MOGO NOT VIABLE AT X: %.2f\n",  mogoX);
    moveToPosition(-30,-30, 110);
    Doinker.set(true);
    Chassis.drive_distance(-5);
    Doinker.set(false);
    Chassis.drive_distance(-5);
  }

  moveToPosition(-110,-110, 110);
}


void auto_Isolation_15()
{

}

#pragma endregion ISOLATION