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
#include <tuple>

using namespace std;
using namespace vex;

// #define MoGo 0
// #define RedRing 1
// #define BlueRing 2



// Calculates the distance to the coordinates from the current robot position

//GENERAL BOOL
// bool ValidTarget = false;
// bool ValidTargetMogo = false;

#pragma region MainFuncs

double distanceTo(double target_x, double target_y, vex::distanceUnits units = vex::distanceUnits::in)
{
    double distance = sqrt(pow((target_x - GPS.xPosition(units)), 2) + pow((target_y - GPS.yPosition(units)), 2));
    return distance;
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


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
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


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


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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
            //MovetoMogo(&Target);
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
                    //MovetoMogo(Path2Follow.PathPoints[i]);
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


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void findScored(AI_RECORD* map, bool isScored)
{
    // Arrays to temporarily store mobile goals and rings
    vector<tuple<DETECTION_OBJECT,int>> MobileGoals;
    vector<tuple<DETECTION_OBJECT,int>> Rings;
    vector<vector<tuple<DETECTION_OBJECT,int>>> scoredObjects;
    DETECTION_OBJECT filterd_detections[MAX_DETECTIONS];
    int32_t newCount;
    const float SCORED_THRESHOLD = 0.4; 

    
    // Step 1: Separate mobile goals and rings from the detection list
    for (int i = 0; i < map->detectionCount; i++) 
    {
        if (map->detections[i].classID == 0) 
            MobileGoals.push_back(make_tuple(map->detections[i],i));
        else 
            Rings.push_back(make_tuple(map->detections[i],i));
    }
    
    for (int j = 0; j < MobileGoals.size(); j++) 
    {
        vector<tuple<DETECTION_OBJECT,int>> scored;
        for (int i = 0; i < Rings.size(); i++) 
        {
    
            float dx = get<0>(Rings[i]).mapLocation.x - get<0>(MobileGoals[j]).mapLocation.x;
            float dy = get<0>(Rings[i]).mapLocation.y - get<0>(MobileGoals[j]).mapLocation.y;
            
            // Calculate 3D distance between ring and mobile goal
            float distance = sqrt(dx*dx + dy*dy);
            
            // If distance is less than threshold, mark as scored
            if (distance < SCORED_THRESHOLD) 
                scored.push_back(Rings[i]);
        }

        if(!scored.empty())
        {
            scored.push_back(MobileGoals[j]);
            scoredObjects.push_back(scored);
        }
    }

    // Print information about scored objects
    fprintf(fp, "\r\n----- Scored Objects Information -----\r\n");
    for(int i = 0; i < scoredObjects.size(); i++)
    {
        // Mobile goal is always the last element in the scored vector
        int mgIndex = scoredObjects[i].size() - 1;
        DETECTION_OBJECT mobileGoal = get<0>(scoredObjects[i][mgIndex]);
        
        fprintf(fp, "Mobile Goal %d @ position: x:(%.2f) y:(%.2f)\r\n", 
                i+1, mobileGoal.mapLocation.x, mobileGoal.mapLocation.y);
        
        // Print info about rings on this mobile goal (all elements except the last one)
        for(int j = 0; j < scoredObjects[i].size() - 1; j++)
        {
            // Determine ring color
            string ringColor = "Unknown";
            if(get<0>(scoredObjects[i][j]).classID == 1)
                ringColor = "Red";
            else if(get<0>(scoredObjects[i][j]).classID == 2)
                ringColor = "Blue";
            
            fprintf(fp, "  Ring %d: %s @ position: x:(%.2f) y:(%.2f)\r\n", 
                    j+1, ringColor.c_str(), 
                    get<0>(scoredObjects[i][j]).mapLocation.x, 
                    get<0>(scoredObjects[i][j]).mapLocation.y);
        }
        fprintf(fp, "\r\n");
    }
    fprintf(fp, "-----------------------------------\r\n");
    
    newCount = 0;
    set<int> scoredIndices;

    if(isScored) // Store all scored objects
    {
        
        for(int i = 0; i < scoredObjects.size(); i++)
        {
            for(int j = 0; j < scoredObjects[i].size(); j++)
            {
                int originalIndex = get<1>(scoredObjects[i][j]);
                scoredIndices.insert(originalIndex);
                
                // Add this object to the filtered detections
                filterd_detections[newCount++] = get<0>(scoredObjects[i][j]);
            }
        }
    }
    else // Store all objects that are not scored
    {
   
        for(int i = 0; i < scoredObjects.size(); i++)
        {
            for(int j = 0; j < scoredObjects[i].size(); j++)
            {
                int originalIndex = get<1>(scoredObjects[i][j]);
                scoredIndices.insert(originalIndex);
            }
        }
        
        // Now add all objects that are not in the scoredIndices set
        for(int i = 0; i < map->detectionCount; i++)
        {
            if(scoredIndices.find(i) == scoredIndices.end())
            {
                filterd_detections[newCount++] = map->detections[i];
            }
        }
    }

    // Copy the filtered detections back to the map
    for(int i = 0; i < newCount; i++)
    {
        map->detections[i] = filterd_detections[i];
    }
    map->detectionCount = newCount;
}



///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


DETECTION_OBJECT findTarget(int type, bool isScored = false)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;

    if(type != 0)
    {
        isScored = false;
    }

    findScored(&local_map, isScored); // Update Local map to remove scored or unscored ubjects 

    if(local_map.detectionCount > 0)
    {
    fprintf(fp,"\r\nNumber of objects in local map: %ld ",local_map.detectionCount);
        for (int i = 0; i < local_map.detectionCount; i++)
        {
            if(local_map.detections[i].classID == type)
            {
                if(local_map.detections[i].probability > 0.70 && local_map.detections[i].probability <= 1) 
                {
                    double Obj_Dist = distanceTo(local_map.detections[i].mapLocation.x*39.37, local_map.detections[i].mapLocation.y*39.37,inches);
                    if (Obj_Dist < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = Obj_Dist; 
                    }
                }
            } 
        }
    }
    else
    {
        if(target.mapLocation.x < -3 || target.mapLocation.x > 3 )
        {
            //target.mapLocation.x = 0.00;
            //target.classID = 99;
        }
        if(target.mapLocation.y < -3 || target.mapLocation.y > 3 )
        {
            //target.mapLocation.y = 0.00;
            target.classID = 99;
        }
    }
    
    fprintf(fp,"\r\n(findtarget)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",
            target.mapLocation.x, target.mapLocation.y, target.classID, target.probability);
    return target;
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void GetRing()
{
    int Ring_Color = 0;
    if (Side == RED)
        Ring_Color = 1;
    if (Side == BLUE)
        Ring_Color = 2;

    float TurnStep = 45;

    DETECTION_OBJECT target = findTarget(Ring_Color);
    // while(target.mapLocation.x == 0 || target.mapLocation.y == 0)
    while(target.classID == 99)
    {
        fprintf(fp,"\r\n(GetRing) Inside turn loop: New heading: %.2f\n", Chassis.get_absolute_heading() + TurnStep );
        Chassis.turn_to_angle(Chassis.get_absolute_heading() + TurnStep);
        wait(1.5,sec);
        target = findTarget(Ring_Color);
        
    }
    fprintf(fp,"\r\n(GetRing) Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
    ScoreRing(target);
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void ScoreRing(DETECTION_OBJECT Target_Ring)
{
    fprintf(fp,"\r\n(ScoreRing) Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",Target_Ring.mapLocation.x, Target_Ring.mapLocation.y, Target_Ring.classID, Target_Ring.probability );
    float Ring_Offset = 6;
    float Tx = Target_Ring.mapLocation.x*39.37 ; 
    float Ty = Target_Ring.mapLocation.y*39.37 ;
    double X_Pos = GPS.xPosition(vex::distanceUnits::in);
    double Y_Pos = GPS.yPosition(vex::distanceUnits::in);
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading(vex::rotationUnits::deg) - targetheading);
    double result = (diff <= 180.0) ? diff : 360.0 - diff;
    if((result > 90))
    {
        targetheading +=  180;
    }
    Chassis.set_heading(GPS.heading(deg));
    Chassis.turn_to_angle(targetheading);
    Intake.spin(fwd);
    //Drive Function
    float distance = distanceTo(Tx, Ty);
    distance = distance - Ring_Offset;
    Chassis.drive_distance(distance);  
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void GetMobileGoal()
{ 
    //add code thats loops scans for mobile goal 
    DETECTION_OBJECT target = findTarget(0);
    // while(target.mapLocation.x == 0 || target.mapLocation.y == 0)
    float TurnStep = 45;

    while(target.classID == 99)
    {
        Chassis.turn_to_angle(Chassis.get_absolute_heading() + TurnStep);
        wait(1.5,sec);
        target = findTarget(0);
    }
    //fprintf(fp,"\r\n(FindMobile Goal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
    GrabMobileGoal(target);
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void GrabMobileGoal(DETECTION_OBJECT Target_MG)
{
    //fprintf(fp,"\r\n(GrabMobileGoal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",Target_MG.mapLocation.x, Target_MG.mapLocation.y, Target_MG.classID, Target_MG.probability );
    Clamp.set(false);
    float MG_Offset = 18;
    float Tx = Target_MG.mapLocation.x*39.37 ; 
    float Ty = Target_MG.mapLocation.y*39.37 ;
    double X_Pos = GPS.xPosition(vex::distanceUnits::in);
    double Y_Pos = GPS.yPosition(vex::distanceUnits::in);
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading(vex::rotationUnits::deg) - targetheading);
    double result = (diff <= 180.0) ? diff : 360.0 - diff;
    if((result > 90))
    {
        targetheading +=  180;
    }
    Chassis.set_heading(GPS.heading(deg));
    Chassis.turn_to_angle(targetheading);
    //Drive Function
    float distance = distanceTo(Tx, Ty);
    distance = distance - MG_Offset ;
    Chassis.drive_distance(distance);
    wait(20,msec);
    Chassis.turn_to_angle(targetheading+180);
    Chassis.drive_with_voltage(-5,-5);
    bool detectMG = false;
    float MG_max = 70;
    float MG_min = 60;
    while(!detectMG)
    {
        if (MobileGoal_Optical.hue() >= MG_min && 
            MobileGoal_Optical.hue() <= MG_max)
        {
            detectMG = true;
        }
    }
    wait(150,msec);
    Clamp.set(true);
    Chassis.drive_with_voltage(0,0);    
}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////









///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////




