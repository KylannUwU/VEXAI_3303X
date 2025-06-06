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

#define MogoHue 60


// Calculates the distance to the coordinates from the current robot position

//GENERAL Variables
bool ValidTarget = false;
bool ValidTargetMogo = false;
bool MogoIsFull = false;




#pragma region MainFuncs

float distanceTo(double target_x, double target_y, vex::distanceUnits unit = vex::distanceUnits::in)
{
    float distance = sqrt(pow((target_x - GPS.xPosition()), 2) + pow((target_y - GPS.yPosition()), 2));
    
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
        double X_Pos = GPS.xPosition();
        double Y_Pos = GPS.yPosition();
        // Check to see if we have arrived to target 
        double threshold = pow((X_Pos - Target->Xcord), 2) + pow((Y_Pos - Target->Ycord),2);
        if(threshold <= pow(ThresholdRad, 3))
        {   
                fprintf(fp,"\rRobot is within the threshold of target\n");
                break;
        }
        // Turn Function
        double intialHeading = calculateBearing(X_Pos, Y_Pos, Target->Xcord, Target->Ycord);
        double diff = fabs(GPS.heading() - intialHeading);
        double result = (diff <= 180.0) ? diff : 360.0 - diff;

        if((result > 90))
        {
            intialHeading +=  180;
        }
        Chassis.set_heading(GPS.heading());
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
        float intialHeading = calculateBearing(GPS.xPosition(), GPS.yPosition(), Target->Xcord, Target->Ycord);
        Chassis.set_heading(GPS.heading());
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        Chassis.drive_distance(distance - field.MG_Offset);



}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, bool GetMobile, bool GetRing, int Dspeed, int Tspeed)
{
    Chassis.drive_max_voltage = Dspeed * 0.12;
    Chassis.turn_max_voltage = Tspeed * 0.12;

    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(), GPS.yPosition());

    if (!field.Check_Obstacle_Intersects(&CurrentPoint, &Target, true))
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
                    if(GetRing)
                    {   
                        double angle = calculateBearing(GPS.xPosition(), GPS.yPosition(), Path2Follow.PathPoints[i]->Xcord, Path2Follow.PathPoints[i]->Ycord);
                        Chassis.turn_to_angle(angle);
                    }
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




void findScored(AI_RECORD* map, bool isScored)
{
    // Temporarily store mobile goals and rings
    vector<tuple<DETECTION_OBJECT,int>> MobileGoals;
    vector<tuple<DETECTION_OBJECT,int>> Rings;
    vector<vector<tuple<DETECTION_OBJECT,int>>> scoredObjects;
    DETECTION_OBJECT filterd_detections[MAX_DETECTIONS];
    int32_t newCount;
    const float SCORED_THRESHOLD = 0.1; 

    
    // Separate mobile goals and rings from the detection map
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
                        fprintf(fp,"\rFound Viable Object at (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
                    }
                }
            } 
        }
    }
    else
    {
        if(target.classID != 0  && target.classID != 1 && target.classID != 2 && abs(target.mapLocation.x) > 1.75 || abs(target.mapLocation.y) > 1.75  )
        {
            //target.mapLocation.x = 0.00;
            target.classID = 99;
        }
        if(abs(target.mapLocation.x) > 1.3 && abs(target.mapLocation.y) > 1.3)
            target.classID = 99;
        // if(target.mapLocation.y < -3 || target.mapLocation.y > 3 )
        // {
        //     //target.mapLocation.y = 0.00;
        //     target.classID = 99;
        // }
    }
    if(target.classID != 0  && target.classID != 1 && target.classID != 2 || abs(target.mapLocation.x) > 1.75 || abs(target.mapLocation.y) > 1.75  )
        {
            //target.mapLocation.x = 0.00;
            target.classID = 99;
        }
        if(abs(target.mapLocation.x) > 1.3 && abs(target.mapLocation.y) > 1.3)
            target.classID = 99;
    
    fprintf(fp,"\r\n(findtarget)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
   

    return target;
}



// void IntakeOn()
// {
//     while(1)
//     {
//         if(!Intake.isSpinning())
//             Intake.spin(fwd);
//         wait(20,msec);
//     }
// }

void ScoreRings()
{
    auto path = getPathToRings();
    task ic(IntakeControl);
    for (const auto& point : path) 
            {
               
                
                fprintf(fp,"Moviendose a: (%.2f, %.2f)\n", point.first, point.second);
                moveToPosition(point.first, point.second,-1,false,true,60,60);
                Chassis.drive_distance(3);
        }

}
bool Act = false;

void ScoreRing(DETECTION_OBJECT target)
{    

    int intakeOffset = 0;
    if(abs(target.mapLocation.x) < 175 && abs(target.mapLocation.y) < 175)
    intakeOffset = -5;
    else
    intakeOffset = 10;
    
    float Tx = target.mapLocation.x*100 ; 
    float Ty = target.mapLocation.y*100 ;
    double X_Pos = GPS.xPosition();
    double Y_Pos = GPS.yPosition();
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading() - targetheading);
    double dx = Tx - X_Pos;
    double dy = Ty - Y_Pos;
    double Totaldistance = distanceTo(Tx, Ty);
    double unitX = dx / Totaldistance;
    double unitY = dy / Totaldistance;
    double newDistance = Totaldistance - intakeOffset;

    double newX = X_Pos + unitX * newDistance;
    double newY = Y_Pos + unitY * newDistance;
    
    fprintf(fp,"\r Original pos %.2f, %.2f, new pos %.2f, %.2f \n", Tx, Ty, newX, newY);
    
    // if(!Act)
    //     thread t3(IntakeControl);
    // Intake.spin(fwd,100,pct);
    moveToPosition(Tx,Ty,-1);
   // double angle = calculateBearing(GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm),target.mapLocation.x* 100,target.mapLocation.y* 100 );
    // double distance = distanceTo(target.mapLocation.x* 100, target.mapLocation.y* 100);
    // Chassis.set_heading(GPS.heading(deg));
    // Chassis.turn_to_angle(angle);
    // Chassis.drive_distance(distance - intakeOffset);
    wait(300,msec);
    Chassis.drive_distance(-5);

    
}
#ifdef MANAGER_ROBOT
void MoveandScoreWallStake()
{
    float desiredAngle = 180;
    static float initAngle = 0;
    
    Chassis.drive_with_voltage(4,4);
    armControl(desiredAngle);
    wait(200,msec);
    armControl(initAngle);
    Chassis.drive_distance(-15);
    Top.set(false);
}

void scoreClosestWallStake()
{
    static Point
    nWallStake(0, 160),
    pWallStake(0, -160);
    float targetX, targetY, theta;

    double negativedistance = distanceTo(nWallStake.Xcord, nWallStake.Ycord);
    double positivedistance = distanceTo(nWallStake.Xcord, nWallStake.Ycord);
     
    if(negativedistance < positivedistance)
    {
        fprintf(fp,"\rScoring at Negative Stake\n");
        targetX = nWallStake.Xcord;
        targetY = nWallStake.Ycord;
        theta = 0;
    }
    else
    {
        fprintf(fp,"\rScoring at Positive Stake\n");
        targetX = pWallStake.Xcord;
        targetY = pWallStake.Ycord;
        theta = 180;
    }


    moveToPosition(targetX, targetY, theta);
    MoveandScoreWallStake();
    
}
#endif
void getRing(bool ScoreWallstake = false)
{
    int type;
    int turn_step = 45;
    int turnItr = 0;
    bool ringDetected = false;

    if(Side == RED)
    type = 1;
    else
    type = 2;
    
    DETECTION_OBJECT target = findTarget(type);
    while(!ringDetected)
    {
        target = findTarget(type);
        if(target.classID == type)
        {
            fprintf(fp,"\r Ring detected at  (%.2f , %.2f)\n",target.mapLocation.x, target.mapLocation.y);
            ringDetected = true;
            break;
        }
        else
        {
            if(turnItr > 4){
                if(Side == RED)
                moveToPosition(-80,-80, -1,false);
                else
                moveToPosition(80,-80, -1,false);
                turnItr=0;
            }
            Chassis.turn_max_voltage = 9;
            fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading() + turn_step);
            Chassis.turn_to_angle(GPS.heading() + turn_step);
            turnItr=turnItr+1;
            vex::wait(500,msec);
            target = findTarget(type);
        }
    }
    #ifdef MANAGER_ROBOT
    if(scoreClosestWallStake)
 
    Top.set(true);
    #endif
    ScoreRings();
    #ifdef MANAGER_ROBOT
    
    if(ScoreWallstake)
        scoreClosestWallStake();
        #endif
    
}




#ifdef MANAGER_ROBOT
void armControl(double target) {

     double KpLB = 20;
     double KiLB = 0.0;
     double KdLB = 0.1;

     double tolerance = 2;
    double previousErrorLB = 0;
    double integralValueLB = 0;
    bool done = false;
    while (!done) {
        double encoderValue = ArmRotation.angle(vex::rotationUnits::deg);
        double error = target - encoderValue;
        
        if (abs(error) <= tolerance) {
            Arm.stop(hold);
            done = true; 
        }

        integralValueLB += error;
        double derivative = error - previousErrorLB;
        double output = KpLB * error + KiLB * integralValueLB + KdLB * derivative;


        if (output > 100) output = 100;
        else if (output < -100) output = -100;
        fprintf(fp, "\r Target: %.2f Current: %.2f Error: %.2f Output: %.2f \n",
            target,     encoderValue    ,error  ,output );

        Arm.spin(fwd, static_cast<int>(output), pct);

        previousErrorLB = error;

        wait(20, msec); 
    }
    Arm.stop(hold);
}

#endif



void GetMobileGoal()
{ 
    // while(!HoldingMogo())
    // {
    //add code thats loops scans for mobile goal 
    DETECTION_OBJECT target = findTarget(0);
    // while(target.mapLocation.x == 0 || target.mapLocation.y == 0)
    float TurnStep = 45;

    while(target.classID == 99 )
    {
        Chassis.turn_to_angle(Chassis.get_absolute_heading() + TurnStep);
        wait(1.5,sec);
        target = findTarget(0);
    }
    //fprintf(fp,"\r\n(FindMobile Goal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",target.mapLocation.x, target.mapLocation.y, target.classID, target.probability );
    
        GrabMobileGoal(target);
        fprintf(fp,"\r\n MOGO GRAB %d\n", HoldingMogo());
    
   // }
    fprintf(fp,"\r\n OUT OF WHILE LOOP\n");

}


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


void GrabMobileGoal(DETECTION_OBJECT Target_MG)
{
    //fprintf(fp,"\r\n(GrabMobileGoal)Returning target: \r\nPosition:(%.2f, %.2f) \r\nClass ID:%ld \r\nProbability:%.2f \n",Target_MG.mapLocation.x, Target_MG.mapLocation.y, Target_MG.classID, Target_MG.probability );
    Clamp.set(false);
    
    float MG_Offset = 10;
    float Tx = Target_MG.mapLocation.x*100 ; 
    float Ty = Target_MG.mapLocation.y*100 ;
    double X_Pos = GPS.xPosition();
    double Y_Pos = GPS.yPosition();
    double targetheading = calculateBearing(X_Pos, Y_Pos, Tx, Ty);
    double diff = fabs(GPS.heading() - targetheading);
    double dx = Tx - X_Pos;
    double dy = Ty - Y_Pos;
    double Totaldistance = distanceTo(Tx, Ty);
    double unitX = dx / Totaldistance;
    double unitY = dy / Totaldistance;
    double newDistance = Totaldistance - MG_Offset;

    double newX = X_Pos + unitX * newDistance;
    double newY = Y_Pos + unitY * newDistance;



    double result = (diff <= 180.0) ? diff : 360.0 - diff;
    if((result > 90))
    {
        targetheading +=  180;
    }
    moveToPosition(Tx,Ty,-1,true);
    // Chassis.set_heading(GPS.heading(deg));
    // Chassis.turn_to_angle(targetheading);
    // //Drive Function
    // float distance = distanceTo(Tx, Ty);
    // distance = distance - MG_Offset ;
    // Chassis.drive_distance(distance);
    // wait(20,msec);
    fprintf(fp,"\r Turning to mogo\n");
    Chassis.turn_to_angle(targetheading+180);
    Chassis.drive_with_voltage(-5,-5);
    bool detectMG = false;
    float MG_max = 70;
    float MG_min = 60;
    int StartTime = Match_timer.time(msec);
    while(!detectMG)
    {
        int currentTime = Match_timer.time(msec);
        if (MogoOptical.hue() >= MG_min && 
        MogoOptical.hue() <= MG_max)
        {
            detectMG = true;
        }
        if(abs(StartTime - currentTime) > 5000)
            detectMG = true;
    }
    fprintf(fp,"\r Clamping mogo\n");
    wait(150,msec);
    Clamp.set(true);
    Chassis.drive_with_voltage(0,0);   

}



void GetMogo()
{
    int turn_step = 45;
    int turnItr = 0;

    Clamp.set(false);

    
    while(!HoldingMogo())
    {
        Clamp.set(false);
        DETECTION_OBJECT target = findTarget(0);
        if(target.classID == 0)
        {   
            Clamp.set(false);

            #ifdef MANAGER_ROBOT
            
            #else
            Chassis.drive_max_voltage = 9;
            #endif
            fprintf(fp, "\r MOGO FOUNDED\n");
            //Chassis.set_heading(GPS.heading(deg));
             double TargetAngle = calculateBearing(GPS.xPosition(), GPS.yPosition(), target.mapLocation.x * 100, target.mapLocation.y * 100);
            // double targetDistance = distanceTo(target.mapLocation.x * 100, target.mapLocation.y* 100);
            // Chassis.turn_to_angle(TargetAngle);
            // Chassis.drive_distance(targetDistance-mogoOffset);
            moveToPosition(target.mapLocation.x,target.mapLocation.y,-1,true);
            vex::wait(250,msec);

            double desiredAngle = TargetAngle + 180; 
            Chassis.turn_to_angle(desiredAngle);
            float startPos = Chassis.get_left_position_in();
            
            while ((MogoOptical.hue() < MogoHue - 10 || MogoOptical.hue() > MogoHue + 10) && Chassis.get_left_position_in() > (startPos -25))                
                Chassis.drive_with_voltage(-8,-8);
            fprintf(fp, "\r hue is %d \n", MogoOptical.hue());
            Chassis.drive_with_voltage(-8,-8);
            wait(200,msec);
            Clamp.set(true);
            wait(90,msec);
            Chassis.drive_with_voltage(0,0);
            wait(400,msec);

          
            
        }
        else
        {
            if(turnItr > 6){
                getRing(true);
                turnItr=0;
            }
            //fprintf(fp,"\rSeanning for ball....\n");
            Chassis.turn_max_voltage = 9;
                 
            fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading() + turn_step);
            Chassis.turn_to_angle(GPS.heading() + turn_step);
            turnItr=turnItr+1;
            vex::wait(500,msec);
            
            
        }
  

    }
    MogoIsFull = false;
}


std::vector<DETECTION_OBJECT> findAllTargets(int type, bool isScored = false)
{
    std::vector<DETECTION_OBJECT> targets;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);

    if (type != 0) {
        isScored = false;
    }

    findScored(&local_map, isScored); 

    if (local_map.detectionCount > 0) {
        //fprintf(fp, "\r\nNumber of objects in local map: %ld ", local_map.detectionCount);
        for (int i = 0; i < local_map.detectionCount; i++) {
            DETECTION_OBJECT obj = local_map.detections[i];

            if (obj.classID == type && obj.probability > 0.70 && obj.probability <= 1) {
                // Filtro de posiciones "válidas"
                if (abs(obj.mapLocation.x) <= 1.75 && abs(obj.mapLocation.y) <= 1.75) {
                   // fprintf(fp, "\r\n Objeto valido: (%.2f, %.2f), clase: %ld, prob: %.2f", 
                    //        obj.mapLocation.x, obj.mapLocation.y, obj.classID, obj.probability);
                    targets.push_back(obj);
                } else {

                }
            }
        }
    }

    return targets;


}



std::vector<std::pair<double, double>> getPathToRings() {
   
    int type = (Side == RED) ? 1 : 2;
   
    std::vector<DETECTION_OBJECT> rings = findAllTargets(type);
    
    // Ordenar por distancia al punto actual
    std::sort(rings.begin(), rings.end(), [=](const DETECTION_OBJECT& a, const DETECTION_OBJECT& b) {
        double distA = distanceTo( a.mapLocation.x*100, a.mapLocation.y*100);
        double distB = distanceTo( b.mapLocation.x*100, b.mapLocation.y*100);
        return distA < distB;
    });

    // Limitar a máximo 6 anillos
    if (rings.size() > 6) {
        rings.resize(6);
    }

    double 
        currentX = GPS.xPosition(), 
        currentY = GPS.yPosition();

    std::vector<std::pair<double, double>> path;

    

    path.emplace_back(currentX, currentY); // Punto de partida

    for (const auto& ring : rings) {
        path.emplace_back(ring.mapLocation.x *100, ring.mapLocation.y *100);
    }

    fprintf(fp,"\r--- Path to Rings ---\n");
    for (size_t i = 0; i < path.size(); ++i) {
        fprintf(fp,"\rPunto %d: (%.2f, %.2f)\n", i, path[i].first, path[i].second);
    }
    fprintf(fp,"\r----------------------\n");


    return path;
}




#pragma endregion MainFuncs

#pragma region ControlFuncs

bool IntakeActivation = false;




bool isAllianceRing(bool sort = false)
{
    int hueTarget = (Side == RED) ? 350: 225,
        tolerance = 15,
        correctReadings = 0,
        totalReadings = 5;
    
    if(sort)
        hueTarget = (Side == RED) ? 225: 350;

    if (!IntakeOptical.isNearObject())
        return false;

    for (int i = 0; i < totalReadings; ++i)
    {
        int currentHue = IntakeOptical.hue();
        int diff = abs(currentHue - hueTarget);
        diff = min(diff, 360 - diff);  
        if (diff < tolerance)
            correctReadings++;

        wait(10,msec);
    }
    
    return (correctReadings >= 4);
}



// bool colorSort()
// {
//     int hueTarget = (Side == RED) ? 225 : 350,
//         tolerance = 15,
//         correctReadings = 0,
//         totalReadings = 5;

//     if (!IntakeOptical.isNearObject())
//         return false;

//     for (int i = 0; i < totalReadings; ++i)
//     {
//         int currentHue = IntakeOptical.hue();
//         int diff = abs(currentHue - hueTarget);
//         diff = min(diff, 360 - diff);  
//         if (diff < tolerance)
//             correctReadings++;

//         wait(16,msec);
//     }

//     return (correctReadings >= 4);
// }


// bool colorSort()
// {
//     bool isWrongColor = false;

//     int 
//         hue = (Side == RED) ? 225 : 350,
//         tolerance = 15,
//         currentHue = IntakeOptical.hue();
    
    

//     int diff = abs(currentHue - hue);
//     diff = min(diff, 360 - diff);  

//     if(diff < 15 && IntakeOptical.isNearObject())
//         isWrongColor = true;
//     //fprintf(fp,"\r diff is %d\n", diff);
//     return isWrongColor;



// }



int counterTask(int counter)
{
    static bool counterState = false;

    if(isAllianceRing())
    {
        if(!counterState)
        {
            counterState = true;
            counter = counter + 1 ;
            fprintf(fp,"\r RING DETECTADO, COUNTER ES %d \n",counter);
        }
    }

    else
    {
        if(counterState) 
            counterState = false;
    }
    return counter;
}



bool isStuck(int threshold)
{
    static double lastPosition = 0;
    static int stuckFrames = 0;

    double currentPosition = Intake.position(deg);
    double efficiency = Intake.efficiency();
    double diff = abs(currentPosition - lastPosition);


 //   fprintf(fp,"\r diff %.2f eff %.2f lastpos %.2f\n", diff, efficiency, lastPosition);

    if(diff < threshold && efficiency < 30)
        stuckFrames++;
    
    else
        stuckFrames = 0;

    lastPosition = currentPosition;

    return stuckFrames > 25;
}


int IntakeControl()
{
         
    #ifdef MANAGER_ROBOT    
    double Intakethreshold = 200;
    #else
    double Intakethreshold = 10;
    #endif
    
    // bool isActivated = true;

    int counter = 0,
    intakecounter = 0;
    bool
        counterState = false,
        sortState = false,
        posState;

    double 
        posCounter,
        lastPos;

    while(1)
    {
        
        while (counter < 6 && counter != -1)
        {
            
    
                
            


                #ifdef MANAGER_ROBOT
            if(!isStuck(Intakethreshold))
            {
                if(isAllianceRing(true))
                    sortState = true;
                
                if(!sortState)
                    Intake.spin(fwd,100,pct);
        
                else
                {
                    if(!Top.value())
                        Top.set(true);
                    
                    Intake.spin(fwd,100,pct);
                    if(isAllianceRing())
                    {
                        wait(600,msec);
                        Top.set(false);
                        sortState = false;
                    }

                }
               
        
            }
            
        
            else
            {
                Intake.spin(vex::directionType::rev, 100,pct);
                wait(400,msec);
            }

            #else
            if(!isStuck(Intakethreshold))
            {
                
                if(isAllianceRing(true))
                    sortState = true;
                
                if(!sortState)
                {
                    Intake.spin(fwd,100,pct);
                    counter = counterTask(counter);   
                }

                else
                {
                    lastPos = Intake.position(deg);
                    Intake1.spin(fwd,100,pct);
                    while( abs(Intake.position(deg) - lastPos) < 50)
                        wait(20,msec);

                    Intake1.spin(fwd,-100,pct);
                    wait(300,msec);
                    sortState = false;

                }
               
        
            }
            
        
            else
            {
                Intake.spin(vex::directionType::rev, 100,pct);
                wait(400,msec);
            }
            #endif
                  
            wait(10,msec);   
        }
        




        if(counter == 6)
        {
            wait(800,msec);
            counter = -1;
            MogoIsFull = true;
        }
        
        wait(10,msec);
        Intake.stop();
    
    }
}
    



bool HoldingMogo()
{
    bool Mogo = false;
    int counter = 0;

    while(counter < 5)
    {
        int hueValue = MogoOptical.hue(); // Guardamos el valor de hue en una variable
        
        // Verificamos si el valor es válido (por ejemplo, en el rango [0, 360])
        if (hueValue >= 0 && hueValue <= 360)
        {
            if(hueValue > 50 && hueValue < 80)
            {
                fprintf(fp, "\rHOLDINGGGGG MOGO, hue is %d \n", hueValue);
                Mogo = true;
                break;
            }
            else
            {
                fprintf(fp, "\rNO MOGO, hue is %d \n", hueValue);
                counter = counter + 1;
            }
        }
        else
        {
            fprintf(fp, "\rINVALID HUE VALUE: %d \n", hueValue);
        }

        wait(20,msec);
    }
    
    fprintf(fp, "\rMOGO  %d \n", Mogo);
    return Mogo;
}

void DropMogo()
{
    double lowestDist = 100000;
    Point closestPoint(0, 0);
    std::vector<Point> Points = 
    {
        Point(43, -43),   // dropPosBlue
        Point(-43, -43),  // dropPosRed
        Point(43, 43),    // dropNegBlue
        Point(-43, 43)    // dropNegRed
    };
    

    for (const Point& p : Points) 
    {
        double newDist = distanceTo(p.Xcord, p.Ycord); 
        if (newDist < lowestDist) {
            lowestDist = newDist;
            closestPoint = p;
        }
        
    }
    
    fprintf(fp, "\r Closest pont is %.2f , %.2f \n", closestPoint.Xcord, closestPoint.Ycord);
    
    

    // Chassis.set_heading(GPS.heading());
    // double angle = calculateBearing(GPS.xPosition(),GPS.yPosition(),closestPoint.Xcord,closestPoint.Ycord);
    // double distance = lowestDist;
    // int backOffset = 5;
    // Chassis.turn_to_angle(angle+180);
    // Chassis.drive_distance(-(distance-backOffset));
    // Clamp.set(false);
    moveToPosition(closestPoint.Xcord, closestPoint.Ycord);
    double angle = calculateBearing(GPS.xPosition(),GPS.yPosition(),closestPoint.Xcord*2,closestPoint.Ycord*2);
    Chassis.turn_to_angle(angle);
    
}



#pragma endregion ControlFuncs

#pragma region MAINTASKS

#ifdef MANAGER_ROBOT
int retArm()
{
    armControl(210);
    return(0);
}




void auto_Isolation_24()
{

    int 
        sideDef = 1,
        ringcolor;
    double 
        angletoSwing,
        leftswing,
        rightswing,
        mogoX,
        mogoY,
        sideCoordsMultiplier,
        target,
        angle;

    bool swing;


    if(Side == RED)
    {
        sideDef = -1;
        angletoSwing = 120;
        swing = Chassis.get_absolute_heading() > 350 || Chassis.get_absolute_heading() < 120;
        ringcolor = 1;
    }

    else
    {
        sideDef = 1;
        angletoSwing = 240;
        swing = Chassis.get_absolute_heading() > 250;
        ringcolor = 2;
    }
    
    Chassis.set_heading(0);

    if(Side == RED)
    Chassis.right_swing_to_angle(315);
    else
    Chassis.left_swing_to_angle(45);
    
    armControl(320);
    Top.set(true);
    task ret(retArm);
    Chassis.drive_distance(-20);
    
    Chassis.turn_to_angle(180);
    //float current = Chassis.get_left_position_in();
    //Chassis.drive_with_voltage(-8,-8);
    // while ((MogoOptical.hue() < MogoHue - 10 || MogoOptical.hue() > MogoHue + 10) && Chassis.get_left_position_in() > (current -20))
    // {
    //     wait(20,msec);
    // }
    Chassis.drive_distance(-28);
    fprintf(fp, "\r Clamp is: %d \n", Clamp.value());
    Clamp.set(true);
    wait(150,msec);
    Chassis.drive_with_voltage(0,0);
    Intake.spin(fwd,100,pct);
    fprintf(fp, "\r Clamp is: %d \n", Clamp.value());
    //int difference = current - Chassis.get_left_position_in();
    IntakePiston.set(true);
    Chassis.drive_distance(35);
    wait(100,msec);
    if(Side == RED)
    Chassis.turn_to_angle(135);
    else
    Chassis.turn_to_angle(225);
    Chassis.drive_distance(20);
    // angle = calculateBearing(GPS.xPosition(),GPS.yPosition(), 90* sideDef, -90);
    // Chassis.turn_to_angle(angle);   
    // target = distanceTo(95 *  sideDef, -95);
    // Chassis.drive_distance(target);

    // if(Side == RED)
    //     Chassis.turn_to_angle(135);
    // else
    //     Chassis.turn_to_angle(225);
    IntakePiston.set(false);
    Chassis.drive_distance(5);
    wait(250,msec);
    Chassis.drive_max_voltage = 5;
    Chassis.drive_distance(-22);
    Chassis.drive_max_voltage = 12;

    if(Side == RED)
        Chassis.turn_to_angle(195);
    else
        Chassis.turn_to_angle(165);

    Chassis.drive_distance(15);
    Chassis.drive_distance(-10);
    
    if(Side == RED)
    {
        Intake.stop();
        Chassis.turn_to_angle(175);
        Chassis.drive_distance(14);
        Chassis.left_swing_to_angle(265);
        Doinker.set(true);
        Chassis.drive_distance(10);
        Chassis.turn_to_angle(310);
        Doinker.set(false);
        Chassis.drive_distance(-5);
        Chassis.turn_to_angle(45);
        Chassis.drive_distance(-10);
        Clamp.set(false);
        Chassis.drive_distance(30);
        
    }

    else
    {
        Chassis.turn_to_angle(115);
        Chassis.drive_distance(12);
        Chassis.left_swing_to_angle(170);
        Chassis.drive_distance(11);
        Doinker.set(true);
        wait(150,msec);
        Chassis.turn_to_angle(220);
        Doinker.set(false);
        Chassis.drive_distance(-5);
        Chassis.turn_to_angle(185);
        // Doinker.set(true);
        // wait(200,msec);
        Chassis.drive_distance(-13);
        Doinker.set(false);
        // Chassis.turn_to_angle(175);
        // Chassis.drive_distance(20);
        // Chassis.drive_distance(-10);
        Chassis.turn_to_angle(325);
        Chassis.drive_distance(-18);
        Clamp.set(false);
        Chassis.turn_to_angle(315);
        Chassis.drive_distance(35);
        
    }



    // double currentPos = Chassis.get_left_position_in(); 
    // while(currentPos < 22)
    // {
    // currentPos = Chassis.get_left_position_in();
    // wait(20,msec);
    // }
    // if(Side == RED)
    // Chassis.drive_with_voltage(8,1);

    // else
    // Chassis.drive_with_voltage(1,8);

    

    // while(swing)
    // {
    // if(Side == RED)
    //     swing = Chassis.get_absolute_heading() > 350 || Chassis.get_absolute_heading() < 120;
    // else
    //     swing = Chassis.get_absolute_heading() > 250;
    //     fprintf(fp, "\r Chassis hd: %.2f \n", Chassis.get_absolute_heading());
    // wait(20,msec);
    // }
    // Chassis.drive_with_voltage(0,0);
    // Doinker.set(true);
    // Chassis.drive_with_voltage(-12,-12);
    // wait(600,msec);
    // Doinker.set(false);
    // Chassis.drive_with_voltage(0,0);

    // wait(100,msec);

    // target = distanceTo(110 *  sideDef, -60);
    // angle = calculateBearing(GPS.xPosition(),GPS.yPosition(), 110* sideDef, -60);
    // Chassis.turn_to_angle(angle);
    // Chassis.drive_distance(target);
    // Chassis.turn_to_angle(180);
    
    // float current = Chassis.get_left_position_in();
    // while ((MogoOptical.hue() < MogoHue - 10 || MogoOptical.hue() > MogoHue + 10) && Chassis.get_left_position_in() > (current -25))
    // {
    //     Chassis.drive_with_voltage(-8,-8);
    //     wait(20,msec);
    // }
    // wait(200,msec);
    // Clamp.set(true);
    // wait(90,msec);
    // Chassis.drive_with_voltage(0,0);
    // wait(200,msec);
    // target = distanceTo(110 * sideDef ,5);
    // fprintf(fp, "\r Target from %.2f , %.2f, running %.2f inches \n", GPS.xPosition(),GPS.yPosition() , target );
    // Chassis.drive_distance(target);

    
    // if(Side == RED)
    //     Chassis.turn_to_angle(270);
    // else
    //     Chassis.turn_to_angle(270);

        
}


void auto_Interaction_24()
{
    fprintf(fp,"\rAuton\n");
    while(1)
    {
        
        
        while(HoldingMogo)
        {
            fprintf(fp,"\r holding a mogo\n");
            if(!MogoIsFull)
                getRing();
             else
                 DropMogo();
        }
        GetMobileGoal();
        wait(1000,msec);


   }
    }



#else

void auto_Isolation_15()
{
    Intake.spin(fwd,-5,pct);
    Chassis.DriveL.resetPosition();
    Chassis.DriveR.resetPosition();
    LeftDriveSmart.resetPosition();
    RightDriveSmart.resetPosition();
    LeftDriveSmart.setStopping(brake);
    RightDriveSmart.setStopping(brake); 
    Chassis.set_heading(GPS.heading());
    Chassis.drive_with_voltage(11,11);
  
    double currentPos = Chassis.get_left_position_in(); 
   if(Side == RED)
    {
        while(currentPos < 28)
    {
      currentPos = Chassis.get_left_position_in();
      wait(20,msec);
    }
    
    }
    else
    {
        while(currentPos < 19)
    {
      currentPos = Chassis.get_left_position_in();
      wait(20,msec);
    }
}

    

    if(Side == RED)
    {
    Chassis.drive_with_voltage(1,8);
    while(Chassis.get_absolute_heading() > 85 )
    wait(20,msec);    
    }
    else
    {
    Chassis.drive_with_voltage(8,1);
    while(Chassis.get_absolute_heading() < 305 )
    wait(20,msec);    
    
    }
  
   
  
    Chassis.drive_with_voltage(0,0);
    Doinker.set(true);
    wait(500,msec);
    if(Side == RED)
    Chassis.drive_with_voltage(-6,-7);
    else
    Chassis.drive_with_voltage(-7,-6);
    wait(600,msec);
    Chassis.drive_with_voltage(-6,-6);
    wait(100,msec);
    Doinker.set(false);
    Chassis.drive_with_voltage(0,0);
    wait(1500,msec);
    Chassis.turn_to_angle(310);
  
    DETECTION_OBJECT target = findTarget(0);
    float mogoX = target.mapLocation.x * 100;
    float mogoY = target.mapLocation.y * 100; 
    bool mogoinside;
    if(Side == RED)
    mogoinside = mogoX < -1 && mogoX > -170;
    else
    mogoinside = mogoX > 1 && mogoX < 170;
    
    if(mogoinside)
    
    {
      fprintf(fp, "\r MOGO VIABLE \n");
      Chassis.set_heading(GPS.heading());
      double TargetAngle = calculateBearing(GPS.xPosition(), GPS.yPosition(), target.mapLocation.x * 100, target.mapLocation.y * 100);
      double desiredAngle = TargetAngle + 180; 
      fprintf(fp, "\r Target in %.2f , %.2f going from %.2f , %.2f\n",
              target.mapLocation.x * 100, target.mapLocation.y * 100,
              GPS.xPosition(), GPS.yPosition());
      fprintf(fp, "\r TURNING TO MOGO FROM %.2f to %.2f\n", TargetAngle, desiredAngle);
      Chassis.turn_to_angle(desiredAngle);
      fprintf(fp, "\r turning to %.2f \n", desiredAngle);
      fprintf(fp, "\r GOING BACK TO MOGO  \n"); 
      Chassis.drive_distance(-28);
      Clamp.set(true);
      wait(400,msec);
    }
  
    else
    {
      
      fprintf(fp, "\r MOGO NOT VIABLE AT X: %.2f\n",  mogoX);
      if(Side == RED)
      moveToPosition(-25,10, -1,true);
      else
      moveToPosition(25,10, -1,true);
      
      if(Side == RED)
      Chassis.turn_to_angle(120);
      else
      Chassis.turn_to_angle(240);
      Doinker.set(true);
      Chassis.drive_distance(-35);
      Doinker.set(false);
      Chassis.drive_distance(-5);
    }
    target = findTarget(0);
    mogoX = target.mapLocation.x * 100;
    mogoY = target.mapLocation.y * 100; 
    if(mogoinside)
    {
    Chassis.set_heading(GPS.heading());
    double TargetAngle = calculateBearing(GPS.xPosition(), GPS.yPosition(), target.mapLocation.x * 100, target.mapLocation.y * 100);
    double desiredAngle = TargetAngle + 180; 
    fprintf(fp, "\r Target in %.2f , %.2f going from %.2f , %.2f\n",
            target.mapLocation.x * 100, target.mapLocation.y * 100,
            GPS.xPosition(), GPS.yPosition());
    fprintf(fp, "\r TURNING TO MOGO FROM %.2f to %.2f\n", TargetAngle, desiredAngle);
    Chassis.turn_to_angle(desiredAngle);
    fprintf(fp, "\r turning to %.2f \n", desiredAngle);
    fprintf(fp, "\r GOING BACK TO MOGO  \n"); 
    Chassis.drive_distance(-28);
    Clamp.set(true);
    wait(400,msec);
    }
   

    Intake.spin(fwd);


    if(Side == RED)
    moveToPosition(-90,90,315);
    else
    moveToPosition(90,90,45);
    

    Chassis.drive_distance(20);
    wait(100,msec);



    if(Side == RED)
    moveToPosition(-60,60,180);
    else
    moveToPosition(60,60,180);
    
    
    Chassis.drive_with_voltage(3,3);
    wait(4000,msec);
}

void auto_Interaction_15()
{
    fprintf(fp,"\rAuton\n");
         
    while(1)
    {

        while(!HoldingMogo())
        {
            GetMobileGoal();
            wait(200,msec);
            
        }
        fprintf(fp,"\r holding a mogo\n");
            if(!MogoIsFull)
            {
                getRing();
                moveToPosition(120,120);
                Chassis.turn_to_angle(180);
            }
            else
                DropMogo();
      wait(200,msec);
    }
//         Clamp.set(true);

//         wait(600,msec);
//         task intakect(IntakeControl);
//         auto path = getPathToRings();

//         for (const auto& point : path) 
//         {
           
            
//             fprintf(fp,"Moviendose a: (%.2f, %.2f)\n", point.first, point.second);
//             moveToPosition(point.first, point.second);
//         }


//         wait(20,msec);
// //    }

}




#endif
#pragma endregion MAIN TASKS