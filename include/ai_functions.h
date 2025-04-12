/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "robot-config.h"
#include "field.h"
#include <field.h>
#include <tuple>
#include <set>


extern Field field;
// Calculates the distance to a given target (x, y)
float distanceTo(double target_x, double target_y,vex::distanceUnits unit);
double calculateBearing(double currX, double currY, double targetX, double targetY);
void moveToPoint(Point* Target, bool FrontFacing);
void MovetoRing(Point* Target);
void moveToPosition(double target_x, double target_y, double target_theta, bool GetBall = false, int Dspeed = 100, int Tspeed = 100);

DETECTION_OBJECT findTarget(bool CheckSide, bool CheckIso);

#if defined(MANAGER_ROBOT)
void armControl(double target);
#endif

bool HoldingMogo();
void GetMogo();
int IntakeControl();


void auto_Interaction_24();
void auto_Interaction_15();

void auto_Isolation_24();
void auto_Isolation_15();

void GetMobileGoal();
void GrabMobileGoal(DETECTION_OBJECT Target_MG);


DETECTION_OBJECT Multi_CheckforMogo();
DETECTION_OBJECT findTarget(int Type, bool isScored);