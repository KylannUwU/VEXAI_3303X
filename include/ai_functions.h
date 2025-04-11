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

double distanceTo(double target_x, double target_y, vex::distanceUnits units);
double calculateBearing(double currX, double currY, double targetX, double targetY);

void moveToPoint(Point* Target, bool FrontFacing);
void moveToPosition(double target_x, double target_y, double target_theta, bool GetRing = false, int Dspeed = 100, int Tspeed = 100);

void findScored(AI_RECORD* map, bool isScored);
DETECTION_OBJECT findTarget(int type, bool isScored);

void GetRing();
void ScoreRing(DETECTION_OBJECT Target_Ring);

void GetMobileGoal();
void GrabMobileGoal(DETECTION_OBJECT Target_MG);
void ScoreMobileGoal();

#if defined(MANAGER_ROBOT)
void GetWSRing();
void ScoreOnWall();
#endif






// // Retrieves an object (e.g. from the ground or a dispenser)
// bool getObject(bool CheckSide, bool CheckIso);
// void ScoreBall();
// void TakeOffAllianceTriball();
// void Hanging();
// void TouchMidPost();
// #if defined(MANAGER_ROBOT)
// int TurnHangTo(int deg);
// bool GetMatchLoad(bool IsShooting, bool IsCheckingForBall);
// void Move2Drop_Pos();
// void ThrowBall();
// void BlockIntake();
// void Shooting(int Targt);
// void StartMech();
// void VHanging();
// void ScoreAllianceTriball();
// #endif

// bool HoldingMogo();
// void GetMogo();
// int IntakeControl_24();
// int IntakeControl_15();

// void auto_Isolation_24();
// void auto_Isolation_15();



// DETECTION_OBJECT Multi_CheckforMogo();
// DETECTION_OBJECT findRing(bool CheckSide = true, bool CheckIso = false);
// DETECTION_OBJECT findMogo(bool CheckSide = true, bool CheckIso = false);
