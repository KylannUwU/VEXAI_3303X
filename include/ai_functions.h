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

#include <field.h>
#include <tuple>
#include <set>


extern Field field;
// Calculates the distance to a given target (x, y)
float distanceTo(double target_x, double target_y,vex::distanceUnits unit);
double calculateBearing(double currX, double currY, double targetX, double targetY);
void moveToPoint(Point* Target, bool FrontFacing);
void MovetoRing(Point* Target);
// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta, bool GetBall = false, int Dspeed = 100, int Tspeed = 100);
// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(bool CheckSide, bool CheckIso);
bool CheckBallColor();
// Retrieves an object (e.g. from the ground or a dispenser)
bool getObject(bool CheckSide, bool CheckIso);
void ScoreBall();
void TakeOffAllianceTriball();
void Hanging();
void TouchMidPost();
#if defined(MANAGER_ROBOT)
int TurnHangTo(int deg);
bool GetMatchLoad(bool IsShooting, bool IsCheckingForBall);
void Move2Drop_Pos();
void ThrowBall();
void BlockIntake();
void Shooting(int Targt);
void StartMech();
void VHanging();
void ScoreAllianceTriball();
#endif

bool HoldingMogo();
void GetMogo();
int IntakeControl_24();
int IntakeControl_15();

void auto_Isolation_24();
void auto_Isolation_15();



DETECTION_OBJECT Multi_CheckforMogo();
DETECTION_OBJECT findTarget(int Type, bool isScored);