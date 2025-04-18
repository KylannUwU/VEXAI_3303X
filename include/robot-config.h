
#include "vex.h"
#include "field.h"
#include "DualGPS.h"

using namespace vex;

enum TeamColor 
{ 
  RED, 
  BLUE 
}; 




///////////////////////////////////
///////////////////////////////////
inline int Side = RED; 
//inline int Side = BLUE; 
//#define  MANAGER_ROBOT    1
///////////////////////////////////
///////////////////////////////////

extern ai::robot_link link;
extern brain Brain;
extern ai::jetson jetson_comms;
extern FILE *fp;
extern controller Controller1;
extern Field field;
extern timer Match_timer;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern Drive Chassis;


//Shared Sensors
extern gps LGPS;
extern gps RGPS;
extern DualGPS GPS;
extern optical IntakeOptical;
extern optical MogoOptical;

//Shared Pnuematics
extern digital_out Doinker;
extern digital_out Clamp;

extern controller Controller;



#if defined(MANAGER_ROBOT)
extern motor_group Intake;
extern motor Arm;
extern digital_out Top;
extern digital_out IntakePiston;
extern digital_out Claw;
extern rotation ArmRotation;
#else
extern motor Intake;
#endif



void tuned_constants();
void vexcodeInit( void );