#include "vex.h"
using namespace vex;

extern brain Brain;
extern Drive Chassis;  
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern gps GPS;
extern optical MogoOptical;
extern optical IntakeOptical;
extern digital_out Clamp;
extern digital_out Doinker;
extern controller Controller;
extern FILE* fp;
extern rotation ArmRotation;

extern int startOfInteractionTime;
extern int breakOutTime;
#define RED true
#define BLUE false

#define  MANAGER_ROBOT  1
#define  Alliance  true

#if defined(MANAGER_ROBOT)
extern motor_group Intake; 
extern motor Arm;
#else
extern motor Intake; 

#endif

