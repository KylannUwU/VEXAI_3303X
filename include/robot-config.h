
#include "vex.h"
#include "field.h"

using namespace vex;


extern FILE *fp;
extern brain Brain;
extern controller Controller1;
extern Field field;


//Shared Sensors
extern gps GPS;

extern optical TopIntakeOptical;
extern optical BtmIntakeOptical;
extern optical MobileGoal_Optical;

//Shared Pnuematics
extern digital_out Doinker;
extern digital_out Clamp;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern Drive Chassis;

extern int Side;

enum TeamColor 
{ 
  RED, 
  BLUE 
}; 



#if defined(MANAGER_ROBOT)
extern motor_group Intake;
extern digital_out Top;
extern digital_out IntakePiston;
extern digital_out Claw;
#else
extern motor Intake;
#endif



void tuned_constants();
void vexcodeInit( void );







