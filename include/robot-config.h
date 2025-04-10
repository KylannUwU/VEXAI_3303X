#include "vex.h"
using namespace vex;

extern brain Brain;
extern Drive Chassis;  
extern gps GPS;
extern optical MogoOptical;
extern optical IntakeOptical;
extern digital_out Clamp;
extern digital_out Doinker;
extern controller Controller;
extern FILE* fp;


extern int startOfInteractionTime;
extern int breakOutTime;
#define RED true
#define BLUE false

//#define  MANAGER_ROBOT  1
#define  Alliance  true

#if defined(MANAGER_ROBOT)
extern motor_group Intake; 
#else
extern motor Intake; 

#endif

