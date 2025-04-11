/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "robot-config.h"


using namespace std;
using namespace vex;

competition Competition;


void testing_tuning()
{
  
}

void pre_auton(void) 
{
  vexcodeInit();
}

void usercontrol(void) 
{
  while(true)
  {

  }
  
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) 
{ 

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Interaction(void) 
{

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) 
{
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();
  firstAutoFlag = false;
  

}


int main() 
{
  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;
  // Run at about 15Hz
  int32_t loop_time = 33;
  // start the status update display
  thread t1(dashboardTask);
  pre_auton(); 

  // Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(auto_Interaction);
  //Match.event(testing_tuning,10);
  // Competition.autonomous(autonomousMain);
  this_thread::sleep_for(loop_time);
  int counter = 0 ;
  //Controller1.Screen.clearScreen();
  while(1) 
  {
      
      jetson_comms.get_data( &local_map ); // get last map data
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );// set our location to be sent to partner robot

      counter += 1 ;
      if (counter > 15)
      {

          //fprintf(fp, "\r Pos %.2f\n",  Chassis.get_left_position_in());
        //fprintf(fp,"\rTimer Value: %.1f\n",Intake.torque(vex::torqueUnits::InLb));
        //fprintf(fp,"\rTimer Value: %.1f\n",Match.time(vex::timeUnits::sec));
        //fprintf(fp,"\rLocal Map Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",local_map.pos.az,local_map.pos.x*100,local_map.pos.y*100);
       // fprintf(fp,"\rGPS Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));
        //fprintf(fp, "\r Timer %.2lu \n", Brain.Timer.system()/1000);

       // DETECTION_OBJECT targetmogo = findMogo();
       // fprintf(fp, "\r Target Depth %.2f \n", targetmogo.depth );
        // Print the current match time in seconds to the controller's screen
        //Controller1.Screen.clearLine();
        //Controller1.Screen.print("Match Time: %d sec",  Brain.Timer.system()/1000);

        counter = 0 ;
      }
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();
      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}