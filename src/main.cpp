/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "ai_functions.h"


using namespace std;
using namespace vex;
competition Competition;
int startOfInteractionTime = 0;

int breakOutTime = 55;

//Realsense Offsets (15in, 24in)
// X(0.25in,0in), Y(-4.25in,12in), Z(9.125in,11in), Heading(0,0), Elevation(0,0)

// GPS Offsets (15in, 24in)
// X(0,0), Y(-6.5in,8in), Z(9.875,11in), Heading(180,180)
/////********************************************************/////

/////*********STOP*************STOP*************STOP*********/////
/////**DONT FORGET TO DEFINE OR COMMENT IN "robot-config.h"**/////
/////********************************************************/////
/////**********Red Side = true || Blue Side = false**********/////
/////********************************************************/////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void pre_auton(void) 
{

  MogoOptical.setLightPower(100,pct);
  IntakeOptical.setLightPower(100,pct);
  MogoOptical.setLight(vex::ledState::on);
  IntakeOptical.setLight(vex::ledState::on);

  Intake.setVelocity(100,pct);
  tuned_constants();

  Chassis.Gyro.calibrate();
  while (Chassis.Gyro.isCalibrating()) 
  {
    wait(25, msec);
  }
  LGPS.calibrate();
  RGPS.calibrate();
  while (LGPS.isCalibrating() || RGPS.isCalibrating() ) 
  {
    wait(25, msec);
  }
  wait(50, msec);
  Brain.Screen.clearScreen();
  // Chassis.Gyro.setHeading(GPS.heading(),deg);
}



bool Astate = false;
bool Xstate = false;
bool Ystate = false;

void usercontrol(void) 
{
  Brain.Timer.reset();
  Intake.setVelocity(100,pct);
  //Hang.setVelocity(100,pct);
  float ThrTest = 0;
  while(1)
  {
    LeftDriveSmart.spin(fwd,Controller.Axis3.position(), pct);
    RightDriveSmart.spin(fwd,Controller.Axis2.position(),pct);
#ifdef MANAGER_ROBOT
    if(Controller.ButtonL1.pressing())
      Arm.spin(fwd,100,pct);

    else if(Controller.ButtonL2.pressing())
      Arm.spin(fwd,-100,pct);
    
    else
      Arm.stop(hold);
#endif

      if(Controller.ButtonR1.pressing())
      Intake.spin(fwd,100,pct);

    else if(Controller.ButtonR2.pressing())
      Intake.spin(fwd,-100,pct);
    
    //else
     // Intake.stop();

    if(Controller.ButtonA.pressing())
    {
      if(!Astate)
      Clamp.set(!Clamp.value());  
      Astate = true;
    }
    else
      Astate = false;

#ifdef MANAGER_ROBOT
    if(Controller.ButtonX.pressing())
      {
      if(!Xstate)
      Top.set(!Top.value());  
      Xstate = true;
    }
    else
      Xstate = false;



    if(Controller.ButtonY.pressing())
      {
        if(!Ystate)
        IntakePiston.set(!IntakePiston.value());  
        Ystate = true;
      }
      else
        Ystate = false;
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool firstAutoFlag = true;

// int endGameTimer()
// {
//   int time2//Hang = 30;
//   while(Match.time(vex::timeUnits::sec)< time2//Hang)
//   {
//     tak::sleep(1000);
//   }
//   wait2Hang = false;

//   return 0;
// }
bool Starting = true;




void testing_tuning()
{
  fprintf(fp, "\r MOGO VIABLE \n");

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
  firstAutoFlag = false;
  #ifdef MANAGER_ROBOT
  auto_Isolation_24();
  #else
  auto_Isolation_15();
  #endif
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
  #ifdef MANAGER_ROBOT
  auto_Interaction_24();
  #else
  auto_Interaction_15();
  #endif

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

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
}


int main() {


  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;
  // Run at about 15Hz
  int32_t loop_time = 33;
  // start the status update display
  thread t1(dashboardTask);
  //thread t2(IntakeOn);
  pre_auton(); 

  // Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(autonomousMain);
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

      //fprintf(fp,"\r Intake %.2f\n",Intake.position(degrees));
      
        // fprintf(fp,"\r FindRing %.1f\n",Intake.torque(vex::torqueUnits::InLb));
        // fprintf(fp,"\r  %.1f\n",Match.time(vex::timeUnits::sec));
      //fprintf(fp,"\rLocal Map Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",local_map.pos.az,local_map.pos.x*100,local_map.pos.y*100);
      //fprintf(fp,"\rGPS Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));
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
      //jetson_comms.request_map();
      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}