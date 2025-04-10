/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "robot-config.h"
using namespace std;
using namespace vex;

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
#define  Alliance  true

#if(Alliance)
#pragma message("Selected Red Side")
#else
#pragma message("Selected Blue Side")
#endif

brain Brain;
#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT15, "24in 3303X", linkType::manager );
//24in Robot Specific Objects
motor leftDriveA = motor(PORT8, ratio6_1, true);  
motor leftDriveB = motor(PORT9, ratio6_1, false);   
motor leftDriveC = motor(PORT10, ratio6_1, true);   
motor rightDriveA = motor(PORT1, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, true);
motor rightDriveC = motor(PORT3, ratio6_1, false);
//motor Catapult = motor(PORT11,ratio36_1,true);
//rotation CatapultEnc = rotation(PORT16);
//limit CatapultLimit = limit(Brain.ThreeWirePort.D);
//optical Balldetect = optical(PORT6);
motor Intake1 = motor(PORT12, ratio6_1, true);//
motor Intake2 = motor(PORT13, ratio6_1, true);
motor_group Intake = motor_group(Intake1, Intake2);

motor Arm = motor(PORT11, ratio36_1, false);
//motor //HangA = motor(PORT13, ratio36_1, false);
//motor //HangB = motor(PORT14, ratio36_1, true);
//rotation //HangEnc = rotation(PORT5);
gps GPS = gps(PORT6, 133, 80, mm, 90);

const int32_t InertialPort = PORT17;
double Robot_x_Offset = 8.75;
double Intake_Offset = 8;
double wheel_size = 2.75;
//digital_out R_Wing = digital_out(Brain.ThreeWirePort.C);
//digital_out L_Wing = digital_out(Brain.ThreeWirePort.E);
//digital_out  //Hang_Ratchet = digital_out(Brain.ThreeWirePort.H);
digital_out Doinker = digital_out(Brain.ThreeWirePort.C);
digital_out Clamp = digital_out(Brain.ThreeWirePort.E);
digital_out Top = digital_out(Brain.ThreeWirePort.F);
digital_out IntakePiston = digital_out(Brain.ThreeWirePort.D);
digital_out Claw = digital_out(Brain.ThreeWirePort.G);




#else

#pragma message("building for the worker")
ai::robot_link       link(PORT15, "15in 3303X", linkType::worker );
//15in Robot Specific Objects
motor leftDriveA = motor(PORT11, ratio6_1, true);  
motor leftDriveB = motor(PORT12, ratio6_1, true);   
motor leftDriveC = motor(PORT13, ratio6_1, true);    
motor rightDriveA = motor(PORT18, ratio6_1, false);
motor rightDriveB = motor(PORT19, ratio6_1, false);
motor rightDriveC = motor(PORT20, ratio6_1, false);
motor Intake = motor(PORT5, ratio6_1, true);
gps GPS = gps(PORT10, -10, -63.5, mm, 270);
const int32_t InertialPort = PORT7;
double Robot_x_Offset = 21;
double Intake_Offset = 10.5;
double wheel_size = 3.25;
digital_out Clamp = digital_out(Brain.ThreeWirePort.D);
digital_out Doinker = digital_out(Brain.ThreeWirePort.E);
optical IntakeOptical = optical(PORT6);
optical MogoOptical = optical(PORT9);

#endif


controller Controller = controller(primary);

competition Competition;
ai::jetson  jetson_comms;// create instance of jetson class to receive location and other

//Universal Objects (Do not comment out)
// Red Side = true || Blue Side = false
Field field(Alliance,Robot_x_Offset,Intake_Offset);
FILE *fp = fopen("/dev/serial2","wb");
timer Match = timer();
motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, wheel_size, 0.75, 360);
//motor_group //Hang = motor_group(//HangA, //HangB);
bool wait2Hang = true;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tuned_constants()
{
  #if defined(MANAGER_ROBOT)

  Chassis.set_turn_constants(12, .42, 0.004, 2.75, 15);
  Chassis.set_drive_constants(12, 1.1, 0.004, 4.8, 10);
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_swing_constants(12, .42, 0.004, 2.75, 15);
  Chassis.set_drive_exit_conditions(1.5, 300, 1500);
  Chassis.set_turn_exit_conditions(1, 300, 1500);
  Chassis.set_swing_exit_conditions(1, 300, 600);
  #else
  Chassis.set_drive_constants(12, 1.5, 0, 10, 0);
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_turn_constants(9, 0.25, 0.0005, 1.15, 15);
  Chassis.set_swing_constants(9, 0.25, 0.0005, 1.15, 15);
  Chassis.set_drive_exit_conditions(1.5, 300, 1500);
  Chassis.set_turn_exit_conditions(1, 300, 1000);
  Chassis.set_swing_exit_conditions(1, 300, 1000);
  #endif
}

void pre_auton(void) 
{


  Intake.setVelocity(100,pct);
  tuned_constants();

  Chassis.Gyro.calibrate();
  while (Chassis.Gyro.isCalibrating()) 
  {
    wait(25, msec);
  }
  GPS.calibrate();
  while (GPS.isCalibrating()) 
  {
    wait(25, msec);
  }
  wait(50, msec);
  Brain.Screen.clearScreen();
  Chassis.Gyro.setHeading(GPS.heading(),deg);
  Chassis.set_heading(GPS.heading());
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
        Claw.set(!Claw.value());  
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
  // Chassis.turn_to_angle(90);
  // Chassis.turn_to_angle(135);
  // Chassis.turn_to_angle(180);
  // Chassis.turn_to_angle(0);
  // moveToPosition(120,120,90,false,100,75);
  // moveToPosition(120,-120,180,false,100,75);
  // moveToPosition(-120,-120,270,false,100,75);
  // moveToPosition(-120,120,0,false,100,75); 
//  Chassis.drive_distance(24);
//  wait(500,msec);
//  Chassis.drive_distance(24);
//  wait(500,msec);
//  Chassis.drive_distance(-48);
  Chassis.DriveL.resetPosition();
  Chassis.DriveR.resetPosition();
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake); 

  Chassis.drive_with_voltage(11,11);

  double currentPos = Chassis.get_left_position_in(); 
  while(currentPos < 18)
  {
    currentPos = Chassis.get_left_position_in();
    wait(20,msec);
  }
  Chassis.drive_with_voltage(8,1);
  fprintf(fp, "\r %.2f \n", Chassis.get_absolute_heading());
  Chassis.drive_with_voltage(8,1);

  while(Chassis.get_absolute_heading() < 305)
  wait(20,msec);

  Chassis.drive_with_voltage(0,0);
  Doinker.set(true);
  wait(200,msec);
  Chassis.drive_with_voltage(-7,-5);
  wait(400,msec);
  Doinker.set(false);
  wait(100,msec);
  Chassis.drive_with_voltage(0,0);
  wait(1500,msec);

  DETECTION_OBJECT target = Multi_CheckforMogo();
  float mogoX = target.mapLocation.x * 100;
  float mogoY = target.mapLocation.y * 100; 


  if(mogoX > 1 && mogoX < 170)
  {
    fprintf(fp, "\r MOGO VIABLE \n");
    double currentAngle = GPS.heading(deg);
    double TargetAngle = calculateBearing(GPS.xPosition(), GPS.yPosition(), mogoX, mogoY);
    fprintf(fp, "\r Turning from %.2f to %.2f \n", currentAngle, TargetAngle);
    double desiredAngle = TargetAngle + 180; 
    if(desiredAngle > 360)
      desiredAngle = desiredAngle - 360;     
    Chassis.turn_to_angle(desiredAngle);
    fprintf(fp, "\r turning to %.2f \n", desiredAngle);
  }

  else
  {

    fprintf(fp, "\r MOGO NOT VIABLE AT X: %.2f\n",  mogoX);
    moveToPosition(40,20, 250);
    Doinker.set(true);
    Chassis.turn_to_angle(225);
    Chassis.drive_distance(-5);
    Doinker.set(false);
    Chassis.drive_distance(-5);
  }

  moveToPosition(110,110, 110);
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
  task ic(IntakeControl_24);

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

  
  while(1)
  {
    while(HoldingMogo())
    {
      fprintf(fp, "\r HOLDING A MOGO  \n");
      getObject(false,false);
    }
    fprintf(fp, "\r NO MOGO\n");
    GetMogo();
  }

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