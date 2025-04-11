#include "robot-config.h"


using namespace vex;

///////////////////////////////////
///////////////////////////////////
int Side = RED; 
//int Side = BLUE; 
//#define  MANAGER_ROBOT    1
///////////////////////////////////
///////////////////////////////////

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT15, "24in 3303X", linkType::manager );
//24in Shared Objects
motor leftDriveA = motor(PORT8, ratio6_1, true);  
motor leftDriveB = motor(PORT9, ratio6_1, false);   
motor leftDriveC = motor(PORT10, ratio6_1, true);   
motor rightDriveA = motor(PORT1, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, true);
motor rightDriveC = motor(PORT3, ratio6_1, false);
gps GPS = gps(PORT6, 133, 80, mm, 90);
const int32_t InertialPort = PORT17;
const int32_t T_IntakePort = PORT21;
const int32_t B_IntakePort = PORT20;
const int32_t Mogo_Port = PORT17;
double wheel_size = 2.75;

//////////////////////////////////////////////////
////////////////24" Robot Specific////////////////
//////////////////////////////////////////////////
motor Intake1 = motor(PORT12, ratio6_1, true);
motor Intake2 = motor(PORT13, ratio6_1, true);
motor_group Intake = motor_group(Intake1, Intake2);
digital_out Top = digital_out(Brain.ThreeWirePort.F);
digital_out IntakePiston = digital_out(Brain.ThreeWirePort.D);
digital_out Claw = digital_out(Brain.ThreeWirePort.G);

//////////////////////////////////////////////////
//////////////////////////////////////////////////

#else
#pragma message("building for the worker")
ai::robot_link       link(PORT15, "15in 3303X", linkType::worker );
//15in Shared Objects
motor leftDriveA = motor(PORT11, ratio6_1, true);  
motor leftDriveB = motor(PORT12, ratio6_1, true);   
motor leftDriveC = motor(PORT13, ratio6_1, true);    
motor rightDriveA = motor(PORT18, ratio6_1, false);
motor rightDriveB = motor(PORT19, ratio6_1, false);
motor rightDriveC = motor(PORT20, ratio6_1, false);
gps GPS = gps(PORT6, 133, 80, mm, 90);
const int32_t InertialPort = PORT17;
const int32_t T_IntakePort = PORT21;
const int32_t B_IntakePort = PORT20;
const int32_t Mogo_Port = PORT17;
double wheel_size = 3.25;
double Robot_x_Offset = 0;
double Intake_Offset = 0;

//////////////////////////////////////////////////
////////////////15" Robot Specific////////////////
//////////////////////////////////////////////////
motor Intake = motor(PORT5, ratio6_1, true);
#endif

//////////////////////////////////////////////////
//////////////////////////////////////////////////

controller Controller = controller(primary);
brain Brain;
ai::jetson  jetson_comms;
FILE *fp = fopen("/dev/serial2","wb");

optical TopIntakeOptical = optical(T_IntakePort);
optical BtmIntakeOptical = optical(B_IntakePort);
optical MobileGoal_Optical = optical(Mogo_Port);

digital_out Doinker = digital_out(Brain.ThreeWirePort.C);
digital_out Clamp = digital_out(Brain.ThreeWirePort.E);

motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, wheel_size, 0.75, 360);

Field field(Robot_x_Offset, Intake_Offset);
timer Match = timer();



void vexcodeInit( void ) 
{
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Device initialization...");
    
    Brain.Screen.setCursor(3, 1);
    wait(200, msec);
    Chassis.Gyro.calibrate();
    Brain.Screen.print("Calibrating Chassis Inertial");
    while (Chassis.Gyro.isCalibrating()) 
    {
      wait(25, msec);
    }
    
    Brain.Screen.setCursor(4, 1);
    wait(200, msec);
    GPS.calibrate();
    Brain.Screen.print("Calibrating GPS");
    while (GPS.isCalibrating()) 
    {
      wait(25, msec);
    }

    MobileGoal_Optical.setLightPower(100,pct);
    MobileGoal_Optical.setLight(vex::ledState::on);

    TopIntakeOptical.setLightPower(100,pct);
    TopIntakeOptical.setLight(vex::ledState::on);

    BtmIntakeOptical.setLightPower(100,pct);
    BtmIntakeOptical.setLight(vex::ledState::on);


    Intake.setVelocity(100,pct);
    tuned_constants();

    wait(50, msec);
    Brain.Screen.clearScreen();
    Chassis.Gyro.setHeading(GPS.heading(),deg);
    Chassis.set_heading(GPS.heading());
}

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