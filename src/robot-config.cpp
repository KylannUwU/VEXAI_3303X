#include "robot-config.h"


using namespace vex;

brain Brain;

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT19, "24in 3303X", linkType::manager );
//24in Shared Objects
motor leftDriveA = motor(PORT8, ratio6_1, true);  
motor leftDriveB = motor(PORT9, ratio6_1, false);   
motor leftDriveC = motor(PORT10, ratio6_1, true);   
motor rightDriveA = motor(PORT1, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, true);
motor rightDriveC = motor(PORT3, ratio6_1, false);
gps LGPS = gps(PORT15, -5.5, -8, vex::distanceUnits::cm, 270);
gps RGPS = gps(PORT4, 5.5, -8, vex::distanceUnits::cm, 90);
const int32_t InertialPort = PORT17;
const int32_t opt_Port = PORT15;
const int32_t MGopt_Port = PORT14;
double wheel_size = 2.75;
double Robot_x_Offset = 2;
double Intake_Offset = 10;
double MG_Offset = 10;
double Arm_Offset = 0;

//////////////////////////////////////////////////
////////////////24" Robot Specific////////////////
//////////////////////////////////////////////////
motor Arm = motor(PORT11, ratio18_1, true);
motor Intake1 = motor(PORT12, ratio6_1, true);
motor Intake2 = motor(PORT13, ratio6_1, true);

rotation ArmRotation = rotation(PORT16, true);
digital_out Top = digital_out(Brain.ThreeWirePort.F);
digital_out IntakePiston = digital_out(Brain.ThreeWirePort.D);
//digital_out Claw = digital_out(Brain.ThreeWirePort.G);

//////////////////////////////////////////////////
//////////////////////////////////////////////////

#else
#pragma message("building for the worker")
ai::robot_link       link(PORT2, "15in 3303X", linkType::worker );
//15in Shared Objects
motor leftDriveA = motor(PORT16, ratio6_1, true);  
motor leftDriveB = motor(PORT17, ratio6_1, true);   
motor leftDriveC = motor(PORT20, ratio6_1, false);    
motor rightDriveA = motor(PORT13, ratio6_1, false);
motor rightDriveB = motor(PORT14, ratio6_1, false);
motor rightDriveC = motor(PORT15, ratio6_1, true);
gps LGPS = gps(PORT2, -5.5, -8, vex::distanceUnits::cm, 270);
gps RGPS = gps(PORT5, 5.5, -8, vex::distanceUnits::cm, 90);
//gps GPS = gps(PORT1, 133, -80, mm, 270);



const int32_t InertialPort = PORT4;
const int32_t opt_Port = PORT6;
const int32_t MGopt_Port = PORT3;
double wheel_size = 3.25;
double Robot_x_Offset = 20;
double Intake_Offset = 12;
double MG_Offset = 15;
double Arm_Offset = 2;

//////////////////////////////////////////////////
////////////////15" Robot Specific////////////////
//////////////////////////////////////////////////
motor Intake1 = motor(PORT10, ratio6_1, true);
motor Intake2 = motor(PORT9, ratio6_1, true);

#endif

//////////////////////////////////////////////////
//////////////////////////////////////////////////



ai::jetson  jetson_comms;
FILE *fp = fopen("/dev/serial2","wb");
controller Controller = controller(primary);
Field field(Robot_x_Offset, Intake_Offset, MG_Offset, Arm_Offset);
timer Match_timer = timer();


motor_group Intake = motor_group(Intake1, Intake2);
motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, wheel_size, 0.75, 360);


DualGPS GPS = DualGPS(LGPS,RGPS, Chassis.Gyro, vex::distanceUnits::cm);

optical IntakeOptical = optical(opt_Port);
optical MogoOptical = optical(MGopt_Port);

digital_out Doinker = digital_out(Brain.ThreeWirePort.C);
digital_out Clamp = digital_out(Brain.ThreeWirePort.G);







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
    // GPS.calibrate();
    Brain.Screen.print("Calibrating GPS");
    // while (GPS.isCalibrating()) 
    // {
    //   wait(25, msec);
    // }

    MogoOptical.setLightPower(100,pct);
    MogoOptical.setLight(vex::ledState::on);

    IntakeOptical.setLightPower(100,pct);
    IntakeOptical.setLight(vex::ledState::on);

    // BtmIntakeOptical.setLightPower(100,pct);
    // BtmIntakeOptical.setLight(vex::ledState::on);


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