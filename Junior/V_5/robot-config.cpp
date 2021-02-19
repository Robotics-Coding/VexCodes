#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftBack = motor(PORT5, ratio18_1, true);
motor RightBack = motor(PORT2, ratio18_1, false);
motor LeftFront = motor(PORT4, ratio18_1, true);
motor RightFront = motor(PORT6, ratio18_1, false);

motor_group Drive_Left = motor_group(LeftBack, LeftFront);
motor_group Drive_Right = motor_group(RightBack, RightFront);


motor_group LeftDriveSmart = motor_group(LeftBack, LeftFront);
motor_group RightDriveSmart = motor_group(RightBack, RightFront);


motor Intake = motor(PORT17, ratio18_1, false);
motor Intake2 = motor(PORT20, ratio18_1, false);
motor highRoller = motor(PORT16, ratio18_1, false);
motor roller = motor(PORT15, ratio18_1, false);

motor index_roller = motor(PORT11, ratio18_1, false);
controller Controller1 = controller(primary);


inertial TurnGyroSmart = inertial(PORT18);
smartdrive Drivetrain= smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart,319.19, 320, 165, mm, 1);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();

  //INSERT UI OVER HERE
}
