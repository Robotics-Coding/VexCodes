/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\emani                                            */
/*    Created:      Fri Jun 19 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 2, 3, 4, 7   
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

float start_val = 50.0;
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

/*
  while(Drivetrain.heading()<90 || Drivetrain.heading()>340){

  }
  */


  /*
  Drivetrain.setTurnVelocity(50, pct);
  Drivetrain.turnFor(90, deg, false);
  
  while(Drivetrain.isTurning()){
    Drivetrain.setTurnVelocity(50, pct);
  }
  */
/*
  
  Drivetrain.driveFor(700, mm, 30,rpm, true);

  Drivetrain.driveFor(-1000, mm, -30, rpm, true);

*/

  Brain.Screen.print(Drivetrain.heading());


  Drivetrain.turnToHeading(90, degrees,80, rpm);
  Brain.Screen.print(Drivetrain.heading());
  wait(150, msec);
  


  Brain.Screen.newLine();

  if(Drivetrain.heading()>90){
    Drivetrain.turnToHeading(90, degrees,-5, rpm);

    wait(500, msec);

  }
  else{
    Drivetrain.turnToHeading(90, degrees,5, rpm);

    wait(500, msec);

  }
  
  
  
  Brain.Screen.print(Drivetrain.heading());
  
  //Drivetrain.turnFor(-90, degrees);
  
  
  


  
}
