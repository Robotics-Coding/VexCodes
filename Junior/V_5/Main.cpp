// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftBack             motor         12              
// RightBack            motor         14              
// LeftFront            motor         13              
// RightFront           motor         18              
// Intake               motor         20              
// Intake2              motor         19              
// Lift1                motor         10              
// roller               motor         15              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h" 
#include "math.h"
using namespace vex;
#include <cmath>
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void sleep(int time){
    vex::task::sleep(time);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


///THESE ARE ALL OF THE AUTON FUNCTIONS:
///
///
///


void stop(){
    RightBack.stop();
    RightFront.stop();
    LeftBack.stop();
    LeftFront.stop();
    
}
void auton_drive (int speed, int time){
    LeftBack.spin(vex::directionType::fwd,speed,vex::velocityUnits::rpm);
    LeftFront.spin(vex::directionType::fwd,speed,vex::velocityUnits::rpm);
    RightBack.spin(vex::directionType::rev,-speed,vex::velocityUnits::rpm);
    RightFront.spin(vex::directionType::rev,-speed,vex::velocityUnits::rpm);
    sleep(time);
    stop();
    
}


// SPINNING ALL OF THE SHOOTER ROLLERS:

void spinwheels() {

  roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  highRoller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  index_roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  
}


//SPINNING ONLY THE BOTTOM ROLLERS (spinning upwards):

void spin_bottom_wheels (){

  roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  index_roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

  
}

//SPINNING ONLY THE TOP ROLLERS (spinning upwards):

void spin_top_wheels (){
  highRoller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  
}

//SPINNING ONLY THE BOTTOM OUT:

void bottom_out (){

  roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  index_roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  
}


//STOP ROLLERS:

void roller_stop (){
  roller.stop();
  highRoller.stop();
  index_roller.stop();

}



//DRIVE FOR DISTANCE AT CONSTANT SPEED:

void auton_drive_forward(double speed, double dist){
  Drivetrain.driveFor(dist, inches, speed, rpm);

}

//SPEED CALCULATOR FOR GRADIENT:

double speed_calc(double revs, double dist){
  double speed;

  if(revs >= -0.25){
    speed = -40+240*revs;
  }
  else if(dist >= -0.25){
    speed = -40+240*dist;
  }
  
  else{
    speed = -130;
  }
  
  

  return speed;
}



//SPEED CALCULATOR FOR TIMED GRADIENT:

double speed_calc_2(double revs){
  double speed;

  if(revs >= -0.25){
    speed = -40+240*revs;
  }

  
  else{
    speed = -130;
  }
  
  

  return speed;
}





//GRADIENT DRIVING FUNCTION:

void gradient_drive(double revs){
  RightBack.setRotation(0, rev);
  LeftBack.setRotation(0, rev);
  RightFront.setRotation(0, rev);
  LeftFront.setRotation(0, rev);
  

    while(RightFront.rotation(rev)> revs){
      double reading = RightFront.rotation(rev);
      double speed = speed_calc(reading, revs-reading);

      Drivetrain.drive(fwd, speed, rpm);
    }

    Drivetrain.stop(hold);

  }




//TIMED GRADIENT FUNCTION:


void timed_gradient(double revs, double t){
  RightBack.setRotation(0, rev);
  LeftBack.setRotation(0, rev);
  RightFront.setRotation(0, rev);
  LeftFront.setRotation(0, rev);

  double t_1 = Brain.timer(msec);
  Brain.Screen.print(t_1);

    while(Brain.timer(msec)-t_1 <= t){
      double reading = RightFront.rotation(rev);

      
      double speed = speed_calc_2(reading);
      Drivetrain.drive(fwd, speed, rpm);
    }


  Drivetrain.stop(coast);
 
}



//MAKING A TURN WITH A TRESHOLD OF ERROR (DO NOT CHANGE)
void make_turn(double speed, double ang, double tresh){
  while(Drivetrain.heading() < ang-tresh || Drivetrain.heading() > ang+tresh){
      Drivetrain.turn(right,speed, rpm);
      
    }
  Drivetrain.stop();
  
}


//TURN TO A SPECIFIC VALUE WITH CORRECTION:


void auton_turn(double speed, double ang){
  make_turn(speed, ang, 10);
  wait(100,msec);
  if(Drivetrain.heading() > ang){
      make_turn(9,ang,1.0);
    }
    
    else if(Drivetrain.heading() < ang){
      make_turn(-9,ang,1.0);
    }


}



//INTAKE SUCKING IN:

void intake_spin(){
  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  Intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);

}


//INTAKE SPINNING OUT:

void intake_out(){
  Intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  Intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

}

//STOPS INTAKE
void intake_stop(){
  
    Intake.stop(vex::brakeType::hold);
    Intake2.stop(vex::brakeType::hold);
      
}


//SHOOTS BALLS:

void shoot_balls(){

  spinwheels();
  wait(1.1,sec);
  roller_stop();
  intake_stop();

}


//MODIFY TIME TO ACCOUNT FOR TWO BALLS IN CORNER:

void shoot_corner_balls(){

  spinwheels();

  wait(1.2,sec);

  roller_stop();
  intake_stop();

}


//SPITS OUT BALLS FOR 1.5 sec:

void poop(){

  bottom_out();
  intake_out();

  wait(1.5,sec);

  roller_stop();
  intake_stop();

}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {

  //1. LAUNCH THE PRELOAD INTO CORNER:
    
    spin_top_wheels();
    wait(0.5,sec);
    roller_stop();


  //2. DRIVE TOWARDS THE FIRST BALL IN THE MIDDLE:

    intake_spin();
    spin_bottom_wheels();
    gradient_drive(-4.84);

    intake_stop();
    roller_stop();

  
    //3. TURN FOR SECOND GOAL AND SHOOT:
    
    auton_turn(80, 180);
    wait(10,msec);

    intake_spin();

    timed_gradient(-2.4, 1875);
    shoot_balls();


    //4. MOVE BACK, SPIT OUT BLUE, AND PICK UP NEXT BALL (near the corner):
    
    auton_drive_forward(100.0, 20.0);

    intake_stop();
    roller_stop();

    poop();
    wait(0.3,sec);
    
    auton_turn(-100, 257);

    intake_spin();

    gradient_drive(-4.0);


    //5. TURN AND SHOOT THE BALL IN THRID TOWER:
 
    auton_turn(50,230);
    auton_drive(-100, 1000);  //can be changed to gradient + lowered at some point

    shoot_balls();

    auton_drive_forward(80.0, 20.0);

    poop();


    ///// FIRST ROW FINISHED, SECOND ROW BEGINS


  // 6. TURN AND PICKUP BALL FOR FOURTH GOAL:


  auton_turn(-80, 0);

  intake_spin();
  spin_bottom_wheels();


  gradient_drive(-3.15);
  roller_stop();
  intake_stop();


  //7. TURN FOR FOURTH GOAL AND SHOOT

  auton_turn(80,270);
  intake_spin();
  timed_gradient(-2.0, 700);

  shoot_balls();  
  auton_drive_forward(100.0, 22.0);

  intake_stop();
  roller_stop();

  poop();
  wait(0.3,sec);


  //8. GET BALL FOR FIFTH TOWER:


  auton_turn(-80,0);

  wait(0.1,sec);

  intake_spin();

  gradient_drive(-3.6);

  wait(0.1,sec);

  intake_stop();



  //9. TURN AND SHOOT FIFTH GOAL:

  auton_turn(80,300);

  intake_spin();

  timed_gradient( -1.7, 1800);

  shoot_balls();
  auton_drive_forward(100.0, 22.0);
  poop();


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



int deadzone(int num){
    
       return 0.5* num*pow(abs(num), 0.75)/(pow(127, 0.75));
       
    
}

void drive_forward() {
    LeftFront.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
}

void forward_turn(){
    RightBack.spin(vex::directionType::fwd, 0.4*deadzone(Controller1.Axis1.value()) - deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, 0.4*deadzone(Controller1.Axis1.value()) + deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, 0.4*deadzone(Controller1.Axis1.value()) - deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd, 0.4*deadzone(Controller1.Axis1.value()) + deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
}

void forward_slow(){
    RightBack.spin(vex::directionType::fwd, 0.2*deadzone(Controller1.Axis3.value()) - 0.1*deadzone(Controller1.Axis1.value()), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, 0.2*deadzone(Controller1.Axis3.value()) + 0.1*deadzone(Controller1.Axis1.value()), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, 0.2*deadzone(Controller1.Axis3.value()) - 0.1*deadzone(Controller1.Axis1.value()), vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd, 0.2*deadzone(Controller1.Axis3.value()) + 0.1*deadzone(Controller1.Axis1.value()), vex::velocityUnits::pct);
}




void turn(){
    LeftFront.spin(vex::directionType::fwd, (-Controller1.Axis4.value()), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, (Controller1.Axis4.value()), vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd, (-Controller1.Axis4.value()), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, (Controller1.Axis4.value()), vex::velocityUnits::pct);

}







void usercontrol( void ) {
  // User control code here, inside the loop
 
  while (1) {
    
    //Brain.Screen.print(speed);
    //wait(1, sec);
      
    //Brain.Screen.clearLine();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo 
    // values based on feedback from the joysticks
     
    wait(5,msec);
      
    
    Drive_Left.spin(vex::directionType::fwd, -0.4*deadzone(Controller1.Axis1.value()) - deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    Drive_Right.spin(vex::directionType::fwd, 0.4*deadzone(Controller1.Axis1.value()) - deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    

    
      
      //THIS IS FOR Roller Motors::

      //SYNCHROUSLY UP:
      
      if(Controller1.ButtonR1.pressing()){

        roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        highRoller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        index_roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
          
          
      } 
      //SYNCHRONOUSLY DOWN:
      else if(Controller1.ButtonR2.pressing()){
        roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        highRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        index_roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
          
      }

      //SPIT OUT A BALL:
       else if(Controller1.ButtonUp.pressing()){
        roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        highRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        index_roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        
          
      }
      //PULL IN A BALL:
      else if(Controller1.ButtonDown.pressing()){
        roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        highRoller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        index_roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        
          
      } 

    else if(Controller1.ButtonL1.pressing()){
        roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        highRoller.stop(vex::brakeType::hold);
        index_roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
        
          
      } 

      else if(Controller1.ButtonL2.pressing()){
        roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        highRoller.stop(vex::brakeType::hold);
        index_roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        
          
      } 

      

      else{
        roller.stop(vex::brakeType::hold);
        highRoller.stop(vex::brakeType::hold);
        index_roller.stop(vex::brakeType::hold);
      }





        
      
      
      
      
      
      
      

      ///
      ///Intake:
    
      
      
      if(Controller1.ButtonL1.pressing()){
          Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
          Intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);

      }
      else if(Controller1.ButtonL2.pressing()){
          Intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
          Intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

      }
      else {
        Intake.stop(vex::brakeType::hold);
        Intake2.stop(vex::brakeType::hold);
      }
      
      
      
      if(Controller1.ButtonY.pressing()){
         //PLACE ANY AUTON CODE HERE FOR SKILLS
          
      }
      


    
         
}
      
      
    
      
      
      
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to 
    // update your motors, etc.
    // ........................................................................
 
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }



//
// Main will set up the competition functions and callbacks.
//
int main() {
    
    //Run the pre-autonomous function. 
    
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    


    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      wait(100,msec);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}
