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
#include <string>
#include <iostream>
#include<stdio.h> 
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


void wheels_down() {

  roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  highRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  index_roller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  
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

//revs is distance from beginning and dist is from end:

double speed_calc(double revs, double dist){
  double speed;
  double max_speed = -170; ///THIS IS THE MAX SPEED

  if(revs >= -0.25){
    //speed = -40+240*revs;
    speed = -40 + (-40 - max_speed)/(0.25) * revs;
  }
  else if(dist >= -1.0){
    //speed = -40 + 84*dist;
    speed = -40 + (-40 - max_speed)/(1.0)*dist;
  }
  
  else{
    speed = max_speed;
  }
  
  

  return speed;
}


double speed_calc_back(double revs, double dist){
  double speed;
  double max_speed = 100; ///THIS IS THE MAX SPEED

  if(revs <= 0.25){
    //speed = -40+240*revs;
    speed = 40 + (40 + max_speed)/(0.25) * revs;
  }
  else if(dist <= 1.0){
    //speed = -40 + 84*dist;
    speed = 40 + (40 + max_speed)/(1.0)*dist;
  }
  
  else{
    speed = max_speed;
  }
  
  

  return speed;
}



//SPEED CALCULATOR FOR TIMED GRADIENT:

double speed_calc_2(double revs){
  double speed;
  double max_speed = -160;

  if(revs >= -0.25){
    speed = speed = -40 + (-40 - max_speed)/(0.25) * revs;;
  }

  
  else{
    speed = max_speed;
  }
  
  

  return speed;
}


double speed_calc_timer(double current_time, double time_left){
  double speed;

  if(current_time <= 0.25){
    speed = -40-240*current_time;
  }

  else if(time_left <= 0.25){
    speed = -40-240*time_left;
  }

  
  else{
    speed = -100;
  }
  
  

  return speed;
}





//GRADIENT DRIVING FUNCTION:

void gradient_drive(double revs){
  RightBack.setRotation(0, rev);
  LeftBack.setRotation(0, rev);
  RightFront.setRotation(0, rev);
  LeftFront.setRotation(0, rev);
  
  wait(0.3,sec);

    while(RightFront.rotation(rev)> revs){
      double reading = RightFront.rotation(rev);
      double speed = speed_calc(reading, revs-reading);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();


      Drivetrain.drive(fwd, speed, rpm);

      wait(1,msec);

    }

    Drivetrain.stop(hold);

  }


  void gradient_back(double revs){
  RightBack.setRotation(0, rev);
  LeftBack.setRotation(0, rev);
  RightFront.setRotation(0, rev);
  LeftFront.setRotation(0, rev);
  
  wait(0.3,sec);

    while(RightFront.rotation(rev)< revs){
      double reading = RightFront.rotation(rev);
      double speed = speed_calc_back(reading, revs-reading);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();


      Drivetrain.drive(fwd, speed, rpm);

      wait(1,msec);

    }

    Drivetrain.stop(hold);

  }

  void gradient_on_time(double revs){
    double initial = Brain.timer(sec);
    double initial_rot = RightFront.rotation(rev);
    double target = -(revs + 0.3375)/(100/60) + 0.5;
    Brain.Screen.print("TARGET: ");
    Brain.Screen.print(target);
    Brain.Screen.newLine();
  

    while(RightFront.rotation(rev) - initial_rot> revs){
      double reading = Brain.timer(sec) - initial;
      double time_left = target -reading;
      double speed = speed_calc_timer(reading, time_left);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(time_left);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();

      wait(100,msec);

      

      Drivetrain.drive(fwd, speed, rpm);

    }

    Drivetrain.stop(hold);

  }



  void gradient_drive_no_reset(double revs){
    double initial = RightFront.rotation(rev);
  
  //wait(0.3,sec);

    while(RightFront.rotation(rev)> revs + initial){
      double reading = RightFront.rotation(rev)- initial;
      double dist = revs-reading;
      double speed = speed_calc(reading, dist);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();


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
  //Brain.Screen.print(t_1);

  while(Brain.timer(msec)-t_1 <= t){
      double reading = RightFront.rotation(rev);

      
      double speed = speed_calc_2(reading);
      Drivetrain.drive(fwd, speed, rpm);
    }


  Drivetrain.stop(coast);
 
}



//MAKING A TURN WITH A TRESHOLD OF ERROR (DO NOT CHANGE)
void make_turn(double speed, double ang, double tresh){

  while(Drivetrain.heading() < ang-tresh || Drivetrain.heading() > ang +tresh){

    Drivetrain.turn(right,speed, rpm);

  }
  Drivetrain.stop();

  /*if(speed < 0){
    //CLOCKWISE:
    while(Drivetrain.heading() < ang-tresh){
      Drivetrain.turn(right,speed, rpm);
      
    }

    Drivetrain.stop();

  }

  else{

    //COUNTER CLOCKWISE:
    while(Drivetrain.heading() > ang+tresh){
      Drivetrain.turn(right,speed, rpm);
      
    }

    Drivetrain.stop();
  }
  */
  
  
  
}

double turn_calc(double head, double ang, double dir, double in_h){
  double speed;
  double real_ang;
  double real_head;

  if (dir < 0){

    if (in_h >ang){
      real_ang = ang + 360;

      if(head < ang){
        real_head = head + 360;

      }
      else{
        real_head = head;
      }

    }

    else{
      real_ang = ang;
      real_head = head;
    }




    if(real_ang - real_head < 45){

      //speed = -85/45*(ang-head) -5;
      speed = (85)/(pow(45,0.7)) * pow(45-real_ang+real_head,0.7) - 90;


    }
    else{
      speed = -90;

    }

  }



//THIS IS FOR COUNTER CW
  else{

    if (in_h  < ang){
      real_ang = ang - 360;

      if(head > ang){
        real_head = head - 360;

      }
      else{
        real_head = head;
      }

    }

    else{
      real_ang = ang;
      real_head = head;
    }

    if(real_head- real_ang < 45){

      speed = -(85)/(pow(45,0.7)) * pow(45-real_head+real_ang,0.7) + 90;


    }
    else{
      speed = 90;

    }
  }



  return speed;
}

void grad_turn(double ang, double dir, double t){
  double in_heading = Drivetrain.heading();
  while(Drivetrain.heading() < ang-1 || Drivetrain.heading() > ang +1){

    double curr_heading = Drivetrain.heading();
    double speed = turn_calc(curr_heading, ang, dir, in_heading);
    

    Drivetrain.turn(right,speed, rpm);



    
    /*char result[50];
    char result2[50]; 
    sprintf(result, "%f", speed);
    sprintf(result2, "%f", curr_heading);

    printf("\n Speed %s", result);
    printf(" Heading %s", result2);
    wait(40,msec);*/

  }
  Drivetrain.stop(hold);
  //wait(0.1,sec);

  if(dir < 0){

    Drive_Right.spin(fwd, -50, rpm);
    Drive_Left.stop(hold);

    wait(t,msec);
    Drivetrain.stop(hold);

  }

  else{

    Drive_Left.spin(fwd, -50, rpm);
    Drive_Right.stop(hold);

    wait(t,msec);
    Drivetrain.stop(hold);


  }
  
}


//TURN TO A SPECIFIC VALUE WITH CORRECTION:


void auton_turn(double speed, double ang, double t_wait){
  make_turn(speed, ang, 10);

  wait(100,msec);
  if(Drivetrain.heading() > ang-0.5){
      make_turn(9,ang,1.0);
    }
    
    else if(Drivetrain.heading() < ang+0.5){
      make_turn(-9,ang,1.0);
    }

  if(speed < 0){

  
    Drive_Left.spin(fwd, -50, rpm);
    Drive_Right.stop(hold);

    wait(t_wait,msec);
    Drivetrain.stop(hold);

  }

  else if(speed > 0){

    Drive_Right.spin(fwd, -50, rpm);
    Drive_Left.stop(hold);

    wait(t_wait,msec);
    Drivetrain.stop(hold);

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
  wait(1.0,sec);
  roller_stop();
  intake_stop();

}


//MODIFY TIME TO ACCOUNT FOR TWO BALLS IN CORNER:

void shoot_corner_balls(){

  spinwheels();

  wait(1.1,sec);

  roller_stop();
  intake_stop();

}


//SPITS OUT BALLS FOR 1.5 sec:

void poop(){


  bottom_out();
  wheels_down();
  intake_out();

  wait(1.5,sec);

  roller_stop();
  intake_stop();
  

}


void drive_straight_test(){
  double initial = Brain.timer(sec);
  TurnGyroSmart.setHeading(90, degrees);
  while(Brain.timer(sec)-initial< 4){
    double current_heading = Drivetrain.heading();
    double correction = 1/3*(current_heading-90);

    Drive_Left.spin(fwd, -50 + correction, rpm);
    Drive_Right.spin(fwd, -50 - correction, rpm);

    
  }

  Drivetrain.stop(hold);
}



void poop_and_back(double dist){
  Drivetrain.driveFor(fwd,dist, inches, 100, rpm, false);

  wait(0.5, sec);

  poop();

  while(Drivetrain.isMoving()){
    wait(20,msec);
}
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






  //gradient_drive(-1.0);


  





  //1. LAUNCH THE PRELOAD INTO GOAL A:
    
    spin_top_wheels();
    wait(0.5,sec);
    roller_stop();
  //2. DRIVE TOWARDS THE FIRST BALL:
    intake_spin();
    //spin_bottom_wheels();
    
    //gradient_drive(-4.9);
    gradient_drive(-1.4);
    intake_stop();
    grad_turn(270, 1,120);
    roller_stop();
    gradient_drive(-3.05);
  
    //3. TURN FOR GOAL B AND SHOOT:
    
    grad_turn(180,1,120);
    
    wait(10,msec);
    intake_spin();
    timed_gradient(-2.4, 700);//1875);
    shoot_balls();
    
    //4. MOVE BACK, SPIT OUT BLUE, AND PICK UP NEXT BALL (near the corner):
    
    //auton_drive_forward(100.0, 20.0);
    //intake_stop();
    //roller_stop();
    poop_and_back(20.0);
    
   // grad_turn(254,-1);
    auton_turn(-100, 254, 120);
  
    intake_spin();
    spin_bottom_wheels();
    gradient_drive(-3.85);
    //5. TURN AND SHOOT THE BALL IN GOAL C:
 
    grad_turn(233, 1,0);
    roller_stop();
    
    
    timed_gradient(-2.4, 700);  
    shoot_corner_balls();
    
    //auton_drive_forward(80.0, 22.2); 
    poop_and_back(22.2);
    
    ///// FIRST ROW FINISHED, SECOND ROW BEGINS
  // 6. TURN AND PICKUP BALL FOR GOAL D:
  //grad_turn(0, -1);
  auton_turn(-80, 0, 120);
  intake_spin();
  spin_bottom_wheels();
  gradient_drive(-3.15); //SECOND VALL 
  
  

  

  
  //7. TURN FOR GOAL D AND SHOOT
  grad_turn(270,1,120);
  roller_stop();
  
  
  
  intake_spin();
  timed_gradient(-2.0, 700);
  shoot_balls(); 

  
  //auton_drive_forward(100.0, 21.0);
  //intake_stop();
  //roller_stop();
  poop_and_back(21.0);
    
  //8. GET BALL FOR GOAL E:
    
  //grad_turn(360, -1);

  auton_turn(-90,0, 130);
  intake_spin();
  gradient_drive(-3.6);
  wait(0.1,sec);
  intake_stop();
  //9. TURN AND SHOOT GOAL E:
  grad_turn(270, 11,120);
  gradient_drive(-0.9);
  grad_turn(315, -1,0);

  
  intake_spin();
  timed_gradient( -1.7, 700);
  shoot_corner_balls();
 
  auton_drive_forward(100.0, 26);
  make_turn(-80, 0, 15);
  poop();

  
 //10. TURN FOR WALL BALL NEAR E
  
  //grad_turn(270, 1);
  auton_turn(90, 270, 130);
  intake_spin();
  spin_bottom_wheels();
  gradient_drive(-1.6);

  //11. BACK UP AND TURN TO I
  

  gradient_back(4.12);
  intake_stop();
  roller_stop();
  grad_turn(180,1,120);
  timed_gradient(-3, 2000);
  shoot_balls();


  //TURN FOR G
  
  gradient_back(0.9);
  //grad_turn(58, 1);
  auton_turn(90, 58, 80);

  intake_spin();
  timed_gradient(-4.0, 3000);

  shoot_corner_balls();

  poop_and_back(22.0);
  

  //timed_gradient(-1.7, 700);  
  
  /*

  intake_spin();

  gradient_drive(-3.1);
  intake_stop();*/
  



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
      
    
    //Drive_Left.spin(vex::directionType::fwd, -0.9*deadzone(Controller1.Axis1.value()) - 1.8*deadzone(Controller1.Axis3.value()), vex::velocityUnits::pct);
    Drive_Right.spin(vex::directionType::fwd, 0.8*deadzone(Controller1.Axis1.value()) - 0.7*Controller1.Axis3.value(), vex::velocityUnits::pct);
    Drive_Left.spin(vex::directionType::fwd,-0.8*deadzone(Controller1.Axis1.value()) - 0.7*Controller1.Axis3.value(), vex::velocityUnits::pct);
    
    Brain.Screen.print(-0.6*deadzone(Controller1.Axis1.value()) - 0.8*Controller1.Axis3.value());
    Brain.Screen.newLine();

    
      
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
