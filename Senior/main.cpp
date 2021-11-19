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
  double max_speed = -190; ///THIS IS THE MAX SPEED

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
  double max_speed = 160; ///THIS IS THE MAX SPEED

  if(revs <= 0.25){
    //speed = -40+240*revs;
    speed = 40 + (-40 + max_speed)/(0.25) * revs;
  }
  else if(dist <= 1.0){
    //speed = -40 + 84*dist;
    speed = 40 + (-40 + max_speed)/(1.0)*dist;
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

double turn_calc(double head, double ang, double dir, double in_h, double p){
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
      speed = (97)/(pow(45,p)) * pow(45-real_ang+real_head,p) - 95;


    }
    else{
      speed = -95;

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

      speed = -(95)/(pow(45,p)) * pow(45-real_head+real_ang,p) + 95;


    }
    else{
      speed = 95;

    }
  }



  return speed;
}

void grad_turn(double ang, double dir,  double pow, double t){
  double in_heading = Drivetrain.heading();
  double last;
  while(Drivetrain.heading() < ang-4 || Drivetrain.heading() > ang +4){

    double curr_heading = Drivetrain.heading();
    double speed = turn_calc(curr_heading, ang, dir, in_heading, pow);
    
    last = speed;

    Drivetrain.turn(right,speed, rpm);



    
    /*char result[50];
    char result2[50]; 
    sprintf(result, "%f", speed);
    sprintf(result2, "%f", curr_heading);

    printf("\n Speed %s", result);
    printf(" Heading %s", result2);
    wait(40,msec);*/

  }

  
  Controller1.Screen.clearScreen();
  Controller1.Screen.print(last);
  Controller1.Screen.print(Drivetrain.heading());
  Controller1.Screen.print(turn_calc(Drivetrain.heading(), ang, dir, in_heading, pow));

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


void grad_turn_nc(double ang, double dir, double pow){
  double in_heading = Drivetrain.heading();
  while(Drivetrain.heading() < ang-3 || Drivetrain.heading() > ang +3){

    double curr_heading = Drivetrain.heading();
    double speed = turn_calc(curr_heading, ang, dir, in_heading, pow);
    

    Drivetrain.turn(right,speed, rpm);



    
  }
  Drivetrain.stop(hold);
  //wait(0.1,sec);

  
  
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

  wait(1.3,sec);

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






void poop_and_back(double dist){
  Drivetrain.driveFor(fwd,dist, inches, 100, rpm, false);

  wait(0.5, sec);

  poop();

  while(Drivetrain.isMoving()){
    wait(20,msec);
}
}



void speed_calc_swerve(double revs, double dist, double speeds[], double curvature){
  double speed_left;
  double speed_right;
  double max_speed = -190; ///THIS IS THE MAX SPEED

  if(revs >= -0.25){
    //speed = -40+240*revs;
    speed_left = -40 + (-40 - max_speed)/(0.25) * revs;
    speed_right = -40 + (-40 - max_speed)/(0.25) * revs;
  }
  else if(dist >= -1.5){
    //speed = -40 + 84*dist;
    speed_left = -190 + curvature;
    speed_right = -190 - curvature;

  }
  
  else{
    speed_left = max_speed;
    speed_right = max_speed;
  }
  

  speeds[0] = speed_left;
  speeds[1] = speed_right;

  //return speeds;
  
}


//revs is straightaway revs
void turn_swerve(double revs, double ang, double curvature, double t){
  double speeds[2];

    RightBack.setRotation(0, rev);
    LeftBack.setRotation(0, rev);
    RightFront.setRotation(0, rev);
    LeftFront.setRotation(0, rev);

    double t_0 = Brain.timer(msec);

    while(Drivetrain.heading()>ang && Brain.timer(msec)- t_0 < t){
      double reading = RightFront.rotation(rev);

      speed_calc_swerve(reading, revs-reading, speeds, curvature);

      
      //double speed = speed_calc_swerve(reading, -1.5-reading);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(-revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speeds[1]);
      Brain.Screen.newLine();


      Drive_Left.spin(fwd, speeds[0], rpm);
      Drive_Right.spin(fwd, speeds[1], rpm);

      wait(1,msec);

    }



    Drivetrain.drive(fwd,-190, rpm);

}


void speed_calc_swerve_left(double revs, double dist, double speeds[], double curvature){
  double speed_left;
  double speed_right;
  double max_speed = -190; ///THIS IS THE MAX SPEED

  /*if(revs <= -0.25){
    if(Drivetrain.heading(deg) > 90.1){

      speed_left  = -20;
      speed_right = -20- 5*(Drivetrain.heading(deg)-90);

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" >L ");
      Brain.Screen.print(5*(Drivetrain.heading(deg) - 90));
      Brain.Screen.newLine();



    }

    else if(Drivetrain.heading(deg) <= 89.9){

      speed_left= -20 - 5*(90- Drivetrain.heading(deg));
      speed_right = -20;

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" >R ");
      Brain.Screen.print(5*(90- Drivetrain.heading(deg)));
      Brain.Screen.newLine();



    }

    else{
      speed_left = -20;
      speed_right = -20;

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" S ");
      Brain.Screen.newLine();
    }
    
  }
  */

  if(revs >= -0.25){

    if(Drivetrain.heading(deg) > 90.1){


      speed_left = -40 + (-40 - max_speed)/(0.25) * revs;
      speed_right = -40 + (-40 - max_speed)/(0.25) * revs- 5*(Drivetrain.heading(deg)-90);

      

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" >L ");
      Brain.Screen.print(5*(Drivetrain.heading(deg) - 90));
      Brain.Screen.newLine();



    }

    else if(Drivetrain.heading(deg) <= 89.9){


      speed_left = -40 + (-40 - max_speed)/(0.25) * revs- 5*(90- Drivetrain.heading(deg));
    speed_right = -40 + (-40 - max_speed)/(0.25) * revs;

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" >R ");
      Brain.Screen.print(5*(90- Drivetrain.heading(deg)));
      Brain.Screen.newLine();



    }

    else{
      speed_left = -40 + (-40 - max_speed)/(0.25) * revs;
      speed_right = -40 + (-40 - max_speed)/(0.25) * revs;

      Brain.Screen.print(Drivetrain.heading(deg));
      Brain.Screen.print(" S ");
      Brain.Screen.newLine();
    }
    //speed = -40+240*revs;
    
  }
  else if(dist >= -1.5){
    //speed = -40 + 84*dist;
    speed_left = -190 - curvature;
    speed_right = -190 + curvature;

  }
  
  else{
    speed_left = max_speed;
    speed_right = max_speed;
  }
  

  speeds[0] = speed_left;
  speeds[1] = speed_right;

  //return speeds;
  
}
void turn_swerve_left(double revs, double ang, double curvature, double t){
  double speeds[2];

    RightBack.setRotation(0, rev);
    LeftBack.setRotation(0, rev);
    RightFront.setRotation(0, rev);
    LeftFront.setRotation(0, rev);

    double t_0 = Brain.timer(msec);

    while(Drivetrain.heading() < ang && Brain.timer(msec)- t_0 < t){
      double reading = RightFront.rotation(rev);

      speed_calc_swerve_left(reading, revs-reading, speeds, curvature);

      
      //double speed = speed_calc_swerve(reading, -1.5-reading);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print(" ");
      Brain.Screen.print(-revs-reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speeds[1]);
      Brain.Screen.newLine();


      Drive_Left.spin(fwd, speeds[0], rpm);
      Drive_Right.spin(fwd, speeds[1], rpm);

      wait(1,msec);

    }

    

    Drivetrain.drive(fwd,-190, rpm);

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

  

 //Controller1.Screen.print(speeds[0]);


  double t_0 = Brain.timer(msec);


  spin_top_wheels();
  wait(0.5,sec);
  roller_stop();

  intake_spin();
  spin_bottom_wheels();

  turn_swerve(-2, 225, 55, 1600);

  //Controller1.Screen.print("OUT");

  

  wait(150,msec);
  intake_stop();
  roller_stop();

  wait(200,msec);

  Drivetrain.stop(coast);

  shoot_corner_balls();

  intake_out();

  gradient_back(2.2);
  
  
  

  grad_turn(90, 1, 0.7, 45);

  //wait(2,sec);
 




  intake_spin();
  spin_bottom_wheels();

  turn_swerve_left(-4.5, 135, 26, 5000);

  


  //Controller1.Screen.print("OUT222");

  

  wait(350,msec);
  intake_stop();
  roller_stop();

  //wait(200,msec);

  Drivetrain.stop(coast);

  shoot_corner_balls();

  intake_out();


  

  

    LeftBack.setRotation(0, rev);

    double max_speed = 190; ///THIS IS THE MAX SPEED

  while(LeftBack.rotation(rev) <= 0.25){
    //speed = -40+240*revs;
    Drive_Left.spin(fwd, 40 + (-40 + max_speed)/(0.25) * LeftBack.rotation(rev), rpm);
    Drive_Right.spin(fwd, 40 + (-40 + max_speed)/(0.25) * LeftBack.rotation(rev), rpm);
  }

  
  while(Drivetrain.heading()> 80){
    //speed = -40 + 84*dist;
    double curv = 100;
    double spd = 190 - curv;
    double head = Drivetrain.heading();

    if(head < 110){
      spd = 190 - curv + curv/pow(25,4) * pow(110 - head, 4);
    }
    

    Drive_Left.spin(fwd, 190, rpm);
    Drive_Right.spin(fwd, spd , rpm);

  }

  LeftBack.setRotation(0, rev);

  while(LeftBack.rotation(rev) <= 0.25){
    //speed = -40 + 84*dist;
    Drive_Left.spin(fwd, 190 , rpm);
    Drive_Right.spin(fwd, 190, rpm);

  }

  
  while(Drivetrain.heading()<  180){
    //speed = -40 + 84*dist;
    double curv = 90;
    double spd = 190 - curv;
    double head = Drivetrain.heading();

    if(head > 155){
      spd = 190 - curv + curv/pow(25,5.4) * pow(head - 155,5.4);
    }
    

    Drive_Left.spin(fwd, spd, rpm);
    Drive_Right.spin(fwd, 190 , rpm);

  }

  LeftBack.setRotation(0, rev);
  RightBack.setRotation(0, rev);

  while(LeftBack.rotation(rev) <= 1.4){
    //speed = -40 + 84*dist;
    Drive_Left.spin(fwd, 190 + (-190 + 50)/(1.4)* LeftBack.rotation(rev) , rpm);
    Drive_Right.spin(fwd, 190 + (-190 + 50)/(1.4)* LeftBack.rotation(rev), rpm);

    Brain.Screen.print(RightBack.rotation(rev));
    Brain.Screen.newLine();

  }

  Drivetrain.stop(hold);

  intake_stop();

  double t_1 = Brain.timer(msec);

  Controller1.Screen.print(t_1-t_0);



  
  
  //return speeds;

  
  
  



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
    
    float x = front_lift.rotation(vex::rotationUnits::deg);
    
    if(Controller1.ButtonR1.pressing()){
      /*if(x > -275){

    
        front_lift.spin(fwd, -50, rpm);


      }

      else{
        front_lift.stop(hold);
      }
*/
          

          front_lift.spin(fwd, -50, rpm);
    
    }

    else if(Controller1.ButtonR2.pressing()){

          front_lift.spin(vex::directionType::fwd, 50, vex::velocityUnits::rpm);
    
    }



    else{

        front_lift.stop(hold);

    }


    if (Controller1.ButtonL1.pressing()){
      roller.spin(fwd, 60, rpm);
    }

    else{
      roller.stop(coast);
    }
          
          /*if(x < -480){
            
            Intake.spin(vex::directionType::fwd, -100,vex::velocityUnits::pct);
            Intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            wait(5, msec);
           }

          
          toggle = false;
          
      } else if(Controller1.ButtonDown.pressing()){
          ladder.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
          toggle = false;
      } else if(Controller1.ButtonRight.pressing()){
          sleep(200);
          ladder.rotateTo(-240,vex::rotationUnits::deg,-70,vex::velocityUnits::pct,false);
          otherToggle = false;
      } else if(!(toggle)){
          ladder.stop(vex::brakeType::hold);
          toggle = true;
      }
      */
      
      
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
