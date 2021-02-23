 void gradient_on_time(double revs){
    double initial = Brain.timer(sec);
    double target = (revs + 0.3375)/(100/60) + 0.5;
    Brain.Screen.print("TARGET: ");
    Brain.Screen.print(target);
    Brain.Screen.newLine();
  

    while(RightFront.rotation(rev)> revs){
      double reading = Brain.timer(sec) - initial;
      double time_left = target -reading;
      double speed = speed_calc_timer(reading, time_left);

      Brain.Screen.print("READING: ");
      Brain.Screen.print(reading);
      Brain.Screen.print("    SPEED: ");
      Brain.Screen.print(speed);
      Brain.Screen.newLine();

      

      Drivetrain.drive(fwd, speed, rpm);

    }

    Drivetrain.stop(hold);

  }
