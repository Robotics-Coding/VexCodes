using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group Drive_Left;
extern motor_group Drive_Right;
extern motor_group rollers;
extern motor LeftBack;
extern motor RightBack;
extern motor LeftFront;
extern motor RightFront;
extern motor Intake;
extern motor Intake2;
extern motor Lift1;
extern motor roller;
extern motor highRoller;
extern motor index_roller;
extern controller Controller1;

extern inertial TurnGyroSmart;

extern smartdrive Drivetrain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
