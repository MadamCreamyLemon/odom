using namespace vex;

extern brain Brain;
extern motor BackLeftMotor;
extern motor FrontLeftMotor;
extern motor FrontRightMotor;
extern motor BackRightMotor;
extern motor flywheel1;
extern motor intake;
extern motor intake2;
extern rotation odomLeft;
extern rotation odomRight;
extern rotation odomCentre;
extern controller Controller1;
extern vision Vision;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
