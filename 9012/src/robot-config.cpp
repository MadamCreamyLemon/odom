#include "vex.h"

using namespace vex;
brain Brain;
#include "vex.h"
motor BackLeftMotor = motor(PORT11, ratio18_1, true); //3
motor FrontLeftMotor = motor(PORT2, ratio18_1, true); //1
motor FrontRightMotor = motor(PORT1, ratio18_1, true); //2
motor BackRightMotor = motor(PORT12, ratio18_1, true); //4
motor flywheel1 = motor(PORT14, ratio6_1, false);
motor intake = motor(PORT15, ratio18_1, false);
motor intake2 = motor(PORT16, ratio18_1,false);
rotation odomLeft = rotation(PORT7);
rotation odomRight= rotation(PORT8);
rotation odomCentre = rotation(PORT5);
controller Controller1;
vision Vision = vision(PORT13);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
*/

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}