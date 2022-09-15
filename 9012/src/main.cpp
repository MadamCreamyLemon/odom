/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel (saurinpatel and noahgelbart)                  */
/*    Created:                                                 */
/*    Description:  Odometry For Precise Autonomous Motion                    */
/*    Credits:      5225A For Pilons POS Document and QUEENS for odom Alg.    */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
#define Pi 3.14159265358979323846
#define SL 4.9 //distance from tracking center to middle of left wheel
#define SR 4.9 //distance from tracking center to middle of right wheel
#define SS 7
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
#define fieldscale 1.66548042705

double DeltaL;
double DeltaR;
double DeltaB;
double currentL;
double currentR;
double currentB;
double PreviousL;
double PreviousR;
double PreviousB;
double DeltaTheta;
double X; //X COORD
double Y; //Y COORD
double Theta;   //CURRENT THETA
double DeltaXSide;
double DeltaYSide;
double SideChord;
double BackChord;
double DeltaXBack;
double DeltaYBack;
double OdomHeading;

/*---------------------------------------------------------------------------*/
/*                            Odometry Functions                             */
/*---------------------------------------------------------------------------*/
void TrackPOS() {
  currentR = odomRight.position(degrees);
  currentL = odomLeft.position(degrees);
  currentB = odomCentre.position(degrees);

  DeltaL = ((currentL - PreviousL) * 8.255) / 360;
  DeltaR = ((currentR - PreviousR) * 8.255) / 360;
  DeltaB = ((currentB - PreviousB) * 8.255) / 360;
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);
  
  if(DeltaTheta == 0) {
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    X += DeltaB * cos (Theta + 1.57079633);
    Y += DeltaB * sin (Theta + 1.57079633);
  } else {
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      BackChord = 2 * ((DeltaB / DeltaTheta) + SS ) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
      X += DeltaB * cos (Theta + 1.57079633);
      Y += DeltaB * sin (Theta + 1.57079633);
    }
    OdomHeading = Theta * 57.295779513;
    PreviousL = currentL;
    PreviousR = currentR;
    PreviousB = currentB;
    DeltaTheta = 0;
//GRAPHICS
 int textadjustvalue = 55;
    int rowadjust = 39;

    //Sets graphical things for our display 
    Brain.Screen.setPenWidth( 1 );
    vex::color redtile = vex::color( 210, 31, 60 );
    vex::color bluetile = vex::color( 14, 77, 146 );
    vex::color graytile = vex::color( 49, 51, 53 );
    Brain.Screen.setFillColor(vex::color( 0, 0, 0 ));
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.setPenColor( vex::color( 222, 49, 99 ) );

    //Displays all the field tiles, text of odom values, and a dot symbolizing the robot
    Brain.Screen.printAt(40,20 + textadjustvalue, "X-Pos:%f",-X);
    Brain.Screen.setPenColor( vex::color( 191, 10, 48 ) );
    Brain.Screen.printAt(40,50 + textadjustvalue, "Y-Pos:%f",Y);
    Brain.Screen.setPenColor( vex::color( 141, 2, 31 ) );
    Brain.Screen.printAt(40,80 + textadjustvalue, "Theta:%f",Theta);
    Brain.Screen.setPenColor( vex::color( 83, 2, 1 ) );
    Brain.Screen.printAt(40,110 + textadjustvalue, "Angle:%f",OdomHeading);
    Brain.Screen.setPenColor( vex::color( 255, 255, 255 ) );
    Brain.Screen.setFillColor( graytile );
    Brain.Screen.drawRectangle( 245, 2, 234, 234 );
    Brain.Screen.drawRectangle( 245, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 197, 39, 39 );
    Brain.Screen.setFillColor( redtile );
    Brain.Screen.drawRectangle( 245, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245, 41, 39, 39 );
    Brain.Screen.setFillColor( bluetile );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 158, 39, 39 );
    Brain.Screen.setPenColor( vex::color( 255,255,255));
    Brain.Screen.setFillColor( vex::color(0,0,0) );
    
    //This draws the robot body for position and arm for angle
    double yfieldvalue = ((-Y)*fieldscale)+245-10;
    double xfieldvalue = ((-X)*fieldscale)+245;
    Brain.Screen.drawCircle(xfieldvalue, yfieldvalue, 10 );
    Brain.Screen.setPenWidth( 4 );
    //Line angle calculation:
    //x1 and y1 are the robot's coordinates, which in our case is xfieldvalue and yfieldvalue
    //angle is the angle the robot is facing, which in our case is Theta
    //(x1,y1, x1 + line_length*cos(angle),y1 + line_length*sin(angle)) = (x1,y1,x2,y2)
    Brain.Screen.drawLine(xfieldvalue, yfieldvalue, xfieldvalue+cos(-Theta-(Pi/2))*15, yfieldvalue+ sin(-Theta-(Pi/2)) *15);
  }
 /* void AutoAim(){
    double aac;
    double YendOfField = 135;
    double XendofField = X;
    double Xgoal = 15;
    double Ygoal = 119;
    double lengthOfArc1;
    double lengthOfArc2;
    double lengthOfArc3;
    double angleToGoal;
    double robotToGoal;
    
    lengthOfArc1 = 135 - Y; //a
    lengthOfArc2 = sqrt((X - Xgoal)*(X - Xgoal)+(Y - Ygoal)*(Y - Ygoal)); //b
    lengthOfArc3 = sqrt((Xgoal - XendofField)*(Xgoal - XendofField)+(Ygoal - YendOfField)*(Ygoal - YendOfField)); //c

    angleToGoal = acos((lengthOfArc2)*(lengthOfArc2)+(lengthOfArc3)*(lengthOfArc3)-(lengthOfArc1)*(lengthOfArc1)/(2*lengthOfArc2)*(lengthOfArc3));
    robotToGoal = angleToGoal - OdomHeading;

  std::cout<<lengthOfArc1;
  std::cout<<lengthOfArc2;
  std::cout<<lengthOfArc3;
  }
*/
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
    vexcodeInit();
    Brain.resetTimer();
    odomRight.resetPosition();
    odomLeft.resetPosition();
    odomCentre.resetPosition();

    X = 0;
    Y = 0;
}
//AUTOM
void autonomous( void ) {

}
//DRIVER CONTROL
void usercontrol( void ) {
  while (1){ 
    Brain.Screen.clearScreen();
    BackLeftMotor.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value() + Controller1.Axis4.value()) , vex::velocityUnits::pct); 
    BackRightMotor.spin(vex::directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis3.value() + Controller1.Axis4.value()), vex::velocityUnits::pct);         
    FrontLeftMotor.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value() - Controller1.Axis4.value()), vex::velocityUnits::pct); 
    FrontRightMotor.spin(vex::directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis3.value() - Controller1.Axis4.value()), vex::velocityUnits::pct); 
    TrackPOS();
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10);
//Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton();
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(5); //Slight delay so the Brain doesn't overprocess
    }
}