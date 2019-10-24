#include "vex.h"
#include "vision.h"
using namespace vex;

brain       Brain;

competition Competition;

controller Controller1 = controller();
motor FL = motor(PORT10, ratio18_1, false);
motor BL = motor(PORT2, ratio18_1, false);
motor FR = motor(PORT20, ratio18_1, true);
motor BR = motor(PORT19, ratio18_1, true);
motor A = motor(PORT13, ratio36_1, false);
motor IL = motor(PORT4, ratio36_1, true);
motor IR = motor(PORT12, ratio36_1, false);
motor C = motor(PORT3, ratio36_1, true);
motor_group LM(FL, BL);
motor_group RM(FR, BR);
drivetrain AutoDrive(LM, RM, 320, 38);

void driveY(double speed_pct, double L_Degrees, double R_Degrees){
  speed_pct *= 2;
  FL.startRotateFor(L_Degrees, deg, speed_pct, rpm);
  BL.startRotateFor(L_Degrees, deg, speed_pct, rpm);
  FR.startRotateFor(R_Degrees, deg, speed_pct, rpm);
  BR.rotateFor(R_Degrees, deg, speed_pct, rpm);
}

void driveX(double speed_pct, double Degrees){
  speed_pct *= 2;
  FL.startRotateFor(-Degrees*100, deg, speed_pct, rpm);
  BL.startRotateFor(Degrees*100, deg, speed_pct, rpm);
  FR.startRotateFor(-Degrees*100, deg, speed_pct, rpm);
  BR.rotateFor(Degrees*100, deg, speed_pct, rpm);
}

void rotate(double d,double speed=100){
  AutoDrive.turnFor(turnType::right, d*1.451, deg, speed, rpm);
}

void pre_auton( void ) {
  C.resetRotation();
  A.resetRotation();
  C.setVelocity(20, pct);
}

void drive(double inches,double vel=100){
  AutoDrive.setVelocity(vel, velocityUnits::pct);
  AutoDrive.driveFor(fwd, inches, distanceUnits::in);
}

void setIntakeSpeed(double speed){
  IL.setVelocity(speed,pct);
  IR.setVelocity(speed,pct);
  IL.spin(fwd);
  IR.spin(fwd);
}

void stopIntake(){
  IR.stop(brake);
  IL.stop(brake);
}

void armUp(){
  A.setVelocity(100, pct);
  A.rotateTo(650,deg);
  A.rotateTo(0,deg);
  A.rotateTo(75,deg);
}

void auto1(){
  armUp();
  setIntakeSpeed(80);
  drive(18);
  stopIntake();
  drive(-10);
  driveX(70,-6);
  rotate(-820);
  drive(9);
  driveX(70,6);
  drive(4);
  setIntakeSpeed(-80);
  task::sleep(1500);
  stopIntake();
  drive(-8);
}

void auto2(){
  armUp();
  setIntakeSpeed(85);
  drive(15);
  stopIntake();
  rotate(820);
  A.rotateTo(700,deg);
  drive(13);
  setIntakeSpeed(-80);
  task::sleep(1500);
  stopIntake();
  drive(-5);
}

void auto3(){
  int dist = 45;
  armUp();
  setIntakeSpeed(80);
  drive(12,80);
  drive(dist-12,12);
  setIntakeSpeed(-50);
  task::sleep(1500);
  stopIntake();
  drive(-dist+5,100);
  rotate(900);
  drive(12,50);
  rotate(300);
  drive(5,60);
  setIntakeSpeed(-20);
  task::sleep(2500);
  stopIntake();
  A.rotateTo(-5,deg);
  C.rotateTo(90,deg);
  drive(2,15);
  drive(-5,100);
}

// 900 is about 90 degrees in rotate function

void autonomous( void ) {
  auto3();
}

void usercontrol( void ) {
  while (1) {
    // DEFINE VARIABLES FOR MECHANUM
    double LX = Controller1.Axis4.position(pct);
    double LY = Controller1.Axis3.position(pct);
    double RX = Controller1.Axis1.position(pct);

    // DRIVE FOR MECHANUM
    FL.spin(fwd, ((LY + RX + LX) * .75), pct);
    BL.spin(fwd, ((LY + RX - LX) * .75), pct);
    FR.spin(fwd, ((LY - RX + LX) * .75), pct);
    BR.spin(fwd, ((LY - RX - LX) * .75), pct);

    if(Controller1.ButtonR1.pressing()){
      A.spin(fwd, 100, pct);
    }
    else if(Controller1.ButtonR2.pressing()){
      A.spin(fwd, -100, pct);
    }
    else{
      A.spin(fwd, 0, rpm);
      A.setStopping(hold);
    }
    float ISpeed;
    if(Controller1.ButtonDown.pressing()){
      ISpeed = 10;
    }
    else{
      ISpeed = 100;
    }
    if(Controller1.ButtonL1.pressing()){
      IL.spin(fwd, ISpeed, pct);
      IR.spin(fwd, ISpeed, pct);
    }
    else if(Controller1.ButtonL2.pressing()){
      IL.spin(fwd, -ISpeed, pct);
      IR.spin(fwd, -ISpeed, pct);
    }
    else {
      IL.spin(fwd, 0, rpm);
      IR.spin(fwd, 0, rpm);
      IL.setStopping(hold);
      IR.setStopping(hold);
    }
    if(Controller1.ButtonA.pressing()){
      if(C.rotation(deg) < 175){
        C.spin(fwd, (15 - 13.5 * ((C.rotation(deg)) / 175)), rpm);
      }  
      else{
        C.spin(fwd, 0, rpm);
        C.setStopping(hold);
      }
    }
    else if(Controller1.ButtonB.pressing()){
      if(C.rotation(deg) > 0){
        C.spin(fwd, -15, rpm);
      }  
      else{
        C.spin(fwd, 0, rpm);
        C.setStopping(hold);
      }
    }
    else{
      C.spin(fwd, 0, rpm);
      C.setStopping(hold);
    }
    Controller1.Screen.print(C.rotation(deg));
    task::sleep(20);
  }
}

int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}