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
motor IL = motor(PORT6, ratio18_1, true);
motor IR = motor(PORT16, ratio18_1, false);
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
  AutoDrive.setDriveVelocity(vel, velocityUnits::pct);
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


void makeTower(){
    setIntakeSpeed(-20);
    task::sleep(1200);
    A.rotateTo(-10,deg);
    setIntakeSpeed(-10);
    C.rotateTo(90,deg);
    C.setVelocity(30, pct);
    C.rotateTo(140,deg);
    drive(2,15);

  // Back off and reset
    setIntakeSpeed(-45);
    drive(-8,50);
    stopIntake();
    C.rotateTo(0, deg);
}

void armUp(){
  A.setVelocity(100, pct);
  A.rotateTo(650,deg);
  A.rotateTo(0,deg);
  A.rotateTo(95,deg);
}

void auto1(){
  armUp();
  setIntakeSpeed(70);
  drive(18,30);
  stopIntake();
  rotate(-90);
  setIntakeSpeed(70);
  drive(10,30);
  stopIntake();
}


void auto3(int rev = 1,bool s=false){ // THIS AUTONOUS GETS 4 BLOCKS ON GROUND (FRONT)

  // Setup
    int dist = s ? 43 : 36; // total amount to drive forward
    armUp(); // Arm all of the way up then down thene up a little
    drive(8,30); // move forward fast
    setIntakeSpeed(65); 
  // Drive Forward
    drive(dist-8,30); // then slow
    task::sleep(500); // stop then intake for another .5 secs
    stopIntake();

  // Drive Backwards
    drive(-dist+11,60); // fast
    drive(-3,20); // slow

  // Get in Corner
    rotate(50*rev,75); 
    if(rev < 0) rotate(50*rev);
    drive(10,50); 

    if(rev>0) rotate(33*rev,80);
    if(rev < 0){
      //drive(3);
    }
    else drive(11,60);


  // Make Tower
    makeTower();

    if(!s) return; // CONTINUE IF SKILLS

    // TURN TOWARDS TOWER

    drive(-8,50);
    rotate(-150*rev);
    drive(5);
    rotate(40*rev);
    A.rotateTo(90,deg,false);
    drive(20,100);
    setIntakeSpeed(55);
    drive(8,30);
    drive(-8);
    stopIntake();
    A.rotateTo(600,deg);
    drive(5,50);
    setIntakeSpeed(-50);
    task::sleep(1000);
    drive(-5);
    stopIntake();
}


void autonomous( void ) {
    //bool SKILLS = false;
    //auto3(1,SKILLS); // 1 for red, -1 for blue
    auto1();
}

void usercontrol( void ) {
  while (true) {
    // DEFINE VARIABLES FOR MECANUM
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
    } else if(Controller1.ButtonR2.pressing()){
      A.spin(fwd, -100, pct);
    } else {
      A.spin(fwd, 0, rpm);
      A.setStopping(hold);
    }
    float ISpeed;
    if(Controller1.ButtonDown.pressing()){
      ISpeed = 10;
    } else {
      ISpeed = 75;
    }
    if(Controller1.ButtonL1.pressing()){
      IL.spin(fwd, ISpeed, pct);
      IR.spin(fwd, ISpeed, pct);
    } else if(Controller1.ButtonL2.pressing()){
      IL.spin(fwd, -ISpeed, pct);
      IR.spin(fwd, -ISpeed, pct);
    } else {
      IL.spin(fwd, 0, rpm);
      IR.spin(fwd, 0, rpm);
      IL.setStopping(hold);
      IR.setStopping(hold);
    }
    if(Controller1.ButtonA.pressing()){
      if(C.rotation(deg) < 175){
        C.spin(fwd, (15 - 13.5 * ((C.rotation(deg)) / 175)), rpm);
      } else {
        C.spin(fwd, 0, rpm);
        C.setStopping(coast);
      }
    } else if(Controller1.ButtonB.pressing()){
      if(C.rotation(deg) > 0){
        C.spin(fwd, -15, rpm);
      } else {
        C.spin(fwd, 0, rpm);
        C.setStopping(coast);
      }
    } else{
      C.spin(fwd, 0, rpm);
      C.setStopping(coast);
    }
    task::sleep(20);
  }
}

int main() {
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    pre_auton();
                               
    while(true) {
      task::sleep(100);
    }    
       
}