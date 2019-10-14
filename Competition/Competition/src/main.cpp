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
motor IL = motor(PORT15, ratio36_1, true);
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
  FL.startRotateFor(-Degrees, deg, speed_pct, rpm);
  BL.startRotateFor(Degrees, deg, speed_pct, rpm);
  FR.startRotateFor(-Degrees, deg, speed_pct, rpm);
  BR.rotateFor(Degrees, deg, speed_pct, rpm);
}

void rotate(double d){
  AutoDrive.turnFor(turnType::right, d*1.451, deg, 100, rpm);
}


void pre_auton( void ) {
  C.resetRotation();
}

void autonomous( void ) {
  //AutoDrive.driveFor(fwd, 14, distanceUnits::in);
  rotate(360);
}

void usercontrol( void ) {
  while (1) {
    double LX = Controller1.Axis4.position(pct);
    double LY = Controller1.Axis3.position(pct);
    double RX = Controller1.Axis1.position(pct);

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