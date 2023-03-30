// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  Xbox driver1 = new Xbox(0);
  Xbox driver2 = new Xbox(1);
  DriveTrain dt = new DriveTrain();
  Arm arm = new Arm();
  Intake intake = new Intake();
  LED led = new LED();
  Auto auto = new Auto(dt, arm, intake);
  boolean balanceMode = false;

  @Override
  public void robotInit() {
    dt.init();
    arm.init();
    intake.init();
    led.init();
    auto.init();
  }

  @Override
  public void disabledPeriodic(){
    if (driver1.getStartButtonPressed()){
      if (auto.getMode() == "PLACE_TAXI"){
        auto.selectDropLeaveBalance();
      } else if (auto.getMode() == "DROP_LEAVE_BALANCE"){
        auto.selectDoubleCube();
      } else if (auto.getMode() == "DOUBLE_CUBE"){
        auto.selectPlaceAndTaxi();
      }

    }
    
    if (driver1.getLeftStickButtonPressed()){
    }
    
    if (driver1.getBButtonPressed()) {
      auto.selectCube();
    }
    if (driver1.getXButtonPressed()){
      auto.selectCone();
    }
    if (driver1.getPOV() == 0){
      auto.selectHighTier();
    }
    if (driver1.getPOV() == 180){
      auto.selectLowTier();
    }
    
    if (driver2.getBButtonPressed()) {
      auto.selectCube();
    }
    if (driver2.getXButtonPressed()){
      auto.selectCone();
    }
    if (driver2.getPOV() == 0){
      auto.selectHighTier();
    }
    if (driver2.getPOV() == 180){
      auto.selectLowTier();
    }
    dt.debug();
    arm.debug();
    auto.debug();
  }

//
//  TELEOP
//

  @Override
  public void teleopInit() {
    dt.reset(); 
    arm.reset();
    balanceMode = false;
  }

  @Override
  public void teleopPeriodic() {

    double upDown = driver1.deadzone(-driver1.getRightY());
    double inOut = driver1.deadzone(driver1.getRightX());
    arm.joystickMove(inOut, upDown);

    //Balance Mode
    
    if (driver1.getStartButtonPressed()) {
      balanceMode = !balanceMode;
    }

    //Loading Selection

    if (driver1.getAButtonPressed() || driver2.getAButtonPressed()){
      if (intake.getTarget() == "CONE"){
        arm.goToFloorIntakeCone();
      } else {
        if (intake.getTarget() == "CUBE"){
          arm.goToFloorIntakeCube();
        }
      }
    }

    if(driver1.getBButtonPressed() || driver2.getBButtonPressed()){
      intake.setTarget("CUBE");
    }

    if(driver1.getXButtonPressed() || driver2.getXButtonPressed()){
      intake.setTarget("CONE");
    }

    if (driver1.getYButtonPressed() || driver2.getYButtonPressed()){            //Velocity drive
      dt.vision.setTarget("STATION");
      if (intake.getTarget() == "CUBE"){
        arm.gotoLoadingZoneCube();
      } else {
        if (intake.getTarget() == "CONE"){
          arm.gotoLoadingZoneCone();
        }
      }
    }

    //Intake
    if (driver1.getRightBumper()){
      intake.on();
    } else {
      if (driver1.getRightTrigger() > 0.85){
        intake.eject();
      } else{
        intake.hold();
      }
    }
    if (balanceMode){
      led.balanceModeColor();
    } else if (intake.getTarget() == "CONE"){
      led.coneColor();
    } else {
      led.cubeColor();
    }

    //Vision and Arm
    if (driver1.getPOV() == 0 || driver2.getPOV() == 0) { // D pad up
       if (intake.getTarget() == "CONE") {
        dt.vision.setTarget("HIGH_CONE");
        arm.gotoConeHigh();
      } else {
        dt.vision.setTarget("CUBE");
        arm.gotoCubeHigh();
      }
    }
    if (driver1.getPOV() == 90 || driver2.getPOV() == 90){ // D pad right
      dt.vision.setTarget("NONE");
      arm.gotoPark();
    }
    if (driver1.getPOV() == 180 || driver2.getPOV() == 180){ // D pad down
      if (intake.getTarget() == "CONE") {
        dt.vision.setTarget("LOW_CONE");
        arm.gotoConeLow();
      } else {
        dt.vision.setTarget("CUBE");
        arm.gotoCubeLow();
      }

    }
    if (driver1.getPOV() == 270 || driver2.getPOV() == 270){ // D pad left
      dt.vision.setTarget("NONE");
      arm.gotoPark();
    }
    if (driver1.getBackButtonPressed()){
      dt.vision.setTarget("NONE");
    }
    
    // Check to see if trying to drive with joysticks. This overrides any other drive method.
    double speedGovenor = 1.0;
    if (arm.isTippy() || balanceMode){
      speedGovenor = 0.6;
    }
    double drive1 = driver1.deadzone(-driver1.getLeftY(), 0.15) * speedGovenor;
    double turn1 = driver1.deadzone(driver1.getLeftX(), 0.15) * 0.7 * speedGovenor;
    
    if(drive1 != 0 || turn1 != 0){   //If joystick is driving in either axis, then do a Teleop drive.
      dt.setTeleopDrive(drive1, turn1);
    } else if (balanceMode){
      dt.setBalanceHold();
    }

    if (driver1.getLeftTrigger() > 0.15){
      if (balanceMode){
        dt.setBalanceDrive();
      } else {
        dt.doVisionDrive(driver1.getLeftTrigger() * 0.6);
      }
    }

    // TESTING with BACK button
    
    if(driver1.getBackButton()){
      dt.setVelocityDrive(10);
    }

    // Testing
    if (driver1.getRightStickButtonPressed()){
      //dt.setDistanceDrive(-70, .4);
    }


    /*
    double drive2 = driver2.deadzone(-driver2.getLeftY(), 0.15) * speedGovenor;
    double turn2 = driver2.deadzone(driver2.getLeftX(), 0.15) * 0.7 * speedGovenor;
    
    if(drive2 != 0 || turn2 != 0){   //If joystick is driving in either axis, then do a Teleop drive.
      dt.setTeleopDrive(drive2, turn2);
    }
*/
    dt.update();
    arm.update();
    debug();
  }

//
//  AUTONOMOUS
//
  @Override
  public void autonomousInit() {
    dt.reset(); 
    arm.reset();
    auto.reset();
    balanceMode = false;
  }

  @Override
  public void autonomousPeriodic() {
    if (intake.getTarget() == "CONE"){
      led.coneColor();
    } else {
      led.cubeColor();
    }
    auto.update();
    debug();
  }

  public void debug() {
    intake.debug();
    //arm.debug();
  }
}