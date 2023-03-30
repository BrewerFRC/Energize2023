package frc.robot;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrain {
    private double drivePower = 0;  // power for drive, derived each robot cycle
    private double turnPower = 0;   // Targeted power for turn, dervied each robot cycle
    private double MAX_VEL = 25.0; // max inches per second sent to the drive motors
    //private double MIN_VEL = 5.0; // min inches per second sent to the drive motors
    private double MAX_DRIVE = 1.0; // max power sent to the drive motors.  Set this to control max power for all robot drive functions.
    private double MIN_DRIVE = 0.3; // min power sent to the drive motors, when very smal powers are provided.  Tune this to just barely move the robot.
    private double MAX_DRIVE_AUTO = 0.6;  // was 0.55 //max power sent to drive motors for distance drive.
    private double MAX_TURN = 0.6;
    private double MIN_TURN_TURNING = 0.3; //min power for angle turns
    private double MIN_TURN_HEADING = 0.1; //min power for heading hold turns
    private double MOTOR_RAMP = .75;  // For closed and open loop
    private double COUNTS_PER_INCH = 146105.0/120.0; //Encouder counts per 120 inches
    private int stallCount = 0; //the number of cycles that the robot has had velocity equal zero
    private double STALL_VELOCITY = 0.25;  //velocities slower than this are considered "stalled".
    private double STALL_CNT_LIMIT = 4; //the number of sucecssive cycles with zero velocity
    private double STALL_DRIVE_POWER = MIN_DRIVE + 0.05; //power used to drive to a wall while watchng for stall.  Must be a bit faster than minimum power to move robot.    
    //PIDS
    private double kP_VEL = 0.02, kI_VEL = 0, kD_VEL = 0; // for velocity pid (Just uses P term)
    private double kP_DIST = 0.005, kI_DIST = 0, kD_DIST = 0; // for distance pid //was 0.005
    private double kP_TURN = 0.0046, kI_TURN = 0, kD_TURN = 0; // for turn pid, Turn was 0.0041
    private PID velPID;
    private PID distPID;
    private PID turnPID;
    private double targetDistance = 0.0;  //target distance for a distance drive in inches
    private double targetVelocity = 0.0;  //target velocity in inches/sec for velocity drive
    private double targetHeading = 0.0; //target heading between 180 and -180 for a PID turn
    private boolean headingHold = false; //When enabled, distance and velocity driving functions will hold current heading.
    //MOTOR CONTROLLERS
    private WPI_TalonSRX frontL = new WPI_TalonSRX(Constants.DRIVE_FL);
    private WPI_TalonSRX backL = new WPI_TalonSRX(Constants.DRIVE_BL);
    private WPI_TalonSRX frontR = new WPI_TalonSRX(Constants.DRIVE_FR);
    private WPI_TalonSRX backR = new WPI_TalonSRX(Constants.DRIVE_BR);
    private MotorControllerGroup left = new MotorControllerGroup(frontL, backL);
    private MotorControllerGroup right = new MotorControllerGroup(frontR, backR);
    DifferentialDrive drive = new DifferentialDrive(left, right);
    //GYRO
    private AHRS navx; // NavX connected over MXP;
    private double PITCH_OFFSET = 0.0; //observed pitch when robot is on a level surface/ was 1.8
    //VISION
    public Vision vision = new Vision();
    

    public enum States {
      TELEOP,        // For normal joystick-based driving
      DIST_DRIVE,    // Drive to a number of inches
      VEL_DRIVE,     // Drive at a set velocity in inches per second.  It is pitch sensitive.
      TURN,          // Turn to a heading
      DRIVE_TO_WALL, // Drive a distance, but continue driving slowly until stalled.
      DRIVE_TO_WALL_2,
      VISION_TRACK,  // Drive to a selected target, adjusting turn and drive speed. Stop if stalled.
      COMPLETE,      // Drive complete, no hold
      HOLD;          // Drive complete, hold the target heading
    }
    
    private States state = States.TELEOP;


 
    public void init() {
        distPID = new PID(kP_DIST, kI_DIST, kD_DIST, 0.0, false, "DistPID", false);
        //distPID.setP(kP_DIST);  distPID.setI(kI_DIST);  distPID.setD(kD_DIST);
        distPID.setOutputLimits(-MAX_DRIVE_AUTO, MAX_DRIVE_AUTO);
        distPID.setMinMagnitude(MIN_DRIVE);

        velPID = new PID(kP_VEL, kI_VEL, kD_VEL, 0.0,false, "VelPID", false);
        //velPID.setP(kP_DIST);  distPID.setI(kI_DIST);  distPID.setD(kD_DIST);
        velPID.setOutputLimits(-MAX_VEL, MAX_VEL);
        velPID.setMinMagnitude(0.0);  //The feedfoward value is used in place of a minimum.

        turnPID = new PID(kP_TURN, kI_TURN, kD_TURN, 0.0, false, "TurnPID", false);
        //turnPID.setP(kP_TURN);  turnPID.setI(kI_TURN);  turnPID.setD(kD_TURN);
        turnPID.setOutputLimits(-MAX_TURN, MAX_TURN);
        turnPID.setMinMagnitude(MIN_TURN_TURNING);

        frontL.configFactoryDefault();
        frontL.setNeutralMode(NeutralMode.Brake);
        //frontL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        frontL.setInverted(true);
        frontL.setSensorPhase(true);
        frontL.configPeakOutputForward(MAX_DRIVE);
        frontL.configPeakOutputReverse(-MAX_DRIVE);
        frontL.configOpenloopRamp(MOTOR_RAMP);
        //frontL.configClosedloopRamp(MOTOR_RAMP);  // Seconds from neutral to full power  (this is for PID control)
        //frontL.config_kP(0,0.1);  //Set kP for PID0
        frontR.configFactoryDefault();
        //frontR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        frontR.setNeutralMode(NeutralMode.Brake);
        frontR.setSensorPhase(true);
        frontR.configPeakOutputForward(MAX_DRIVE);
        frontR.configPeakOutputReverse(-MAX_DRIVE);
        frontR.configOpenloopRamp(MOTOR_RAMP);
        //frontR.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        //frontR.config_kP(0,0.1);  //Set kP for PID0
        backL.configFactoryDefault();
        backL.setNeutralMode(NeutralMode.Coast);
        //backL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        backL.setInverted(true);
        backL.setSensorPhase(true);
        backL.configPeakOutputForward(MAX_DRIVE);
        backL.configPeakOutputReverse(-MAX_DRIVE);
        backL.configOpenloopRamp(MOTOR_RAMP);
        //backL.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        //backL.config_kP(0,0.1);  //Set kP for PID0
        backR.configFactoryDefault();
        //backR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10); // Choose the encoder for PID feedback on PID 0, timeout of 10ms.
        backR.setNeutralMode(NeutralMode.Coast);
        backR.setSensorPhase(true);
        backR.configPeakOutputForward(MAX_DRIVE);
        backR.configPeakOutputReverse(-MAX_DRIVE);
        backR.configOpenloopRamp(MOTOR_RAMP);
        //backR.configClosedloopRamp(MOTOR_RAMP);            // Seconds from neutral to full power  (this is for PID control)
        //backR.config_kP(0,0.1);  //Set kP for PID0

        // Setup NavX
        navx = new AHRS(SPI.Port.kMXP);
        
        vision.init();
    }

    public void distDriveAcceleration(String accel){
      if (accel == "LOW"){
        kP_DIST = 0.005;
      } else {
        if (accel == "MEDIUM"){
          kP_DIST = 0.0085;
        } else {
          if (accel == "HIGH"){
            kP_DIST = 0.012;
          }
        }
      }
      distPID.setP(kP_DIST);
    }

    public void reset() {
      targetDistance = 0.0;
      targetHeading = 0.0;
      targetVelocity = 0.0;   
      distPID.reset();
      velPID.reset();
      turnPID.reset(); 
      resetEncoder();  
      stopMotors();
      resetGyro();
      vision.setTarget("NONE");
      distDriveAcceleration("LOW");
      state = States.COMPLETE;
  }

    private void stopMotors() {
        backR.stopMotor();
        backL.stopMotor();
        frontR.stopMotor();
        frontL.stopMotor();
    }

    private void setTurningPower(){
      turnPID.setMinMagnitude(MIN_TURN_TURNING);
    }

    private void setHeadingHoldPower(){
      turnPID.setMinMagnitude(MIN_TURN_HEADING);
    }

    // Get velocity in inches/second for left and right drive motors and average
    public double getVelocity(){
        return (frontL.getSelectedSensorVelocity(0)*10/COUNTS_PER_INCH + frontR.getSelectedSensorVelocity(0)*10/COUNTS_PER_INCH) / 2;
    }
    
    // Raw encoder from left and right enconders, averaged
    public double getEncoderCounts() {
        return ((frontL.getSelectedSensorPosition(0) + frontR.getSelectedSensorPosition(0)) / 2);
    }

    public double getInches(){
        return countsToInches(getEncoderCounts());
    }

    public double countsToInches(double encoderCount){
      return encoderCount/COUNTS_PER_INCH;
    }

    public double inchesToCounts(double inches){
      return COUNTS_PER_INCH*inches;
    }
    
    // If the robot has not been moving for the for the past STALL CNT LIMIT cycles, then it is considered stalled
    // Be sure to reset stallCount, before initiating a drive function that is testing the isStalled().
    public boolean isStalled(){
      if (stallCount >= STALL_CNT_LIMIT){
        return true;
      } else {
        return false;
      }
    }

    // Increment stall counter if robot velocity is less than STALL_VELOCITY.
    // This function should be called every update() cycle.
    private void checkForStall(){
      if (Math.abs(getVelocity()) < STALL_VELOCITY){  // if the velocity is less then 0.5 inches per second consider robot to be stalled
        stallCount += 1;
      } else {
        stallCount = 0;
      }
    }

    // get Yaw from navX, 0-360 degrees
    public double getGyroYaw() {
      /*if (navx.isMagnetometerCalibrated()) {
        // We will only get valid fused headings if the magnetometer is calibrated
        return navx.getFusedHeading();
      }*/
      return navx.getYaw();
    }

    // Reset gyro yaw (heading) to 0. Gyro pitch can't be reset.
    public void resetGyro() {
      navx.reset();
    }

    // Read gyro and covert heading in +/- 180 degree reading
    public double getSignedAngle() {
      double angle = getGyroYaw(); 
      if (angle > 180) { 
        angle = angle - 360;
      }
      return angle;
    }

    // Reset the Talon encoders
    public void resetEncoder(){
      frontL.setSelectedSensorPosition(0);
      frontR.setSelectedSensorPosition(0);
      backL.setSelectedSensorPosition(0);
      backR.setSelectedSensorPosition(0);
      
    }
    
    // Get pitch from NavX and adjust by offset, pitch is zero when robot on level surface.
    public double getPitch() {
      return -navx.getRoll() - PITCH_OFFSET;
    }

    // Calculate a feedforward value, assuming robot is on level surface, for a given target inches/sec.
    // Uses linear interpolation by on observed motor powers for velocities from 6 to 15 inches/sec.
    private double minFeed(double inchesPerSec){
      return Common.map(inchesPerSec, 6, 15, 0.31, 0.365);
    }

    // Calculate a feedfordward value, assuming robot is on an 18% incline, for a given target inches/sec.
    // Uses linear interpolation by on observed motor powers for velocities from 6 to 15 inches/sec.
    private double maxFeed(double inchesPerSec){
      return Common.map(inchesPerSec, 6, 15, 0.28, 0.397);  //minOutput was 0.34
    }

    // Calculate a feedForward value for velocity drive based on target inchesPerSec and current robot pitch from NavX
    private double feedForward(double inchesPerSec){
      double MIN_ANGLE = 0;  //minimum angle used to mesure feed foward power
      double MAX_ANGLE = 18; //maximum angle used to mesure feed foward power
      double minFF = minFeed(inchesPerSec);
      double maxFF = maxFeed(inchesPerSec);
      double angle = getPitch();
      double slope = (maxFF - minFF) / (MAX_ANGLE - MIN_ANGLE);
      double power;
      if (angle > 2) {
        power = slope * angle + minFF;
      } else if (angle < -2) {
        power = slope * angle - minFF;
      } else {
        power = 0.0;
      }
      return power;
    }

    public void setTeleopDrive(double drive, double turn) {
      drivePower = drive;
      turnPower = turn;
      state = States.TELEOP;
    }
    /* 
    public void setBalanceHold(){
      double KP = 0.45 / 20;     //aiming for 30% power at 20 degrees was .3
      drivePower = getPitch() * KP;
      turnPower = 0.0;
    }
    */
    
    /* Applies power to motors to keep from rolling of angled platform */
    public void setBalanceHold(){
      setVelocityDrive(0.0);
    }

    /* Climb the ramp slowly until balanced.  Balances forward and backward. */
    public void setBalanceDrive() {
      double KP = 0.4;
      double inchesPerSec = KP * getPitch();
      setVelocityDrive(inchesPerSec);
    }

    public void setDistanceDrive(double inches) {
      setDistanceDrive(inches, 0.6, "LOW");
    }
    
    public void setDistanceDrive(double inches, double maxPower, String accel) {
      targetDistance = inches;
      distPID.setOutputLimits(-maxPower, maxPower);
      distPID.setTarget(inches);
      distDriveAcceleration(accel);
      setHeadingHoldPower();
      resetEncoder();
      state = States.DIST_DRIVE;
    }

    public void setWallDrive(double inches){
      if (state != States.DRIVE_TO_WALL  && state != States.DRIVE_TO_WALL_2) {
        targetDistance = inches;
        distPID.setTarget(inches);
        setHeadingHoldPower();
        resetEncoder();
        stallCount = 0;
        state = States.DRIVE_TO_WALL;
      }
    }

    public void setVelocityDrive(double inchesPerSec) {
      targetVelocity = inchesPerSec;
      velPID.setTarget(inchesPerSec);
      setTargetAngle(getSignedAngle());
      setHeadingHoldPower();
      resetEncoder();
      state = States.VEL_DRIVE;   // Drive at a set velocity in inches per second.  It is pitch sensitive.
    }

    public void doVisionDrive(double drive){
      drivePower = drive;
      turnPower = vision.calcTurnPower();
      setTargetAngle(getSignedAngle());
      setHeadingHoldPower();
      state = States.VISION_TRACK;
    }

    public void setTargetAngle(double signedAngle){
      targetHeading = signedAngle;
      turnPID.setTarget(signedAngle);
      setTurningPower();
    }

    //Set turn to a signedAngle between 180 and -180
    public void setTurnTo(double signedAngle){
      setTargetAngle(signedAngle);
      state = States.TURN;
    }

    public void adjustTurnPIDTarget(double adjust){
      turnPID.setTarget(turnPID.getTarget() + adjust);
    }

    public boolean isComplete() {
      if (state == States.COMPLETE || state == States.HOLD) {
        stopMotors();
        return true;
      } else{
        return false;
      }
    }

    public void update() {
      switch(state) {
        case TELEOP:
          setTargetAngle(getSignedAngle());
          break;
        case DIST_DRIVE:
          double error = Math.abs(targetDistance - getInches());
          if (error < 2) {
            drivePower = 0.0;
            turnPower = 0.0;
            state = States.COMPLETE;
          } else {
            drivePower = distPID.calc(getInches());
            turnPower = turnPID.calc(getSignedAngle());
          }
          break;
        case VEL_DRIVE:
            velPID.setFeedForward(feedForward(targetVelocity));
            drivePower = velPID.calc(getVelocity());
            //turnPower = turnPID.calc(getSignedAngle());
            turnPower = 0.0;
          break;
        case TURN:
          drivePower = 0.0;
          turnPower = turnPID.calc(getSignedAngle());
          if (Math.abs(turnPID.getError()) < 2) {
            turnPower = 0.0;
            Common.debug("dt: TURN Complete");
            state = States.COMPLETE;
          }
          break;
        case DRIVE_TO_WALL:
          drivePower = distPID.calc(getInches());
          turnPower = turnPID.calc(getSignedAngle());
          if (Math.abs(drivePower) < STALL_DRIVE_POWER){
            state = States.DRIVE_TO_WALL_2;
          }
          if (isStalled()){
            drivePower = 0.0;
            state = States.COMPLETE;
          }
          break;
        case DRIVE_TO_WALL_2:
          drivePower = STALL_DRIVE_POWER;
          turnPower = turnPID.calc(getSignedAngle());
          if (isStalled()){
            drivePower = 0.0;
            state = States.COMPLETE;
          }
          break;
        case VISION_TRACK:
          state = States.TELEOP;
          break;
        case COMPLETE:
          drivePower = 0.0;
          turnPower = 0.0;
          break;
			  case HOLD:
				  drivePower = 0.0;
				  turnPower = turnPID.calc(getSignedAngle());
				  break;
      }
      // Now power the drivetrain
      drivePower = Common.constrain(drivePower, -MAX_DRIVE, MAX_DRIVE);
      turnPower = Common.constrain(turnPower, -MAX_TURN, MAX_TURN);

      drive.arcadeDrive(drivePower, turnPower);  
      checkForStall();   //Update stall count, if not moving.
      debug();

      drivePower = 0.0;
      turnPower = 0.0;
    }

    public String getState() {
      return state.toString();
    }

    public void debug() {
      SmartDashboard.putNumber("DT: Inches: ", getInches());
      SmartDashboard.putNumber("DT: Encoder Counts: ", getEncoderCounts());
      SmartDashboard.putNumber("DT: Velocity", getVelocity());
      SmartDashboard.putNumber("DT: drivePower", drivePower);
      SmartDashboard.putNumber("DT: turnPower", turnPower);
      SmartDashboard.putNumber("DT: velPID ff", velPID.getFeedForward());
      SmartDashboard.putNumber("DT: feed foward(0)", feedForward(0));
      SmartDashboard.putNumber("DT: pitch", getPitch());
      SmartDashboard.putString("DT: state", getState());
      SmartDashboard.putNumber("DT: distPIDTarget", distPID.getTarget());
      SmartDashboard.putNumber("DT: turnPIDTarget", turnPID.getTarget());
      SmartDashboard.putNumber("DT: gyro", getGyroYaw());
      vision.debug();
    }


}
