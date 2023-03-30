// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class Arm {

    // Constants for power and position constraints
    public double MAX_UP_POWER_UPPER = 0.35;     //was 0.35   // Max power allowed to move upper arm up
    public double MAX_DOWN_POWER_UPPER = -0.25;  //was -0.25    // Max power allowed to move upper arm down
    private double MIN_DEGREES_UPPER = -30;         // Min angle when upper arm is full retraced into robot (0 degrees is straight down).  Will go negative.
    private double MAX_DEGREES_UPPER = 160;         // Max angle when upper arm is reaching upwards.
    private double TOLERANCE_UPPER = 1.0;           // Number of degrees of allowable difference to consider target angle move complete.
    public double MAX_OUT_POWER_LOWER = 0.45;    //was 0.45   // Max power to move lower arm outward.
    public double MAX_IN_POWER_LOWER = -0.4;    //was -0.25   // Max power to move lower arm inward.
    private double MIN_DEGREES_LOWER = -73;      //was -67   // Min angle of lower arm when retracted into robot
    private double MAX_DEGREES_LOWER = 75;          // Max angle of lower arm when extended from robot
    private double TOLERANCE_LOWER = 2.0;           // Number of degrees of allowable difference to consider target angle move complete.
 
    // PIDs
    private double targetUpper = 0; 
    private double targetLower = 0;
    private double feedforwardUpper = 0;        // The feedforward value for the upper arm, to overcome gravity.
    private double kP_UPPER = 1.0/20.0, kI_UPPER = 0, kD_UPPER = 0; // upperarm position. pid aiming for 100% power at 20 degrees of error
    private double kP_LOWER = 1.0/25.0, kI_LOWER = 0, kD_LOWER = 0; // lowerarm position. pid aiming for 100% power at 40 degrees of error
    private PID upperPID;
    private PID lowerPID;

    // Hardware 
    private WPI_TalonSRX upperArm = new WPI_TalonSRX(Constants.UPPER_ARM);
    private WPI_TalonSRX lowerArm = new WPI_TalonSRX(Constants.LOWER_ARM);    
    private CANCoder upCanCoder = new CANCoder(Constants.CANCODER_U);
    private CANCoder lowCanCoder = new CANCoder(Constants.CANCODER_L);

    // States
    private enum States {
        INIT,               // Check Arm position and determine starting start as either STOW or UNSTOW
        PARKED,               // Tuck arm into normal stowed position
        STOWED,             // Arm in stowed position.  Must be unstowed to be used.
        UNSTOW,             // Move arm to unstowed position
        READY,           // Arm unstowed, can be moved to other positions, except STOWED
        FLOOR_INTAKE,       // Arm moving or at floor level to intake Cube or Cone
        LOADING_ZONE,       // Arm moving or at loading station level
        CONE_LOW,           // Arm moving or at cone low
        CONE_HIGH_ONE,          // Arm moving or at cone high
        CONE_HIGH_TWO,
        CUBE_LOW,           // Arm moving or at cube low
        CUBE_HIGH_ONE,
        CUBE_HIGH_TWO;          // Arm moving or at cube high
    }
    private States state = States.INIT;

    /* Initialize the Arm subsystem.  Run this when robot first initializes. */
    public void init() {
        // MOTORS
        // Upper arm is a Falcon that will be PID controlled with a potentiometer to measure arm angle
        upperArm.configFactoryDefault();
        upperArm.setNeutralMode(NeutralMode.Brake);
        upperArm.setInverted(false);
        upperArm.setSensorPhase(false);
        upperArm.configPeakOutputForward(MAX_UP_POWER_UPPER);
        upperArm.configPeakOutputReverse(MAX_DOWN_POWER_UPPER);
        upperArm.configOpenloopRamp(.15);
        // Lower arm is a Falcon with an external absolute encoder connected to it.
        lowerArm.configFactoryDefault();
        lowerArm.setNeutralMode(NeutralMode.Brake);
        //lowerArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // Note: We wont be using the controllers PID, just reading the enconder from Java.
        lowerArm.setInverted(true);
        lowerArm.setSensorPhase(false);
        lowerArm.configPeakOutputForward(MAX_OUT_POWER_LOWER);
        lowerArm.configPeakOutputReverse(MAX_IN_POWER_LOWER);           
        // PIDS  (Note: PIDs are running in Java, not on the motor controller)
        upperPID = new PID(kP_UPPER, kI_UPPER, kD_UPPER, 0.0, false, "UpperPID", false);
        upperPID.setOutputLimits(MAX_DOWN_POWER_UPPER, MAX_UP_POWER_UPPER);
        //upperPID.setMinMagnitude(MIN_POWER_UPPER);
        lowerPID = new PID(kP_LOWER, kI_LOWER, kD_LOWER, 0.0, false, "LowerPID", false);
        lowerPID.setOutputLimits(MAX_IN_POWER_LOWER, MAX_OUT_POWER_LOWER);
        //lowerPID.setMinMagnitude(MIN_POWER_LOWER);
       
        reset();
    }

    /* Reset PIDS and put into INIT state.  Run this before enabling the robot. */
    public void reset() {
        upperArm.set( 0);
        lowerArm.set( 0);
        upperPID.reset();
        lowerPID.reset();
        targetUpper = getAngleUpper();
        targetLower = getAngleLower();
        state = States.INIT;
    }


    /* Returns lower arm angle as measured off absolute encoder connected to lower motor controller */
    private double getAngleLower() {
        final double OFFSET = 218.0;  // Encoder reading when arm is straight up and down 
        double angle = lowCanCoder.getAbsolutePosition();
        return angle - OFFSET;
    }

    /* Returns upper arm angle as measured off a multi-turn potentiometer
     * Reading is linear, so a straight line function is used to calculate angle based 
     *on observed endpont readings.
     */
    private double getAngleUpper() {
        double SCALEFACTOR = 180.0 / 204.0; // for 180 degrees of arm swing
        final double OFFSET = 196.0;  // Encoder reading when arm is straight up and down 
        double angle = upCanCoder.getAbsolutePosition() * SCALEFACTOR;
        return -(angle - OFFSET);
    }
    
    /* Set Lower arm target angle for PID control.  You MUST make sure target angle is safe for arm to move to.
     * Value will be constrained to the absolute range min/max range the arm is supposed to move.
     */
    public void setTargetLower(double degrees) {
        targetLower = Common.constrain(degrees, MIN_DEGREES_LOWER, MAX_DEGREES_LOWER);
    }

    /* Set Upper arm target angle for PID control.  You MUST make sure target angle is safe for arm to move to.
     * Value will be constrained to the absolute range min/max range the arm is supposed to move.
     */    
    public void setTargetUpper(double degrees) {
        targetUpper = Common.constrain(degrees, MIN_DEGREES_UPPER, MAX_DEGREES_UPPER);
    }

    private void PIDUpdateWithSafety(){
        double safeUpper = targetUpper;
        double safeLower = targetLower;
        if (getAngleLower() < -12){     //When lower arm is inside robot, upper arm movement is restricted.  
            if (getAngleUpper() < -9 && getAngleLower() < -14){   //was -6 on upper//If upper arm is inside robot too, then don't move it and only move lower arm outwards to safety
                safeUpper = getAngleUpper();
                safeLower = Common.constrain(targetLower, -12, MAX_DEGREES_LOWER);
            } else {                    //If upper arm is outside of robot, constain it from going inside, because lower is still inside robot.
                safeUpper = Common.constrain(targetUpper, 0, MAX_DEGREES_UPPER);
            }
        }
        upperPID.setTarget(safeUpper);
        lowerPID.setTarget(safeLower);
    }

    /* Test to see if Upper arm has reached Target */
    public boolean isCompleteUpper() {
        if (state != States.INIT  && Common.isNear(getAngleUpper(), targetUpper, TOLERANCE_UPPER)) {
            return true;
        } else {
            return false;
        }
    }

    /* Test to see if Upper arm has reached Target */
    public boolean isCompleteLower() {
        if (state != States.INIT  && Common.isNear(getAngleLower(), targetLower, TOLERANCE_LOWER)) {
            return true;
        } else {
            return false;
        }
    }

    /* Returns true when both Arms are within specified target angles.  
     * Note: If arm state is in INIT, will return false.
     */
    public boolean isComplete() {
        if (state != States.INIT && isCompleteLower() && isCompleteUpper()) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * Return true if the arm is an a state that is ready for general use.
     */
    private boolean isReady() {
        return ! (state == States.INIT || state == States.STOWED || state == States.UNSTOW);
    }

    /*
     * Return true if the arm is up in the air
     */
    public boolean isTippy(){
        if (getAngleUpper() > 40){
            return true;
        } else {
            return false;
        }
    }

    /*
     * Based on the current angle of the Upper arm, assess the impact of gravity and provide a feedforward
     * value to the PID.
     */
    private void updateFeedForwardUpper() {
        if (state != States.INIT) {
            final double kP_GRAVITY = 0.10;  //Calibrate this by determining holding power required to hold arm steady at 90 degrees (ie parallel to floor).
            double gravity = Math.sin(Math.toRadians(getAngleUpper())); // Influence of gravity at a given angle (+ values when arm is out in front)
            feedforwardUpper = kP_GRAVITY * gravity;
            upperPID.setFeedForward(feedforwardUpper);
        }
    }

    /* Initiate parking the arm. This is only allowed if the arm is ready. */
    public void gotoPark() {
        if (isReady()) {
            setTargetLower(-73);       
            setTargetUpper(0);
            state = States.PARKED;
        }
    }

    /* Initiate moving arm to FLOOR_INTAKE. This is only allowed if the arm is ready to be moved. */
    public void goToFloorIntakeCone() {
        if (isReady()) {
            setTargetLower(42);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(21);
            state = States.FLOOR_INTAKE;
        }
    }

    /* Initiate moving arm to FLOOR_INTAKE. This is only allowed if the arm is ready to be moved. */
    public void goToFloorIntakeCube() {
        if (isReady()) {
            setTargetLower(75);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(27);
            state = States.FLOOR_INTAKE;
        }
    }

    /* Initiate moving arm to LOADING_ZONE position. This is only allowed if the arm is ready to be moved. */
    public void gotoLoadingZoneCube() {
        if (isReady()) {
            setTargetLower(-73);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(88);
            state = States.LOADING_ZONE;
        }
    }

    public void gotoLoadingZoneCone() {
        if (isReady()) {
            setTargetLower(-73);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(92);   //was 81
            state = States.LOADING_ZONE;
        }
    }
    
    /* Initiate moving arm to Cone_High position. This is only allowed if the arm is ready to be moved. */
    public void gotoConeHigh() {
        if (isReady() && (state != States.CONE_HIGH_ONE && state != States.CONE_HIGH_TWO)) {
            //setTargetLower(10);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(68);     //was 117
            state = States.CONE_HIGH_ONE;
        }
    }
    
    /* Initiate moving arm to Cone_Low position. This is only allowed if the arm is ready to be moved. */
    public void gotoConeLow() {
        if (isReady()) {
            setTargetLower(-73);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(68); 
            state = States.CONE_LOW;
        }
    }
    
    /* Initiate moving arm to Cube_High position. This is only allowed if the arm is ready to be moved. */
    public void gotoCubeHigh() {
        if (isReady() && (state != States.CUBE_HIGH_ONE && state != States.CUBE_HIGH_TWO)) {
            //setTargetLower(35);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(65);
            state = States.CUBE_HIGH_ONE;
        }
    }
    
    /* Initiate moving arm to Cube_Low position. This is only allowed if the arm is ready to be moved. */
    public void gotoCubeLow() {
        if (isReady()) {
            setTargetLower(-61);       //Initiate move to unstore lower arm.  **Need to determine what is right angle for Unstowin */
            setTargetUpper(65);
            state = States.CUBE_LOW;
        }
    }

    public void joystickMove(double lowerArm, double upperArm){
        setTargetUpper(targetUpper + upperArm);
        setTargetLower(targetLower + lowerArm);
    }

    public void update() {
        switch (state) {
            case INIT:
                // If the upper arm is inside the robot then make sure lower arm > -10 degrees 
                if (getAngleUpper() < -5 && getAngleLower() < -14) {
                    setTargetLower(-10);
                }
                state = States.UNSTOW;
                break;

            case PARKED:      // Parked position.  Ready for movement.
                break;

            case UNSTOW:    // Wait for unstow to complete
                if (isComplete()) {
                    state = States.READY;
                }
                break;
                
            case READY:           // Ready for use, just stay put.
                break;
                
            case FLOOR_INTAKE:       // Move to floor level to intake Cube or Cone
                break;
                
            case LOADING_ZONE:       // Arm moving or at loading station level
                break;

            case CONE_LOW:           // Arm moving or at cone low
                break;
            
            case CONE_HIGH_ONE:          // Arm moving or at cone high
                if (isComplete()){ 
                    setTargetUpper(118);    //was 117
                    setTargetLower(10);
                    state = States.CONE_HIGH_TWO;
                }
                break;

            case CONE_HIGH_TWO:
                break;

            case CUBE_LOW:           // Arm moving or at cube low
                break;

            case CUBE_HIGH_ONE:          // Arm moving or at cube high
                if (isComplete()){
                    setTargetUpper(122);// was 120
                    setTargetLower(35);
                    state = States.CUBE_HIGH_TWO;
                }
                break;

            case CUBE_HIGH_TWO:
                break;
        }

        updateFeedForwardUpper();
        PIDUpdateWithSafety();
        double armUpperPower = upperPID.calc(getAngleUpper());
        double armLowerPower = lowerPID.calc(getAngleLower());
        upperArm.set(armUpperPower);
        lowerArm.set(armLowerPower);
        debug();
    }

    public void debug() {
        Common.dashNum("Arm: targetUpper", targetUpper);
        Common.dashNum("Arm: targetLower", targetLower);
        Common.dashNum("Arm: targetUpperPID", upperPID.getTarget());
        Common.dashNum("Arm: targetLowerPID", lowerPID.getTarget());
        Common.dashNum("Arm: feedforwardUpper", feedforwardUpper);
        Common.dashStr("Arm: state",state.toString());
        Common.dashNum("Arm: angleUpper", getAngleUpper());
        Common.dashNum("Arm: angleLower", getAngleLower());
        Common.dashNum("Arm: currentPIDupper", upperPID.getLastCalc());
        Common.dashNum("Arm: currentPIDlower", lowerPID.getLastCalc());
        Common.dashNum("Arm: UpperCanReading", upCanCoder.getAbsolutePosition());
        Common.dashBool("Arm: isComplete", isComplete());
    }
}
