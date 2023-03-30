package frc.robot;

public class Auto {
    DriveTrain dt;
    Intake intake;
    Arm arm;

    public enum PlaceAndTaxiStates {
        INIT,                   //Start backing away so we have room to raise arm
        BACK_UP,                //Wait for backup to complete and then raise arm
        ARM_UP,                 //When arm is up, drive foward for drop off
        DRIVE_FORWARD,          //Wait for drive to finish
        DROP_OFF,               //Wait a second for object to eject
        INITIAL_BACK_OUT,       //Backup some before parking arm
        BACK_OUT_COMMUNITY,     //Complete backing out of community
        THOMAS_180,             //Rotate 180 toward center field
        COMPLETE;               //Auto finished
    }

    public enum DropLeaveAndBalanceStates {
        INIT,
        ARM_UP,
        PLACE_CONE,
        PLACE_CUBE1,
        PLACE_CUBE2,
        BACK_OUT,
        BACK_ONTO_RAMP,
        BACK_OFF_RAMP,
        GO_TO_BALANCE_POINT,
        BALANCE,
        COMPLETE;
    }

    

    public enum DoubleCubeStates {
        STEP1A,
        STEP1B,
        STEP1C,
        STEP2,
        STEP3,
        STEP4,
        STEP5,
        STEP6,
        STEP7,
        STEP8,
        STEP9,
        COMPLETE;
    }

    private enum Modes {
        PLACE_TAXI,
        DROP_LEAVE_BALANCE,
        TEST,
        DOUBLE_CUBE;
    }

    private enum Objects {
        CONE,
        CUBE;
    }

    private enum Tier {
        LOW,
        HIGH;
    }

    public static enum Positions {
        LOADING_ZONE,
        MID,
        FAR;
    }    

    private Modes autoMode = Modes.PLACE_TAXI;
    private PlaceAndTaxiStates pTState = PlaceAndTaxiStates.INIT;
    private DropLeaveAndBalanceStates dLState = DropLeaveAndBalanceStates.INIT;
    private DoubleCubeStates dCState = DoubleCubeStates.STEP1A;
    private Objects autoObject = Objects.CONE;
    private Tier autoTier = Tier.LOW;  
    private Long startTime;

    public Auto(DriveTrain dt, Arm arm, Intake intake){
        this.dt = dt;
        this.arm = arm;
        this.intake = intake;
    }

    public void init(){
        reset();
    }

    public void reset(){
        dt.reset();
        pTState = PlaceAndTaxiStates.INIT;
        dLState = DropLeaveAndBalanceStates.INIT;
        dCState = DoubleCubeStates.STEP1A;
    }

    public void selectLowTier(){
        autoTier = Tier.LOW;
    }

    public void selectHighTier(){
        autoTier = Tier.HIGH;
    }
    
    public void selectCone(){
        autoObject = Objects.CONE;
        intake.setTarget("CONE");
    }

    public void selectCube(){
        autoObject = Objects.CUBE;
        intake.setTarget("CUBE");
    }

    public void selectPlaceAndTaxi(){
        autoMode = Modes.PLACE_TAXI;
    }

    public void selectDropLeaveBalance(){
        autoMode = Modes.DROP_LEAVE_BALANCE;
    }

    public void selectDoubleCube(){
        autoMode = Modes.DOUBLE_CUBE;
        selectCube();
    }
    
    /*public void selectTest(){
        autoMode = Modes.TEST;
        selectCube();
    }*/

    public String getMode(){
        return autoMode.toString();
    }

    /* Update method for when performing Place and Taxi auto */
    public void placeAndTaxiAuto() {
        switch (pTState) {
            case INIT:
            if (arm.isComplete()) {
                Common.debug("Auto: Arm is ready. Start drive back.");
                dt.setDistanceDrive(-20);//was -32
                arm.gotoPark();
                intake.hold();
                pTState = PlaceAndTaxiStates.BACK_UP;
            }
            break;

            case BACK_UP:
            if(dt.isComplete()) {
                Common.debug("Auto: Drive back complete, intiating arm up");
                if (autoObject == Objects.CONE && autoTier == Tier.HIGH){
                    arm.gotoConeHigh();
                }
                if (autoObject == Objects.CONE && autoTier == Tier.LOW){
                    arm.gotoConeLow();
                }
                if (autoObject == Objects.CUBE && autoTier == Tier.HIGH){
                    arm.gotoCubeHigh();
                }
                if (autoObject == Objects.CUBE && autoTier == Tier.LOW){
                    arm.gotoCubeLow();
                }
                startTime = Common.time();
                pTState = PlaceAndTaxiStates.ARM_UP;
            }
            break;
            
            case ARM_UP:
            if (arm.isComplete() && Common.time() - startTime > 1000) {  //was 2500
                Common.debug("Auto: Arm up complete, intiating drive forward");
                dt.setDistanceDrive(19);//was 31
                pTState = PlaceAndTaxiStates.DRIVE_FORWARD;
            }
            break;
            
            case DRIVE_FORWARD:
            if (dt.isComplete()) {
                Common.debug("Auto: Drive forward complete, intiating eject");
                if(autoObject == Objects.CONE){
                    intake.eject();
                }
                if(autoObject == Objects.CUBE){
                    intake.eject();
                }
                startTime = Common.time();
                pTState = PlaceAndTaxiStates.DROP_OFF;
            }
            break;
            
            case DROP_OFF:
            if (Common.time() - startTime > 500) { //was 1000
                Common.debug("Auto: Eject complete, intiating drive back");
                dt.setDistanceDrive(-183);//was -194
                pTState = PlaceAndTaxiStates.INITIAL_BACK_OUT;
            }
            break;

            case INITIAL_BACK_OUT:  //As we head out of community, park arm
            if (dt.getInches() <= -32) {   //Only when we've driven away far enough can we bring arm down.
                Common.debug("Auto: Initiating park and intake hold");
                arm.gotoPark();
                intake.hold();
                pTState = PlaceAndTaxiStates.BACK_OUT_COMMUNITY;
            }
            break;

            case BACK_OUT_COMMUNITY:
            if (dt.isComplete()) {
                if (autoObject == Objects.CUBE) {
                    dt.setTurnTo(180);
                    Common.debug("Auto: Backout of community complete and intiating turn");
                    pTState = PlaceAndTaxiStates.THOMAS_180;
                } else {
                    Common.debug("Auto: Backout of community complete, auto complete");
                    pTState = PlaceAndTaxiStates.COMPLETE;
                }
            }
            break;

            
            case THOMAS_180:
                if (dt.isComplete()){ 
                    intake.on();
                    if (autoObject == Objects.CUBE){
                        arm.goToFloorIntakeCube();
                    }
                    if (autoObject == Objects.CONE){
                        arm.goToFloorIntakeCone();
                    }
                    Common.debug("Auto: Completed 180 turn. Auto done");
                    // What would you initiate next?
                    pTState = PlaceAndTaxiStates.COMPLETE;
                }


            case COMPLETE:
            break;
        }
    }
    
    /* Drop object and leave community */
    public void dropLeaveAndBalance(){
        switch(dLState){
            case INIT:
            //if (arm.isComplete()) {
                if (autoObject == Objects.CONE) {
                    arm.setTargetLower(0);
                    arm.setTargetUpper(15);
                    intake.eject();
                    dLState = DropLeaveAndBalanceStates.PLACE_CONE;
                } else {
                    intake.hold();
                    arm.setTargetLower(-22);
                    arm.setTargetUpper(38);
                    dLState = DropLeaveAndBalanceStates.PLACE_CUBE1;
                }
            //}
            break;

            case PLACE_CONE:
            if (arm.isComplete()){
                Common.debug("Auto: Cone drop completed, initiating backup onto ramp.");
                dt.setDistanceDrive(-136);
                dLState = DropLeaveAndBalanceStates.BACK_ONTO_RAMP;
            }
            break;

            case PLACE_CUBE1:
            if (arm.isComplete()){
                Common.debug("Auto: Arm extended to place cube, initiating timed eject.");
                startTime = Common.time();
                intake.eject();
                dLState = DropLeaveAndBalanceStates.PLACE_CUBE2;
            }
            break;

            case PLACE_CUBE2:
            if (Common.time() - startTime > 99){ // Was 100
                Common.debug("Auto: Cube eject complete, initiating backup onto ramp.");
                intake.hold();
                arm.gotoPark();
                dt.setDistanceDrive(-136);
                dLState = DropLeaveAndBalanceStates.BACK_ONTO_RAMP;
            }
            break;

            case BACK_ONTO_RAMP:
            if (dt.getInches() < -20) {
                arm.gotoPark();
            }
            if (dt.isComplete() || dt.getPitch()>13){
                if (dt.isComplete()) {
                    Common.debug("Auto: Drive dist completed, initiating back off ramp.");
                } else {
                    Common.debug("Auto: Pitch completed drive onto ramp, initiating back off ramp.");
                }
                intake.hold();
                arm.goToFloorIntakeCone();
                dt.setDistanceDrive(-78, 0.7, "MEDIUM");  //was -88
                dLState = DropLeaveAndBalanceStates.BACK_OFF_RAMP;
            }
            break;

            case BACK_OFF_RAMP:
            if (dt.isComplete()){
                Common.debug("Auto: Complete Back Off, drive up to balance point.");
                dt.setDistanceDrive(120);  //was 105
                dLState = DropLeaveAndBalanceStates.GO_TO_BALANCE_POINT;
            }
            break;

            case GO_TO_BALANCE_POINT:
            if (dt.isComplete()  || (dt.getInches()>70 && dt.getPitch() < 5)){  //was 80 inches// if drive finishes or we are on ramp and the angle is down to less then 5 degrees
                if (dt.isComplete()) {
                    Common.debug("Auto: Dist drive to balance point completed, initiating balance hold.");
                } else {
                    Common.debug("Auto: Low pitch complete balance point drive, initiating balance hold.");
                }
                //arm.gotoPark();
                dt.setBalanceDrive();
                dLState = DropLeaveAndBalanceStates.BALANCE;
            }
            break;

            case BALANCE:
            dt.setBalanceDrive();
            if (dt.isComplete()){
            }
            break;

            case COMPLETE:
            break;
        }
    }

    // Double Cube auto
    private void doubleCubeAuto(){
        switch (dCState){

            case STEP1A:
            intake.hold();
            arm.setTargetLower(-22);
            arm.setTargetUpper(38);
            dt.vision.setTarget("CUBE");
            dCState = DoubleCubeStates.STEP1B;
            break;

            case STEP1B:
            if (arm.isComplete()){
                startTime = Common.time();
                intake.eject();
                dCState = DoubleCubeStates.STEP1C;
            }
            break;

            case STEP1C:
            if (Common.time() - startTime > 750){
                intake.hold();
                arm.gotoPark();
                dt.setDistanceDrive(-184, 0.8, "HIGH");   //was -185 //18 FT 8 INCHES LESS ROBOT LENGTH OF 29 INCHES
                dCState = DoubleCubeStates.STEP2;
            }
            break;

            case STEP2:
            if (dt.isComplete()){
                dt.setTurnTo(180);
                dCState = DoubleCubeStates.STEP3;
            }
            break;


            case STEP3:
            if (dt.isComplete()){
                arm.goToFloorIntakeCube();
                intake.on();
                startTime = Common.time();
                dCState = DoubleCubeStates.STEP4;
            }
            break;

            case STEP4:
            if (Common.time() - startTime > 1250){
                arm.gotoPark();
                dt.setTurnTo(2); // was 1
                dCState = DoubleCubeStates.STEP5;
            }
            break;

            case STEP5:
            if (dt.isComplete()){
                intake.hold();
                dt.setDistanceDrive(192, 0.8, "HIGH"); //was 193
                dCState = DoubleCubeStates.STEP6;
            }
            break;

            case STEP6:
            double turnError = dt.vision.calcTurnError();
            if (turnError > 1){
                // dt.adjustTurnPIDTarget(+0.2);
            } else if (turnError < -1){
                // dt.adjustTurnPIDTarget(-0.2);
            }
            if (dt.getInches() > 55){   //was 100
                dt.distDriveAcceleration("MEDIUM");
                arm.gotoCubeHigh();
                dCState = DoubleCubeStates.STEP7;
            }
            break;

            case STEP7:
            if(dt.isComplete() && arm.isComplete()){
                startTime = Common.time();
                intake.eject();
                dCState = DoubleCubeStates.STEP8;
            }
            break;

            case STEP8:
            if (Common.time() - startTime > 500){
                intake.hold();
                arm.setTargetLower(-70);
                dCState = DoubleCubeStates.COMPLETE;
            }
            break;

            case COMPLETE:
            break;
        }
    }

    /* Auto update will call the appropriate update mehtod based on the autonomous mode selected. */
    public void update() {
        switch (autoMode) {
            case PLACE_TAXI:
                placeAndTaxiAuto();
            break;

            case DROP_LEAVE_BALANCE:
                dropLeaveAndBalance();
            break;

            case DOUBLE_CUBE:
                doubleCubeAuto();
            break;

            /*case DOUBLE_CUBE:
            doubleCubeAuto();
            break;*/
        }
        dt.update();
        arm.update();
        debug();
    }
    
    public void debug(){
        Common.dashStr("Auto: Mode", autoMode.toString());
        Common.dashStr("Auto: dCState", dCState.toString());
        Common.dashBool("Auto: Object", autoObject.toString() == "CUBE");
        Common.dashStr("Auto: Tier", autoTier.toString());
        //Common.dashStr("Auto: DLState", dLState.toString());
    }

}