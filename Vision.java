package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta;       //tx is degrees off target left right, ty is degrees off target up down, ta is target area
    private String currentTarget = "NONE";      //values are NONE, LOW_CONE, HIGH_CONE, CUBE (low, ids 1,2,3,6,7,8), STATION (high, ids 4,5)
    private double turnPower;   //Calculated power to turn the robot towards target
    private double drivePower;  //Calculated power to drive the robot towards target
    private double MIN_DISTANCE = 12; //How close to target until we get to minimum power. Min Power is applied from this point forward
    private double MAX_DISTANCE = 120;//Max distance the LimeLight detects target.
    private double MIN_POWER = 0.3;  //Power applied when closer than min distance.(about 12" per second)
    private double MAX_POWER = 0.53;   //Max power that we will drive the robot.
    private double targetDistance = 0.0; //Desired distance between target and camera. This distance varies by target type. 

    public void init(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        setTarget("NONE");
    }

    private void changeTarget(){
        if (currentTarget == "LOW_CONE"){
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
          targetDistance = 47.4;
        }
        if (currentTarget == "HIGH_CONE"){
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
          targetDistance = 60.7;
        }
        if (currentTarget == "CUBE"){
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
          targetDistance = 44.4;
        }
        if (currentTarget == "STATION"){
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
          targetDistance = 34;
        }
        if (currentTarget == "NONE"){
            cameraMode();
        } else {
            trackMode();
        }
    }

    private void trackMode(){
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }

    private void cameraMode(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }

    private void enableLight(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    private void disableLight(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public double distanceFinder(){
        double h1 = 31;   // height of camera in inches
        double h2 = 0;  //Height of the cone target
        double distance;
        if (currentTarget == "LOW_CONE" || currentTarget == "HIGH_CONE"){
          if (currentTarget == "LOW_CONE"){
          h2 = 24.25;
          }
          if (currentTarget == "HIGH_CONE"){
          h2 = 44;
          }
    
          double cameraY = 2.0; // Camera angle
          double angleY = ty.getDouble(0.0) + cameraY;
          angleY = Math.toRadians(angleY);
          distance = (h2-h1) / Math.tan(angleY);
          if (distance > 200) {
            distance = 0;                                                                                    
          }
        } else {
          double area = ta.getDouble(0.0);
          distance = (89.2069 + -32.4584 * area + 4.0585 * (area * area));
        }
        return distance;
    }

    public double calcTurnError(){
        double x = tx.getDouble(0.0);
        return x;
    }
        
    
    public double calcTurnPower(){
        double x = tx.getDouble(0.0);
        double target = 0;
        double Kp = 0.01; //.025
        double current = x;
        double error = current - target;
        if (Math.abs(error) < 1){
          turnPower = 0;
        } else {
          turnPower = Kp * error;
          if (turnPower < 0){
            turnPower -= 0.15;
          } else {
          turnPower += 0.15;
          }
        }
        
        return turnPower;
      }

      public double calcDrivePower(){
        double slope = (MAX_POWER-MIN_POWER)/(MAX_DISTANCE-MIN_DISTANCE);
        double intercept = (MIN_POWER - (slope * MIN_DISTANCE));
        double distanceToGo = (distanceFinder() - targetDistance);
        double power;
        if (distanceToGo < 0){
            distanceToGo = 0;
        }
        if (distanceToGo < MIN_DISTANCE){
          power = MIN_POWER;
        } else {
          power = slope * distanceToGo + intercept;
        }
        return power;

        /**double Kp = 0.3 / 50.0; // 30 percent additional power at 50 inches
        double current = distanceFinder(); // inches from cone node
        double error = current - target;
        if (current == 0) {
          error = 0;  
        }
        if (Math.abs(error) < 1.0) {
          drivePower = 0.0;
        } else {
        drivePower = 0.45 + Kp * error;
        }
        return drivePower;  **/
    }

    public void setTarget(String target){
        if (target == "NONE" || target == "LOW_CONE" || target == "HIGH_CONE" || target == "CUBE" || target == "STATION"){
            currentTarget = target;
        } else {
            Common.debug("Vision target name is not valid: " + target);
            currentTarget = "NONE";
        }
        if (currentTarget == "NONE" || currentTarget == "CUBE" || currentTarget == "STATION"){
            disableLight();
        } else {
            enableLight();
        }
        changeTarget();
    }

    private double x(){
      return tx.getDouble(0.0);
    }

    private double y(){
      return ty.getDouble(0.0);
    }

    private double area(){
      return ta.getDouble(0.0);
    }
  
    public void debug(){
      //Common.dashNum("distanceToGo", (distanceFinder() - targetDistance));
      //Common.dashNum("Distance Finder", distanceFinder());
      Common.dashNum("Vision: Turn Error", calcTurnError());
    }
}