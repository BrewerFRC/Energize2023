package frc.robot;
/* REFERENCE https://gitlab.com/robonautseverybot/everybot-2023/-/blob/main/src/main/java/frc/robot/Robot.java#L249 */


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {
    // MOTOR CONSTANTS
    // Positve values pull in a Cone and eject a Cube.
    // Cone
    private final double INTAKE_POWER_CONE = 0.75;
    private final double EJECT_POWER_CONE = -0.4;
    private final double HOLD_POWER_CONE = 0.05;
    // Cube
    private final double INTAKE_POWER_CUBE = -0.75;
    private final double EJECT_POWER_CUBE = 0.4;
    private final double HOLD_POWER_CUBE = -0.05;

    static final int INTAKE_CURRENT_LIMIT_A = 20;
    static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;


    private String currentTarget = "CONE";
    CANSparkMax motor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);


    public void init(){
        setTarget("CONE");
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        motor.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT_A);
    }

    /* Set the intakes target.  Do this before running the intake. */
    public void setTarget(String target) {
        if (target == "CONE" || target == "CUBE"){
            currentTarget = target;
        }
    }

    public String getTarget() {
        return currentTarget;
    }

    /* Turn intake on to load a CUBE or CONE */
    public void on() {
        if (currentTarget == "CONE") {
            motor.set(INTAKE_POWER_CONE);
        } else {
            motor.set(INTAKE_POWER_CUBE);
        }
    }

    public void eject() {
        if (currentTarget == "CONE") {
            motor.set(EJECT_POWER_CONE);
        } else {
            motor.set(EJECT_POWER_CUBE);
        }
    }

    public void hold() {
        if (currentTarget == "CONE") {
            motor.set(HOLD_POWER_CONE);
        } else {
            motor.set(HOLD_POWER_CUBE);
        }
    }

    
    public void debug() {
        Common.dashStr("Intake: currentTarget", currentTarget);
    }
}
