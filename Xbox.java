package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends XboxController{
    
    private Map<String, Supplier<Boolean>> functionMap = new HashMap<String, Supplier<Boolean>>();
    private Map<String, Boolean> whenMap = new HashMap<String, Boolean>();
    private Map<String, Boolean> fallingMap = new HashMap<String, Boolean>();
    public enum buttons {
        x, y, a, b, start, back, dPadDown, dPadLeft, dPadRight, dPadUp, leftBumper,
        leftTrigger, rightBumper, rightTrigger, rightThumb, leftThumb 
    }


    public Xbox(int port) {
		super(port);
		setupFunctions();
	}

	public double deadzone(double input, double deadzone) {
		if (Math.abs(input) < deadzone) {
			return(0);
		} else {
			if(input > 0 ){
				return (input - deadzone) * 1.0/(1.0-deadzone);
			} else {
				return (input + deadzone) * 1.0/(1.0-deadzone);
			}
		}
	}
    
	public double deadzone(double input) {
		return deadzone(input, 0.2);
	}
    
	public double getLeftTrigger() {
		return deadzone(getLeftTriggerAxis());
	}
    
	public double getRightTrigger() {
		return deadzone(getRightTriggerAxis());
	}
    
	private void setupFunctions() {
		//Changed to private because I didn't see an reason to be public -Brent 10/11/18
		functionMap.put(buttons.a.toString(), this::getAButton);
		whenMap.put(buttons.a.toString(), false);
		fallingMap.put(buttons.a.toString(), false);
		
		functionMap.put(buttons.b.toString(), this::getBButton);
		whenMap.put(buttons.b.toString(), false);
		fallingMap.put(buttons.b.toString(), false);
		
		functionMap.put(buttons.x.toString(), this::getXButton);
		whenMap.put(buttons.x.toString(), false);
		fallingMap.put(buttons.x.toString(), false);
		
		functionMap.put(buttons.y.toString(), this::getYButton);
		whenMap.put(buttons.y.toString(), false);
		fallingMap.put(buttons.y.toString(), false);
		
		functionMap.put(buttons.start.toString(), this::getStartButton);
		whenMap.put(buttons.start.toString(), false);
		fallingMap.put(buttons.start.toString(), false);
		
		functionMap.put(buttons.back.toString(), this::getBackButton);
		whenMap.put(buttons.back.toString(), false);
		fallingMap.put(buttons.back.toString(), false);
		
		functionMap.put(buttons.dPadUp.toString(), () -> {
			return (this.getPOV() == -1) ? false : Math.abs(0 - this.getPOV()) < 45 || Math.abs(360 - this.getPOV()) < 45;
		});
		whenMap.put(buttons.dPadUp.toString(), false);
		fallingMap.put(buttons.dPadUp.toString(), false);
		
		functionMap.put(buttons.dPadRight.toString(), () -> {
			return (this.getPOV() == -1) ? false : Math.abs(90 - this.getPOV()) < 45;
		});
		whenMap.put(buttons.dPadRight.toString(), false);
		fallingMap.put(buttons.dPadRight.toString(), false);
		
		functionMap.put(buttons.dPadDown.toString(), () -> {
			return (this.getPOV() == -1) ? false : Math.abs(180 - this.getPOV()) < 45;
		});
		whenMap.put(buttons.dPadDown.toString(), false);
		fallingMap.put(buttons.dPadDown.toString(), false);
		
		functionMap.put(buttons.dPadLeft.toString(), () -> {
			return (this.getPOV() == -1) ? false : Math.abs(270 - this.getPOV()) < 45;
		});
		whenMap.put(buttons.dPadLeft.toString(), false);
		fallingMap.put(buttons.dPadLeft.toString(), false);
		
		functionMap.put(buttons.leftBumper.toString(), () -> {
			return this.getLeftBumper();
		});
		whenMap.put(buttons.leftBumper.toString(), false);
		fallingMap.put(buttons.leftBumper.toString(), false);
		
		functionMap.put(buttons.rightBumper.toString(), () -> {
			return this.getRightBumper();
		});
		whenMap.put(buttons.rightBumper.toString(), false);
		fallingMap.put(buttons.rightBumper.toString(), false);
		
		functionMap.put(buttons.leftTrigger.toString(), () -> {
			return deadzone(this.getLeftTrigger()) > 0;
		});
		whenMap.put(buttons.leftTrigger.toString(), false);
		fallingMap.put(buttons.leftTrigger.toString(), false);
		
		functionMap.put(buttons.rightTrigger.toString(), () -> {
			return deadzone(this.getRightTrigger()) > 0;
		});
		whenMap.put(buttons.rightTrigger.toString(), false);
		fallingMap.put(buttons.rightTrigger.toString(), false);
		
		functionMap.put(buttons.rightThumb.toString(), () -> {
			return this.getRawButton(10);
		});
		whenMap.put(buttons.rightThumb.toString(), false);
		fallingMap.put(buttons.rightThumb.toString(), false);

		functionMap.put(buttons.leftThumb.toString(), () -> {
			return this.getRawButton(9);
		});
		whenMap.put(buttons.leftThumb.toString(), false);
		fallingMap.put(buttons.leftThumb.toString(), false);
	}
}
