package frc.robot;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Common {
    
	public static double deadZone(double input, double minPut){
        double output=input;
		if (input > 0){
			if (input < minPut){
				output=0;
			}
		} else {
        	if (input > -minPut){
        	    output=0;
       	 	}
		}
        return output;
    }

	/**
	 * Some common functions to make our lives easier.
	 * 
	 * @author Brewer FIRST Robotics Team 4564
	 * @author Evan McCoy
	 * @author Jacob Cote
	 * @author Brent Roberts
	 * @author Samuel Woodward
	 */
	private static final DateFormat formatter = new SimpleDateFormat("HH:mm:ss:SSS");

	/**
	 * Returns true if x is between low and high values inclusive.
	 * 
	 * @param x value being tested.
	 * @param low lowest value
	 * @param high highest value
	 * @return boolean response
	 */
	public static boolean isBetween(double x, double low, double high) {
		if (x>= low && x<= high) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Returns true if x is within (+ or -) tolerance of the target.
	 * 
	 * @param x value being tested.
	 * @param target 
	 * @param tolerance 
	 * @return boolean response
	 */
	public static boolean isNear(double x, double target, double tolerance) {
		if (x >= target-tolerance && x <= target+tolerance) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Constrain a value between min and max values
	 * @param x
	 * @param low
	 * @param high
	 * @return x constrained
	 */
	public static double constrain(double x, double low, double high) {
		if (x < low) {
			x = low;
		} else {
			if (x > high) {
				x = high;
			}
		}
		return x;
	}


	/**
	 * Prints text in console.
	 * 
	 * @param text to print in console.
	 */
	public static void debug(String text) {
		System.out.println(formatter.format(new Date(time())) + ": "+ text);
	}
	
	/**
	 * Posts a string to the SmartDashboard.
	 * 
	 * @param title of the string to be output.
	 * @param a string to be output.
	 */
	public static void dashStr(String title, String a) {
		SmartDashboard.putString(title + ": ", a);
	}
	
	/**
	 * Posts a integer to the SmartDashboard
	 * 
	 * @param title of the integer to be posted.
	 * @param a integer to be posted.
	 */
	public static void dashNum(String title, int a) {
		SmartDashboard.putNumber(title + ": ", a);
	}
	
	
	/**
	 * Posts a double to the SmartDashboard
	 * 
	 * @param title of the double to be posted.
	 * @param a double to be posted. 
	 */
	public static void dashNum(String title, double a) {
		SmartDashboard.putNumber(title + ": ", a);
	}
	
	/**
	 * Posts a boolean to the SmartDashboard
	 * 
	 * @param title of the double to be posted.
	 * @param a boolean to be posted
	 */
	public static void dashBool(String Title, boolean a) {
		SmartDashboard.putBoolean(Title, a);
	}
	
	/**
	 * Returns the time in milliseconds.
	 * 
	 * @return The time in milliseconds.
	 */
	public static long time() {
		return Calendar.getInstance().getTimeInMillis();
	}
	
	/**
	 * Gets an double from smartDashboard.
	 * 
	 * @param title of value to get, dashNum adds a colon.
	 * @return value from smartDashboard, 999 is an default value.
	 */
	public static double getNum(String title) {
		return getNum(title, 999);
	}
	
	/**
	 * Gets an double from smartDashboard.
	 * 
	 * @param title of value to get.
	 * @param defaultValue the value that is returned if no value is found
	 * @return value from smartDashboard, defaultValue if no value is found
	 */
	public static double getNum(String title, double defaultValue) {
		return SmartDashboard.getNumber(title+": ", defaultValue);
	}
	
	
	/**
	 * Takes a value and a range it falls within and converts it to a different range.
	 * Defaults to minimum input if it exceeds the min or max input.
	 * 
	 * @param input Value to be converted to a different range
	 * @param minInput Minimum value for the range of the input
	 * @param maxInput Maximum value for the range of the input
	 * @param minOutput Minimum value for the range of the output
	 * @param maxOutput Maximum value for the range of the output
	 * @return double A value in the output range that is proportional to the input
	 */
	public static double map(double input, double minInput, double maxInput, double minOutput, double maxOutput) {
		input = Math.min(Math.max(input, minInput), maxInput);
		double inputRange = maxInput - minInput;
		double inputPercentage = (input-minInput)/inputRange;
		double outputRange = maxOutput - minOutput;
		double output = (outputRange * inputPercentage) + minOutput;
		return output;
	}


	 /**
     * Straight line acceleration calulator
	 * 
	 * (Needs testing!!!!!)
	 * 
     * @param currPower
     * @param targetPower
     * @param minPower (power need to get robot moving)
     * @param accel  (power increment to add per robot cycle)
     * @return
     */
    public double accelerate(double currPower, double targetPower, double minPower, double accel) {
		double diff = targetPower - currPower;
		if (diff > 0) {
			if (currPower > -minPower && currPower < minPower) {  //Skip past deadzone of the motor, if below the START_POWER
				diff = minPower;
			} else {
				if (diff > accel) {
					diff = accel;
				}
			}
		} else {
			if (currPower > -minPower && currPower < minPower) {  //Skip past deadzone of the motor, if below the START_POWER
				diff = -minPower;
			} else {
				if (diff < -accel) {
					diff = -accel;
				}
			}   
		}
  
		return currPower + diff;
	}

}
