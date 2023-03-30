package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED{
    AddressableLED led = new AddressableLED(Constants.PWM_LED);
    // Reuse buffer
    // Default to a length of 30, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);
  
    //@Override
    public void init(){
        led.setLength(ledBuffer.getLength());
        // Set the data
        led.setData(ledBuffer);
        led.start();
    }
  
    public void coneColor(){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for yellow
        ledBuffer.setRGB(i,255,191,0);
        }
        led.setData(ledBuffer);
    }

    public void cubeColor(){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for purple
        ledBuffer.setRGB(i,64,0,128);
        }
        led.setData(ledBuffer);
    }

    public void balanceModeColor(){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for blue
        ledBuffer.setRGB(i,0,0,255);
        }
        led.setData(ledBuffer);
    }
}