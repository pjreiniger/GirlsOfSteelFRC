/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package girlsofsteel.objects;

import java.lang.Math;

/**
 *
 * @author Sylvie
 */
public class ShooterPoint {
    double voltage;
    double encoderSpeed;
    double battery;
    
    public ShooterPoint(double voltage, double encoderSpeed, double battery){
        this.voltage = voltage;
        this.encoderSpeed = encoderSpeed;
        this.battery = battery;
    }
    
    public double getVoltage(){
        return voltage;
    }
    
    public double getEncoderSpeed(){
        return encoderSpeed;
    }
    
    public double getBattery(){
        return battery;
    }
}