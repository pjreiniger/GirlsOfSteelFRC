package org.usfirst.frc.team3504.robot.subsystems;

import org.usfirst.frc.team3504.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class TestBoardPID extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private CANTalon testMotor;
	private double encOffSetValue = 0;
	
	public TestBoardPID() {
		testMotor = new CANTalon(12);
		testMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		testMotor.setPID(0.0, 0.0, 0.0);
	}
	
	public void setPosition(double output) {
		testMotor.setSetpoint(output);
	}
	
	public double getEncoder() {
		return testMotor.getEncPosition();
	}
	
	public double getEncoderDistance() {
		return (getEncoder() - encOffSetValue) * RobotMap.DISTANCE_PER_PULSE;
	}
	
	public void resetEncoder() {
		encOffSetValue = getEncoder();
		System.out.println(getEncoder());
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

