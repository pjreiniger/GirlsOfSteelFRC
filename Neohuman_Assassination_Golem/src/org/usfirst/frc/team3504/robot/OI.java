package org.usfirst.frc.team3504.robot;

import org.usfirst.frc.team3504.robot.commands.*;
import org.usfirst.frc.team3504.robot.commands.autonomous.*;
import org.usfirst.frc.team3504.robot.commands.buttons.*;
import org.usfirst.frc.team3504.robot.commands.camera.*;
import org.usfirst.frc.team3504.robot.subsystems.TestBoardPID;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public enum DriveDirection {kFWD, kREV}; 
	
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

	Joystick operatorStick = new Joystick(2);
	Joystick drivingStickForward = new Joystick(0);
	Joystick drivingStickBackward = new Joystick(1); 

	
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button. 
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
	//JOYSTICK BUTTONS
	private JoystickButton collectBallButton;
	private JoystickButton releaseBallButton;
	
	private JoystickButton shiftUpButton;
	private JoystickButton shiftDownButton;
	
	private JoystickButton testAutonomous;
	private JoystickButton testBoardPID;

	private DriveDirection driveDirection = DriveDirection.kFWD; 
	
	private JoystickButton switchCam;

	private JoystickButton switchToForward; 
	private JoystickButton switchToBackward; 
	
	
	private JoystickButton testDesiredRotationAngle;
	
	private JoystickButton switchToCamPivot;
	
	//TODO: figure out how to write buttons for button board:
	//Flap: Rocker + 2 buttons, Pivot: 3 buttons, Claw: 2 Buttons, Other: 3 Buttons (defenses & scoring), Shooter: 2 buttons - total 12 buttons + rocker
	
	public OI() {
		collectBallButton = new JoystickButton(operatorStick, 1);
		collectBallButton.whileHeld(new CollectBall());
		releaseBallButton = new JoystickButton(operatorStick, 2);
		releaseBallButton.whileHeld(new ReleaseBall());
		
		shiftUpButton = new JoystickButton(drivingStickForward, 3);
		shiftUpButton.whenPressed(new ShiftUp());
		shiftDownButton = new JoystickButton(drivingStickForward, 4);
		shiftDownButton.whenPressed(new ShiftDown());
		
		testAutonomous = new JoystickButton(drivingStickForward, 5);
		testAutonomous.whenPressed(new AutoDriveDistance(60.0));
		
		switchCam = new JoystickButton(drivingStickForward, 10);
		switchCam.whenPressed(new SwitchCam());

		switchToForward = new JoystickButton(drivingStickForward, 1); 
		switchToForward.whenPressed(new SwitchToForward()); 

		switchToBackward = new JoystickButton(drivingStickBackward, 1);
		switchToBackward.whenPressed(new SwitchToBackward());
		
		switchToCamPivot = new JoystickButton(drivingStickForward, 6);
		switchToCamPivot.whenPressed(new UpdateCam());
		
		testBoardPID = new JoystickButton(drivingStickForward,12);
		testBoardPID.whenPressed(new TestBoardPositionPID());
		

		testDesiredRotationAngle = new JoystickButton(drivingStickForward, 6);
		testDesiredRotationAngle.whenPressed(new RotateToDesiredAngle(.2, 90));
		
	}
	
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	
	public double getDrivingJoystickY() {
		if (driveDirection == DriveDirection.kFWD){
			return drivingStickForward.getY();
		}
		else {
			return -drivingStickBackward.getY(); 
		}
	}
	
	public double getDrivingJoystickX() {
		if (driveDirection == DriveDirection.kFWD){
			return drivingStickForward.getX();
		}
		else {
			return drivingStickBackward.getX(); 
		}
	}
	
	public double getOperatorStickThrottle() {
		return operatorStick.getThrottle();
	}
	
	public void setDriveDirection(DriveDirection driveDirection) {
		this.driveDirection = driveDirection; 
		System.out.println("Drive direction set to: " + driveDirection);
	}
	
	public boolean isJoystickReversed() {
		return (driveDirection == DriveDirection.kREV); 
	}
	
}

