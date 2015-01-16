package org.usfirst.frc.team3504.robot.commands;

import org.usfirst.frc.team3504.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/*
 * 
 */
public class DriveLeft extends Command {
	
	public DriveLeft() {
		requires(Robot.chassis);
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		Robot.chassis.driveLeft(Robot.oi.getChassisJoystick());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.chassis.stop();
	}

	@Override
	protected void interrupted() {
		end();
	}
	
}
