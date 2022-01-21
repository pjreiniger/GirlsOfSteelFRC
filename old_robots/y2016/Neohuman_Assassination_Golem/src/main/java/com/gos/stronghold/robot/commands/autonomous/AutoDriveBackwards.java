package com.gos.stronghold.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.gos.stronghold.robot.subsystems.Chassis;

/**
 *
 */
public class AutoDriveBackwards extends Command {

    private final Chassis m_chassis;
    private final double m_inches;
    private final double m_speed;

    public AutoDriveBackwards(Chassis chassis, double distance, double speed) {
        m_chassis = chassis;
        requires(m_chassis);
        m_inches = distance;
        this.m_speed = speed;
        SmartDashboard.putBoolean("Autonomous is Finished!", false);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        m_chassis.resetEncoderDistance();
        SmartDashboard.putNumber("Autonomous Distance", m_inches);
        SmartDashboard.putNumber("Encoder distance initially AutoDriveDistance:", m_chassis.getEncoderDistance());
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        m_chassis.driveSpeed(-m_speed);
        m_chassis.printEncoderValues();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return m_chassis.getEncoderDistance() >= Math.abs(m_inches);    //for competition bot
        //return m_chassis.getEncoderDistance() >= inches;    //this is what works on practice bot.. don't know if its the same as competition since we switched it on practice

    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        m_chassis.stop();
        SmartDashboard.putBoolean("Autonomous is Finished!", true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}