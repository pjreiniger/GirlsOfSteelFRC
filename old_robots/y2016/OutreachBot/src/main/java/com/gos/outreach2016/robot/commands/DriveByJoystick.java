package com.gos.outreach2016.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.gos.outreach2016.robot.OI;
import com.gos.outreach2016.robot.subsystems.DriveSystem;

/**
 *
 */
public class DriveByJoystick extends CommandBase {

    private final DriveSystem m_driveSystem;
    private final OI m_oi;

    public DriveByJoystick(OI oi, DriveSystem driveSystem) {
        m_oi = oi;
        m_driveSystem = driveSystem;
        addRequirements(m_driveSystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_driveSystem.takeJoystickInputs(m_oi.getDriveStick());

        SmartDashboard.putNumber("Drive Left Encoder ", m_driveSystem.getEncoderLeft());
        SmartDashboard.putNumber("Drive Right Encoder ", m_driveSystem.getEncoderRight());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveSystem.stop();
    }


}
