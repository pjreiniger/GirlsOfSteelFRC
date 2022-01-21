package com.gos.stronghold.robot.commands.camera;

import edu.wpi.first.wpilibj.command.Command;
import com.gos.stronghold.robot.subsystems.Camera;

/**
 *
 */
public class SwitchCam extends Command {

    private final Camera m_camera;

    public SwitchCam(Camera camera) {
        m_camera = camera;
        requires(m_camera);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        m_camera.switchCam();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}