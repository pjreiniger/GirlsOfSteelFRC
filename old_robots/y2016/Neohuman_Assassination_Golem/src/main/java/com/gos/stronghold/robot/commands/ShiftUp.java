package com.gos.stronghold.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.stronghold.robot.subsystems.Shifters;
import com.gos.stronghold.robot.subsystems.Shifters.Speed;

/**
 *
 */
public class ShiftUp extends CommandBase {
    private final Shifters m_shifters;

    public ShiftUp(Shifters shifters) {
        m_shifters = shifters;
        addRequirements(m_shifters);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_shifters.shiftLeft(Speed.kHigh);
        m_shifters.shiftRight(Speed.kHigh);
        //Robot.ledlights.greenLight();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // The solenoid setting commands should complete immediately
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }


}
