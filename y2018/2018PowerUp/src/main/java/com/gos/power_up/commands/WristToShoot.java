package com.gos.power_up.commands;

import com.gos.power_up.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class WristToShoot extends CommandBase {
    private final Wrist m_wrist;

    public WristToShoot(Wrist wrist) {
        m_wrist = wrist;
        addRequirements(m_wrist);
    }


    @Override
    public void initialize() {
        m_wrist.setGoalWristPosition(Wrist.WRIST_SHOOT);
        System.out.println("WristToShoot initialized");
    }


    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {
    }



}
