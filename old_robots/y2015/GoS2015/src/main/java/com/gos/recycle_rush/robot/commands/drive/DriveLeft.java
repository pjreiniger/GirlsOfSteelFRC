package com.gos.recycle_rush.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.recycle_rush.robot.OI;
import com.gos.recycle_rush.robot.subsystems.Chassis;

/*
 *
 */
public class DriveLeft extends CommandBase {
    private final Joystick m_joystick;
    private final Chassis m_chassis;

    public DriveLeft(OI oi, Chassis chassis) {
        m_chassis = chassis;
        m_joystick = oi.getChassisJoystick();
        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_chassis.driveLeft(m_joystick);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.stop();
    }



}
