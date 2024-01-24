package com.gos.crescendo2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import com.gos.crescendo2024.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopDriveWhileStaringAtAngleCommand extends BaseTeleopSwerve {
    private final double m_angleRad;

    public TeleopDriveWhileStaringAtAngleCommand(ChassisSubsystem chassisSubsystem, CommandXboxController joystick, double angleDeg) {
        super(chassisSubsystem, joystick);
        this.m_angleRad = Math.toRadians(angleDeg);
    }

    @Override
    protected void handleJoystick(double xLeft, double yLeft, double xRight, double yRight) {
        m_subsystem.turnToAngleWithVelocity(
            yLeft,
            yRight,
            m_angleRad);
    }
}
