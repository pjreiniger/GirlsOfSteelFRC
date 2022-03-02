package com.gos.power_up.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.gos.power_up.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class TurnInPlace extends CommandBase {

    private static final double kP = .005;
    private static final double kI = 0;
    private static final double kD = 0;

    private boolean m_targetReached;

    private final double m_headingTarget;
    private double m_errorLast;
    private double m_iError;

    private final Chassis m_chassis;
    private final WPI_TalonSRX m_leftTalon;
    private final WPI_TalonSRX m_rightTalon;

    public TurnInPlace(Chassis chassis, double degrees) {
        m_chassis = chassis;
        m_leftTalon = m_chassis.getLeftTalon();
        m_rightTalon = m_chassis.getRightTalon();
        m_headingTarget = degrees;
        addRequirements(m_chassis);
    }


    @Override
    public void initialize() {
        System.out.println("Trying to initialize");
        m_chassis.setInverted(false);
        m_chassis.zeroSensors();
        System.out.println("Turn in place initialized Heading = " + m_headingTarget);

    }



    @Override
    public void execute() {

        double currentPos = m_chassis.getYaw();
        double error = m_headingTarget - currentPos;
        double dError = (error - m_errorLast) / .02;


        double tempError = m_iError + (error * .02);
        if (Math.abs(tempError * kI) < .5) {
            m_iError = tempError;
        }
        System.out.println("current position " + currentPos);

        m_leftTalon.set(ControlMode.PercentOutput, (kP * error) + (kD * dError) + (kI * m_iError));
        m_rightTalon.set(ControlMode.PercentOutput, (kP * error) + (kD * dError) + (kI * m_iError));

        if (error < 1 && dError < 10) {
            m_targetReached = true;
        }

        m_errorLast = error;

        //        if (headingTarget > 0) {
        //            leftTalon.set(ControlMode.Position, encoderTicks);
        //            rightTalon.set(ControlMode.Position, encoderTicks);
        //        } else {
        //            leftTalon.set(ControlMode.Position, -encoderTicks);
        //            rightTalon.set(ControlMode.Position, -encoderTicks);
        //        }
        //
        //        System.out.println("Left Error: " + (leftTalon.getSelectedSensorPosition(1) - encoderTicks));
        //        System.out.println("Right Error: " + (rightTalon.getSelectedSensorPosition(1) + encoderTicks));
        //
    }


    @Override
    public boolean isFinished() {
        return m_targetReached;
    }



    @Override
    public void end(boolean interrupted) {
        System.out.println("TurnInPlace Finished");
        m_chassis.stop();
    }
}
