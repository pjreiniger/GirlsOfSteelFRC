package com.gos.crescendo2024.subsystems.sysid;

import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSysId {
    private static final Angle ANGLE_UNIT = Rotations;
    private static final Velocity<Angle> VELOCITY_UNIT = RotationsPerSecond;

    private final SysIdRoutine m_sysIdRoutine;
    private final ShooterSubsystem m_shooter;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_angle = mutable(ANGLE_UNIT.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(VELOCITY_UNIT.of(0));

    public ShooterSysId(ShooterSubsystem shooter) {
        m_shooter = shooter;

        m_sysIdRoutine =  new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1.0)),
                Units.Volts.of(3.0),
                Units.Seconds.of(10)
            ),
            new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> {
                m_shooter.setVoltage(voltage.in(Volts));
            }, this::logSysId, m_shooter)
        );
    }

    private void logSysId(SysIdRoutineLog log) {
        log.motor("shooter")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_shooter.getMotorVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(m_shooter.getDistance(), ANGLE_UNIT))
            .angularVelocity(
                m_velocity.mut_replace(m_shooter.getRPM(), VELOCITY_UNIT));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}
