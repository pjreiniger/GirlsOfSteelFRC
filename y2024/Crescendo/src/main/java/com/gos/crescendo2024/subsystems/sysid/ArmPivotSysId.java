package com.gos.crescendo2024.subsystems.sysid;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
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
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ArmPivotSysId {
    private final SysIdRoutine m_sysIdRoutine;
    private final ArmPivotSubsystem m_armPivot;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

    public ArmPivotSysId(ArmPivotSubsystem armPivot) {
        m_armPivot = armPivot;

        m_sysIdRoutine =  new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Seconds.of(1.0)),
                Units.Volts.of(3.0),
                Units.Seconds.of(10)
            ),
            new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> {
                m_armPivot.setVoltage(voltage.in(Volts));
            }, this::logSysId, armPivot)
        );
    }

    private void logSysId(SysIdRoutineLog log) {
        log.motor("arm")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_armPivot.getMotorVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(m_armPivot.getAngle(), Degrees))
            .angularVelocity(
                m_velocity.mut_replace(m_armPivot.getVelocity(), DegreesPerSecond));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
