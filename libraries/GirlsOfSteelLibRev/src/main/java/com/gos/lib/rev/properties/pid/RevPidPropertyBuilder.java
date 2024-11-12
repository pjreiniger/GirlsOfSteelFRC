package com.gos.lib.rev.properties.pid;

import com.gos.lib.properties.pid.IPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class RevPidPropertyBuilder extends PidProperty.Builder implements IPidPropertyBuilder {
    private final SparkBase m_motor;
    private final SparkBaseConfig m_config;
    private final ClosedLoopConfig.ClosedLoopSlot m_slot;

    public RevPidPropertyBuilder(String baseName, boolean isConstant, SparkMax motor, ClosedLoopConfig.ClosedLoopSlot slot) {
        this(baseName, isConstant, motor, new SparkMaxConfig(), slot);
    }

    public RevPidPropertyBuilder(String baseName, boolean isConstant, SparkFlex motor, ClosedLoopConfig.ClosedLoopSlot slot) {
        this(baseName, isConstant, motor, new SparkMaxConfig(), slot);
    }

    public RevPidPropertyBuilder(String baseName, boolean isConstant, SparkBase motor, SparkBaseConfig config, ClosedLoopConfig.ClosedLoopSlot slot) {
        super(baseName, isConstant);
        m_motor = motor;
        m_config = config;
        m_slot = slot;
    }

    @Override
    public IPidPropertyBuilder addP(double defaultValue) {
        addP(defaultValue, (double gain) -> {
            m_config.closedLoop.p(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addI(double defaultValue) {
        addI(defaultValue, (double gain) -> {
            m_config.closedLoop.i(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addD(double defaultValue) {
        addD(defaultValue, (double gain) -> {
            m_config.closedLoop.d(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addFF(double defaultValue) {
        addFF(defaultValue, (double gain) -> {
            m_config.closedLoop.velocityFF(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxVelocity(double defaultValue) {
        addMaxVelocity(defaultValue, (double gain) -> {
            m_config.closedLoop.maxMotion.maxVelocity(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxAcceleration(double defaultValue) {
        addMaxAcceleration(defaultValue, (double gain) -> {
            m_config.closedLoop.maxMotion.maxAcceleration(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    private void configureMotor() {
        m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
