package com.gos.lib.rev.properties.pid;

import com.gos.lib.properties.pid.IPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.SparkMaxUtil;
import com.revrobotics.SparkPIDController;

public final class RevPidPropertyBuilder extends PidProperty.Builder implements IPidPropertyBuilder {
    private final SparkPIDController m_pidController;
    private final int m_slot;

    public RevPidPropertyBuilder(String baseName, boolean isConstant, SparkPIDController pidController, int slot) {
        super(baseName, isConstant);
        m_pidController = pidController;
        m_slot = slot;
    }

    @Override
    public IPidPropertyBuilder addP(double defaultValue) {
        addP(defaultValue, (double gain) -> SparkMaxUtil.setP(m_pidController, gain, m_slot));
        return this;
    }

    @Override
    public IPidPropertyBuilder addI(double defaultValue) {
        addI(defaultValue, (double gain) -> SparkMaxUtil.setI(m_pidController, gain, m_slot));
        return this;
    }

    @Override
    public IPidPropertyBuilder addD(double defaultValue) {
        addD(defaultValue, (double gain) -> SparkMaxUtil.setD(m_pidController, gain, m_slot));
        return this;
    }

    @Override
    public IPidPropertyBuilder addFF(double defaultValue) {
        addFF(defaultValue, (double gain) -> SparkMaxUtil.setFF(m_pidController, gain, m_slot));
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxVelocity(double defaultValue) {
        addMaxVelocity(defaultValue, (double gain) -> SparkMaxUtil.setSmartMotionMaxVelocity(m_pidController, gain, m_slot));
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxAcceleration(double defaultValue) {
        addMaxAcceleration(defaultValue, (double gain) -> SparkMaxUtil.setSmartMotionMaxAccel(m_pidController, gain, m_slot));
        return this;
    }
}
