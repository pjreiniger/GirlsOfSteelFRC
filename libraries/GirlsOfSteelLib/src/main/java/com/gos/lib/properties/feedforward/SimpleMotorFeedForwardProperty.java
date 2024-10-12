package com.gos.lib.properties.feedforward;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleMotorFeedForwardProperty extends BaseFeedForwardProperty {
    private SimpleMotorFeedforward m_feedForward;

    public SimpleMotorFeedForwardProperty(String baseName, boolean isConstant) {
        this(baseName, isConstant, new SimpleMotorFeedforward(0, 0));
    }

    public SimpleMotorFeedForwardProperty(String baseName, boolean isConstant, SimpleMotorFeedforward feedforward) {
        super(baseName + ".smff.", isConstant);
        m_feedForward = feedforward;
    }

    public SimpleMotorFeedForwardProperty addKff(double defaultValue) {
        m_properties.add(createDoubleProperty("kff", defaultValue,
            (v) -> m_feedForward = new SimpleMotorFeedforward(m_feedForward.getKs(), v, m_feedForward.getKa())));
        return this;
    }

    public SimpleMotorFeedForwardProperty addKs(double defaultValue) {
        m_properties.add(createDoubleProperty("ks", defaultValue,
            (v) -> m_feedForward = new SimpleMotorFeedforward(v, m_feedForward.getKv(), m_feedForward.getKa())));
        return this;
    }


    public SimpleMotorFeedForwardProperty addKa(double defaultValue) {
        m_properties.add(createDoubleProperty("ka", defaultValue,
            (v) -> m_feedForward = new SimpleMotorFeedforward(m_feedForward.getKs(), m_feedForward.getKv(), v)));
        return this;
    }

    public double calculate(double velocity) {
        return m_feedForward.calculate(velocity, 0);
    }

    public double calculate(double velocity, double acceleration) {
        return m_feedForward.calculate(velocity, acceleration);
    }

    public double getKs() {
        return m_feedForward.getKs();
    }

    public double getKFf() {
        return m_feedForward.getKv();
    }

    public double getKa() {
        return m_feedForward.getKa();
    }

}
