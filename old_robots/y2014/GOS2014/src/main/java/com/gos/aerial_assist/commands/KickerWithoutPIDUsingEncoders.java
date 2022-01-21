/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.gos.aerial_assist.commands;

import com.gos.aerial_assist.subsystems.Kicker;

/**
 * @author < ( *0* ) >
 */
public class KickerWithoutPIDUsingEncoders extends CommandBase {

    private static final double m_loadingEncoderPosition = .45; // this value is "about"; currentEncoderValue % 360

    private final Kicker m_kicker;

    //0 = loading; 1 = shooting
    @SuppressWarnings("PMD.UnusedFormalParameter")
    public KickerWithoutPIDUsingEncoders(Kicker kicker, int loadingOrShooting) {
        m_kicker = kicker;
        requires(m_kicker);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        if (m_kicker.getEncoder() % 360 < m_loadingEncoderPosition) {
            m_kicker.setJag(1.0);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        m_kicker.stopJag();
    }

    @Override
    protected void interrupted() {
        end();
    }

}