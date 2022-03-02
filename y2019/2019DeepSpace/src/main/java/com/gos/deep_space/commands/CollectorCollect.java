/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.gos.deep_space.commands;

import com.gos.deep_space.subsystems.Collector;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CollectorCollect extends CommandBase {
    private final Collector m_collector;

    public CollectorCollect(Collector collector) {
        m_collector = collector;
        addRequirements(m_collector);
    }


    @Override
    public void initialize() {
        System.out.println("init CollectorCollect");
    }


    @Override
    public void execute() {
        m_collector.collect();
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        m_collector.slowCollect();
        System.out.println("end CollectorCollect");
    }


}
