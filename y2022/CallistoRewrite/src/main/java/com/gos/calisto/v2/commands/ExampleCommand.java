// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gos.calisto.v2.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.calisto.v2.subsystems.ExampleSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "FieldCanBeLocal"})
    private final ExampleSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ExampleCommand(ExampleSubsystem subsystem) {
        this.m_subsystem = subsystem;
        addRequirements(subsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}