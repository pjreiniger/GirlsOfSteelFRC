package com.gos.crescendo2024.commands;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import com.gos.crescendo2024.subsystems.IntakeSubsystem;
import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CombinedCommandsUtil {

    public static Command createIntakeFromGroundCommand(IntakeSubsystem intake, ArmPivotSubsystem pivot) {
        return Commands.parallel(
            intake.createIntakeUntilPiece(),
            pivot.createGoToIntakeCommand()
        );
    }

    public static Command createShootAtFixedAngle(double angle, ShooterSubsystem shooter, ArmPivotSubsystem pivot) {
        return Commands.sequence(
            pivot.createMoveArmToAngle(angle),
            shooter.createTunePercentShootCommand().withTimeout(1)
        );
    }
}
