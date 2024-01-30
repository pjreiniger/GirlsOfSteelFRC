package com.gos.crescendo2024.commands;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import com.gos.crescendo2024.subsystems.ChassisSubsystem;
import com.gos.crescendo2024.subsystems.IntakeSubsystem;
import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CombinedCommands {


    public static Command intakePieceCommand(ArmPivotSubsystem armPivot, IntakeSubsystem intake) {
        return armPivot.createMoveArmToGroundIntakeAngleCommand()
            .alongWith(intake.createMoveIntakeInCommand())
            .until(intake::hasGamePiece)
            .withName("Intake Piece");
    }

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

    public static Command speakerAimAndShoot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter, ChassisSubsystem chassis, IntakeSubsystem intake) {
        return new SpeakerAimAndShootCommand(armPivot, chassis, intake, shooter)
            .withName("Auto shoot into speaker");
    }

    public static Command ampShooterCommand(ArmPivotSubsystem armPivot, IntakeSubsystem intake) {
        return armPivot.createMoveArmToAmpAngleCommand()
            .until(armPivot::isArmAtGoal)
            .andThen(intake.createMoveIntakeOutCommand())
            .withName("Auto shoot into amp");
    }
}

