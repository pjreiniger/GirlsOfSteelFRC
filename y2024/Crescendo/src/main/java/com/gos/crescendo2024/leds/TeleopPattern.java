package com.gos.crescendo2024.leds;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import com.gos.crescendo2024.subsystems.ChassisSubsystem;
import com.gos.crescendo2024.subsystems.IntakeSubsystem;
import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import com.gos.lib.led.LEDBoolean;
import com.gos.lib.led.LEDPolkaDots;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class TeleopPattern {
    // Subsystems
    private final IntakeSubsystem m_intake;
    private final ArmPivotSubsystem m_pivot;
    private final ChassisSubsystem m_chassis;
    private final ShooterSubsystem m_shooter;

    // Patterns
    private final HasGamePiecePattern m_hasGamePiecePattern;
    private final LEDBoolean m_isArmAngleGoodPattern;
    private final LEDBoolean m_isShooterRpmGoodPattern;
    private final LEDBoolean m_isChassisAimedPattern;

    public TeleopPattern(AddressableLEDBuffer ledBuffer, IntakeSubsystem intake, ArmPivotSubsystem pivot, ChassisSubsystem chassis, ShooterSubsystem shooter) {
        m_intake = intake;
        m_pivot = pivot;
        m_chassis = chassis;
        m_shooter = shooter;

        // Patterns
        m_hasGamePiecePattern = new HasGamePiecePattern(new LEDPolkaDots(ledBuffer, 0, ledBuffer.getLength()), 2);

        int THIRD_OF_LIGHTS = ledBuffer.getLength() / 3;
        m_isArmAngleGoodPattern = new LEDBoolean(ledBuffer, 0 * THIRD_OF_LIGHTS, 0 * THIRD_OF_LIGHTS + ledBuffer.getLength() / 3, Color.kGreen, Color.kRed);
        m_isShooterRpmGoodPattern = new LEDBoolean(ledBuffer, 1 * THIRD_OF_LIGHTS, 1 * THIRD_OF_LIGHTS + ledBuffer.getLength() / 3, Color.kGreen, Color.kRed);
        m_isChassisAimedPattern = new LEDBoolean(ledBuffer, 2 * THIRD_OF_LIGHTS, 2 * THIRD_OF_LIGHTS + ledBuffer.getLength() / 3, Color.kGreen, Color.kRed);
    }

    public void update() {
        m_hasGamePiecePattern.setPiece(m_intake.hasGamePiece());

        if (m_hasGamePiecePattern.isPatternActive()) {
            m_hasGamePiecePattern.writeLeds();
        } else {
            System.out.println(m_pivot.isArmAtGoal() + ", " + m_shooter.isShooterAtGoal() + ", " + m_chassis.isAngleAtGoal());
            m_isArmAngleGoodPattern.setStateAndWrite(m_pivot.isArmAtGoal());
            m_isShooterRpmGoodPattern.setStateAndWrite(m_shooter.isShooterAtGoal());
            m_isChassisAimedPattern.setStateAndWrite(m_chassis.isAngleAtGoal());
        }
    }
}
