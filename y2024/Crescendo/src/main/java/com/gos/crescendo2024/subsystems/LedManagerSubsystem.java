package com.gos.crescendo2024.subsystems;

import com.gos.crescendo2024.leds.HasGamePiecePattern;
import com.gos.lib.led.LEDPolkaDots;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedManagerSubsystem extends SubsystemBase {
    private static final int MAX_INDEX_LED = 30;

    // Led core
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLED m_led;

    // Subsystems
    private final IntakeSubsystem m_intake;

    // Patterns
    private final HasGamePiecePattern m_hasGamePiecePattern;

    public LedManagerSubsystem(IntakeSubsystem intake) {

        m_intake = intake;

        // Setup LEDS
        m_buffer = new AddressableLEDBuffer(MAX_INDEX_LED);
        m_led = new AddressableLED(0);

        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();

        // Patterns
        m_hasGamePiecePattern = new HasGamePiecePattern(new LEDPolkaDots(m_buffer, 0, MAX_INDEX_LED), 2);
    }

    @Override
    public void periodic() {
        clear();

        m_hasGamePiecePattern.setPiece(m_intake.hasGamePiece());
        m_hasGamePiecePattern.writeLeds();

        // Write LEDs
        m_led.setData(m_buffer);
    }

    private void clear() {
        for (int i = 0; i < MAX_INDEX_LED; i++) {
            m_buffer.setRGB(i, 0, 0, 0);
        }
    }
}
