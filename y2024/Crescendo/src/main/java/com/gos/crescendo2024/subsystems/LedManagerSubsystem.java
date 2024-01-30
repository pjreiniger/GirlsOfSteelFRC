package com.gos.crescendo2024.subsystems;

import com.gos.crescendo2024.leds.TeleopPattern;
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
    private final TeleopPattern m_teleopPattern;

    public LedManagerSubsystem(IntakeSubsystem intake, ArmPivotSubsystem pivot, ChassisSubsystem chassis, ShooterSubsystem shooter) {

        m_intake = intake;

        // Setup LEDS
        m_buffer = new AddressableLEDBuffer(MAX_INDEX_LED);
        m_led = new AddressableLED(0);

        m_led.setLength(m_buffer.getLength());

        m_led.setData(m_buffer);
        m_led.start();

        // Patterns
        m_teleopPattern = new TeleopPattern(m_buffer, intake, pivot, chassis, shooter);
    }

    @Override
    public void periodic() {
        clear();

        m_teleopPattern.update();

        // Write LEDs
        m_led.setData(m_buffer);
    }

    private void clear() {
        for (int i = 0; i < MAX_INDEX_LED; i++) {
            m_buffer.setRGB(i, 0, 0, 0);
        }
    }
}
