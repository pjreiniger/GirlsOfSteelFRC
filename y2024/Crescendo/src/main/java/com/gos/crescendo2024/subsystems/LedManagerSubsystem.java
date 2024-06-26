package com.gos.crescendo2024.subsystems;


import com.gos.crescendo2024.Constants;
import com.gos.crescendo2024.DriverStationLedDriver;
import com.gos.crescendo2024.auton.Autos;
import com.gos.crescendo2024.led_patterns.DisabledPatterns;
import com.gos.crescendo2024.led_patterns.DriverStationPatterns;
import com.gos.crescendo2024.led_patterns.EnabledPatterns;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedManagerSubsystem extends SubsystemBase {

    private static final int MAX_INDEX_LED = 70;

    // Led core
    protected final AddressableLEDBuffer m_buffer;
    protected final AddressableLED m_led;

    private final EnabledPatterns m_enabledPatterns;
    private final DisabledPatterns m_disabledPatterns;
    private final DriverStationPatterns m_driverStationPatterns;


    public LedManagerSubsystem(DriverStationLedDriver driverStationLedDriver, IntakeSubsystem intakeSubsystem, Autos autoModeFactory, ChassisSubsystem chassis, ArmPivotSubsystem arm, ShooterSubsystem shooter) {
        // Setup LED's
        m_buffer = new AddressableLEDBuffer(MAX_INDEX_LED);
        m_led = new AddressableLED(Constants.LED_PORT);
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();

        // Patterns
        m_enabledPatterns = new EnabledPatterns(m_buffer, MAX_INDEX_LED, intakeSubsystem, chassis, arm, shooter);
        m_disabledPatterns = new DisabledPatterns(m_buffer, MAX_INDEX_LED, autoModeFactory, chassis);
        m_driverStationPatterns = new DriverStationPatterns(driverStationLedDriver, intakeSubsystem, chassis, arm);
    }


    @Override
    public void periodic() {
        clear();

        if (DriverStation.isEnabled()) {
            m_enabledPatterns.writeLED();
        } else {
            m_disabledPatterns.writeAutoPattern();
        }

        m_driverStationPatterns.writeLeds();

        m_led.setData(m_buffer);
    }

    private void clear() {
        for (int i = 0; i < MAX_INDEX_LED; i++) {
            m_buffer.setRGB(i, 0, 0, 0);
        }
    }
}
