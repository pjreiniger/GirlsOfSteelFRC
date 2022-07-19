package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.codelabs.pid.SmartDashboardNames;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PunchSubsystem extends SubsystemBase {

    private final Solenoid m_punchSolenoid;

    public PunchSubsystem() {
        m_punchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_PUNCH);
    }

    @Override
    public void periodic() {

    }

    public boolean isExtended() {
        return m_punchSolenoid.get();
    }

    public void extend() {
        m_punchSolenoid.set(true);
    }

    public void retract() {
        m_punchSolenoid.set(false);
    }
}
