package com.gos.rapidreact.subsystems;


import com.gos.rapidreact.Constants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;


public class HangerSubsystem extends SubsystemBase {
    public static final double HANGER_UP_SPEED = 1.0;
    public static final double HANGER_DOWN_SPEED = -HANGER_UP_SPEED;
    private static final double GEAR = 80;

    private final SimableCANSparkMax m_leftHanger;
    private final SimableCANSparkMax m_rightHanger;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private ISimWrapper m_leftSimulator;
    private ISimWrapper m_rightSimulator;

    // Logging
    private final NetworkTableEntry m_leftHangerHeightEntry;
    private final NetworkTableEntry m_rightHangerHeightEntry;

    public HangerSubsystem() {
        m_leftHanger = new SimableCANSparkMax(Constants.HANGER_LEFT_SPARK, MotorType.kBrushless);
        m_leftHanger.restoreFactoryDefaults();
        m_rightHanger = new SimableCANSparkMax(Constants.HANGER_RIGHT_SPARK, MotorType.kBrushless);
        m_rightHanger.restoreFactoryDefaults();
        m_leftEncoder = m_leftHanger.getEncoder();
        m_rightEncoder = m_rightHanger.getEncoder();

        m_leftHanger.setIdleMode(IdleMode.kBrake);
        m_rightHanger.setIdleMode(IdleMode.kBrake);

        m_leftEncoder.setPositionConversionFactor(GEAR);
        m_rightEncoder.setPositionConversionFactor(GEAR);

        m_leftHanger.burnFlash();
        m_rightHanger.burnFlash();

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Hanger");
        m_leftHangerHeightEntry = loggingTable.getEntry("LeftHeight");
        m_rightHangerHeightEntry = loggingTable.getEntry("RightHeight");

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(2);
            ElevatorSim leftElevatorSim = new ElevatorSim(gearbox, GEAR, Units.lbsToKilograms(10), Units.inchesToMeters(2), Units.feetToMeters(0), Units.feetToMeters(4), true, 0);
            m_leftSimulator = new ElevatorSimWrapper(leftElevatorSim,
                new RevMotorControllerSimWrapper(m_leftHanger),
                RevEncoderSimWrapper.create(m_leftHanger));
            ElevatorSim rightElevatorSim = new ElevatorSim(DCMotor.getNeo550(2), GEAR, Units.lbsToKilograms(10), Units.inchesToMeters(2), Units.feetToMeters(0), Units.feetToMeters(4), true, 0);
            m_rightSimulator = new ElevatorSimWrapper(rightElevatorSim,
                new RevMotorControllerSimWrapper(m_rightHanger),
                RevEncoderSimWrapper.create(m_rightHanger));
        }
    }


    @Override
    public void periodic() {
        m_leftHangerHeightEntry.setNumber(getLeftHangerHeight());
        m_rightHangerHeightEntry.setNumber(getRightHangerHeight());
    }

    public double getLeftHangerSpeed() {
        return m_leftHanger.getAppliedOutput();
    }

    public double getLeftHangerHeight() {
        return m_leftEncoder.getPosition();
    }

    public void setLeftHangerSpeed(double speed) {
        m_leftHanger.set(speed);
    }

    public double getRightHangerSpeed() {
        return m_rightHanger.getAppliedOutput();
    }

    public double getRightHangerHeight() {
        return m_rightEncoder.getPosition();
    }

    public void setRightHangerSpeed(double speed) {
        m_rightHanger.set(speed);
    }

    @Override
    public void simulationPeriodic() {
        m_leftSimulator.update();
        m_rightSimulator.update();
    }

    public void stop() {
        m_leftHanger.set(0);
        m_rightHanger.set(0);
    }
}
