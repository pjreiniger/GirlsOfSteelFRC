package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ElevatorSubsystem extends SubsystemBase {
    public static final GosDoubleProperty GRAVITY_COMPENSATION = new GosDoubleProperty(false, "Elevator.GravityCompensationSpeed", 0);

    public static final double ALLOWABLE_POSITION_ERROR = .25;

    public enum Positions {
        LOW(Units.inchesToMeters(10)),
        MID(Units.inchesToMeters(35)),
        HIGH(Units.inchesToMeters(45));

        public final double m_heightMeters;

        Positions(double heightMeters) {
            m_heightMeters = heightMeters;
        }
    }


    private final SimableCANSparkMax m_liftMotor;
    private final RelativeEncoder m_liftEncoder;
    private final DigitalInput m_lowerLimitSwitch;
    private final DigitalInput m_upperLimitSwitch;
    private final SparkPIDController m_pidController;
    private final PidProperty m_pidProperty;
    private double m_desiredHeight;

    private ISimWrapper m_elevatorSim;

    public ElevatorSubsystem() {
        m_liftMotor = new SimableCANSparkMax(Constants.CAN_LIFT_MOTOR, MotorType.kBrushless);
        m_liftEncoder = m_liftMotor.getEncoder();
        m_pidController = m_liftMotor.getPIDController();

        m_lowerLimitSwitch = new DigitalInput(Constants.DIO_LIFT_LOWER_LIMIT);
        m_upperLimitSwitch = new DigitalInput(Constants.DIO_LIFT_UPPER_LIMIT);

        m_pidProperty = new RevPidPropertyBuilder("Elevator", false, m_pidController, 0)
                .addP(0)
                .addFF(0)
                .addMaxAcceleration(0.1)
                .addMaxVelocity(0.1)
                .build();

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSimWrapper(Constants.ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_liftMotor),
                    RevEncoderSimWrapper.create(m_liftMotor));
        }
    }

    @Override
    public void periodic() {
        m_pidProperty.updateIfChanged();
    }

    public boolean goToPosition(Positions position) {
        return goToPosition(position.m_heightMeters);
    }

    public boolean goToPosition(double position) {
        m_desiredHeight = position;
        m_pidController.setReference(position, ControlType.kSmartMotion, 0, GRAVITY_COMPENSATION.getValue(), ArbFFUnits.kPercentOut);
        return false;
    }

    public boolean isAtLowerLimit() {
        return m_lowerLimitSwitch.get();
    }

    public boolean isAtUpperLimit() {
        return m_upperLimitSwitch.get();
    }

    public void setSpeed(double speed) {
        m_desiredHeight = -999;
        m_liftMotor.set(speed);
    }

    public double getHeightMeters() {
        return m_liftEncoder.getPosition();
    }

    public double getHeightInches() {
        return Units.metersToInches(getHeightMeters());
    }

    public double getVelocityMps() {
        return m_liftEncoder.getVelocity();
    }

    public double getVelocityInchesPerSec() {
        return Units.metersToInches(getVelocityMps());
    }

    public double getDesiredHeightMeters() {
        return m_desiredHeight;
    }

    public double getDesiredHeightInches() {
        return Units.metersToInches(getDesiredHeightMeters());
    }

    public double getMotorSpeed() {
        return m_liftMotor.getAppliedOutput();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.update();
    }
}
