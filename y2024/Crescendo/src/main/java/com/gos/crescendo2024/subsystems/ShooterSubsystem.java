package com.gos.crescendo2024.subsystems;

import com.gos.crescendo2024.Constants;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkFlex;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ShooterSubsystem extends SubsystemBase {

    public static final GosDoubleProperty TUNABLE_SHOOTER_RPM = new GosDoubleProperty(false, "ShooterTunableRpm", 5500);
    public static final GosDoubleProperty SPEAKER_SHOT_SHOOTER_RPM = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "ShooterSpeakerShotRpm", 5000);
    public static final GosDoubleProperty AMP_SHOT_SHOOTER_RPM = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "ShooterAmpShotRpm", 800);
    private static final GosDoubleProperty SHOOTER_SPEED = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "ShooterSpeed", 0.5);
    public static final GosDoubleProperty SHOOT_NOTE_TO_ALLIANCE_RPM = new GosDoubleProperty(false, "Feeding: Shoot note to alliance with rpm", 3000);
    private static final double ALLOWABLE_ERROR = 125;


    private final SimableCANSparkFlex m_shooterMotorLeader;
    private final SimableCANSparkFlex m_shooterMotorFollower;
    private final SparkMaxAlerts m_shooterMotorErrorAlerts;
    private final SparkMaxAlerts m_shooterFollowerErrorAlerts;
    private final RelativeEncoder m_shooterEncoder;
    private final SparkPIDController m_pidController;
    private final PidProperty m_pidProperties;
    private final LoggingUtil m_networkTableEntries;
    private ISimWrapper m_shooterSimulator;
    private double m_shooterGoalRPM;
    private final DigitalInput m_photoelectricSensor;

    public ShooterSubsystem() {
        m_shooterMotorLeader = new SimableCANSparkFlex(Constants.SHOOTER_MOTOR_LEADER, MotorType.kBrushless);
        m_shooterMotorLeader.restoreFactoryDefaults();
        m_shooterMotorLeader.setInverted(true);
        m_shooterEncoder = m_shooterMotorLeader.getEncoder();
        m_pidController = m_shooterMotorLeader.getPIDController();
        m_pidController.setFeedbackDevice(m_shooterEncoder);
        m_pidProperties = new RevPidPropertyBuilder("Shooter", Constants.DEFAULT_CONSTANT_PROPERTIES, m_pidController, 0)
            .addP(1.5e-4)
            .addI(0.0)
            .addD(0.0)
            .addFF(0.000185)
            .build();

        m_shooterMotorLeader.setIdleMode(IdleMode.kCoast);
        m_shooterMotorLeader.setSmartCurrentLimit(60);
        m_shooterMotorLeader.enableVoltageCompensation(10);
        m_shooterMotorLeader.burnFlash();

        m_shooterMotorFollower = new SimableCANSparkFlex(Constants.SHOOTER_MOTOR_FOLLOWER, MotorType.kBrushless);
        m_shooterMotorFollower.restoreFactoryDefaults();
        m_shooterMotorFollower.setIdleMode(IdleMode.kCoast);
        m_shooterMotorFollower.setSmartCurrentLimit(60);
        m_shooterMotorFollower.follow(m_shooterMotorLeader, true);
        m_shooterMotorFollower.burnFlash();

        m_shooterMotorErrorAlerts = new SparkMaxAlerts(m_shooterMotorLeader, "shooter motor");
        m_shooterFollowerErrorAlerts = new SparkMaxAlerts(m_shooterMotorFollower, "shooter follower");

        m_photoelectricSensor = new DigitalInput(Constants.SHOOTER_SENSOR);

        m_networkTableEntries = new LoggingUtil("Shooter Subsystem");
        m_networkTableEntries.addDouble("Current Amps", m_shooterMotorLeader::getOutputCurrent);
        m_networkTableEntries.addDouble("Output", m_shooterMotorLeader::getAppliedOutput);
        m_networkTableEntries.addDouble("Velocity (RPM)", m_shooterEncoder::getVelocity);
        m_networkTableEntries.addBoolean("Shooter At Goal", this::isShooterAtGoal);
        m_networkTableEntries.addBoolean("Is Piece in Shooter", this::isPieceInShooter);

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(2);
            FlywheelSim shooterFlywheelSim = new FlywheelSim(gearbox, 1.0, 0.01);
            this.m_shooterSimulator = new FlywheelSimWrapper(
                shooterFlywheelSim,
                new RevMotorControllerSimWrapper(this.m_shooterMotorLeader),
                RevEncoderSimWrapper.create(this.m_shooterMotorLeader));
        }

    }

    @Override
    public void periodic() {
        m_shooterMotorErrorAlerts.checkAlerts();
        m_shooterFollowerErrorAlerts.checkAlerts();
        m_networkTableEntries.updateLogs();
        m_pidProperties.updateIfChanged();

    }

    @Override
    public void simulationPeriodic() {
        this.m_shooterSimulator.update();
    }

    public void tuneShootPercentage() {
        m_shooterMotorLeader.set(SHOOTER_SPEED.getValue());
    }

    public void stopShooter() {
        m_shooterMotorLeader.set(0);
    }

    public void setPidRpm(double rpm) {
        this.m_pidController.setReference(rpm, ControlType.kVelocity);
        m_shooterGoalRPM = rpm;
    }

    public void setVoltage(double outputVolts) {
        m_shooterMotorLeader.setVoltage(outputVolts);
    }

    public double getRPM() {
        return m_shooterEncoder.getVelocity();
    }

    public double getShooterMotorPercentage() {
        return m_shooterMotorLeader.getAppliedOutput();
    }

    public double getVoltage() {
        if (RobotBase.isReal()) {
            return m_shooterMotorLeader.getBusVoltage();
        }
        return m_shooterMotorLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
    }

    public double getEncoderPos() {
        return m_shooterEncoder.getPosition();
    }

    public boolean isShooterAtGoal() {
        double error = m_shooterGoalRPM - getRPM();
        return Math.abs(error) < ALLOWABLE_ERROR;
    }

    public boolean isPieceInShooter() {
        return !m_photoelectricSensor.get();
    }

    public void clearStickyFaults() {
        m_shooterMotorLeader.clearFaults();
        m_shooterMotorFollower.clearFaults();
    }

    /////////////////////////////////////
    // Command Factories
    /////////////////////////////////////
    public Command createTunePercentShootCommand() {
        return this.runEnd(this::tuneShootPercentage, this::stopShooter).withName("TuneShooterPercentage");
    }

    public Command createSetRPMCommand(double rpm) {
        return this.runEnd(() -> this.setPidRpm(rpm), this::stopShooter).withName("set shooter rpm " + rpm);
    }

    public Command createRunTunableRpmCommand() {
        return this.runEnd(() -> this.setPidRpm(TUNABLE_SHOOTER_RPM.getValue()), this::stopShooter).withName("set shooter to tunable rpm");
    }

    public Command createRunSpeakerShotRPMCommand() {
        return this.runEnd(() -> this.setPidRpm(SPEAKER_SHOT_SHOOTER_RPM.getValue()), this::stopShooter).withName("set shooter to speaker shot rpm");
    }

    public Command createRunAmpShotRPMCommand() {
        return this.runEnd(() -> this.setPidRpm(AMP_SHOT_SHOOTER_RPM.getValue()), this::stopShooter).withName("set shooter to amp shot rpm");
    }

    public Command createStopShooterCommand() {
        return this.run(this::stopShooter).withName("stop shooter");
    }

    public Command createShootNoteToAllianceRPMCommand() {
        return this.runEnd(() -> this.setPidRpm(SHOOT_NOTE_TO_ALLIANCE_RPM.getValue()), this::stopShooter).withName("Shoot note to alliance with rpm");
    }
}
