package com.gos.codelabs.basic_simulator.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SimableCANSparkMax;
import com.gos.codelabs.basic_simulator.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.module_wrappers.wpi.ADXRS450GyroWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

public class ChassisSubsystem extends SubsystemBase implements AutoCloseable {

    private final SimableCANSparkMax m_leftDriveA;
    private final SimableCANSparkMax m_leftDriveB;
    private final SimableCANSparkMax m_rightDriveA;
    private final SimableCANSparkMax m_rightDriveB;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final DifferentialDrive m_differentialDrive;
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_field;

    private final ADXRS450_Gyro m_gyro;

    private DifferentialDrivetrainSimWrapper m_simulator;

    public ChassisSubsystem() {

        m_leftDriveA = new SimableCANSparkMax(Constants.CAN_CHASSIS_LEFT_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftDriveB = new SimableCANSparkMax(Constants.CAN_CHASSIS_LEFT_B, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftDriveB.follow(m_leftDriveA);

        m_rightDriveA = new SimableCANSparkMax(Constants.CAN_CHASSIS_RIGHT_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_rightDriveB = new SimableCANSparkMax(Constants.CAN_CHASSIS_RIGHT_B, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_rightDriveB.follow(m_rightDriveA);
        m_rightDriveA.setInverted(true);

        m_leftEncoder = m_leftDriveA.getEncoder();
        m_rightEncoder = m_rightDriveA.getEncoder();

        m_gyro = new ADXRS450_Gyro();

        m_differentialDrive = new DifferentialDrive(m_leftDriveA, m_rightDriveA);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d());
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        if (RobotBase.isSimulation()) {
            m_simulator = new DifferentialDrivetrainSimWrapper(
                    Constants.DrivetrainConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_leftDriveA),
                    new RevMotorControllerSimWrapper(m_rightDriveA),
                    RevEncoderSimWrapper.create(m_leftDriveA),
                    RevEncoderSimWrapper.create(m_rightDriveA),
                    new ADXRS450GyroWrapper(m_gyro));
            m_simulator.setRightInverted(false);

            m_differentialDrive.setSafetyEnabled(false);
        }
    }

    @Override
    public void close() {
        m_leftDriveA.close();
        m_leftDriveB.close();
        m_rightDriveA.close();
        m_rightDriveB.close();
        m_differentialDrive.close();
        m_gyro.close();
    }

    public void arcadeDrive(double speed, double steer) {
        // TODO implement
    }

    public void stop() {
        // TODO implement
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }

    public double getHeading() {
        // TODO implement
        return 0;
    }

    public double getLeftDistance() {
        // TODO implement
        return 0;
    }

    public double getRightDistance() {
        // TODO implement
        return 0;
    }

    public double getAverageDistance() {
        // TODO implement
        return 0;
    }
}
