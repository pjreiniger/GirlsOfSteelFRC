/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.gos.codelabs.basic_simulator;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /////////////////////////////////
    // Sensor / Actuator ports
    /////////////////////////////////

    // PWM

    // Digital IO
    public static final int DIO_LIFT_LOWER_LIMIT = 0;
    public static final int DIO_LIFT_UPPER_LIMIT = 1;

    // Solenoids
    public static final int SOLENOID_PUNCH = 0;

    // CAN
    public static final int CAN_CHASSIS_LEFT_A = 1;
    public static final int CAN_CHASSIS_LEFT_B = 2;
    public static final int CAN_CHASSIS_RIGHT_A = 3;
    public static final int CAN_CHASSIS_RIGHT_B = 4;
    public static final int CAN_LIFT_MOTOR = 5;
    public static final int CAN_SPINNING_MOTOR = 6;

    public static final boolean SIMULATE_SENSOR_NOISE = false;

    private Constants() {

    }


    public static final class DrivetrainConstants {

        public static final DCMotor DRIVE_GEARBOX = DCMotor.getCIM(2);
        public static final double K_DRIVE_GEARING = 8;

        public static final double K_TRACK_WIDTH_METERS = 0.69;
        public static final double K_WHEEL_DIAMETER_METERS = 0.15;

        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLT_SECONDS_PER_METER = 1.98;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.2;
        public static final double KV_VOLT_SECONDS_PER_RADIAN = 2.5;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_RADIAN = 0.8;

        public static final LinearSystem<N2, N2, N2> K_DRIVETRAIN_PLANT =
                LinearSystemId.identifyDrivetrainSystem(KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER,
                        KV_VOLT_SECONDS_PER_RADIAN, KA_VOLT_SECONDS_SQUARED_PER_RADIAN);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

        public static DifferentialDrivetrainSim createSim() {
            return new DifferentialDrivetrainSim(
                    K_DRIVETRAIN_PLANT,
                    DRIVE_GEARBOX,
                    K_DRIVE_GEARING,
                    K_TRACK_WIDTH_METERS,
                    K_WHEEL_DIAMETER_METERS / 2.0,
                SIMULATE_SENSOR_NOISE ? VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) : null); // NOPMD
        }

        private DrivetrainConstants() {

        }
    }


    public static final class FlywheelSimConstants {
        public static final DCMotor K_GEARBOX = DCMotor.getVex775Pro(2);
        public static final double K_GEARING = 4;
        public static final double K_INERTIA = 0.008;

        public static FlywheelSim createSim() {
            return new FlywheelSim(FlywheelSimConstants.K_GEARBOX, FlywheelSimConstants.K_GEARING, FlywheelSimConstants.K_INERTIA,
                    SIMULATE_SENSOR_NOISE ? VecBuilder.fill(0.5) : null); // NOPMD
        }

        private FlywheelSimConstants() {

        }
    }
}
