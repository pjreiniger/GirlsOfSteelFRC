// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3504.BlueTeamEOPP;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveSystemleftTalon;
    public static SpeedController driveSystemrightTalon;
    public static RobotDrive driveSystemRobotDrive21;
    public static SpeedController shootershooterTalon;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSystemleftTalon = new Talon(1, 1);
    LiveWindow.addActuator("DriveSystem", "leftTalon", (Talon) driveSystemleftTalon);

        driveSystemrightTalon = new Talon(1, 2);
    LiveWindow.addActuator("DriveSystem", "rightTalon", (Talon) driveSystemrightTalon);

        driveSystemRobotDrive21 = new RobotDrive(driveSystemleftTalon, driveSystemrightTalon);

        driveSystemRobotDrive21.setSafetyEnabled(true);
        driveSystemRobotDrive21.setExpiration(0.1);
        driveSystemRobotDrive21.setSensitivity(0.5);
        driveSystemRobotDrive21.setMaxOutput(1.0);


        shootershooterTalon = new Talon(1, 3);
    LiveWindow.addActuator("Shooter", "shooterTalon", (Talon) shootershooterTalon);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}