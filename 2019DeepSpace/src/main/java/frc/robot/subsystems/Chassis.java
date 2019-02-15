package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveByJoystick;
import frc.robot.LidarLitePWM;
public class Chassis extends Subsystem {

	private WPI_TalonSRX masterLeft;
	private WPI_TalonSRX driveLeft_A;
	private WPI_TalonSRX driveLeft_B;

	private WPI_TalonSRX masterRight;
	private WPI_TalonSRX driveRight_A;
	private WPI_TalonSRX driveRight_B;
	
	private DifferentialDrive drive;

	private LidarLitePWM lidar;
	
	public double LIDAR_TOLERANCE = 1; //tune


	private double speed = 1; //tune this!!

	public Chassis () {
		masterLeft = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER_TALON);
		driveLeft_A = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FOLLOWER_TALON);

		//driveLeft_B = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_B_PORT);

		
		masterRight = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER_TALON); 
		driveRight_A = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOWER_TALON); 

		//driveRight_B = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_B_PORT); 
		
		masterLeft.setNeutralMode(NeutralMode.Brake);
		driveLeft_A.setNeutralMode(NeutralMode.Brake);
		//driveLeft_B.setNeutralMode(NeutralMode.Brake);

		masterRight.setNeutralMode(NeutralMode.Brake);
		driveRight_A.setNeutralMode(NeutralMode.Brake);
		//driveRight_B.setNeutralMode(NeutralMode.Brake);
		
		driveLeft_A.follow(masterLeft, FollowerType.PercentOutput); 
		//driveLeft_B.follow(masterLeft, FollowerType.PercentOutput);
		
		driveRight_A.follow(masterRight, FollowerType.PercentOutput);
		//driveRight_B.follow(masterRight, FollowerType.PercentOutput); 
		
		drive = new DifferentialDrive(masterLeft, masterRight);
		drive.setSafetyEnabled(true);
		drive.setExpiration(0.1);
		drive.setMaxOutput(0.8);

		lidar = new LidarLitePWM(RobotMap.LIDAR_LITE_DIO);
	}

	// Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveByJoystick()); 
	}
	
	public WPI_TalonSRX getLeftTalon(){
		return driveLeft_A;
	}

	public WPI_TalonSRX getRightTalon(){
		return driveRight_A;
	}
    
    public void driveByJoystick(double yDir, double xDir) {
    	SmartDashboard.putString("driveByJoystick?", yDir + "," + xDir); 
    	drive.arcadeDrive(yDir, xDir);
	}
	
	public void driveForward(){
		driveLeft_A.set(speed);//TODO; ADJUST SPEED
		driveRight_A.set(speed); //TODO: ADJUST SPEED
	}

	public void driveBackwards(){
		driveLeft_A.set(-speed);//TODO; ADJUST SPEED
		driveRight_A.set(-speed); //TODO: ADJUST SPEED
	}
	
	public double getLidarDistance(){
		return lidar.getDistance(); 
	  }

    public void stop() {
    	drive.stopMotor(); 
	}
}

