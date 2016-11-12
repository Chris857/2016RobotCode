package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;

public class RobotMap {
	
	//Motors
	public static VictorSP leftDriveMotor;
	public static VictorSP rightDriveMotor;
	public static Victor leftShooterMotor;
	public static Victor rightShooterMotor;
	public static Victor kickerMotor;
	public static Victor angleMotor;
	public static Victor leftWedgeMotor;
	public static Victor rightWedgeMotor;
	
	//Encoders
	public static Encoder angleEncoder;
	public static AnalogInput anglePotentiometer;
	public static Counter leftShooterSpeed;
	public static Counter rightShooterSpeed;
	
	//Switches
	public static DigitalInput angleLowerLimitSwitch;
	public static DigitalInput angleUpperLimitSwitch;
	
	//Cameras
	public static String shooterCamera;
	public static String driveCamera;
	
	//I2C
	public static LidarSubsystem lidar;
	
	public static void init() {
		
		//Motors
		leftDriveMotor = new VictorSP(0);
		rightDriveMotor = new VictorSP(1);
		leftShooterMotor = new Victor(3);
		rightShooterMotor = new Victor(4);
		kickerMotor = new Victor(2);
		angleMotor = new Victor(5);
		leftWedgeMotor = new Victor(6);
		rightWedgeMotor = new Victor(7);
		
		//Encoders
		angleEncoder = new Encoder(0,1,2);
		angleEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		anglePotentiometer = new AnalogInput(0);
		leftShooterSpeed = new Counter(3);
		rightShooterSpeed = new Counter(4);
		leftShooterSpeed.setPIDSourceType(PIDSourceType.kRate);
		rightShooterSpeed.setPIDSourceType(PIDSourceType.kRate);
		
		//Switches
		angleLowerLimitSwitch = new DigitalInput(5);
		angleUpperLimitSwitch = new DigitalInput(6);
		
		//Cameras
		shooterCamera = "cam0";
		driveCamera = "cam1";
		
		//I2C
		lidar = new LidarSubsystem(I2C.Port.kMXP);
		
	}
}
