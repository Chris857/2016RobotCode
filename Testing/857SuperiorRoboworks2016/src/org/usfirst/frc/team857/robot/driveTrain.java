package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class driveTrain {

	private Counter leftEncoder; //encoder for the motors
	private Counter rightEncoder;
	private RobotDrive	robot;
	
	public driveTrain(SpeedController leftDrive, SpeedController rightDrive, Counter leftEncoder, Counter rightEncoder) // driveTrain constuctor
	{
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.robot = new RobotDrive(leftDrive, rightDrive);
		
	}
	
	
	public void drive(double left, double right){
		robot.tankDrive(left,  right);
	}
	
	public void auto1(){
		
	}
	
}
