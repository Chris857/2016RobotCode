package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class wedgeControl {

	private SpeedController leftWedge;
	private SpeedController rightWedge;
	
	private double motorSpeed = .09;
	
	public wedgeControl(SpeedController leftWedge, SpeedController rightWdge) {
		
		this.leftWedge = leftWedge;
		this.rightWedge = rightWedge;
		
	}
	
	public void wedgeUp(){
		
		leftWedge.set(motorSpeed);
		rightWedge.set(motorSpeed);
		
	}
	
	public void wedgeDown(){
		
		leftWedge.set(-motorSpeed);
		rightWedge.set(-motorSpeed);
		
	}
	

}
