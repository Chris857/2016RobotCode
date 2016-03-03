package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.Joystick;

public class getGamePadButton {

	
	private Joystick joystick;
	private double deadbandLeft, deadbandRight, deadbandTrigger; //create deadband class objects, left right and trigger
	
	public getGamePadButton(int port, double dL, double dR, double dT) //constructor with port number: deadband left, right and trigger
	{
		joystick = new Joystick(port);	 
		
		deadbandLeft = dL;
		deadbandRight = dR;
		deadbandTrigger = dT;
	}
	 
	private double limit(double value, double deadband)  //find limit to determine if movement is necessary 
	{  
		if(Math.abs(value)-deadband<0) return 0; 
		else return value;
	}
	
	
    public boolean getButtonA(){return joystick.getRawButton(1);}
	public boolean getButtonB(){return joystick.getRawButton(2);}
	public boolean getButtonX(){return joystick.getRawButton(3);}
	public boolean getButtonY(){return joystick.getRawButton(4);}
	
	public boolean getLeftBumper(){return joystick.getRawButton(5);}
	public boolean getRightBumper(){return joystick.getRawButton(6);}
	
	public boolean getBackButton(){return joystick.getRawButton(7);}
	public boolean getStartButton(){return joystick.getRawButton(8);}
	public boolean getLeftStickDown(){return joystick.getRawButton(9);}
	public boolean getRightStickDown(){return joystick.getRawButton(10);}
	
	public double getLeftAxisX(){return getLeftAxisX(deadbandLeft);}
	public double getLeftAxisX(double deadband){return limit(joystick.getRawAxis(0),deadband);}
	public double getRightAxisX(){return getRightAxisX(deadbandRight);}
	public double getRightAxisX(double deadband){return limit(joystick.getRawAxis(4),deadband);}
	public double getLeftAxisY(){return getLeftAxisY(deadbandLeft);}
	public double getLeftAxisY(double deadband){return limit(joystick.getRawAxis(1),deadband);}
	public double getRightAxisY(){return getRightAxisY(deadbandRight);}
	public double getRightAxisY(double deadband){return limit(joystick.getRawAxis(5),deadband);}
	public double getRightTrigger(){return getRightTrigger(deadbandTrigger);}
	public double getRightTrigger(double deadband){return limit(joystick.getRawAxis(3),deadband);}
	public double getLeftTrigger(){return getLeftTrigger(deadbandTrigger);}
	public double getLeftTrigger(double deadband){return limit(joystick.getRawAxis(2),deadband);}
	
	
}
