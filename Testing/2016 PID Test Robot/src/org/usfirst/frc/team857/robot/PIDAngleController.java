package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class PIDAngleController extends PIDController {
	
	private double m_F;
	
	//Kf = mass * gravity * distance * resistance / K_t
	
	public PIDAngleController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output) {
		super(Kp, Ki, Kd, Kf, source, output);
		m_F = Kf;
	}
	
	public PIDAngleController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output, double period) {
		super(Kp, Ki, Kd, Kf, source, output, period);
		m_F = Kf;
	}
	
	protected double calculateFeedForward() {
	        return  m_F * Math.cos(getSetpoint()*180.0/Math.PI);
	}
}
