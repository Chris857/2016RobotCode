package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDAngleController extends PIDController {
	
	private double m_F;
	private double _setpoint;
	private double _prevSetpoint;
	private double _time;
	private double _period;
	
	//Kf = mass * gravity * distance * resistance / K_t
	
	public PIDAngleController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output) {
		super(Kp, Ki, Kd, Kf, source, output);
		m_F = Kf;
		_period = PIDController.kDefaultPeriod;
	}
	
	public PIDAngleController(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output, double period) {
		super(Kp, Ki, Kd, Kf, source, output, period);
		m_F = Kf;
		_period = period;
	}
	
	  public synchronized void setSetpoint(double setpoint) {
		  super.setSetpoint(setpoint);
		  _time = 0;
	  }

	  protected void calculate() {
		  super.calculate();
		  _time += _period;
	  }
	  
	protected double calculateFeedForward() {
		return  m_F * Math.cos(getSetpoint()*180.0/Math.PI);
	}

	public synchronized double getSetpoint() {
		return calculateSetpoint(_prevSetpoint, _setpoint);
  }

	private synchronized double calculateSetpoint(double oldsetpoint, double setpoint) {
		double zeta = 0.9;
		double freq = 0.25;
		
		double wn = freq * 2.0 * Math.PI;
		
		double deltht = oldsetpoint - setpoint; 	//Old target - new target
		double deltime = _time;						//Elapsed time in seconds since filter start
		
		double tmp0 = Math.sqrt(1.0 - Math.pow(zeta, 2));
		double tmp1 = Math.exp(-1.0 * zeta * wn * deltime);
		double tmp2 = wn * tmp0 * deltime;
		SmartDashboard.putNumber("DEBUG oldsetpoint", oldsetpoint);
		SmartDashboard.putNumber("DEBUG setpoint", setpoint);
		SmartDashboard.putNumber("DEBUG time", _time);

		SmartDashboard.putNumber("Calculated Setpoint", setpoint + deltht * (1 - tmp1 * Math.cos(tmp2) - zeta / tmp0 * tmp1 * Math.sin(tmp2)));
		return setpoint + deltht * (1 - tmp1 * Math.cos(tmp2) - zeta / tmp0 * tmp1 * Math.sin(tmp2));
	}
	
}