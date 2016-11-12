package org.usfirst.frc.team857.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

public class YetiPIDController implements PIDInterface {
	private abstract class Filter <E> {
		private E[] value;
		
		E[] getValue(){
			return value;
		}
		
		abstract E func(E... in);
		public Filter (E... in){
			value = in;
		}
	}
	
	private class OneToOne <E> extends Filter <E>{

		@Override
		E func(E... in) {
			return in [1];
		}
		
	}
	
	private class PIDTask extends TimerTask {
		
		private YetiPIDController _controller;
		
		public PIDTask(YetiPIDController controller) {
			if (controller == null) {
				throw new NullPointerException("YetiPIDController can not be null");
			}
			_controller = controller;
	    }
		
		@Override
		public void run() {
			_controller.calculate();
		}
	}

	java.util.Timer _controlLoop;
	private Timer _setpointTimer;
	private double _P;
	private double _I;
	private double _D;
	private double _F;
	private double _period;
	private double _setpoint;
	private boolean _enabled;
	private double totalError;
	private double prevError=0;
	private PIDOutput _pidOutput;
	private PIDSource _pidSource;
	private double _result;
	
	public YetiPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source,
		      PIDOutput output, double period) {

	    _controlLoop = new java.util.Timer();
	    _setpointTimer = new Timer();
	    _setpointTimer.start();

	    _P = Kp;
	    _I = Ki;
	    _D = Kd;
	    _F = Kf;
	    
	    _period = period;
	    
	    _enabled = false;
	    
	    _pidOutput = output;
	    _pidSource = source;
	    
	    _controlLoop.schedule(new PIDTask(this), 0L, (long) (_period * 1000));
	}
	
	@Override
	public synchronized void setPID(double p, double i, double d) {
		_P = p;
		_I = i;
		_D = d;
	}
	
	public synchronized void setPID(double p, double i, double d, double f) {
		_P = p;
		_I = i;
		_D = d;
		_F = f;
	}

	//Just toggle the output for testing
	public void calculate() {
		boolean enabled;
		double curErr;
		double result = 0;
		
	    synchronized (this) {
	      enabled = this.isEnabled();
	      curErr = this.getError();
	      totalError += curErr * _period;
	      _result = (_P*curErr) + (_I * totalError) + (_D * (curErr - prevError) / _period)
	    		  + calculateFeedForward();
	      prevError = curErr;
	      result = _result;
	    }

	    if (enabled) {
	        _pidOutput.pidWrite(result);
	    }
	}
	
	public double get() {
		return _result;
	}
	
	public double calculateFeedForward() {
		return _F * _setpoint;
	}

	public synchronized double getP() {
		return _P;
	}

	public synchronized double getI() {
		return _I;
	}

	public synchronized double getD() {
		return _D;
	}
	
	public synchronized double getF() {
		return _F;
	}

	public synchronized void setSetpoint(double setpoint) {
		_setpoint = setpoint;
	}

	public synchronized double getSetpoint() {
		return _setpoint;
	}

	public synchronized double getError() {
		return getSetpoint() - _pidSource.pidGet();
	}

	public synchronized void enable() {
		_enabled = true;	
	}

	public synchronized void disable() {
		_pidOutput.pidWrite(0);
	    _enabled = false;	
	}

	public synchronized boolean isEnabled() {
		return _enabled;
	}

	public synchronized void reset() {
		prevError = 0;
		totalError = 0;
	}
	
	public synchronized double getReference() {
		return _pidSource.pidGet();
	}
}
