package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class AngleController extends PIDAngleSubsystem {
	private SpeedController _motor;
	private Encoder _encoder;
	private AnalogInput _potentiometer;
	
	public AngleController(final double p, final double i, final double d, final double f, Encoder encoder, SpeedController motor) {
		
		//Call PIDSubsystem constructor
		super("Angle", p, i, d, f);
		
		//Setup IO
		_motor = motor;
		_encoder = encoder;
		
		//Set controller settings
		setAbsoluteTolerance(0.05);
		getPIDAngleController().setContinuous(false);
		getPIDAngleController().setInputRange(0,  360);
	}
	
	public AngleController(final double p, final double i, final double d, final double f, AnalogInput potentiometer, SpeedController motor) {
		
		//Call PIDSubsystem constructor
		super("Angle", p, i, d, f);
		
		//Setup IO
		_motor = motor;
		_potentiometer = potentiometer;
		
		//Set controller settings
		setAbsoluteTolerance(0.05);
		getPIDAngleController().setContinuous(true);
		
		//TODO: Set input range here!
		getPIDAngleController().setInputRange(2.4,  2.8);
		
		//getPIDController().setOutputRange(-.2, .2);
	}
	
	//Compute angle given sensor.
	protected double returnPIDInput() {
		//TODO: Convert voltage to angle!
		if (_encoder == null) {
			return _potentiometer.getAverageVoltage();
		} else {
			//return ((_encoder.get() * 360.0 / 1024.0)+360.0) % 360;
			return _encoder.get();
		}
	}
	
	protected void usePIDOutput(double output) {
		_motor.pidWrite(output);
	}

	protected void initDefaultCommand() {
		
	}
}
