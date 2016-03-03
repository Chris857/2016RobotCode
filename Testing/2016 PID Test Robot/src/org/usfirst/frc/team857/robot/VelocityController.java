package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class VelocityController extends PIDSubsystem {

	private SpeedController _motor;
	private Counter _encoder;
	
	public VelocityController(String name, double p, double i, double d,
			double f, Counter encoder, SpeedController motor) {
		super(name, p, i, d, f);
		//Setup IO
		_motor = motor;
		_encoder = encoder;
		
		//Set controller settings
		setAbsoluteTolerance(0.05);
		getPIDController().setContinuous(true);
		getPIDController().setInputRange(0,  5500);
	}

	protected double returnPIDInput() {
		return 60.0 / _encoder.getPeriod();
	}

	protected void usePIDOutput(double output) {
		_motor.pidWrite(output);
	}

	protected void initDefaultCommand() {
	}
}
