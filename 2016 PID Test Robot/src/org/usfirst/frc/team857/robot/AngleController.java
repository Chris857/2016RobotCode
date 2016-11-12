package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		getPIDAngleController().setInputRange(-30,  100);
	}
	
	public AngleController(final double p, final double i, final double d, final double f, AnalogInput potentiometer, SpeedController motor) {
		
		//Call PIDSubsystem constructor
		super("Angle", p, i, d, f);
		
		//Setup IO
		_motor = motor;
		_potentiometer = potentiometer;
		
		//Set controller settings
		setAbsoluteTolerance(0.05);
		getPIDAngleController().setContinuous(false);

		getPIDAngleController().setInputRange(-30,  100);
	}
	
	
	//Compute angle given sensor.
	protected double returnPIDInput() {
		if (_encoder == null) {
			double a = -108.1918;
			double b = 283.3593;

			SmartDashboard.putNumber("POT Voltage", _potentiometer.getVoltage());
			SmartDashboard.putNumber("POT Angle", _potentiometer.getVoltage() * a + b);
			return _potentiometer.getVoltage() * a + b;
		} else {
			return _encoder.get();
		}
	}
	
	protected void usePIDOutput(double output) {
		SmartDashboard.putNumber("Angle PIDOutput", output);
		_motor.pidWrite(output);
	}

	protected void initDefaultCommand() {
		
	}
}
