package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDSourceType;


public class Robot_Test extends SampleRobot {
	private YetiPIDController pid;
	
	public void robotInit() {
		AnalogInput a = new AnalogInput(3);
		a.setPIDSourceType(PIDSourceType.kRate);
		
		
		pid = new YetiPIDController(0,0,0,0, 
				new AnalogInput(3), new VictorSP(8), 
				0.05);
		
		SmartDashboard.putNumber("P",0);
		SmartDashboard.putNumber("I",0);
		SmartDashboard.putNumber("D",0);
		SmartDashboard.putNumber("F", 0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("Output", 0);
		SmartDashboard.putBoolean("Reset", false);
	}

	public void autonomous() {
		while (isAutonomous() && isEnabled()) {

			
		}
	}
    
	public void operatorControl() {
		pid.enable();
		double prevSetpoint = SmartDashboard.getNumber("Setpoint");
        while (isOperatorControl() && isEnabled()) {

    		if (SmartDashboard.getBoolean("Reset")) {
    			pid.reset();
    			SmartDashboard.putBoolean("Reset", false);    			
    		}
    		
        	pid.setPID(SmartDashboard.getNumber("P"),
			    		SmartDashboard.getNumber("I"),
			    		SmartDashboard.getNumber("D"),
			    		SmartDashboard.getNumber("F"));
        	
        	double setpoint = SmartDashboard.getNumber("Setpoint");
        	if (setpoint != prevSetpoint) {
        		pid.setSetpoint(setpoint);
        	}
        	
    		SmartDashboard.putNumber("Output", pid.get());
        }
        
        //Disable the PID loop and reset to prevent errors on robot enable
        pid.disable();
        pid.reset();
	}
}
